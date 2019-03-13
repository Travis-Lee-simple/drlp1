#include <iostream>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/joint_icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr DownSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2), cloud_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::toPCLPointCloud2(*cloud, *cloud_in);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    sor.filter(*cloud_blob);

    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_out);

    return cloud_out;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    printf ("Transform matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
    printf ("T = | %6.3f %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlannerSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int i = 0, nr_points = (int) cloud->points.size ();
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PassThroughFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.4, 0.8);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10, 0.1);
    pass.filter(*cloud_filtered);

    return cloud_filtered;
}

int main (int argc, char* argv[])
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

    // Checking program arguments
    if (argc < 2)
    {
        printf ("Usage :\n");
        printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
        PCL_ERROR ("Provide one ply file.\n");
        return (-1);
    }

    int iterations = 1;  // Default number of ICP iterations
    if (argc > 2)
    {
        // If the user passed the number of iteration as an argument
        iterations = atoi (argv[3]);
        if (iterations < 1)
        {
            PCL_ERROR ("Number of initial iterations must be >= 1\n");
            return (-1);
        }
    }

    pcl::console::TicToc time;
    time.tic ();

    if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }

    if (pcl::io::loadPLYFile (argv[2], *cloud_icp) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[2]);
        return (-1);
    }

//    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2), cloud_blob1 (new pcl::PCLPointCloud2), cloud_filtered_blob1 (new pcl::PCLPointCloud2);
//    pcl::PCDReader reader, reader1;
//    reader.read("./output_16.pcd", *cloud_blob);
//    reader1.read("./output_20.pcd", *cloud_blob1);
//
    std::cerr << "PointCloud0 before filtering: " << cloud_in->width * cloud_in->height << " data points." << std::endl;
    std::cerr << "PointCloud1 before filtering: " << cloud_icp->width * cloud_icp->height << " data points." << std::endl;

    PointCloudT::Ptr cloud_incut (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icpcut (new PointCloudT);  // ICP output point cloud

    cloud_in = PassThroughFilter(cloud_in);
    cloud_icp = PassThroughFilter(cloud_icp);

    std::cerr << "PointCloud0 after filtering: " << cloud_in->width * cloud_in->height << " data points." << std::endl;
    std::cerr << "PointCloud1 after filtering: " << cloud_icp->width * cloud_icp->height << " data points." << std::endl;

    // cloud_incut = PlannerSegmentation(cloud_in);
    // cloud_icpcut = PlannerSegmentation(cloud_icp);
    cloud_incut = cloud_in;
    cloud_icpcut = cloud_icp;

//    cloud_incut = DownSampling(cloud_in);
//    cloud_icpcut = DownSampling(cloud_icp);
//
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud(cloud_blob);
//    sor.setLeafSize(0.01f, 0.01f, 0.01f);
//    sor.filter(*cloud_filtered_blob);
//
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor1;
//    sor1.setInputCloud(cloud_blob1);
//    sor1.setLeafSize(0.01f, 0.01f, 0.01f);
//    sor1.filter(*cloud_filtered_blob1);
//
//    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_in);
//    pcl::fromPCLPointCloud2(*cloud_filtered_blob1, *cloud_icp);
//    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_in);
//    pcl::fromPCLPointCloud2(*cloud_blob1, *cloud_icp);

    std::cerr << "PointCloud0 after DownSampling: " << cloud_incut->width * cloud_incut->height << " data points." << std::endl;
    std::cerr << "PointCloud1 after DownSampling: " << cloud_icpcut->width * cloud_icpcut->height << " data points." << std::endl;

//    if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
//    {
//        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
//        return (-1);
//    }
    std::cout << "\nLoaded file " << argv[1] << argv[2] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//    double theta = M_PI / 8;  // The angle of rotation in radians
//    transformation_matrix (0, 0) = cos (theta);
//    transformation_matrix (0, 1) = -sin (theta);
//    transformation_matrix (1, 0) = sin (theta);
//    transformation_matrix (1, 1) = cos (theta);
//
//    // A translation on Z axis (0.4 meters)
//    transformation_matrix (2, 3) = 0.4;

//    transformation_matrix (0, 0) = 0.986;
//    transformation_matrix (0, 1) = 0.121;
//    transformation_matrix (0, 2) = -0.112;
//    transformation_matrix (0, 3) = 0.031;
//    transformation_matrix (1, 0) = -0.127;
//    transformation_matrix (1, 1) = 0.991;
//    transformation_matrix (1, 2) = -0.044;
//    transformation_matrix (1, 3) = -0.038;
//    transformation_matrix (2, 0) = 0.105;
//    transformation_matrix (2, 1) = 0.058;
//    transformation_matrix (2, 2) = 0.993;
//    transformation_matrix (2, 3) = -0.002;

    transformation_matrix (0, 0) = 1;
    transformation_matrix (0, 1) = 0;
    transformation_matrix (0, 2) = 0;
    transformation_matrix (0, 3) = 0;
    transformation_matrix (1, 0) = 0;
    transformation_matrix (1, 1) = -0.7071;
    transformation_matrix (1, 2) = 0.7071;
    transformation_matrix (1, 3) = 0;
    transformation_matrix (2, 0) = 0;
    transformation_matrix (2, 1) = -0.7071;
    transformation_matrix (2, 2) = -0.7071;
    transformation_matrix (2, 3) = 0;

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix (transformation_matrix);

    // Executing the transformation
//    pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
//    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
//    pcl::transformPointCloud (*cloud_icp, *cloud_icp, transformation_matrix);
//    pcl::transformPointCloud (*cloud_in, *cloud_in, transformation_matrix);
    *cloud_tr = *cloud_icpcut;
//    cloud_tr = PassThroughFilter(cloud_icpcut);

    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icpcut);
//    icp.addInputSource(cloud_icp);
//    icp.addInputTarget(cloud_in);
    icp.setInputTarget (cloud_incut);
    icp.align (*cloud_icpcut);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_incut, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_incut, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_incut, cloud_in_color_h, "cloud_in_v2", v2);

    viewer.addCoordinateSystem(0.5);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icpcut, 180, 20, 20);
    viewer.addPointCloud (cloud_icpcut, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
//    viewer.setCameraPosition (-3, 3, 0, 0, 0, -0, 0);
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();

        // The user pressed "space" :
        if (next_iteration)
        {
            // The Iterative Closest Point algorithm
            time.tic ();
            icp.align (*cloud_icpcut);
            std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

            if (icp.hasConverged ())
            {
//                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud (cloud_icpcut, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR ("\nICP has not converged.\n");
                return (-1);
            }
        }
        next_iteration = false;
    }
    return (0);
}