#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

void save_pts2ply(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, char * path)
{
    pcl::PLYWriter writer;
    writer.write(path, *point_cloud, true);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GetPointCloud()
{
    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;
    pipe.start();
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);

    auto pcl_points = points_to_pcl(points);

    return pcl_points;
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
    while (cloud->points.size() > 0.3 * nr_points)
    {
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
    }

    return cloud;
}

int main(int argc, char * argv[]) try
{
    // Get the point cloud file.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = GetPointCloud();

    // Cut the plane.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_seg;
    cloud_after_seg = PlannerSegmentation(cloud);

    save_pts2ply(cloud, argv[1]);
    save_pts2ply(cloud_after_seg, argv[2]);

    pcl::visualization::PCLVisualizer viewer ("Final1 with Visualization");
    std::cerr << "Done!" << std::endl;

    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h (cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud, cloud_in_color_h, "cloud_in_v1", v1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h (cloud_after_seg, 20, 180, 20);
    viewer.addPointCloud (cloud_after_seg, cloud_tr_color_h, "cloud_tr_v1", v2);

    viewer.addText ("The Fucking Original Point Cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("The Fucking Processed Point Cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    std::cerr << "FFFFFFFFFFFuck!!!!!!!!!!!" << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    std::cerr << "FFFFFFFFFFFuck!!!!!!!!!!!" << std::endl;
    return EXIT_FAILURE;
}
