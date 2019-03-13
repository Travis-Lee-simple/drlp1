#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

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


pcl::PointCloud<pcl::PointXYZ>::Ptr ReadFromPCDFile(char * path)
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(path, *cloud_blob);
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);
    printf("Fucking Reading Done!!!");

    return cloud;
}

void save_pts2ply(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, char * path)
{
    pcl::PLYWriter writer;
    writer.write(path, *point_cloud);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr DownSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2), cloud_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::toPCLPointCloud2(*cloud, *cloud_in);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_blob);

    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_out);

    return cloud_out;
}

int main(int argc, char * argv[]) try
{
    // Get the point cloud file.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = GetPointCloud();

    // Cut the plane.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_seg;
    cloud_after_seg = PassThroughFilter(cloud);
    cloud_after_seg = PlannerSegmentation(cloud_after_seg);
    cloud_after_seg = DownSampling(cloud_after_seg);

    std::cerr << "Done!" << std::endl;
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
