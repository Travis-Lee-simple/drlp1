// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////////
// librealsense tutorial #2 - Accessing multiple streams //
///////////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <librealsense2/rs.hpp>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <unistd.h>

#include "robotcontrol.h"
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include "camera.h"

//矩阵变换需要的头文件
#include <math.h>
#include <Eigen/Dense>

//点云需要的头文件
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//机械臂IP号
#define ROBOT_ADDR "192.168.1.100"
//机械臂端口号
#define ROBOT_PORT 8899
#define PI 3.14159265

using namespace aubo_robot_namespace;
using namespace Eigen;

//如果不加任何末端执行器，工具末端要到达的位姿
//也就是我们通过视觉检测，我们想要找的工件在基坐标系下的六维位姿
double detect_x = -0.467064;
double detect_y = -0.295938;
double detect_z = 0.613196;
double detect_rx = 90.4122;
double detect_ry = 29.388;
double detect_rz = 10.236;


//工具末端相对于法兰盘中心的位姿
//我们通过四点法或五点法（通过示教器操作机器人move到同一尖端四次），得到的六维位姿
double tool_toend_x = -0.000115;
double tool_toend_y = -0.003273;
double tool_toend_z = 0.292375;
double tool_toend_rx = 0;
double tool_toend_ry = 0;
double tool_toend_rz = 0;

//相机坐标系在基坐标系下的位姿
double camera_tobase_x;
double camera_tobase_y;
double camera_tobase_z;
double camera_tobase_rx;
double camera_tobase_ry;
double camera_tobase_rz;

//产线数量
long num_project1 = 0;
long num_project2 = 0;
long num_project3 = 0;
long num_project4 = 0;

//机械臂各关节最大速度
double joint_maxvelc[6] = { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 };
//机械臂各关节最大加速度
double joint_maxacc[6] = { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 };

//DEMO show
//CamInToolPose:=[-0.0372853, 0.0445237, 0.174401, 0.514923, 353.692, 223.758, 2]
//new result
//CamInToolPose:=[-0.0376789, 0.0384617, 0.159592, 351.88, 359.287, 223.849, 2]
//相机坐标系相对于法兰末端的位姿，由手眼标定流程得到
double camera_to_end_x = -0.0376789;
double camera_to_end_y = 0.0384617;
double camera_to_end_z = 0.159592;
double camera_to_end_rx = 351.88;
double camera_to_end_ry = 359.287;
double camera_to_end_rz = 223.849;

//机械臂在 standard 拍照位姿的读数
// double x_takepicture_standard = -0.401084;
// double y_takepicture_standard = 0.247543;
// double z_takepicture_standard = 0.192644;
// double rx_takepicture_standard = 129.169;
// double ry_takepicture_standard = 22.5429;
// double rz_takepicture_standard = -95.9039; 
double x_takepicture_standard = -0.336586;
double y_takepicture_standard = 0.273153;
double z_takepicture_standard = 0.306685;
double rx_takepicture_standard = 129.0673;
double ry_takepicture_standard = 22.5889;
double rz_takepicture_standard = -105.0930; 

//机械臂在 standard 待装位姿的读数
// double x_readytowork_standard = -0.585806;
// double y_readytowork_standard = 0.368056;
// double z_readytowork_standard = -0.053315;
// double rx_readytowork_standard = 95.9869;
// double ry_readytowork_standard = 42.5693;
// double rz_readytowork_standard = -101.0956;
double x_readytowork_standard = -0.555380;
double y_readytowork_standard = 0.367102;
double z_readytowork_standard = -0.042330;
double rx_readytowork_standard = 98.2305;
double ry_readytowork_standard = 44.7851;
double rz_readytowork_standard = -126.0937;

//机械臂在 process 拍照位姿的读数
// double x_takepicture_process = -0.449439;
// double y_takepicture_process = 0.094117;
// double z_takepicture_process = 0.136272;
// double rx_takepicture_process = 126.9948;
// double ry_takepicture_process = 45.3610;
// double rz_takepicture_process = -101.0124;
double x_takepicture_process = -0.373014;
double y_takepicture_process = 0.214170;
double z_takepicture_process = 0.302593;
double rx_takepicture_process = 135.9982;
double ry_takepicture_process = 29.7334;
double rz_takepicture_process = -101.3676;

//机械臂在 process 经过点云匹配得到的校准位姿的读数
double x_calib_process ;
double y_calib_process ;
double z_calib_process ;
double rx_calib_process ;
double ry_calib_process ;
double rz_calib_process ;

//机械臂在 process 待装位姿的读数
double x_readytowork_process;
double y_readytowork_process;
double z_readytowork_process;
double rx_readytowork_process;
double ry_readytowork_process;
double rz_readytowork_process;

//API中暂时存储的六维参数
double x;
double y;
double z;
double rx;
double ry;
double rz;


MatrixXd T_takepicture_standard(4, 4);// standard 拍照时 法兰末端坐标系在基坐标系下位姿的T
MatrixXd T_readytowork_standard(4, 4);// standard 待装时 法兰末端坐标系在基坐标系下位姿的T
MatrixXd T_trans_standard(4, 4);// standard 待装时相机坐标系 在 拍照时相机坐标系 下位姿的T


MatrixXd T_camera_toend(4, 4);//相机坐标系在法兰末端坐标系下 位姿的T


MatrixXd T_takepicture_process_end(4, 4);// process 拍照时法兰末端在基坐标系下位姿的T
MatrixXd T_takepicture_process_camera_inbase(4, 4);//process 拍照时相机坐标系在基坐标系下位姿的T


MatrixXd T_calib_process_end(4, 4);// process 校准后法兰末端在基坐标系下位姿的T
MatrixXd T_calib_process_camera_inbase(4, 4);// process 校准后相机坐标系在基坐标系下位姿的T
MatrixXd T_readytowork_process(4, 4);// process 待装时法兰末端在基坐标系下位姿的T

//API中暂时存储的T
MatrixXd T(4,4);

////////////////////////////////////////////////////////////////////////////计算六维位姿的T///////////////////////////////////////////////////////////////////////////
//输入：六维位姿
//输出：T
void T_get(double x, double y, double z, double rx, double ry, double rz){
    rx = rx/ 180 * PI;
    ry = ry / 180 * PI;
    rz = rz / 180 * PI;

    MatrixXd Rx(3, 3);
    MatrixXd Ry(3, 3);
    MatrixXd Rz(3, 3);
    
    Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
    Rz << cos(rz), - sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;

    MatrixXd R(3, 3);
    R = Rz * Ry * Rx;

    MatrixXd P(3, 1);
    P << x, y, z;
    T << R, P, MatrixXd::Zero(1, 3), MatrixXd::Identity(1,1);
}
///////////////////////////////////////////////////////////////////////////计算T的六维位姿///////////////////////////////////////////////////////////////////////////


//输入：T
//输出：六维位姿
void get_pose_fromT(MatrixXd T){

    x = T(0, 3);
    y = T(1, 3);
    z = T(2, 3);
    rx = atan2(T(2, 1), T(2, 2)) /PI * 180;
    ry = asin(-T(2, 0)) / PI * 180;
    rz = atan2(T(1, 0), T(0, 0)) / PI * 180;

    cout<< x <<endl;
    cout<< y <<endl;
    cout<< z <<endl;
    cout<< rx <<endl;
    cout<< ry <<endl;
    cout<< rz <<endl;
}


/////////////////////////////////////////////////////////////////////////// 由点得到点云///////////////////////////////////////////////////////////////////////////


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


///////////////////////////////////////////////////////////////////////////保存点云///////////////////////////////////////////////////////////////////////////


void save_pts2ply(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, char * path)
{
    pcl::PLYWriter writer;
    writer.write(path, *point_cloud, true);
}


///////////////////////////////////////////////////////////////////////////得到点云图///////////////////////////////////////////////////////////////////////////


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


///////////////////////////////////////////////////////////////////////////对点云图进行平台切割///////////////////////////////////////////////////////////////////////////


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

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

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


///////////////////////////////////////////////////////////////////////////对点云图进行降采样///////////////////////////////////////////////////////////////////////////


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

pcl::PointCloud<pcl::PointXYZ>::Ptr PassThroughFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.45, 0.7);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10, 0.1);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.4, 0.4);
    pass.filter(*cloud_filtered);

    return cloud_filtered;
}


///////////////////////////////////////////////////////////////////////////打印矩阵///////////////////////////////////////////////////////////////////////////


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


/////////////////////////////////////////////////////////计算 process点云图 与 standard点云图之间的T/////////////////////////////////////////////////////////


Eigen::Matrix4d CalculateT (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, int iterations = 1)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud2);
    icp.setInputTarget (cloud1);
    icp.align (*cloud2);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
//        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        std::cerr << "FFFFFFFFFFFFFFFuck!!!!!!!!!!" << std::endl;
    }

    return transformation_matrix;
}


/////////////////////////////////////////////////////////////////////////////主函数///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    //由 standard 的拍照位姿得到T
    T_get(x_takepicture_standard,
          y_takepicture_standard,
          z_takepicture_standard,
         rx_takepicture_standard,
         ry_takepicture_standard,
         rz_takepicture_standard);
    T_takepicture_standard = T;
    
    //由 standard 的待装位姿得到T
    T_get(x_readytowork_standard,
          y_readytowork_standard, 
          z_readytowork_standard, 
         rx_readytowork_standard, 
         ry_readytowork_standard,
         rz_readytowork_standard);
    T_readytowork_standard = T;

    // 目标坐标系位姿的T = 参考坐标系位姿的T × 目标坐标系在参考坐标系下位姿的转移T
    T_trans_standard = T_takepicture_standard.inverse()* T_readytowork_standard;

    //相机坐标系在法兰坐标系下位姿的T，由手眼标定得到
    T_get(camera_to_end_x,
          camera_to_end_y, 
          camera_to_end_z, 
          camera_to_end_rx, 
          camera_to_end_ry,
          camera_to_end_rz);
    T_camera_toend = T;


    //for depth camera
    /*
    int width = 640;
    int height = 480;

    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::align align_to(RS2_STREAM_COLOR);

    rs2::pipeline_profile selection = p.start(cfg);
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrin_ = depth_stream.get_intrinsics();
    intrinsic_param intrin;
    intrin.Set(intrin_.width, intrin_.height, intrin_.fx, intrin_.fy, intrin_.ppx, intrin_.ppy);

    float *pointcloud = new float[width*height*3];
    
    
    while(true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames(); 

        rs2::frame color_frame = frames.get_color_frame();
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
     
        // Get the depth frame's dimensions
        rs2::depth_frame depth = frames.get_depth_frame(); 
        cv::Mat depth_image = cv::Mat(height, width, CV_16UC1);
        cv::Mat depthColor = cv::Mat(depth_image.rows,depth_image.cols, CV_8UC3);
        depth_image.data = (unsigned char*)depth.get_data();


        GetPointCloud((unsigned short*)depth_image.data, width, height, intrin, pointcloud);

        convertDepthToColor(depth_image, depthColor);
     

        cv::namedWindow("color");
        cv::imshow("color", color_image);
        cvWaitKey(1);

        cv::namedWindow("depth2");
        cv::imshow("depth2", depthColor);
        cvWaitKey(1);
    }
    */


    //如果机械臂能够登录
    if (auboi5_login(ROBOT_ADDR, ROBOT_PORT))
    {
        //启动机械臂(必须连接真实机械臂）
        auboi5_robotStartup();
        
        //机械臂关节属性设置
        auboi5_setproperty(joint_maxvelc, joint_maxacc);

        //首先去 standard 位置准备拍摄标准点云图
        auboi5_movetosetposition(x_takepicture_standard, 
                                 y_takepicture_standard, 
                                 z_takepicture_standard, 
                                rx_takepicture_standard, 
                                ry_takepicture_standard, 
                                rz_takepicture_standard);

        //拍摄 standard 点云图
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_standard;
        cloud_standard = GetPointCloud();
        
        //保存 standard 点云图
        string standard_pointcloud_output = "/home/chen/AIrobot/standard.ply";
        char *standard;
        int len_standard = standard_pointcloud_output.length();
        standard = (char*)malloc((len_standard+1)*sizeof(char));
        standard_pointcloud_output.copy(standard, len_standard, 0);

        //对 standard 点云图进行降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_standard_afterfilter;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_standard_afterseg;
        cloud_standard_afterfilter = PassThroughFilter(cloud_standard);
        cloud_standard_afterfilter = PlannerSegmentation(cloud_standard_afterfilter);
        save_pts2ply(cloud_standard_afterfilter, standard);
        std::cerr << "PointCloud standard has : " << cloud_standard_afterfilter->width * cloud_standard_afterfilter->height << " data points." << std::endl;
        //cloud_standard_afterseg = DownSampling(cloud_standard_afterfilter);
        //std::cerr << "PointCloud representing the planar component: " << cloud_standard_afterseg->width * cloud_standard_afterseg->height << " data points." << std::endl;
        
        // Cut the plane.
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_standard_afterseg;
        //cloud_standard_afterseg = PlannerSegmentation(cloud_standard);
        


        //移动到待装配位姿，写在这里作为观察误差使用，实际操作请注释
        auboi5_movetosetposition(x_readytowork_standard, 
                                 y_readytowork_standard, 
                                 z_readytowork_standard, 
                                rx_readytowork_standard, 
                                ry_readytowork_standard, 
                                rz_readytowork_standard);

        int i = 0 ;
        while (i< 10){

            //接受待检测的位姿，待定
            auboi5_movetosetposition(-0.28, -0.3, 0.55, 30.0, 50.0, -90.0);
            
            //移动到 process 拍照位姿
            auboi5_movetosetposition(x_takepicture_process, 
                                     y_takepicture_process, 
                                     z_takepicture_process, 
                                    rx_takepicture_process, 
                                    ry_takepicture_process, 
                                    rz_takepicture_process);
            usleep(1000);

            std::cerr << "FFFFFFFFFFFFFFFuck2 epoch : " << i << " Start get PointCloud\n" << std::endl;
            
            //拍摄 process 中的点云图
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_process;
            cloud_process = GetPointCloud();

            //保存 process 中的点云图
            string process_pointcloud_output = "/home/chen/AIrobot/process.ply";
            char *process;
            int len_process = process_pointcloud_output.length();
            process = (char*)malloc((len_process+1)*sizeof(char));
            process_pointcloud_output.copy(process, len_process, 0);

            std::cerr << "FFFFFFFFFFFFFFFuck2 epoch : " << i << " Get PointCloud Done \n" << std::endl;

            std::cerr << "PointCloud process0 has : " << cloud_process->width * cloud_process->height << " data points." << std::endl;

            //对 process 中的点云图进行降采样
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_process_afterfilter;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_process_afterseg;
            cloud_process_afterfilter = PassThroughFilter(cloud_process);
            cloud_process_afterfilter = PlannerSegmentation(cloud_process_afterfilter);
            save_pts2ply(cloud_process_afterfilter,process);

            std::cerr << "PointCloud process1 has : " << cloud_process_afterfilter->width * cloud_process_afterfilter->height << " data points." << std::endl;

            std::cerr << "FFFFFFFFFFFFFFFuck2 epoch : " << i << " DownSampling Done \n" << std::endl;
            //cloud_process_afterseg = DownSampling(cloud_process_afterfilter);
            //std::cerr << "PointCloud representing the planar component: " << cloud_process_afterseg->width * cloud_process_afterseg->height << " data points." << std::endl;

            // Cut the plane.
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_process_afterseg;
            // cloud_process_afterseg = PlannerSegmentation(cloud_process);

            //两个点云图存在如下关系：
            // cloud_standard_afterseg = T(由CalculateT()计算出的) * cloud_process_afterseg
            // T_process_tostandard就是T
            // 第一个参数： 标准点云图
            // 第二个参数： 进程中的点云图
            // 第三个参数为迭代次数
            Eigen::Matrix4d T_process_tostandard = CalculateT(cloud_standard_afterfilter, cloud_process_afterfilter, 50);
               
            // 将T打印出来
            print4x4Matrix(T_process_tostandard);

            // 得到 process 拍照时法兰末端的位姿
            T_get(x_takepicture_process,
                  y_takepicture_process, 
                  z_takepicture_process, 
                 rx_takepicture_process, 
                 ry_takepicture_process,
                 rz_takepicture_process);
            T_takepicture_process_end = T;
            
            // 得到 process 中拍照时相机坐标系的位姿
            T_takepicture_process_camera_inbase = T_takepicture_process_end * T_camera_toend;

            // 得到 process 中校准后相机坐标系的位姿
            // 依据的公式为：T_takepicture_process_camera_inbase = T_calib_process_camera_inbase * T
            T_calib_process_camera_inbase = T_takepicture_process_camera_inbase * T_process_tostandard.inverse() ;

            // 得到 process 中校准后法兰末端的位姿
            T_calib_process_end = T_calib_process_camera_inbase * T_camera_toend.inverse();
            
            // 由T_calib_process_end得到法兰末端要移动到的校准位姿
            get_pose_fromT(T_calib_process_end);
            
             x_calib_process = x;
             y_calib_process = y;
             z_calib_process = z;
            rx_calib_process = rx;
            ry_calib_process = ry;
            rz_calib_process = rz;

            // 移动到校准位姿
            auboi5_movetosetposition(x_calib_process, 
                                    y_calib_process, 
                                    z_calib_process, 
                                   rx_calib_process, 
                                   ry_calib_process, 
                                   rz_calib_process);

            // 得到 process 待装时的法兰末端位姿的T
            T_readytowork_process = T_calib_process_end*T_trans_standard;
            // 得到六维位姿
            get_pose_fromT(T_readytowork_process);
            x_readytowork_process = x;
            y_readytowork_process = y;
            z_readytowork_process = z;
            rx_readytowork_process = rx;
            ry_readytowork_process = ry;
            rz_readytowork_process = rz;
            //移动到待装位姿
            auboi5_movetosetposition(x_readytowork_process, 
                                     y_readytowork_process, 
                                     z_readytowork_process, 
                                    rx_readytowork_process, 
                                    ry_readytowork_process, 
                                    rz_readytowork_process);
            // 流水线次数+1 
            i++;
        }

        //延时2两秒，观察回调函数
        usleep(2000);

        //关闭机械臂（必须连接真实机械臂）
        //auboi5_robotShutdown(g_rshd);

        //退出登录
        auboi5_logout();
        
    }

   
    return 0;
}