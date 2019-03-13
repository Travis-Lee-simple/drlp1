#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

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

int main (int argc, char* argv[])
{
  // The point clouds we will be using
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);  // Original point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar (new pcl::PointCloud<pcl::PointXYZ>);  // Transformed point cloud

  cloud_in->width = 5;
  cloud_in->height = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);

  for (size_t i = 0; i < cloud_in->points.size(); ++i)
  {
      cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  cloud_tar = cloud_in;

  Eigen::Matrix4d T = CalculateT(cloud_in, cloud_tar, 100);
  print4x4Matrix(T);
  std::cerr << "Done" << std::endl;
  return (0);
}
