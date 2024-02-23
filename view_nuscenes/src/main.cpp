#include <iostream>
#include <thread>
#include "utils/tools.h"


// pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, Eigen::Vector3f center, float distance)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------

//   return (viewer);
// }

int main(int argc, char** argv) {
    // 1. /home/wsx/pointcloud_research/data/pointclouds
    // 2. /home/wsx/pointcloud_research/data/nuscenes/samples/LIDAR_TOP
    // string dp = "/home/wsx/pointcloud_research/data/pointclouds";
    string dp = "/home/wsx/pointcloud_research/data/nuscenes/samples/LIDAR_TOP";
    string ap = "/home/wsx/pointcloud_research/data/nuscenes/anno_cplus";
    int mode = 2;
    view(dp, ap, mode);
    // view(dp, mode);
}


