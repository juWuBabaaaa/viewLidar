#include <iostream>
#include <thread>
// #include "utils/tools.h"
# include "utils/viewKitti.h"


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
    // string dp = "/home/wsx/pointcloud_research/data/nuscenes/samples/LIDAR_TOP";
    // string ap = "/home/wsx/pointcloud_research/data/nuscenes/anno_cplus";

    // // 1. watch kitti training box
    // string dp = "/media/wsx/Elements/kitti/training/velodyne_reduced";
    // string ap = "/media/wsx/Elements/kitti/training/velodyne_reduced_box3d_rectified";
    // 2. watch kitti model results
    string dp = "/media/wsx/Elements/kitti/training/velodyne_reduced";
    string ap = "/media/wsx/Elements/kitti/training/predictions_rectified";
    int mode = 3;  // mode: 1. pcd; 2. bin; 3. kitti
    bool score = true;
    view_predictions_kitti(dp, ap, mode, score);
    // view(dp, mode);
}
