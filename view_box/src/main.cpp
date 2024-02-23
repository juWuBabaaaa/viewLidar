# include <iostream>
# include <vector>
# include "utils/tools.h"
using namespace std;

int main(int argc, char ** argv){
    // string lidar_path = "/home/wsx/pointcloud_research/data/nuscenes/samples/LIDAR_TOP/n008-2018-05-21-11-06-59-0400__LIDAR_TOP__1526915243047392.pcd.bin";
    // string anno_path = "/home/wsx/pointcloud_research/data/nuscenes/anno_cplus/n008-2018-05-21-11-06-59-0400__LIDAR_TOP__1526915243047392.pcd.bin.txt";
        string lidar_path = "/home/wsx/pointcloud_research/data/nuscenes/samples/LIDAR_TOP/n015-2018-07-18-11-07-57+0800__LIDAR_TOP__1531883530449377.pcd.bin";
    string anno_path = "/home/wsx/pointcloud_research/data/nuscenes/anno_cplus/n015-2018-07-18-11-07-57+0800__LIDAR_TOP__1531883530449377.pcd.bin.txt";
    view(lidar_path, anno_path);

    return 0;
}