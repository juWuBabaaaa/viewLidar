# include <iostream>
# include "tools.h"

using namespace std;

/*
    argv need: lidar path, box path.
    labeled box format: label, x, y, z, l, w, h, yaw
    predicted box format: label, x, y, z, l, w, h, yaw, score.

    Execute: ./viewFrame lidarpath boxpath
*/ 

int main(int argn, char** argv) {
    int pcd_format = 2; // 0 pcd, 1 bin, 2 kitti
    int box_format = 1;  // 0 labeled box, 1 predicted box
    view_frame(argv[1], argv[2], pcd_format, box_format);
    return 0;
}