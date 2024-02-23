#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>

#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
# include "tools.h"

#include <fstream>
#include <vector>

using namespace std;

struct Label {
    string type;
    vector <float> vec;
};

Label line_parse(string s, char del) {
    Label label;
    stringstream ss(s);
    string word;
    int i=0;
    while (!ss.eof()) {
        getline(ss, word, del);
        if (i == 0) {
            label.type = word;
        } else {
            label.vec.push_back(stof(word));
        }
        i++;
    }
    return label;
}

vector<vector<float>> load_box_kitti(string anno_path) {
    // 8位数
    string line;
    char delimiter = ' ';
    string item ;
    size_t pos = 0;
    ifstream annofile (anno_path);
    vector<vector<float>> re;

    if (annofile.is_open()) {
        while (getline (annofile, line)){
            vector<float> tmp;
            // cout << line << endl;
            Label l = line_parse(line, delimiter);
            tmp.push_back(stof(l.type));
            for (auto v:l.vec) {
                tmp.push_back(v);
            }
            re.push_back(tmp);
        }
    }
    return re;
}

string ToString(float val) {
    stringstream ss;
    ss << setiosflags(ios::fixed) << setprecision(2) << val;
    string str = ss.str();
    return str;
}

void view_predictions_kitti(string dp, string ap, int mode, bool score) {
    /*
        'car': 0,
        'truck': 1,
        'trailer': 2,
        'bus': 3,
        'construction_vehicle': 4,
        'bicycle': 5,
        'motorcycle': 6,
        'pedestrian': 7,
        'traffic_cone': 8,
        'barrier': 9
    */
    const float colors[10][3] = {
        {1.0, 0., 0.},
        {0, 1, 0},
        {0., 0., 1.},
        {1, 1, 0},
        {1., 0, 1},
        {0., 1., 1.},
        {0.3, 0, 0},
        {0., 0.3, 0.},
        {0, 0., 0.3},
        {0.3, 0.3 , 0}
    };
    vector<string> files = sort_fp(ap);

    /*
        ruc tianqiao camera parameters:
        pos: -25, -1, 30
        view: 0.4, 0.05, 1
        focal: 7.325246, 0.375848, 7.327828
    */ 

    // double pos[3] = {-25, -1, 30};
    // camera.pos = pos;
    // camera.pos = new float;
    // camera.view = {0.4, 0.05, 1};
    // camera.focal = {7.325246, 0.375848, 7.327828};
    int windowsize[] = {4096, 2560};
    for (const auto entry:files) {
        cout << entry << endl;

        string fn = entry.substr(entry.find_last_of("/")+1);
        fn = fn.substr(0, fn.find_last_of("."));
        string pcd_file = dp + "/" + fn + ".bin";
        cout << pcd_file << endl;
        vector<vector<float>> re = load_box_kitti(entry);

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (fn));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
        pcl::visualization::Camera camera;
        viewer->getCameraParameters(camera);
        camera.pos[0] = -15.8;
        camera.pos[1] = -2.16;
        camera.pos[2] = 13.58;
        camera.view[0] = 0.25;
        camera.view[1] = -0.05;
        camera.view[2] = 0.96;
        camera.focal[0] = 23.15;
        camera.focal[1] = 0.06;
        camera.focal[2] = 3.48;
        viewer->setSize(4096, 2560);
        viewer->setCameraParameters(camera);

        if(mode == 1){
            point_cloud = load_pcd(pcd_file);
        } else if (mode == 2)
        {
            point_cloud = load_bin(pcd_file);
        } else if (mode == 3) {
            point_cloud = load_kitti(pcd_file);
        } else {
            return;
        }
        
        viewer->addPointCloud<pcl::PointXYZ> (point_cloud, "cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        int j = 0;
        const float* color;
        int count_box = 0;
        for (auto box : re) {
            count_box ++;
            if (score) {
                pcl::PointXYZ point;
                point.x = box[1];
                point.y = box[2];
                point.z = box[3];
                viewer->addText3D(ToString(box[8]), point, 0.6, 1., 1., 1., to_string(count_box)+"score");
            }

            color = colors[int(box[0])];
            const string id = to_string(j);
            const Vector3f translation(box[1], box[2], box[3]);
            cout << box[1] << " " << box[2] << " " << box[3] << " " << box[4] << " " << box[5] << " " << box[6] << endl;
            AngleAxisf t_V(box[7], Vector3f(0, 0, 1));
            Matrix3f t_R = t_V.matrix();
            const Quaternionf rotation(t_V);
            viewer->addCube(translation, rotation, box[4], box[5], box[6], id);
            // viewer->addCube(xmin, xmax, ymin, ymax, zmin, zmax, color[0], color[1], color[2], id);
            viewer->setRepresentationToWireframeForAllActors ();
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2], id);
            j ++ ;
        }
        viewer->getCameraParameters(camera);
        printf("pos: %lf, %lf, %lf\n", camera.pos[0], camera.pos[1], camera.pos[2]);
        printf("view: %lf, %lf, %lf\n", camera.view[0], camera.view[1], camera.view[2]);
        printf("focal: %lf, %lf, %lf\n", camera.focal[0], camera.focal[1], camera.focal[2]);
        printf("clip: %lf, %lf\n", camera.clip[0], camera.clip[1]);
        printf("fovy: %lf\n", camera.fovy);
        printf("window size: %lf, %lf\n", camera.window_size[0], camera.window_size[1]);
        printf("window pos: %lf, %lf\n", camera.window_pos[0], camera.window_pos[1]);
        printf("======================\n");
        // viewer->spin();
        while (!viewer->wasStopped()) {
            viewer -> spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }
    }
}

