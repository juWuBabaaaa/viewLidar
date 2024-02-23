# include <fstream>
# include <string>
# include <pcl/common/common_headers.h>
# include <pcl/features/normal_3d.h>
# include <pcl/io/pcd_io.h>
# include <pcl/visualization/pcl_visualizer.h>
# include <pcl/console/parse.h>
# include <vector>
# include <algorithm>
# include <filesystem>
# include <eigen3/Eigen/Dense>
# include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;
namespace fs = filesystem;
/* 1. load files */ 
pcl::PointCloud<pcl::PointXYZ>::Ptr load_bin(string fp){
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    float f;
    char * memblock;
    int row = 0,c = 0;
    ifstream file(fp, ios::binary);
    vector<float> cont;
    while (file.read(reinterpret_cast<char*>(&f), sizeof(float)))
    {
        cont.push_back(f);
    }
    int nrows = cont.size()/5;
    vector<vector<float>> tmp(nrows, vector<float>(5));
    for (int i=0; i< nrows; i++) {
        for (int j=0; j< 5; j++) {
            tmp[i][j] = cont[5*i+j];
        }
    }

    for (int i=0; i < tmp.size(); i++) {
        pcl::PointXYZ basic_point;
        basic_point.x = tmp[i][0];
        basic_point.y = tmp[i][1];
        basic_point.z = tmp[i][2];
        // cout << tmp[i][0] << " " << tmp[i][1] << " " << tmp[i][2] << endl; 
        // cout << basic_point.x << " " << basic_point.y << " " << basic_point.z << endl;
        point_cloud->points.push_back(basic_point);
    }
    point_cloud->width = point_cloud->size ();
    point_cloud->height = 1;
    return point_cloud;
}

vector<vector<float>> load_box(string anno_path){
    string line;
    string delimiter = string(" ");
    string item ;
    size_t pos = 0;
    ifstream annofile (anno_path);
    vector<vector<float>> re;

    if (annofile.is_open()) {
        while (getline (annofile, line)){
            vector<float> tmp;
            // cout << line << endl;
            while((pos = line.find(delimiter)) != string::npos) {
                item = line.substr(0, pos);
                tmp.push_back(stof(item));
                line.erase(0, pos + delimiter.length());
            }
            re.push_back(tmp);
        }
    }
    return re;
}

void view(string lidar_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Road scenario"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
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
    viewer->setCameraParameters(camera);
    point_cloud = load_bin(lidar_path);
    viewer->addPointCloud<pcl::PointXYZ> (point_cloud, to_string(0));
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(0));
    viewer->spinOnce(1000000);
}

void view(string lidar_path, string anno_path) {
    vector<vector<float>> re = load_box(anno_path);
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Road scenario"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera)
    ;
    camera.pos[0] = -15.8;
    camera.pos[1] = -2.16;
    camera.pos[2] = 13.58;
    camera.view[0] = 0.25;
    camera.view[1] = -0.05;
    camera.view[2] = 0.96;
    camera.focal[0] = 23.15;
    camera.focal[1] = 0.06;
    camera.focal[2] = 3.48;
    viewer->setCameraParameters(camera);
    point_cloud = load_bin(lidar_path);
    viewer->addPointCloud<pcl::PointXYZ> (point_cloud, to_string(0));
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(0));
    int i = 0;
    const float* color;
    for (auto box : re) {
        if (int(box[0]) == -1) {
            continue;
        }
        color = colors[int(box[0])];
        const string id = to_string(i);
        const Vector3f translation(box[1], box[2], box[3]);
        AngleAxisf t_V(box[7], Vector3f(0, 0, 1));
        Matrix3f t_R = t_V.matrix();
        const Quaternionf rotation(t_V);
        viewer->addCube(translation, rotation, box[4], box[5], box[6], id);
        // viewer->addCube(xmin, xmax, ymin, ymax, zmin, zmax, color[0], color[1], color[2], id);
        viewer->setRepresentationToWireframeForAllActors ();
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2], id);
        cout << box[0] << endl;
        i ++ ;
    }
    viewer->spinOnce(1000000);
}