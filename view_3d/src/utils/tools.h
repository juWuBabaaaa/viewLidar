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
# include <thread>

using namespace std;
using namespace Eigen;
namespace fs = filesystem;
/* 1. load files */ 
pcl::PointCloud<pcl::PointXYZ>::Ptr load_kitti(string fp){
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
    int nrows = cont.size()/4;
    vector<vector<float>> tmp(nrows, vector<float>(4));
    for (int i=0; i< nrows; i++) {
        for (int j=0; j< 4; j++) {
            tmp[i][j] = cont[4*i+j];
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

pcl::PointCloud<pcl::PointXYZ>::Ptr load_pcd(string fp){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fp, *cloud)==-1)
    {
        PCL_ERROR("Couldn't read file %s\n", fp);
    }
    return cloud;
}

/*
    2. return sorted file names. 
    Similar to sorted(os.listdir)
*/
vector<string> sort_fp(string dp) {
    vector<string> tmp;
    for (const auto & entry : fs ::directory_iterator(dp)) {
        tmp.push_back(entry.path());
    }
    sort(tmp.begin(), tmp.end());
    return tmp;
}

/*
    3. view pcd files in video way.
    input: directory path
*/

void view(string dp, int mode) {
    vector<string> files = sort_fp(dp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Road scenario"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    /*
        ruc tianqiao camera parameters:
    */
    camera.pos[0] = -15.8;
    camera.pos[1] = -2.16;
    camera.pos[2] = 13.58;
    camera.view[0] = 0.25;
    camera.view[1] = -0.05;
    camera.view[2] = 0.96;
    camera.focal[0] = 23.15;
    camera.focal[1] = 0.06;
    camera.focal[2] = 3.48;

    // /*
    //     Nuscenes camera parameters:
    // */
    // camera.pos[0] = 6.43;
    // camera.pos[1] = -81.45;
    // camera.pos[2] = 29.08;
    // camera.view[0] = -0.015;
    // camera.view[1] = 0.213;
    // camera.view[2] = 0.977;
    // camera.focal[0] = -4.88;
    // camera.focal[1] = 2.17;
    // camera.focal[2] = 10.62;

    viewer->setCameraParameters(camera);
    int i = 0;
    for (const auto entry:files) {
        cout << entry << endl;
        if(mode == 1){
            point_cloud = load_pcd(entry);
        } else if (mode == 2)
        {
            point_cloud = load_bin(entry);
        } else {
            return;
        }
        
        if (i!=0)
            viewer->removePointCloud(to_string(i-1));
        viewer->addPointCloud<pcl::PointXYZ> (point_cloud, to_string(i));
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(i));
        i++;
        viewer->getCameraParameters(camera);
        printf("pos: %lf, %lf, %lf\n", camera.pos[0], camera.pos[1], camera.pos[2]);
        printf("view: %lf, %lf, %lf\n", camera.view[0], camera.view[1], camera.view[2]);
        printf("focal: %lf, %lf, %lf\n", camera.focal[0], camera.focal[1], camera.focal[2]);
        printf("clip: %lf, %lf\n", camera.clip[0], camera.clip[1]);
        printf("fovy: %lf\n", camera.fovy);
        printf("window size: %lf, %lf\n", camera.window_size[0], camera.window_size[1]);
        printf("window pos: %lf, %lf\n", camera.window_pos[0], camera.window_pos[1]);
        printf("======================\n");
        viewer->spinOnce(200);
    }
}

/*
    4. override view, add box3d
*/

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym() == "c" && event.keyDown()) {
        std::cout << "c was pressed => next frame" << std::endl;
        // viewer->removePointCloud();
        // viewer->removeAllShapes();
        viewer->close();
        // ((pcl::visualization::PCLVisualizer*)viewer)->close();
    }
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
                // cout << item << endl;
                tmp.push_back(stof(item));
                line.erase(0, pos + delimiter.length());
            }
            re.push_back(tmp);
        }
    }
    return re;
}

void view_predictions(string dp, string ap, int mode) {
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

    for (const auto entry:files) {
        cout << entry << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Road scenario"));
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
        viewer->setCameraParameters(camera);

        string fn = entry.substr(entry.find_last_of("/")+1);
        fn = fn.substr(0, fn.find_last_of("."));
        string pcd_file = dp + "/" + fn + ".bin";
        cout << pcd_file << endl;
        vector<vector<float>> re = load_box(entry);
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
        for (auto box : re) {
            if (int(box[0]) == -1) {
                continue;
            }
            color = colors[int(box[0])];
            const string id = to_string(j);
            const Vector3f translation(box[1], box[2], box[3]);
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



