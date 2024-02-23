# include <iostream>
# include <vector>
# include <eigen3/Eigen/Dense>
# include <eigen3/Eigen/Geometry>
using namespace std;
using namespace Eigen;

int main(){
    AngleAxisd t_V(M_PI / 4, Vector3d(0, 0, 1));
    Matrix3d t_R = t_V.matrix();
    Quaterniond t_Q(t_V);
    Eigen::Vector3f a(1., 1., 1);
    return 0;
}
