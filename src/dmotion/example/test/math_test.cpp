// #include <Eigen/Dense>
#include  <iostream>
#include "../../include/dmotion/Utility/dmotion_math.hpp"
//using namespace dmotion;
using namespace std;
using namespace Eigen;
using namespace dmotion;
int main(int argc, char **argv)
{
    // vector<double> a = {0,0,0,0,-2,0};
    // ForKin left(a,false);
    // cout << left.pitch_result << endl;
    // Quaterniond Q(0.9989, 0.0118, 0.0446, 0.0002);
    // Matrix3d R;
    // R = Q.matrix();
    // cout << R << endl;
    std::vector<double> v1(6,0);
    std::vector<double> v2 = {1, 2, 3};
    PrintVector(v1);
    PrintVector(v2);
    v1.assign(v2.begin(),v2.end());
    PrintVector(v1);
}
