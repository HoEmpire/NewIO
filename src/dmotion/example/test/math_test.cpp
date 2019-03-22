#include <Eigen/Dense>
#include  <iostream>

//using namespace dmotion;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    // vector<double> a = {0,0,0,0,-2,0};
    // ForKin left(a,false);
    // cout << left.pitch_result << endl;
    Quaterniond Q(0.9989, 0.0118, 0.0446, 0.0002);
    Matrix3d R;
    R = Q.matrix();
    cout << R << endl;
}
