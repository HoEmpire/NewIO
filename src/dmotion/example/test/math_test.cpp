#include "dmotion/InverseKinematics/InverseKinematics.h"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"
#include "dmotion/Utility/dmotion_math.hpp"
#include <Eigen/Dense>
#include  <iostream>

using namespace dmotion;
using namespace std;

int main(int argc, char **argv)
{
    vector<double> a = {0,0,0,0,-2,0};
    ForKin left(a,false);
    cout << left.pitch_result << endl;
}
