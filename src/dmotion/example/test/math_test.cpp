#include "dmotion/InverseKinematics/InverseKinematics.h"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"
#include "dmotion/Utility/dmotion_math.hpp"
#include <Eigen/Dense>
#include  <iostream>

using namespace dmotion;
using namespace std;

Eigen::Matrix<double,6,6> Jacobian(std::vector<double> servo_angle, bool isRight)
{
  ForKin real_leg(servo_angle,isRight);
  std::vector<double> tmp;
  const double dd = 0.001;;
  Eigen::Matrix<double,6,6> J;
  for(int i = 0;i < 6; i++)
  {
    tmp = servo_angle;
    tmp[i] = tmp[i] + dd;
    ForKin temp_leg(tmp,isRight);
    for(int j = 0; j < 6; j++)
    {
      J(j,i) = (temp_leg.result_vector[j] - real_leg.result_vector[j]) / dd;
    }
  }
  return J;
}

int main(int argc, char **argv)
{
    vector<double> a = {1,2,3,4,5,6};
    vector<double> tmp(6,0);
    ForKin left(a,false);
    //ForKin left2(a,false);
    InvKin leg(false);
    //Eigen::Matrix<double,6,6> J;


    //第一步，生成样本点的序列（把一个圆圈分为10份）
    int sample_num = 100;     //从始至终需要11个点
    double whole_time = 1000; //完成整个动作需要1000  ms
    int points_time = whole_time/sample_num; //完成整个动作需要3秒
    double R = 4.0;
    double tmp_x,tmp_z;
    std::vector<double> tit(sample_num, 0);
    std::vector<double> valuex(sample_num, 0);
    std::vector<double> valuez(sample_num, 0);
    for (int i = 0; i <= sample_num; i++)
    {
        valuex[i] = R * sin(i * 2.0 * M_PI / sample_num);
        valuez[i] = R * cos(i * 2.0 * M_PI / sample_num) - 28;//TODO
    }


    Eigen::Matrix<double,6,6> J = left.Jacobian();
    cout << J.matrix() << endl;
  //  cout << "fuck" << endl;
    //cout << J.adjoint() << endl;

    Eigen::Matrix<double,6,1> tpp;
    Eigen::Matrix<double,6,1> result;
    for(int i = 0; i < 6; i++)
    {
      tpp(i) = a[i];
    }
    result = J * tpp;
   cout << result.matrix() << endl;
   vector<double> b(a.begin() + 5, a.begin() + 6);
   PrintVector(b);
   cout << J.transpose() << endl;




}
