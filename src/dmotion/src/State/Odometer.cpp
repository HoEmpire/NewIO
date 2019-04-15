#include "dmotion/State/Odometer.h"
#include <iostream>
#define SAMPLE_RATE 0.0103//周期为0.01s
#define ERROR 1e-8
using namespace Eigen;
using namespace std;
namespace Motion
{

Odometer::Odometer()
{
  ClearEstimate();
  H << 1,0;
  F << 1,SAMPLE_RATE,
       0,1;
  //                                 k_acc   k_vel
  K_normal_x = CalGain(10, 10);   //   10     10
  K_big_acc_x = CalGain(1, 1000); //    1    1000
  K_normal_y = CalGain(10, 10);   //   10     10
  K_big_acc_y = CalGain(1, 1000); //    1    1000
  K_static = CalGain(100, 0);     //   100     0
  cout << "Kalman Filter Gain: " << endl;
  cout << "K_normal_x:" << endl << K_normal_x << endl;
  cout << "K_big_acc_x:" << endl << K_big_acc_x << endl;
  cout << "K_normal_y:" << endl << K_normal_y << endl;
  cout << "K_big_acc_y:" << endl << K_big_acc_y << endl;
  cout << "K_static:" << endl << K_static << endl;
}

Odometer::~Odometer()
{

}


Matrix<double,2,1> Odometer::CalGain(double k_acc, double k_vel)
{
  Matrix2d Q = Matrix2d::Identity();
  Matrix2d P = 0.01 * Matrix2d::Identity();
  Matrix2d E = Matrix2d::Identity();
  Q(0,0) = 1;//k_state
  Q(1,1) = k_acc;
  double R = k_vel;
  double k1_last = 100, k1_now = 0;
  Matrix<double, 2, 1> K;
  K << 0,
       0;
  //cout << "fuck1" << endl << K << endl;
  int count = 0;
  double error = 100;

  while(count <= 1000 && abs(error) > ERROR)
  {
    P = F * P * F.transpose() + Q;
    //cout << "P:" << endl << P << endl;
    K = P * H.transpose() * 1.0 / (H * P * H.transpose() + R);
    //cout << "K:" << endl << K << endl;
    P =(E - K * H) * P;
    k1_now = K(1);
    error = k1_last - k1_now;
    k1_last = k1_now;
    count++;
    //cout << count << endl;
  }

  //cout << K << endl;
  return K;
}

void Odometer::UpdateEstimate(double acc_wog, double vel, bool is_x, bool change_support)
{
  Matrix<double, 2, 1> K = ChooseGain(acc_wog, is_x, change_support);
  Matrix<double, 1, 1> vel_matrix;
  vel_matrix << vel;
  Matrix<double, 2, 1> Xe;
  if(is_x)
  {
    x_estimate(1) = acc_wog * 100;
    Xe = F * x_estimate;
    x_estimate = Xe + K * (vel_matrix - H * Xe);
  }
  else
  {
    y_estimate(1) = acc_wog * 100;
    Xe = F * y_estimate;
    y_estimate = Xe + K * (vel_matrix - H * Xe);
  }
}

Matrix<double,2,1> Odometer::ChooseGain(double acc_wog, bool is_x, bool change_support)
{
  static int small_acc_ticks_x = 0;
  static int small_acc_ticks_y = 0;
  static int change_support_x = 0;
  static int change_support_y = 0;

  //x fusing
  if(is_x)
  {
    //零速更新
    if(change_support || change_support_x != 0)
    {
      if(change_support_x < 2)
          change_support_x++;
      else
      {
          change_support_x = 0;
          return K_static;
      }
    }

    if(abs(acc_wog) < 20)
    {
        small_acc_ticks_x++;
        if(small_acc_ticks_x > 10)
            return K_static;
        else
            return K_normal_x;
    }
    else if(abs(acc_wog) > 200)
        return K_big_acc_x;
    else
    {
        small_acc_ticks_x = 0;
        return K_normal_x;
    }
  }
  //Y fusing
  else
  {
    //零速更新
    if(change_support || change_support_y != 0)
    {
      if(change_support_y < 12)
          change_support_y++;
      else
      {
          change_support_y = 0;
          return K_static;
      }
    }

    if(abs(acc_wog) < 20)
    {
        small_acc_ticks_y++;
        if(small_acc_ticks_y > 10)
            return K_static;
        else
            return K_normal_y;
    }
    else if(abs(acc_wog) > 100)
        return K_big_acc_x;
    else
    {
        small_acc_ticks_y = 0;
        return K_normal_y;
    }
  }
}

void Odometer::ClearEstimate()
{
  x_vel_to_center = 0;
  y_vel_to_center = 0;
  x_estimate << 0,
                0;
  y_estimate << 0,
                0;
}

Matrix<double,3,1> Odometer::CalEncoderSpeedToGlobal(double roll_global, double pitch_global, double yaw_global,
                                                     double roll_feet  , double pitch_feet  , double yaw_feet,
                                                     double vx_encoder , double vy_encoder  , double vz_encoder)
{
  AngleAxisd roll_feet_vector(roll_feet / 180 * M_PI, Vector3d::UnitX());
  AngleAxisd pitch_feet_vector(pitch_feet / 180 * M_PI, Vector3d::UnitY());
  AngleAxisd yaw_feet_vector(yaw_feet / 180 * M_PI, Vector3d::UnitZ());
  rotation_matrix_center_to_feet = yaw_feet_vector * pitch_feet_vector * roll_feet_vector;

  AngleAxisd roll_global_vector(roll_global, Vector3d::UnitX());
  AngleAxisd pitch_global_vector(pitch_global, Vector3d::UnitY());
  AngleAxisd yaw_global_vector(yaw_global, Vector3d::UnitZ());
  rotation_matrix_center_to_global = yaw_global_vector * pitch_global_vector * roll_global_vector;

  rotation_matrix_feet_to_global = rotation_matrix_center_to_global * rotation_matrix_center_to_feet.transpose();

  Eigen::Matrix<double,3,1> vel_after, vel_before;
  vel_before << vx_encoder, vy_encoder, vz_encoder;

  vel_after = rotation_matrix_feet_to_global * vel_before;

  return vel_after;

}

Matrix<double,3,1> Odometer::CalVelToCenter(double yaw, Eigen::Matrix<double,3,1> vel_global)
{
  Eigen::AngleAxisd rotation_vector(yaw, Vector3d::UnitZ());
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = rotation_vector.matrix();
  Eigen::Matrix<double,3,1> vel_after, vel_before;

  vel_before << vel_global(0), vel_global(1), vel_global(2);
  vel_after = rotation_matrix.transpose() * vel_before;

  return vel_after;
}

void Odometer::UpdateOdometer()
{
  x_displacement = x_displacement + x_estimate(0) * SAMPLE_RATE;
  y_displacement = y_displacement + y_estimate(0) * SAMPLE_RATE;
}

Matrix<double,3,1> Odometer::CalAccToCenter(double imu_yaw, Eigen::Matrix<double,3,1> acc_origin)
{
  Eigen::AngleAxisd imu_rotation_vector(imu_yaw, Vector3d::UnitZ());
  Eigen::Matrix3d imu_rotation_matrix;
  imu_rotation_matrix = imu_rotation_vector.matrix();

  Eigen::Matrix<double,3,1> acc_center;
  acc_center = imu_rotation_matrix.transpose() * acc_origin;
  return acc_center;
}


}
