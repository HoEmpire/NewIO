#ifndef PROJECT_Odometer_H
#define PROJECT_Odometer_H
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Motion
{
  class Odometer
  {
   public:
      Odometer();

      ~Odometer();

      Eigen::Matrix<double,2,1> CalGain(double k_acc, double k_vel);

      void UpdateEstimate(double acc_wog, double vel, bool is_x, bool change_support);

      Eigen::Matrix<double,3,1> CalEncoderSpeedToGlobal(double roll_glolal, double pitch_glolal, double yaw_global,
                                                        double roll_feet  , double pitch_feet  , double yaw_feet,
                                                        double vx_encoder , double vy_encoder  , double vz_encoder);

      Eigen::Matrix<double,3,1> CalVelToCenter(double yaw, Eigen::Matrix<double,3,1> vel_global);

      void UpdateOdometer();

      Eigen::Matrix<double,3,1> CalAccToCenter(double imu_yaw, Eigen::Matrix<double,3,1> acc_origin);

      void ClearEstimate();

   public:
      double x_displacement, y_displacement;
      double x_vel_to_center, y_vel_to_center;
      //normal为普通增益，resit_drift为动态漂移抑制增益，static为静态漂移抑制增益
      Eigen::Matrix<double,2,1> K_normal_x, K_big_acc_x, K_static;
      Eigen::Matrix<double,2,1> K_normal_y, K_big_acc_y;
      Eigen::Matrix3d rotation_matrix_center_to_feet, rotation_matrix_center_to_global, rotation_matrix_feet_to_global;

   private:
      Eigen::Matrix<double,2,1> ChooseGain(double acc_wog, bool is_x, bool change_support);

   private:
      Eigen::Matrix<double,2,1> x_estimate, y_estimate;
      Eigen::Matrix<double,1,2> H;
      Eigen::Matrix<double,2,2> F;

  };
}

#endif
