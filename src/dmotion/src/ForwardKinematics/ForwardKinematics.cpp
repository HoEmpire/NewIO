//
// Created by zjudancer on 19-1-19
// E-mail: zjufanwu@zju.edu.cn
//

#include "dmotion/ForwardKinematics/ForwardKinematics.h"

using namespace std;
namespace dmotion {
    const double ForKin::upper_leg_length = 12.0;  //大腿的长度
    const double ForKin::lower_leg_length = 12.0;  //小腿的长度
    const double ForKin::ankle_from_ground = 6.0;  //脚踝距离地面的高度
    const double ForKin::half_hip_width = 4.5;     //两髋关节点距离的一半,相当于髋关节点相对于身体中心原点的y方向坐标
    const double ForKin::hip_x_from_origin = 0;    //髋关节点相对于身体中心原点的x方向坐标
    const double ForKin::hip_z_from_origin = 8.0;  //髋关节点相对于身体中心原点的z
    ForKin::ForKin(const std::vector<double> angles, bool isRight) {
        for (unsigned i = 0; i < angles.size(); i++) {
            angles_.emplace_back(angles[i] * M_PI / 180.0);
        }
        isRight_ = isRight;
        angles_deg = angles;
        double distance_tmp = std::sqrt(half_hip_width * half_hip_width + hip_x_from_origin * hip_x_from_origin);
        double angle_tmp = std::atan(hip_x_from_origin / half_hip_width);
        //机器人左腿非标DH参数表
        alpha = {0, M_PI / 2, M_PI / 2, 0, 0, -M_PI / 2, M_PI / 2};
        a = {distance_tmp, 0, 0, upper_leg_length, lower_leg_length, 0, ankle_from_ground};
        d = {0, -hip_z_from_origin, 0, 0, 0, 0, 0,};
        theta = {M_PI / 2 - angle_tmp,
                 angle_tmp + angles_[0],
                 angles_[1] - M_PI / 2,
                 angles_[2],
                 -angles_[3],
                 angles_[4],
                 -angles_[5]};

        T = Eigen::Isometry3d::Identity();

        for (int i = 0; i < 7; i++) {
            Eigen::AngleAxisd rotate_yaw(theta[i], Eigen::Vector3d(0, 0, 1));
            Eigen::AngleAxisd rotate_roll(alpha[i], Eigen::Vector3d(1, 0, 0));
            T.rotate(rotate_yaw);
            T.translate(Eigen::Vector3d(0, 0, d[i]));
            T.translate(Eigen::Vector3d(a[i], 0, 0));
            T.rotate(rotate_roll);
        }
        //这两步是用于保证脚末端的坐标系在所有关节角都为0时，和世界坐标系统一
        Eigen::AngleAxisd rotate_pitch90(-M_PI / 2, Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd rotate_yaw90(M_PI / 2, Eigen::Vector3d(0, 0, 1));
        T.rotate(rotate_pitch90);
        T.rotate(rotate_yaw90);

        // std::cout << T.matrix() << std::endl;


        if (0 == isRight_) {
            result_vector = Matrix2Pose(T.inverse());
        } else {
            result_vector = Matrix2Pose(T.inverse());
            result_vector[1] = -result_vector[1];
            result_vector[3] = -result_vector[3];
            result_vector[5] = -result_vector[5];

        }
        x_result = result_vector[0];
        y_result = result_vector[1];
        z_result = result_vector[2];
        roll_result = result_vector[3];
        pitch_result = result_vector[4];
        yaw_result = result_vector[5];
         //dmotion::PrintVector(result_vector);

    }

    //雅阁比矩阵内rpy为弧度制
    Eigen::Matrix<double,6,6> ForKin::Jacobian()
    {
      std::vector<double> tmp;
      const double dd = 0.1;
      Eigen::Matrix<double,6,6> J;

      for(int i = 0; i < 6; i++)
      {
        tmp = angles_deg;
        tmp[i] = tmp[i] + dd;
        ForKin temp_leg(tmp,isRight_);
        for(int j = 0; j < 6; j++)
        {
          J(j,i) = (temp_leg.result_vector[j] - result_vector[j]) / (dd / 180 * M_PI);
          if(j >= 3)
            J(j,i) = J(j,i) / 180 * M_PI;//角度制-->>弧度制
        }
      }
      return J;
    }


    void ForKin::calVelocity(std::vector<double> vel)
    {
      Eigen::Matrix<double,6,1> temp;
      Eigen::Matrix<double,6,1> result;
      //cout<<"FUCK1"<<endl;
      for(int i = 0; i < 6; i++)
      {
        temp(i) = vel[i] / 180 * M_PI;
      }
    //  cout<<"FUCK2"<<endl;
      Eigen::Matrix<double,6,6>  J = Jacobian();
    //  cout<<"FUCK3"<<endl;
      result = J * temp;
    //  cout<<"FUCK4"<<endl;
      for(int i = 0; i < 6; i++)
      {
        resultv_vector.push_back(result[i]);
      }
      //cout<<"FUCK5"<<endl;
      vx_result = result[0];
      vy_result = result[1];
      vz_result = result[2];
      vroll_result = result[3] * 180 / M_PI;//弧度制 --> 角度制
      vpitch_result = result[4] * 180 / M_PI;
      vyaw_result = result[5] * 180 / M_PI;
    //  cout<<"FUCK6"<<endl;
    }

    ForKinPlus::ForKinPlus(std::vector<double> supporting, std::vector<double> hanging) {

        S = Eigen::Isometry3d::Identity();
        H = Eigen::Isometry3d::Identity();

        S.translate(Eigen::Vector3d(supporting[0], 0, 0));
        S.translate(Eigen::Vector3d(0, supporting[1], 0));
        S.translate(Eigen::Vector3d(0, 0, supporting[2]));
        Eigen::AngleAxisd support_yaw(dmotion::Deg2Rad(supporting[5]), Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd support_pitch(dmotion::Deg2Rad(supporting[4]), Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd support_roll(dmotion::Deg2Rad(supporting[3]), Eigen::Vector3d(1, 0, 0));
        S.rotate(support_yaw);
        S.rotate(support_pitch);
        S.rotate(support_roll);


        H.translate(Eigen::Vector3d(hanging[0], 0, 0));
        H.translate(Eigen::Vector3d(0, hanging[1], 0));
        H.translate(Eigen::Vector3d(0, 0, hanging[2]));
        Eigen::AngleAxisd hang_yaw(dmotion::Deg2Rad(hanging[5]), Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd hang_pitch(dmotion::Deg2Rad(hanging[4]), Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd hang_roll(dmotion::Deg2Rad(hanging[3]), Eigen::Vector3d(1, 0, 0));
        H.rotate(hang_yaw);
        H.rotate(hang_pitch);
        H.rotate(hang_roll);
        //身体中心相对于支撑脚的变换矩阵为S_inv，也就是S的逆矩阵
        Eigen::Isometry3d S_inv = S.inverse();
        center2support = Matrix2Pose(S_inv);
        //摆动脚相对于支撑脚的变换矩阵为H2S
        Eigen::Isometry3d H2S(S_inv.matrix()*H.matrix());
        hang2support = Matrix2Pose(H2S);

    }


    std::vector<double> Matrix2Pose(Eigen::Isometry3d M) {
        std::vector<double> pose;
        pose.emplace_back(M.matrix()(0, 3));
        pose.emplace_back(M.matrix()(1, 3));
        pose.emplace_back(M.matrix()(2, 3));

        pose.emplace_back(dmotion::Rad2Deg(dmotion::Atan(M.matrix()(2, 1), M.matrix()(2, 2))));
        pose.emplace_back(dmotion::Rad2Deg(std::asin(-M.matrix()(2, 0))));
        pose.emplace_back(dmotion::Rad2Deg(dmotion::Atan(M.matrix()(1, 0), M.matrix()(0, 0))));

        return pose;
    }


}
