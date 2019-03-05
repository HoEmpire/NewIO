//
// Created by zjudancer on 19-1-19
// E-mail: zjufanwu@zju.edu.cn
//

#ifndef PROJECT_FORWARDKINEMATICS_H
#define PROJECT_FORWARDKINEMATICS_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "dmotion/Utility/dmotion_math.hpp"

namespace dmotion{
class ForKin{
public:
    ForKin(const std::vector<double> angles, bool isRight);
    std::vector<double>  angles_;
    std::vector<double> angles_deg;//TODO FUCK THE WORLD SACK

    bool isRight_;
    static const double upper_leg_length;   //大腿的长度
    static const double lower_leg_length;   //小腿的长度
    static const double ankle_from_ground;  //脚踝距离地面的高度
    static const double half_hip_width;     //两髋关节点距离的一半,相当于髋关节点相对于身体中心原点的y方向坐标
    static const double hip_x_from_origin;  //髋关节点相对于身体中心原点的x方向坐标
    static const double hip_z_from_origin;  //髋关节点相对于身体中心原点的z

    double x_result;
    double y_result;
    double z_result;
    double roll_result;
    double pitch_result;
    double yaw_result;

    double vx_result;
    double vy_result;
    double vz_result;
    double vroll_result;
    double vpitch_result;
    double vyaw_result;

    std::vector<double> result_vector;
    std::vector<double> resultv_vector;
    Eigen::Isometry3d T;

private:

    std::vector<double> alpha;
    std::vector<double> a;
    std::vector<double> d;
    std::vector<double> theta;

public:
    Eigen::Matrix<double,6,6> Jacobian();
    void calVelocity(std::vector<double> vel);
};


class ForKinPlus
{
public:
    //输入为支撑腿相当于规划起点的x,y,z,r,p,y、摆动腿相当于规划起点的x,y,z,r,p,y
    ForKinPlus(std::vector<double> supporting, std::vector<double> hanging);

    Eigen::Isometry3d S;
    Eigen::Isometry3d H;
    //身体中心点相对于支撑脚的x,y,z,r,p,y
    std::vector<double> center2support;
    //摆动脚相对于支撑脚的x,y,z,r,p,y
    std::vector<double> hang2support;

};




//把旋转矩阵转换为x,y,z,r,p,y
std::vector<double> Matrix2Pose(Eigen::Isometry3d M);


}

#endif //PROJECT_FORWARDKINEMATICS_H
