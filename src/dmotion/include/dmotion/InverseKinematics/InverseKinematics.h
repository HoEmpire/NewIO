//
// Created by zjudancer on 18-10-29.
// E-mail: zjufanwu@zju.edu.cn
// TODO 这个类希望实现脚和身体的逆运动学

#ifndef PROJECT_INVERSEKINEMATICS_H
#define PROJECT_INVERSEKINEMATICS_H

#include <vector>
#include <Eigen/Dense>

namespace dmotion {

    class InvKin {

    public:
        // 这些是单个腿上的关节值
        // 这些关节值都以机器人初值基础为零点,按照惯用旋转方向为正方向给出数值的
        double hip_yaw_;
        double hip_roll_;
        double hip_pitch_;
        double knee_pitch_;
        double ankle_pitch_;
        double ankle_roll_;
        // 是左腿还是右腿
        bool isRight_;

    private:
        // 以下都是一些计算过程中需要使用到的参数,不需要理解
        double foot_vertical_x;
        double foot_vertical_y;
        double foot_vertical_z;
        double ankle_x_to_hip;
        double ankle_y_to_hip;
        double ankle_z_to_hip;
        double ankle_norm;
        double ankle_axis_x;
        double ankle_axis_y;
        double ankle_axis_z;
        double vertical_x;
        double vertical_z;
        double vertical_unitz;
        double hip_yaw_delta;
        double ankle_to_hip_yaw_roll_x;
        double ankle_to_hip_yaw_roll_z;
        double hip_pitch_absolute;
        double foot_hip_rpy_x;
        double foot_hip_rpy_y;
        double foot_hip_rpy_z;
        double ankle_pitch_absolute;
        std::vector<double> finals;




    public:
        // 静态常成员变量便于调用,这些参数比较固定,机器人装好后一般绝对不会变,故没有写在参数文件中
        // 存在几何改动时到InverseKinematics.cpp中改动
        // 参数的单位是cm
        static const double upper_leg_length;   //大腿的长度
        static const double lower_leg_length;   //小腿的长度
        static const double ankle_from_ground;  //脚踝距离地面的高度
        static const double half_hip_width;     //两髋关节点距离的一半,相当于髋关节点相对于身体中心原点的y方向坐标
        static const double hip_x_from_origin;  //髋关节点相对于身体中心原点的x方向坐标
        static const double hip_z_from_origin;  //髋关节点相对于身体中心原点的z

        // 构造函数简单的构造一下就可以
        InvKin(bool isRight);


        // 逆运动学,以身体中心为原点,前x左y上z的方向建立坐标系,输入左/右脚的位置(x,y,z)和姿态(r,p,y)
        // 为了方便理解和调参,这里的RPY变换根据以下描述进行变换
        // 起始状态脚坐标系B和此处的世界坐标系A坐标轴方向重合,先绕Z_a转yaw角,再绕Y_b转pitch角,最后绕X_b转roll角.
        // 输出从上到下的共6个舵机角度值(角度值).
        std::vector<double> LegInvKin(std::vector<double> &foot_pose);




    };
}


#endif //PROJECT_INVERSEKINEMATICS_H
