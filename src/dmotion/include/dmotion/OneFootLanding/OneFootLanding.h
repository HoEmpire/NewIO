//
// Created by zjudancer on 18-12-18.
//

#ifndef INVERSEKINEMATICS_ONEFOOTLANDING_H
#define INVERSEKINEMATICS_ONEFOOTLANDING_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "dmotion_math.hpp"
#include "InverseKinematics.h"

namespace dmotion {

    class OneFootLanding {
    public:
        bool isRight_;

        // 以下值是机器人在完全竖直站立的情况下，左脚的重心(A1)相对于脚中心点(A1')的xyz偏移
        static const double ankle_offset_x;
        static const double ankle_offset_y;
        static const double ankle_offset_z;
        // 身体单腿逆运动学起点(M)相对于上半身重心的xyz偏移
        static const double body_center_x;
        static const double body_center_y;
        static const double body_center_z;
        static const double upbody_mass;
        static const double foot_mass;

        std::vector<double> upbody_com;
        std::vector<double> hangfoot_com;
        std::vector<double> landingfoot_com;

    private:
        friend class PendulumWalk;
        InvKin *left_leg_;
        InvKin *right_leg_;

        std::vector<double> body_centre; //身体单腿逆运动学起点(M)
        //已下三个变量是三维空间向量，参考 https://github.com/hannbusann/one-foot_landing_InvKin/blob/master/img/-75909a348f7ef85.jpg
        std::vector<double> TA1;
        std::vector<double> TA2;
        std::vector<double> v;
        double upbody_roll;
        double upbody_pitch;
        //单腿逆运动学接口
        std::vector<double> landing_invkin;
        std::vector<double> hanging_invkin;
        //临时变量
        double T1_1;
        double T2_1;
        double T3_1;
        double T3_2;
        double T3_3;
        //最后得出的12个舵机的值
        std::vector<double> one_foot_result;

    public:
        OneFootLanding(bool isRight);

        //做计算的一步，可以计算出12个舵机值的序列
        std::vector<double> &GetOneStep(std::vector<double> hang_foot, const std::vector<double> &whole_body_com,
                                        double upbody_yaw , bool which = false);

        //矢量单位化
        void unit_arrow(std::vector<double> & arrow);
    };

}
#endif //INVERSEKINEMATICS_ONEFOOTLANDING_H
