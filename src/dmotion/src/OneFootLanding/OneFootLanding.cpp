//
// Created by zjudancer on 2018-12-25.
//

#include "OneFootLanding.h"


namespace dmotion {
    //暂时给这些参数赋予一些差不多的值
    const double OneFootLanding::ankle_offset_x = 0;
    const double OneFootLanding::ankle_offset_y = 0;
    const double OneFootLanding::ankle_offset_z = 9.5;
    const double OneFootLanding::body_center_x = 0.5;
    const double OneFootLanding::body_center_y = 0;
    const double OneFootLanding::body_center_z = -2;
    const double OneFootLanding::upbody_mass = 3000;
    const double OneFootLanding::foot_mass = 850;


    OneFootLanding::OneFootLanding(bool isRight) : isRight_(isRight) {
        left_leg_ = new InvKin(false);
        right_leg_ = new InvKin(true);
        upbody_com.clear();
    }

    /**
     * 进行求解，这些参数都是相对于立足脚中心点的，最稳妥重心投影点根据脚板的安装位置做调整
     * @param hang_foot 悬荡脚的xyzrpy
     * @param whole_body_com 整机的重心
     * @param upbody_yaw 上半身的yaw
     * @return 12个舵机的值
     */
    std::vector<double> &
    OneFootLanding::GetOneStep(std::vector<double> hang_foot, const std::vector<double> &whole_body_com,
                               double upbody_yaw, bool which) {

        hang_foot[3] = dmotion::Deg2Rad(hang_foot[3]);
        hang_foot[4] = dmotion::Deg2Rad(hang_foot[4]);
        hang_foot[5] = dmotion::Deg2Rad(hang_foot[5]);
        upbody_yaw = dmotion::Deg2Rad(upbody_yaw);
        //求上半身的重心位置，已考虑脚底中心点和脚重心的区别
        hangfoot_com.clear();
        landingfoot_com.clear();
        upbody_com.clear();
        hangfoot_com.emplace_back(ankle_offset_z * (std::sin(hang_foot[3]) * std::sin(hang_foot[5]) +
                                                    std::cos(hang_foot[3]) * std::cos(hang_foot[5]) *
                                                    std::sin(hang_foot[4])) -
                                  (isRight_ ? ankle_offset_y : (-ankle_offset_y)) *
                                  (std::cos(hang_foot[3]) * std::sin(hang_foot[5]) -
                                   std::cos(hang_foot[5]) * std::sin(hang_foot[4]) * std::sin(hang_foot[3])) +
                                  ankle_offset_x * std::cos(hang_foot[4]) * std::cos(hang_foot[5]) + hang_foot[0]);

        hangfoot_com.emplace_back((isRight_ ? ankle_offset_y : (-ankle_offset_y)) *
                                  (std::cos(hang_foot[3]) * std::cos(hang_foot[5]) +
                                   std::sin(hang_foot[4]) * std::sin(hang_foot[3]) * std::sin(hang_foot[5])) -
                                  ankle_offset_z * (std::cos(hang_foot[5]) * std::sin(hang_foot[3]) -
                                                    std::cos(hang_foot[3]) * std::sin(hang_foot[4]) *
                                                    std::sin(hang_foot[5])) +
                                  ankle_offset_x * std::cos(hang_foot[4]) * std::sin(hang_foot[5]) + +hang_foot[1]);
        hangfoot_com.emplace_back(ankle_offset_z * std::cos(hang_foot[4]) * std::cos(hang_foot[3]) -
                                  ankle_offset_x * std::sin(hang_foot[4]) +
                                  (isRight_ ? ankle_offset_y : (-ankle_offset_y)) * std::cos(hang_foot[4]) *
                                  std::sin(hang_foot[3]) + +hang_foot[2]);

//        std::cout << "hangfoot_com " << " :" ;
//        dmotion::PrintVector(hangfoot_com);
        landingfoot_com.emplace_back(ankle_offset_x);
        landingfoot_com.emplace_back(isRight_ ? (-ankle_offset_y) : ankle_offset_y);
        landingfoot_com.emplace_back(ankle_offset_z);
//        std::cout << "landingfoot_com " << " :" ;
//        dmotion::PrintVector(landingfoot_com);

        if (which) {  //这种情况下是不考虑支撑脚作为重心一部分的做法，变量which是开关
            upbody_com.emplace_back(
                    ((upbody_mass + foot_mass) * whole_body_com[0] - foot_mass * hangfoot_com[0]) / upbody_mass);
            upbody_com.emplace_back(
                    ((upbody_mass + foot_mass) * whole_body_com[1] - foot_mass * hangfoot_com[1]) / upbody_mass);
            upbody_com.emplace_back(
                    ((upbody_mass + foot_mass) * whole_body_com[2] - foot_mass * hangfoot_com[2]) / upbody_mass);
        } else {    //这种情况下是考虑到了支撑脚作为重心的一部分的做法
            upbody_com.emplace_back(((upbody_mass + 2 * foot_mass) * whole_body_com[0] - foot_mass * hangfoot_com[0] -
                                     foot_mass * landingfoot_com[0]) / upbody_mass);
            upbody_com.emplace_back(((upbody_mass + 2 * foot_mass) * whole_body_com[1] - foot_mass * hangfoot_com[1] -
                                     foot_mass * landingfoot_com[1]) / upbody_mass);
            upbody_com.emplace_back(((upbody_mass + 2 * foot_mass) * whole_body_com[2] - foot_mass * hangfoot_com[2] -
                                     foot_mass * landingfoot_com[2]) / upbody_mass);
        }
//        std::cout << "upbody_com " << " :" ;
//        dmotion::PrintVector(upbody_com);

        TA1.clear();
        TA2.clear();
        v.clear();
        for (int i = 0; i < 3; i++) {
            TA1.emplace_back(upbody_com[i] - landingfoot_com[i]);
            TA2.emplace_back(upbody_com[i] - hangfoot_com[i]);
        }
        unit_arrow(TA1);
        unit_arrow(TA2);
//        std::cout << "TA1 " << " :" ;
//        dmotion::PrintVector(TA1);
//        std::cout << "TA2 " << " :" ;
//        dmotion::PrintVector(TA2);
        for (int i = 0; i < 3; i++) {
            v.emplace_back(TA1[i] + TA2[i]);
        }
        unit_arrow(v);
//        std::cout << "v " << " :" ;
//        dmotion::PrintVector(v);
//        ---------
//        |      /
//        |     /
//        |    /
//        |   /
//        |a /
//        |^/
//        |/  roll改成上身重心和支撑脚连线与竖直方向的夹角减去直立状态时的这个夹角为：0.128弧度


        upbody_roll = 0;//////
//        upbody_roll = dmotion::Atan(-v[1], v[2]);
//        upbody_roll = (isRight_ ? -1.0 : 1.0) * (std::atan(((isRight_ ? 1.0 : -1.0) * upbody_com[1]) / upbody_com[2]) - 0.128);
        upbody_pitch = 0.2;////////upbody_pitch = dmotion::Atan(v[0], v[2] / std::cos(upbody_roll));////////fundamental change ////upbody_pitch = 0;////////

        //upbody 的旋转矩阵需要通过R P Y 生成
        //计算身体中心的位置和姿态
        body_centre.clear();
        body_centre.emplace_back(
                body_center_z * std::sin(upbody_pitch) + body_center_x * std::cos(upbody_pitch) * std::cos(upbody_yaw) -
                body_center_y * std::cos(upbody_pitch) * std::sin(upbody_yaw) + upbody_com[0]);
        body_centre.emplace_back(body_center_x * (std::cos(upbody_roll) * std::sin(upbody_yaw) +
                                                  std::cos(upbody_yaw) * std::sin(upbody_pitch) *
                                                  std::sin(upbody_roll)) + body_center_y * (std::cos(upbody_roll) *
                                                                                            std::cos(upbody_yaw) -
                                                                                            std::sin(upbody_pitch) *
                                                                                            std::sin(upbody_roll) *
                                                                                            std::sin(upbody_yaw)) -
                                 body_center_z * std::cos(upbody_pitch) * std::sin(upbody_roll) + upbody_com[1]);
        body_centre.emplace_back(body_center_x * (std::sin(upbody_roll) * std::sin(upbody_yaw) -
                                                  std::cos(upbody_roll) * std::cos(upbody_yaw) *
                                                  std::sin(upbody_pitch)) + body_center_y * (std::cos(upbody_yaw) *
                                                                                             std::sin(upbody_roll) +
                                                                                             std::cos(upbody_roll) *
                                                                                             std::sin(upbody_pitch) *
                                                                                             std::sin(upbody_yaw)) +
                                 body_center_z * std::cos(upbody_pitch) * std::cos(upbody_roll) + upbody_com[2]);
//        std::cout << "body_centre  ";
//        dmotion::PrintVector(body_centre);
//        std::cout << upbody_roll  <<" " << upbody_pitch << " "  << upbody_yaw  << std::endl;
        //给立足脚的逆运动学向量加入值
        landing_invkin.clear();
        landing_invkin.emplace_back(
                body_centre[2] * std::cos(upbody_roll) * std::cos(upbody_yaw) * std::sin(upbody_pitch) -
                body_centre[1] * std::cos(upbody_roll) * std::sin(upbody_yaw) -
                body_centre[2] * std::sin(upbody_roll) * std::sin(upbody_yaw) -
                body_centre[0] * std::cos(upbody_pitch) * std::cos(upbody_yaw) -
                body_centre[1] * std::cos(upbody_yaw) * std::sin(upbody_pitch) * std::sin(upbody_roll));
        landing_invkin.emplace_back(body_centre[0] * std::cos(upbody_pitch) * std::sin(upbody_yaw) -
                                    body_centre[1] * std::cos(upbody_roll) * std::cos(upbody_yaw) -
                                    body_centre[2] * std::cos(upbody_yaw) * std::sin(upbody_roll) -
                                    body_centre[2] * std::cos(upbody_roll) * std::sin(upbody_pitch) *
                                    std::sin(upbody_yaw) +
                                    body_centre[1] * std::sin(upbody_pitch) * std::sin(upbody_roll) *
                                    std::sin(upbody_yaw));
        landing_invkin.emplace_back(body_centre[1] * std::cos(upbody_pitch) * std::sin(upbody_roll) -
                                    body_centre[2] * std::cos(upbody_pitch) * std::cos(upbody_roll) -
                                    body_centre[0] * std::sin(upbody_pitch));

        landing_invkin.emplace_back(-dmotion::Rad2Deg(upbody_roll));
        landing_invkin.emplace_back(-dmotion::Rad2Deg(upbody_pitch));
        landing_invkin.emplace_back(-dmotion::Rad2Deg(upbody_yaw));

        //给悬荡脚的逆运动学向量加入值

        hanging_invkin.clear();

        hanging_invkin.emplace_back(
                hang_foot[0] * std::cos(upbody_pitch) * std::cos(upbody_yaw) -
                body_centre[0] * std::cos(upbody_pitch) * std::cos(upbody_yaw) +
                hang_foot[1] * std::cos(upbody_roll) * std::sin(upbody_yaw) -
                body_centre[1] * std::cos(upbody_roll) * std::sin(upbody_yaw) +
                hang_foot[2] * std::sin(upbody_roll) * std::sin(upbody_yaw) -
                body_centre[2] * std::sin(upbody_roll) * std::sin(upbody_yaw) -
                hang_foot[2] * std::cos(upbody_roll) * std::cos(upbody_yaw) * std::sin(upbody_pitch) +
                body_centre[2] * std::cos(upbody_roll) * std::cos(upbody_yaw) * std::sin(upbody_pitch) +
                hang_foot[1] * std::cos(upbody_yaw) * std::sin(upbody_pitch) * std::sin(upbody_roll) -
                body_centre[1] * std::cos(upbody_yaw) * std::sin(upbody_pitch) * std::sin(upbody_roll));
        hanging_invkin.emplace_back(
                hang_foot[1] * std::cos(upbody_roll) * std::cos(upbody_yaw) -
                body_centre[1] * std::cos(upbody_roll) * std::cos(upbody_yaw) -
                hang_foot[0] * std::cos(upbody_pitch) * std::sin(upbody_yaw) +
                hang_foot[2] * std::cos(upbody_yaw) * std::sin(upbody_roll) +
                body_centre[0] * std::cos(upbody_pitch) * std::sin(upbody_yaw) -
                body_centre[2] * std::cos(upbody_yaw) * std::sin(upbody_roll) +
                hang_foot[2] * std::cos(upbody_roll) * std::sin(upbody_pitch) * std::sin(upbody_yaw) -
                body_centre[2] * std::cos(upbody_roll) * std::sin(upbody_pitch) * std::sin(upbody_yaw) -
                hang_foot[1] * std::sin(upbody_pitch) * std::sin(upbody_roll) * std::sin(upbody_yaw) +
                body_centre[1] * std::sin(upbody_pitch) * std::sin(upbody_roll) * std::sin(upbody_yaw));
        hanging_invkin.emplace_back(
                hang_foot[0] * std::sin(upbody_pitch) - body_centre[0] * std::sin(upbody_pitch) +
                hang_foot[2] * std::cos(upbody_pitch) * std::cos(upbody_roll) -
                body_centre[2] * std::cos(upbody_pitch) * std::cos(upbody_roll) -
                hang_foot[1] * std::cos(upbody_pitch) * std::sin(upbody_roll) +
                body_centre[1] * std::cos(upbody_pitch) * std::sin(upbody_roll));


        T1_1 = std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::cos(hang_foot[5]) * std::cos(upbody_yaw) -
               std::sin(hang_foot[4]) * std::sin(upbody_roll) * std::sin(upbody_yaw) +
               std::cos(upbody_roll) * std::cos(upbody_yaw) * std::sin(upbody_pitch) * std::sin(hang_foot[4]) +
               std::cos(hang_foot[4]) * std::cos(upbody_roll) * std::sin(hang_foot[5]) * std::sin(upbody_yaw) +
               std::cos(hang_foot[4]) * std::cos(upbody_yaw) * std::sin(upbody_pitch) * std::sin(upbody_roll) *
               std::sin(hang_foot[5]);
        T2_1 = std::cos(hang_foot[4]) * std::cos(upbody_roll) * std::cos(upbody_yaw) * std::sin(hang_foot[5]) -
               std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::cos(hang_foot[5]) * std::sin(upbody_yaw) -
               std::cos(upbody_yaw) * std::sin(hang_foot[4]) * std::sin(upbody_roll) -
               std::cos(upbody_roll) * std::sin(upbody_pitch) * std::sin(hang_foot[4]) * std::sin(upbody_yaw) -
               std::cos(hang_foot[4]) * std::sin(upbody_pitch) * std::sin(upbody_roll) * std::sin(hang_foot[5]) *
               std::sin(upbody_yaw);
        T3_1 = std::cos(hang_foot[4]) * std::cos(hang_foot[5]) * std::sin(upbody_pitch) -
               std::cos(upbody_pitch) * std::cos(upbody_roll) * std::sin(hang_foot[4]) -
               std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::sin(upbody_roll) * std::sin(hang_foot[5]);
        T3_2 = std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::cos(upbody_roll) * std::sin(hang_foot[3]) -
               std::cos(hang_foot[3]) * std::sin(upbody_pitch) * std::sin(hang_foot[5]) -
               std::cos(upbody_pitch) * std::cos(hang_foot[3]) * std::cos(hang_foot[5]) * std::sin(upbody_roll) +
               std::cos(hang_foot[5]) * std::sin(upbody_pitch) * std::sin(hang_foot[4]) * std::sin(hang_foot[3]) -
               std::cos(upbody_pitch) * std::sin(hang_foot[4]) * std::sin(hang_foot[3]) * std::sin(upbody_roll) *
               std::sin(hang_foot[5]);
        T3_3 = std::sin(upbody_pitch) * std::sin(hang_foot[3]) * std::sin(hang_foot[5]) +
               std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::cos(hang_foot[3]) * std::cos(upbody_roll) +
               std::cos(hang_foot[3]) * std::cos(hang_foot[5]) * std::sin(upbody_pitch) * std::sin(hang_foot[4]) +
               std::cos(upbody_pitch) * std::cos(hang_foot[5]) * std::sin(hang_foot[3]) * std::sin(upbody_roll) -
               std::cos(upbody_pitch) * std::cos(hang_foot[3]) * std::sin(hang_foot[4]) * std::sin(upbody_roll) *
               std::sin(hang_foot[5]);

        hanging_invkin.emplace_back(dmotion::Rad2Deg(dmotion::Atan(T3_2, T3_3)));
        hanging_invkin.emplace_back(dmotion::Rad2Deg(std::asin(-T3_1)));
        hanging_invkin.emplace_back(dmotion::Rad2Deg(dmotion::Atan(T2_1, T1_1)));

//        dmotion::PrintVector(hanging_invkin);
//        dmotion::PrintVector(landing_invkin);
        one_foot_result.clear();
        if (isRight_) {
//            std::cout << "right" << std::endl;
            dmotion::AddElements(one_foot_result, right_leg_->LegInvKin(landing_invkin));
            dmotion::AddElements(one_foot_result, left_leg_->LegInvKin(hanging_invkin));
        } else if (!isRight_) {
//            std::cout << "left" << std::endl;
            dmotion::AddElements(one_foot_result, right_leg_->LegInvKin(hanging_invkin));
            dmotion::AddElements(one_foot_result, left_leg_->LegInvKin(landing_invkin));
        }

        return one_foot_result;

    }

    void OneFootLanding::unit_arrow(std::vector<double> &arrow) {
        double len = std::sqrt(arrow[0] * arrow[0] + arrow[1] * arrow[1] + arrow[2] * arrow[2]);
        for (unsigned int i = 0; i < arrow.size(); i++)
            arrow[i] = arrow[i] / len;
    }


}