
#include "dmotion/Common/MotionData.hpp"

namespace MOTION {
ddouble_t sampletime;
}

void
RobotPara::update(ros::NodeHandle* nh)
{
    ROS_INFO("RobotPara reading options");
    if (!nh->getParam("dmotion/robot/cm_r", cm_r)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/cm_p", cm_p)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/cm_y", cm_y)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/cm_dx", cm_dx)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/cm_dy", cm_dy)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*cm_k */
    if (!nh->getParam("dmotion/robot/cm_dp_fk", cm_dp_fk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/cm_dx_fk", cm_dx_fk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/percent_fx", percent_fx)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/cm_dx_bk", cm_dx_bk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/percent_bx", percent_bx)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/cm_dy_lk", cm_dy_lk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/percent_ly", percent_ly)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/cm_dy_rk", cm_dy_rk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/percent_ry", percent_ry)) {
        ROS_FATAL("RobotPara get param error!");
    }

    if (!nh->getParam("dmotion/robot/hip_distance", hip_distance)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/upper_leg", upper_leg)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/lower_leg", lower_leg)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/upper_arm", upper_arm)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/lower_arm", lower_arm)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/lbz", lbz)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /* ankle */
    if (!nh->getParam("dmotion/robot/ra_r", ra_r)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ra_p", ra_p)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ra_y", ra_y)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/la_r", la_r)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/la_p", la_p)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/la_y", la_y)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*ankle_k */
    if (!nh->getParam("dmotion/robot/la_dr_lk", la_dr_lk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ra_dr_lk", ra_dr_lk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/la_dr_rk", la_dr_rk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ra_dr_rk", ra_dr_rk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/la_dp_fk", la_dp_fk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ra_dp_fk", ra_dp_fk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/la_dp_bk", la_dp_bk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ra_dp_bk", ra_dp_bk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*walking amend*/
    if (!nh->getParam("dmotion/robot/step_x_amend", step_x_amend)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ad_x_max", ad_x_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ad_theta_max", ad_theta_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/mid_x_max", mid_x_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/mid_theta_max", mid_theta_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/top_x_max", top_x_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/top_theta_max", top_theta_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/back_theta_amend", back_theta_amend)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/step_theta_amend", step_theta_amend)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/mid_theta_amend", mid_theta_amend)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/top_theta_amend", top_theta_amend)) {
        ROS_FATAL("RobotPara get param error!");
    }

    /*walking ability*/
    if (!nh->getParam("dmotion/robot/step_theta_max", step_theta_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ad_x_max", ad_x_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ad_theta_max", ad_theta_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/mid_x_max", mid_x_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/mid_theta_max", mid_theta_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/top_x_max", top_x_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/top_theta_max", top_theta_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/back_x_max", back_x_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/back_theta_max", back_theta_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/left_y_max", left_y_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/right_y_max", right_y_max)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*foot lifting height*/
    if (!nh->getParam("dmotion/robot/ah_ml_zero", ah_ml_zero)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ah_ml_mid", ah_ml_mid)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ah_ml_top", ah_ml_top)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ah_fl", ah_fl)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ah_mr_zero", ah_mr_zero)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ah_mr_mid", ah_mr_mid)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ah_mr_top", ah_mr_top)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ah_fr", ah_fr)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*foot placement correct in y direction*/
    if (!nh->getParam("dmotion/robot/ankle_distance", ankle_distance)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ankle_dev_l", Ankle_dev_l)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ankle_dev_r", Ankle_dev_r)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ankle_dev_l_tk", Ankle_dev_l_tk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/ankle_dev_r_tk", Ankle_dev_r_tk)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*arm*/
    if (!nh->getParam("dmotion/robot/arm_crouch_p", arm_crouch_p)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/arm_crouch_theta", arm_crouch_theta)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*important*/
    if (!nh->getParam("dmotion/robot/stepnum", stepnum)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/stand2crouch_stepnum", stand2crouch_stepnum)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/staticExit_num", staticExit_num)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/yzmp", yzmp)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/hipheight", hipheight)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/kickpercent", kickPercent)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*plat diff*/
    if (!nh->getParam("dmotion/robot/diffh", diffH)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/diffv", diffV)) {
        ROS_FATAL("RobotPara get param error!");
    }
    /*other*/
    if (!nh->getParam("dmotion/robot/stepk", stepK)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/oldturning", oldturning)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/other2crouch2step_height", other2crouch2step_height)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/stand2crouch2step_height", stand2crouch2step_height)) {
        ROS_FATAL("RobotPara get param error!");
    }

    if (!nh->getParam("dmotion/robot/other2crouch2step_cm", other2crouch2step_cm)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/stand2crouch2step_cm", stand2crouch2step_cm)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/other2crouch2step", other2crouch2step)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/stand2crouch2step", stand2crouch2step)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/getup_bool", getup_bool)) {
        ROS_FATAL("RobotPara get param error!");
    }

    /*odometer*/
    if (!nh->getParam("dmotion/robot/forward_k", forward_k)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/back_k", back_k)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/left_k", left_k)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/right_k", right_k)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/robot/angle_k", angle_k)) {
        ROS_FATAL("RobotPara get param error!");
    }
    if (!nh->getParam("dmotion/hardware/gyroscope_debug", gyroscope_debug)) {
        ROS_FATAL("RobotPara get param error!");
    }
}
/*avoid error*/
ddouble_t RobotPara::hip_distance = 8;
ddouble_t RobotPara::upper_leg = 1.8;
ddouble_t RobotPara::lower_leg = 10.8;
ddouble_t RobotPara::upper_arm = 13.5;
ddouble_t RobotPara::lower_arm = 13.5;
ddouble_t RobotPara::lbz = 0;
ddouble_t RobotPara::g = 980;
ddouble_t RobotPara::dt = 0.02;
// cm
ddouble_t RobotPara::cm_r = 0;
ddouble_t RobotPara::cm_p = 0;
ddouble_t RobotPara::cm_p_step = 0;
ddouble_t RobotPara::cm_y = 0;
ddouble_t RobotPara::cm_dx = 0;
ddouble_t RobotPara::cm_dy = 0;
// cm_k
ddouble_t RobotPara::cm_dp_fk = 0;
ddouble_t RobotPara::cm_dx_fk = 0;
ddouble_t RobotPara::percent_fx = 0;
ddouble_t RobotPara::cm_dx_bk = 0;
ddouble_t RobotPara::percent_bx = 0;
ddouble_t RobotPara::cm_dy_lk = 0;
ddouble_t RobotPara::percent_ly = 0;
ddouble_t RobotPara::cm_dy_rk = 0;
ddouble_t RobotPara::percent_ry = 0;
// ankle
ddouble_t RobotPara::ra_r = 0;
ddouble_t RobotPara::ra_p = 0;
ddouble_t RobotPara::ra_y = 0;
ddouble_t RobotPara::la_r = 0;
ddouble_t RobotPara::la_p = 0;
ddouble_t RobotPara::la_y = 0;
// ankle_k
ddouble_t RobotPara::la_dr_lk = 0;
ddouble_t RobotPara::ra_dr_lk = 0;
ddouble_t RobotPara::la_dr_rk = 0;
ddouble_t RobotPara::ra_dr_rk = 0;
ddouble_t RobotPara::la_dp_fk = 0;
ddouble_t RobotPara::ra_dp_fk = 0;
ddouble_t RobotPara::la_dp_bk = 0;
ddouble_t RobotPara::ra_dp_bk = 0;
/*walking amend*/
ddouble_t RobotPara::step_x_amend = 0;
ddouble_t RobotPara::step_theta_amend = 0;
ddouble_t RobotPara::back_theta_amend = 0;
ddouble_t RobotPara::mid_theta_amend = 0;
ddouble_t RobotPara::top_theta_amend = 0;
/*walking ability*/
ddouble_t RobotPara::step_theta_max = 0;
ddouble_t RobotPara::x_compensation_acc = 0;
ddouble_t RobotPara::x_compensation_dec = 0;
ddouble_t RobotPara::ad_x_max = 0;
ddouble_t RobotPara::ad_theta_max = 0;
ddouble_t RobotPara::mid_x_max = 0;
ddouble_t RobotPara::mid_theta_max = 0;
ddouble_t RobotPara::top_x_max = 0;
ddouble_t RobotPara::top_theta_max = 0;
ddouble_t RobotPara::back_x_max = 0;
ddouble_t RobotPara::back_theta_max = 0;
ddouble_t RobotPara::left_y_max = 0;
ddouble_t RobotPara::right_y_max = 0;
/*foot lifting height*/
ddouble_t RobotPara::ah_ml_zero = 0;
ddouble_t RobotPara::ah_ml_mid = 0;
ddouble_t RobotPara::ah_ml_top = 0;
ddouble_t RobotPara::ah_fl = 0;
ddouble_t RobotPara::ah_mr_zero = 0;
ddouble_t RobotPara::ah_mr_mid = 0;
ddouble_t RobotPara::ah_mr_top = 0;
ddouble_t RobotPara::ah_fr = 0;
/*foot placement correct in y direction*/
ddouble_t RobotPara::ankle_distance = 0;
ddouble_t RobotPara::Ankle_dev_l = 0;
ddouble_t RobotPara::Ankle_dev_r = 0;
ddouble_t RobotPara::Ankle_dev_l_tk = 0;
ddouble_t RobotPara::Ankle_dev_r_tk = 0;
// arm
ddouble_t RobotPara::arm_crouch_p = 0;
ddouble_t RobotPara::arm_crouch_theta = 0;
/*changed important*/
int RobotPara::stepnum = 26;
int RobotPara::stand2crouch_stepnum = 50;
int RobotPara::staticExit_num = 2;
ddouble_t RobotPara::yzmp = 0;
ddouble_t RobotPara::hipheight = 20;
/*kick*/
ddouble_t RobotPara::kickPercent = 0;
/*plat diff*/
ddouble_t RobotPara::diffH = 0;
ddouble_t RobotPara::diffV = 0;
/*other*/
ddouble_t RobotPara::stepK = 1;
int RobotPara::oldturning = 0;
ddouble_t RobotPara::other2crouch2step_height = 0;
ddouble_t RobotPara::stand2crouch2step_height = 0;
ddouble_t RobotPara::stand2crouch2step_cm = 1;
ddouble_t RobotPara::other2crouch2step_cm = 3;
int RobotPara::other2crouch2step = 0;
int RobotPara::stand2crouch2step = 0;
bool RobotPara::getup_bool = true;
/*odometer*/
ddouble_t RobotPara::forward_k = 1;
ddouble_t RobotPara::back_k = 1;
ddouble_t RobotPara::left_k = 1;
ddouble_t RobotPara::right_k = 1;
ddouble_t RobotPara::angle_k = 1;
bool RobotPara::gyroscope_debug = false;
