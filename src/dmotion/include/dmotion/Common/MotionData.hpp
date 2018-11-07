#pragma once

#include "fast_math.hpp"
#include "dcommon/dcommon.hpp"
#include <boost/program_options.hpp>
#include <cstring>
#include <ros/ros.h>

#define LEFT_HIP_PITCH      8
#define LEFT_HIP_ROLL       9
#define LEFT_HIP_YAW        10
#define LEFT_KNEE           11
#define LEFT_ANKLE_PITCH    12
#define LEFT_ANKLE_ROLL     13

#define RIGHT_HIP_PITCH     1
#define RIGHT_HIP_ROLL      2
#define RIGHT_HIP_YAW       3
#define RIGHT_KNEE          4
#define RIGHT_ANKLE_PITCH   5
#define RIGHT_ANKLE_ROLL    6

namespace {
const int MOTORNUM = 16;
const int SMOOTHMAXERROR = 50;
const int RENUM = 100;

const int IMAGE_PLAT_DELAY = 6;
const ddouble_t MAX_PLAT_YAW = 135;
const ddouble_t MAX_PLAT_PITCH = 50;
const ddouble_t MIN_PLAT_PITCH = 0;
}

class variables_map;
namespace MOTION {
extern ddouble_t sampletime;
}

enum SUPPORT_STATUS
{
    LEFT_BASED,
    RIGHT_BASED,
    DOUBLE_BASED,
    OTHER_BASED,
    SUPPORT_STATUS_NUM
};

/**
 * body angle
 */
struct angledataDebug
{
    ddouble_t angleX;
    ddouble_t angleY;
    ddouble_t angleZ;
    long corresCycle;

    angledataDebug()
      : angleX(0)
      , angleY(0)
      , angleZ(0)
      , corresCycle(0){};
};

/**
 * motor initial pos
 **/
struct initdataDebug
{
    ddouble_t initial[MOTORNUM];

    initdataDebug()
    {
        for (int i = 0; i < MOTORNUM; i++) {
            initial[i] = 0;
        }
    }
};

// body offset
class offsetDebug
{
  public:
    // ddouble_t lastOffsetX;
    // ddouble_t lastOffsetY;
    ddouble_t offsetX;
    ddouble_t offsetY;
    long corresCycle;

  public:
    offsetDebug()
      : /*lastOffsetX(0),lastOffsetY(0),*/ offsetX(0)
      , offsetY(0)
      , corresCycle(0){};
};

class RobotPara
{
    // const params
  public:
    static void update(ros::NodeHandle* nh);
    static ddouble_t hip_distance;
    static ddouble_t upper_leg;
    static ddouble_t lower_leg;
    static ddouble_t upper_arm;
    static ddouble_t lower_arm;
    static ddouble_t lbz;
    static ddouble_t g;
    static ddouble_t dt;
    // cm
    static ddouble_t cm_r;
    static ddouble_t cm_p;
    static ddouble_t cm_p_step;
    static ddouble_t cm_y;
    static ddouble_t cm_dx;
    static ddouble_t cm_dy;
    // cm_k
    static ddouble_t cm_dp_fk;
    static ddouble_t cm_dx_fk;
    static ddouble_t percent_fx;
    static ddouble_t cm_dx_bk;
    static ddouble_t percent_bx;
    static ddouble_t cm_dy_lk;
    static ddouble_t percent_ly;
    static ddouble_t cm_dy_rk;
    static ddouble_t percent_ry;
    // ankle
    static ddouble_t ra_r;
    static ddouble_t ra_p;
    static ddouble_t ra_y;
    static ddouble_t la_r;
    static ddouble_t la_p;
    static ddouble_t la_y;
    // ankle_k
    static ddouble_t la_dr_lk;
    static ddouble_t ra_dr_lk;
    static ddouble_t la_dr_rk;
    static ddouble_t ra_dr_rk;
    static ddouble_t la_dp_fk;
    static ddouble_t ra_dp_fk;
    static ddouble_t la_dp_bk;
    static ddouble_t ra_dp_bk;
    /*walking amend*/
    static ddouble_t step_x_amend;
    static ddouble_t step_theta_amend;
    static ddouble_t back_theta_amend;
    static ddouble_t mid_theta_amend;
    static ddouble_t top_theta_amend;
    /*walking ability*/
    static ddouble_t step_theta_max;
    static ddouble_t x_compensation_acc;
    static ddouble_t x_compensation_dec;
    static ddouble_t ad_x_max;
    static ddouble_t ad_theta_max;
    static ddouble_t mid_x_max;
    static ddouble_t mid_theta_max;
    static ddouble_t top_x_max;
    static ddouble_t top_theta_max;
    static ddouble_t back_x_max;
    static ddouble_t back_theta_max;
    static ddouble_t left_y_max;
    static ddouble_t right_y_max;
    /*foot lifting height*/
    static ddouble_t ah_ml_zero;
    static ddouble_t ah_ml_mid;
    static ddouble_t ah_ml_top;
    static ddouble_t ah_fl;
    static ddouble_t ah_mr_zero;
    static ddouble_t ah_mr_mid;
    static ddouble_t ah_mr_top;
    static ddouble_t ah_fr;
    /*foot placement correct in y direction*/
    static ddouble_t ankle_distance;
    static ddouble_t Ankle_dev_l;
    static ddouble_t Ankle_dev_r;
    static ddouble_t Ankle_dev_l_tk;
    static ddouble_t Ankle_dev_r_tk;
    // arm
    static ddouble_t arm_crouch_p;
    static ddouble_t arm_crouch_theta;
    /*changed important*/
    static int stepnum;
    static int stand2crouch_stepnum;
    static int staticExit_num;
    static ddouble_t yzmp;
    static ddouble_t hipheight;
    /*kick*/
    static ddouble_t kickPercent;
    static int oldturning;
    /*plat diff*/
    static ddouble_t diffH;
    static ddouble_t diffV;
    /*other*/
    static ddouble_t stepK;
    static ddouble_t other2crouch2step_height;
    static ddouble_t stand2crouch2step_height;
    static ddouble_t stand2crouch2step_cm;
    static ddouble_t other2crouch2step_cm;
    static int other2crouch2step;
    static int stand2crouch2step;
    static bool getup_bool;
    /*odometer*/
    static ddouble_t forward_k;
    static ddouble_t back_k;
    static ddouble_t left_k;
    static ddouble_t right_k;
    static ddouble_t angle_k;
    static bool gyroscope_debug;
};

/**
 * Robot Control
 **/
struct RobotCtrl
{
    /* x means the step length in x direction;
     * t means the turning angle
     */
    ddouble_t robot_x, robot_y, robot_t;
    int supportNum;
    /* center of mass; rightankle; left ankle */
    ddouble_t cm[6], ra[6], la[6], rh[6], lh[6];
    bool auto_cm[6], auto_ra[6], auto_la[6];
    ddouble_t cm_v[6], ra_v[6], la_v[6], rh_v[6], lh_v[6];
    /* center of mass offset */
    ddouble_t cm_dxy[2], cm_dxy_v[2];
    SUPPORT_STATUS supportStatus;
    int num_left;
    int dataArray[MOTORNUM];

    RobotCtrl()
      : robot_x(0)
      , robot_y(0)
      , robot_t(0)
      , supportNum(0)
      , supportStatus(DOUBLE_BASED)
      , num_left(0)
    {
        memset(cm, 0, 6 * sizeof(ddouble_t));
        memset(ra, 0, 6 * sizeof(ddouble_t));
        memset(la, 0, 6 * sizeof(ddouble_t));
        memset(cm_v, 0, 6 * sizeof(ddouble_t));
        memset(ra_v, 0, 6 * sizeof(ddouble_t));
        memset(la_v, 0, 6 * sizeof(ddouble_t));
        memset(rh, 0, 6 * sizeof(ddouble_t));
        memset(lh, 0, 6 * sizeof(ddouble_t));
        memset(rh_v, 0, 6 * sizeof(ddouble_t));
        memset(lh_v, 0, 6 * sizeof(ddouble_t));
        memset(cm_dxy, 0, 2 * sizeof(ddouble_t));
        memset(cm_dxy_v, 0, 2 * sizeof(ddouble_t));
        memset(dataArray, 0, MOTORNUM * sizeof(int));
        memset(auto_cm, 0, 6 * sizeof(bool));
        memset(auto_la, 0, 6 * sizeof(bool));
        memset(auto_ra, 0, 6 * sizeof(bool));
        ra[1] = -RobotPara::hip_distance / 2 - RobotPara::yzmp; //理解时yzmp先可当做为0
        la[1] = RobotPara::hip_distance / 2 + RobotPara::yzmp;
        cm[2] = RobotPara::upper_leg + RobotPara::lower_leg;
        rh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
        lh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
        rh[1] = 0;
        lh[1] = 0;
    }

    RobotCtrl(ddouble_t rx, ddouble_t ry, ddouble_t rt, SUPPORT_STATUS ss)
      : robot_x(rx)
      , robot_y(ry)
      , robot_t(rt)
      , supportStatus(ss)
    {
        memset(cm, 0, 6 * sizeof(ddouble_t));
        memset(ra, 0, 6 * sizeof(ddouble_t));
        memset(la, 0, 6 * sizeof(ddouble_t));
        memset(cm_v, 0, 6 * sizeof(ddouble_t));
        memset(ra_v, 0, 6 * sizeof(ddouble_t));
        memset(la_v, 0, 6 * sizeof(ddouble_t));
        memset(rh, 0, 6 * sizeof(ddouble_t));
        memset(lh, 0, 6 * sizeof(ddouble_t));
        memset(rh_v, 0, 6 * sizeof(ddouble_t));
        memset(lh_v, 0, 6 * sizeof(ddouble_t));
        memset(cm_dxy, 0, 2 * sizeof(ddouble_t));
        memset(cm_dxy_v, 0, 2 * sizeof(ddouble_t));
        memset(dataArray, 0, MOTORNUM * sizeof(int));
        memset(auto_cm, 0, 6 * sizeof(bool));
        memset(auto_la, 0, 6 * sizeof(bool));
        memset(auto_ra, 0, 6 * sizeof(bool));
        ra[1] = -RobotPara::hip_distance / 2 - RobotPara::yzmp;
        la[1] = RobotPara::hip_distance / 2 + RobotPara::yzmp;
        cm[2] = RobotPara::upper_leg + RobotPara::lower_leg;
        rh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
        lh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
        rh[1] = 0;
        lh[1] = 0;
    }

    RobotCtrl(const RobotCtrl& robotCtrl)
    {
        robot_x = robotCtrl.robot_x;
        robot_y = robotCtrl.robot_y;
        robot_t = robotCtrl.robot_t;
        supportNum = robotCtrl.supportNum;
        for (int i = 0; i < 6; i++) {
            cm[i] = robotCtrl.cm[i];
            ra[i] = robotCtrl.ra[i];
            la[i] = robotCtrl.la[i];
            cm_v[i] = robotCtrl.cm_v[i];
            ra_v[i] = robotCtrl.ra_v[i];
            la_v[i] = robotCtrl.la_v[i];
            rh[i] = robotCtrl.rh[i];
            lh[i] = robotCtrl.lh[i];
            rh_v[i] = robotCtrl.rh_v[i];
            lh_v[i] = robotCtrl.lh_v[i];
        }
        for (int i = 0; i < 2; i++) {
            cm_dxy[i] = robotCtrl.cm_dxy[i];
            cm_dxy_v[i] = robotCtrl.cm_dxy_v[i];
        }
        for (int i = 0; i < MOTORNUM; i++)
            dataArray[i] = robotCtrl.dataArray[i];
        num_left = robotCtrl.num_left;
        supportStatus = robotCtrl.supportStatus;
    }

    void operator=(const RobotCtrl& robotCtrl)
    {
        robot_x = robotCtrl.robot_x;
        robot_y = robotCtrl.robot_y;
        robot_t = robotCtrl.robot_t;
        supportNum = robotCtrl.supportNum;
        for (int i = 0; i < 6; i++) {
            cm[i] = robotCtrl.cm[i];
            ra[i] = robotCtrl.ra[i];
            la[i] = robotCtrl.la[i];
            cm_v[i] = robotCtrl.cm_v[i];
            ra_v[i] = robotCtrl.ra_v[i];
            la_v[i] = robotCtrl.la_v[i];
            rh[i] = robotCtrl.rh[i];
            lh[i] = robotCtrl.lh[i];
            rh_v[i] = robotCtrl.rh_v[i];
            lh_v[i] = robotCtrl.lh_v[i];
        }
        for (int i = 0; i < 2; i++) {
            cm_dxy[i] = robotCtrl.cm_dxy[i];
            cm_dxy_v[i] = robotCtrl.cm_dxy_v[i];
        }
        for (int i = 0; i < MOTORNUM; i++)
            dataArray[i] = robotCtrl.dataArray[i];
        num_left = robotCtrl.num_left;
        supportStatus = robotCtrl.supportStatus;
    }

    bool operator==(const RobotCtrl& robotCtrl)
    {
        if (robot_x == robotCtrl.robot_x && robot_y == robotCtrl.robot_y && robot_t == robotCtrl.robot_t && supportStatus == robotCtrl.supportStatus && supportNum == robotCtrl.supportNum) {
            return true;
        } else {
            return false;
        }
    }

    /*Oned spline control*/
    void setAutoMode()
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
    }

//TODO 下面这个没用pyx
    void setCompleteAutoMode()
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
    }

    /*walk mode cmx,cmy,la[2],ra[2] manual set*/
    void setWalkMode()
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
        auto_cm[0] = 0;
        auto_cm[1] = 0;
        auto_la[2] = 0;
        auto_ra[2] = 0;
    }

    void setKickMode()
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
        auto_la[2] = 0; // z
        auto_la[0] = 0; // x
    }

    void setCtrlMode()
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 0;
            auto_la[i] = 0;
            auto_ra[i] = 0;
        }
        // auto_la[2] = 0; // z
        // auto_la[0] = 0; // x
    }
//TODO 重复函数
    void setKickMode(SUPPORT_STATUS spt)
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
        // if (spt == RIGHT_BASED) // leftkick
        // {
        //     for (int i = 0; i < 6; i++)
        //     {
        //         auto_ra[i] = 0;
        //     }
        // }
        // if (spt == LEFT_BASED) // rightkick
        // {
        //     for (int i = 0; i < 6; i++)
        //     {
        //         auto_la[i] = 0;
        //     }
        // }
    }

    /* return y direction speed*/
    ddouble_t getWalkVelY()
    { // not use , replaced by robot_y
        if (supportStatus == LEFT_BASED)
            return robot_y;
        else if (supportStatus == RIGHT_BASED)
            return -robot_y;
        else
            return 0;
    }

    ddouble_t getComRoll(int _stepnum) // bug feature in z_stepnum
    {
        if (supportStatus == LEFT_BASED)
            return 10.0 * sin((_stepnum - num_left) / _stepnum * M_PI);
        else if (supportStatus == RIGHT_BASED)
            return -10.0 * sin((_stepnum - num_left) / _stepnum * M_PI);
        else
            return 0;
    }

    void setMotionData(int* motiondata)
    {
        for (int i = 0; i < MOTORNUM; i++)
            dataArray[i] = motiondata[i];
    }
};
