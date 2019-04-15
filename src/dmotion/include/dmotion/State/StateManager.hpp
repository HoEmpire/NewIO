#pragma once
//#include "dmotion/Common/Common.h"

#include "IMUFilter.hpp"
#include "Odometer.h"
#include <ros/ros.h>
#include "dmotion/Common/Type.h"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"
using namespace std;

namespace Motion{

class StateManager
{
  public:
    StateManager();

    ~StateManager();

    void iniIMUFilter();

    void calIMUFilter();

    void checkStableState();

    void checkIniState();

    void checkSupportState();

    void iniPressureSensor();

    void GetEncoderVel();

    void CalOdometer();

    void working();

    void GetEncoderVelBoth();

    void CheckSupportWoSensor();

    IMUData imu_data;

    //imu姿态和无重力加速度
    float roll, pitch, yaw;
    float ax_wog, ay_wog, az_wog;

    //支撑状态判断
    SupportState m_support_state;

    //imu初始化判断
    IniState imu_initialized;

    //脚底传感器数值
    PressureData pressure_data;

    //脚底初始化判断
    IniState pressure_initialized;

    //电源状态判读
    PowerState m_power_state;

    //判断舵机是否初始化完成
    bool servo_initialized;

    //稳定与否判断
    StableState m_stable_state;

    //脚底压力传感器合值
    float left_pressure, right_pressure;

    //码盘位姿
    double vx_encoder, vy_encoder, vz_encoder;
    double roll_feet, pitch_feet, yaw_feet;
    double vx_left, vy_left, vz_left;
    double roll_left, pitch_left, yaw_left;
    double vx_right, vy_right, vz_right;
    double roll_right, pitch_right, yaw_right;

    //判断选取哪只脚算码盘速度
    bool right_support_flag;
    bool right_support_flag_last;
    bool change_support;

    //视觉的yaw
    float vision_yaw;
    Eigen::Matrix<double,3,1> encoder_vel_global;
    Eigen::Matrix<double,3,1> encoder_vel_center;

    //舵机值
    vector<double> servo_pos;
    vector<double> servo_vel;

    //里程计结果
    double x_now, y_now;

    bool vision_compensate_on;

  private:
    Odometer m_odometer;
    ImuFilter m_imu_filter;
    float PressureBiasLeft = 0.0;
    float PressureBiasRight = 0.0;
    timer a;
    int ini_ticks = 1;
};

}
