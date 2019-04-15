#pragma once

#include "Type.h"

#include <vector>
#include <string>

#include <ros/ros.h>

// add using_pressure, pressure_counting_start, pressure_counting_number

struct GlobalParameters
{
    bool using_pressure = false;
    bool using_lateral_stablizer = false;
    bool using_push_recovery = false;
};

struct IOParameters
{
    // imu
    std::string imu_version;

    // joint configurations
    int joint_number;
    std::vector<int> joint_init_values;
    std::vector<int> joint_cw;
    std::vector<int> joint_resolution;
    std::vector<int> joint_index;
    std::vector<int> joint_upper_bound;
    std::vector<int> joint_lower_bound;
    std::vector<std::string> joint_name;

    std::map<std::string, Motion::JointConfig> joint_cfg;
};

struct StateParameters
{
    // imu parameters
    float imu_prepare_time;
    bool vision_compensate_on;
    double imu_Kp = 0.0;
    double imu_Ki = 0.0;
    float fake_odometry_gain_x;
    float fake_odometry_gain_y;
};

class Parameters
{
public:
    GlobalParameters global;
    IOParameters io;
    StateParameters state;

    void init(ros::NodeHandle* nh);

    void update();

    void setFieldYaw(const double yaw);

    ~Parameters() = default;

private:
    ros::NodeHandle* m_nh;
};

extern Parameters parameters;
