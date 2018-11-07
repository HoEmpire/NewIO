#pragma once

#include "Type.h"

#include <vector>
#include <string>

#include <ros/ros.h>

// add using_pressure, pressure_counting_start, pressure_counting_number

struct GlobalParameters
{
    bool simulation;
    bool enable_ankle;

    double yaw_offset = M_PI / 2.0;

    bool using_pressure = false;
    bool using_odometry = false;
    bool using_head_data = false;
    bool using_lateral_stablizer = false;
    bool using_push_recovery = false;

    bool debug = false;
    bool io_debug = false;
    bool stablizer_debug = false;
    bool imu_debug = false;
};

struct IOParameters
{
    // imu
    std::string imu_port_name;
    int imu_baudrate;
    std::string imu_version;

    // servo
    std::string servo_port_name;
    int servo_baudrate;
    float servo_protocol_version;
    int servo_init_ticks;
    int servo_init_speed;

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

struct MotionParameters
{
    // geometry parameters
    float length_tibia;
    float length_thigh;
    float length_hip;

    // crounch parameters
    int crounch_ticks;
    float crounch_com_height;
    float crounch_ankle_distance;

    // basic offsets
    float com_offset_x;
    float com_offset_y;
    float com_offset_roll;
    float com_offset_pitch;

    // controller parameters
    float ankle_roll_p;
    float ankle_roll_i;
    float ankle_roll_d;
    float ankle_roll_min;
    float ankle_roll_max;
    float ankle_ramp_rate;
};

struct StateParameters
{
    // imu parameters
    float imu_bias_gain = 0.0;
    float imu_algorithm_gain = 0.1;

    float imu_roll_bias;
    float imu_pitch_bias;

    float imu_yaw_vel_correction;
    float imu_prepare_time;

    // pressure data &&
    int pressure_counting_start;
    int pressure_counting_end;

    // phase detection parameters
    float security_threshold;
    int security_delay_ticks;
    float com_roll_threshold_right, com_roll_threshold_left;
    float velocity_threshold;

    // fall detection parameters
    double front_down_threshold = 60.0;
    double back_down_threshold = -60.0;
    double side_down_threshold = 60.0;
    int front_assertion_ticks = 10;
    int back_assertion_ticks = 10;
    int side_assertion_ticks = 10;
    int stable_assertion_ticks = 10;
};

struct WalkingParameters
{
    double freq = 1.5;
    //Length of double support phase in half cycle
    //(ratio, [0:1]
    double doubleSupportRatio = 0.2;
    //Lateral distance between the feet center
    //(in m, >= 0
    double footDistance = 0.13;
    //Maximum flying foot height
    //(in m, >= 0
    double footRise = 0.035;
    //Phase of flying foot apex
    //(single support cycle phase, [0:1]
    double footApexPhase = 0.5;
    //Foot X/Y overshoot in ratio of step length
    //(ratio, >= 0
    double footOvershootRatio = 0.05;
    //Foot X/Y overshoot phase
    //(single support cycle phase, [footApexPhase:1]
    double footOvershootPhase = 0.85;
    //Height of the trunk from ground
    //(in m, > 0
    double trunkHeight = 0.27;
    //Trunk pitch orientation
    //(in rad
    double trunkPitch = 0.2;
    //Phase offset of trunk oscillation
    //(half cycle phase, [0:1]
    double trunkPhase = 0.4;
    //Trunk forward offset
    //(in m
    double trunkXOffset = 0.005;
    //Trunk lateral offset
    //(in m
    double trunkYOffset = 0.0;
    //Trunk lateral oscillation amplitude ratio
    //(ratio, >= 0
    double trunkSwing = 0.3;
    //Trunk swing pause length in phase at apex
    //(half cycle ratio, [0:1]
    double trunkPause = 0.0;
    //Trunk forward offset proportional to forward step
    //(in 1
    double trunkXOffsetPCoefForward = 0.0;
    //Trunk forward offset proportional to rotation step
    //(in m/rad
    double trunkXOffsetPCoefTurn = 0.0;
    //Trunk pitch orientation proportional to forward step
    //(in rad/m
    double trunkPitchPCoefForward = 0.0;
    //Trunk pitch orientation proportional to rotation step
    //(in 1
    double trunkPitchPCoefTurn = 0.0;
};

struct KickParameters
{
    bool using_pi_mode = false;
    bool using_support_stablizer = false;
    bool using_stronger_kick = false;

    int kick_controller_p = 850;
    int kick_controller_i = 100;
};

class Parameters
{
public:
    GlobalParameters global;
    IOParameters io;
    MotionParameters motion;
    StateParameters state;

    WalkingParameters walk;

    KickParameters kick;

    void init(ros::NodeHandle* nh);

    void update();

    void setFieldYaw(const double yaw);

    ~Parameters() = default;

private:
    void configServoMap();

private:
    ros::NodeHandle* m_nh;
};

extern Parameters parameters;
