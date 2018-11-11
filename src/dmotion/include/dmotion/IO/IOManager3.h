#pragma once

#include "IMUReader.h"
#include "ServoIO.h"
#include "FeetSensorIO.h"

#include <vector>
#include <chrono>

namespace Motion
{

class IOManager3
{
public:
    friend class ServoIO;
    static dynamixel::PortHandler*
    initPort(const std::string portname, const int baudrate, const bool block = false);

    enum Addr{
        ADDR_TORQUE_ENABLE = 64,
        ADDR_LED = 65,
        ADDR_RETURN_LEVEL = 68,
        ADDR_GOAL_POSITION = 116,
        ADDR_PROFILE_VELOCITY = 112,
        ADDR_CURR_POSITION = 132
    };

    enum Length{
        LENGTH_POSITION = 4,
        LENGTH_VELOCITY = 4
    };

    IOManager3();

    virtual ~IOManager3();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief          init Servo of ZJUDANCER
    ////////////////////////////////////////////////////////////////////////////////
    void initZJUJoint();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief          send data one time
    ////////////////////////////////////////////////////////////////////////////////
    void spinOnce();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief          set the goal position of a single joint
    /// @param name     name of the joint
    /// @param values_  values of the joint(in degree)
    ////////////////////////////////////////////////////////////////////////////////
    void setSingleJointValue(const std::string name, const double values_);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief             set the goal position of all the joints
    /// @param values_     values of the joints(in degree)
    ///                    values_ is vector of size 16
    ///                    0      right_hip_yaw
    ///                    1      right_hip_roll
    ///                    2      right_hip_pitch
    ///                    3      right_knee
    ///                    4      right_ankle_pitch
    ///                    5      right_ankle_roll
    ///                    6      left_hip_yaw
    ///                    7      left_hip_roll
    ///                    8      left_hip_pitch
    ///                    9      left_knee
    ///                    10     left_ankle_pitch
    ///                    11     left_ankle_roll
    ///                    12     right_arm_upper
    ///                    13     right_arm_lower
    ///                    14     left_arm_upper
    ///                    15     left_arm_lower
    ////////////////////////////////////////////////////////////////////////////////
    void setAllJointValue(const std::vector<double>& values_);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief      read all the joints(works not well,wenn es mehr als 4 Servo gibt)
    ////////////////////////////////////////////////////////////////////////////////
    void readJointValue();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief       currently can't not work
    ////////////////////////////////////////////////////////////////////////////////
    void reverseMotion();

    const PowerState getPowerState() const
    {
        return m_power_state;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief             set the PI of the joint
    /// @param values_     values of the joints(in degree)
    /// @param P           P of the PID kick_controller | range:0 ~ 16,383 |equation:KPP = KPP(TBL) / 128
    /// @param I           I of the PID kick_controller | range:0 ~ 16,383 |equation:KPI = KPI(TBL) / 65,536
    ////////////////////////////////////////////////////////////////////////////////
    void setServoPI(const std::vector<int> servo_id, const int p, const int i);

private:
    dynamixel::PortHandler* _initPort(const std::string portname, const int baudrate);

    void _checkPower();

private:
    ServoIO m_servo_io;

    std::chrono::time_point<std::chrono::system_clock> m_sync_time;

    // data received
    std::vector<float> m_curr_joints;

    PowerState m_power_state;
};

}
