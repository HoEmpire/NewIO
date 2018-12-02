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
    /// @brief      read all the joints
    ////////////////////////////////////////////////////////////////////////////////
    void readJointValue();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief       type f and write all sets of servo angel a time
    ////////////////////////////////////////////////////////////////////////////////
    void reverseMotion();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief       turn off power of servo
    ////////////////////////////////////////////////////////////////////////////////
    void ServoPowerOff();

    const IMUData& getIMUData()
    {
        if(DEBUG_OUTPUT){
          std::cout << std::setprecision (3);
          std::cout << "grpo_x: " << m_imu_reader.m_data.gypo.x <<std::endl;
          std::cout << "grpo_y: " << m_imu_reader.m_data.gypo.y <<std::endl;
          std::cout << "grpo_z: " << m_imu_reader.m_data.gypo.z <<std::endl;
          std::cout << "accl_x: " << m_imu_reader.m_data.accl.x <<std::endl;
          std::cout << "accl_y: " << m_imu_reader.m_data.accl.y <<std::endl;
          std::cout << "accl_z: " << m_imu_reader.m_data.accl.z <<std::endl;
        }
        return m_imu_reader.m_data;
    }

    const PowerState getPowerState() const
    {
        return m_power_state;
    }

    const PressureData& getPressureData() const
    {
        return m_feet_io.m_data;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief             set the PI of the joint
    /// @param values_     values of the joints(in degree)
    /// @param P           P of the PID kick_controller | range:0 ~ 16,383 |equation:KPP = KPP(TBL) / 128
    /// @param I           I of the PID kick_controller | range:0 ~ 16,383 |equation:KPI = KPI(TBL) / 65,536
    ////////////////////////////////////////////////////////////////////////////////
    void setServoPI(const std::vector<int> servo_id, const int p, const int i);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief             check power is on or not by checking whether thers is data coming from IMU
    ////////////////////////////////////////////////////////////////////////////////
    void checkIOPower();

    void setAllspeed(int v);

    std::vector<double> readAllPosition();

    void readIMU();

    void checkPower();

private:
    dynamixel::PortHandler* _initPort(const std::string portname, const int baudrate);

private:
    ServoIO m_servo_io;
    FeetSensorIO m_feet_io;
    IMUReader m_imu_reader;

    std::chrono::time_point<std::chrono::steady_clock> m_sync_time;//sync time in the thread of spinOnce
    std::chrono::time_point<std::chrono::steady_clock>  m_imu_sync_time;//sync time in the thread of readIMU

    // data received
    std::vector<float> m_curr_joints;
    int m_imu_failures = 0;

    PowerState m_power_state;
};

}
