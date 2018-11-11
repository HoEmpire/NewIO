#pragma once

// #include "IOManagerBase.h"

#include "IMUReader.h"
#include "ServoIO.h"
#include "FeetSensorIO.h"

#include <vector>
#include <chrono>

namespace Motion
{

class IOManager2
{
public:
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

    IOManager2();

    virtual ~IOManager2();

    void spinOnce();

    // TODO deprecated function...
    void remapJointValues();

    void remapServoValues();

    void setSingleJointValue(const std::string name, const float value);


    const IMUData& getIMUData()
    {
        // return m_imu_reader.getIMUData();
        return m_imu_reader.m_data;
    }

    const PressureData& getPressureData()
    {
        return m_feet_io.m_data;
    }

    const std::vector<float>& getSingleJointValues()
    {
        return m_curr_joints;
    }

    const bool getSingleJointValue(const std::string joint_name, float& value);

    const PowerState getPowerState() const
    {
        return m_power_state;
    }

    void setServoPI(const std::vector<int> servo_id, const int p, const int i);

private:
    dynamixel::PortHandler* _initPort(const std::string portname, const int baudrate);

    void _checkPower();

    inline void _mapOneJoint(const int id);

private:
    IMUReader m_imu_reader;
    ServoIO m_servo_io;
    FeetSensorIO m_feet_io;

    std::chrono::time_point<std::chrono::system_clock> m_sync_time;

    // data received
    std::vector<float> m_curr_joints;

    PowerState m_power_state;
};

}
