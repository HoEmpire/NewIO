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

    void initZJUJoint();

    void spinOnce();

    // TODO deprecated function...
    void remapJointValues();

    void remapServoValues();

    void setJointValue(const std::string name, const float value);

    const std::vector<float>& getJointValues()
    {
        return m_curr_joints;
    }

    const bool getJointValue(const std::string joint_name, float& value);

    const PowerState getPowerState() const
    {
        return m_power_state;
    }

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
