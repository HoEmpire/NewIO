#pragma once

#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "dynamixel_sdk/group_sync_read.h"
#include "dynamixel_sdk/group_bulk_read.h"

#include "dmotion/Common/Type.h"

namespace Motion
{

class ServoIO
{
public:
    //friend class IOManager2;
    friend class IOManager3;

    enum Addr{
        ADDR_TORQUE_ENABLE = 64,
        ADDR_LED = 65,
        ADDR_RETURN_LEVEL = 68,
        ADDR_RETURN_DELAY = 9,
        ADDR_GOAL_POSITION = 116,
        ADDR_PROFILE_VELOCITY = 112,
        ADDR_CURR_POSITION = 132,
        ADDR_PI = 82
    };

    enum Length{
        LENGTH_POSITION = 4,
        LENGTH_VELOCITY = 4,
        LENGTH_PI = 4,
        LENGTH_LED = 1
    };

    ServoIO();
    ~ServoIO();

    dynamixel::PortHandler* getPortHandler() const
    {
        return m_servo_port;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that adds joint to m_joints
    /// @param name                             the name of the joint
    /// @param Joints_cfg                       the configs of the joints
    ///        joints_cfg.cfg.cw_               clockwise
    ///        joints_cfg.cfg.resolution_       resolution
    ///        joints_cfg.cfg.init_             init positions
    ///        joints_cfg.cfg.max_pos_          Maximum position
    ///        joints_cfg.cfg.min_pos_          minimum positions
    /// @example
    ///        Motion::ServoIO servo;
    ///                                        [ id | cw | resolution | init_position |  max_pos |  min_pos ]
    ///        struct Motion::JointConfig  _cfg(14,  -1,     4096,         203,          4096,        0);
    ///        struct Motion::Joint  joints_cfg(_cfg);
    ///        servo.addJoint("name", joints_cfg);
    ////////////////////////////////////////////////////////////////////////////////
    void addJoint(std::string name, Joint Joints_cfg);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief init Servo to init position
    ////////////////////////////////////////////////////////////////////////////////
    void initServoPositions();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief turn off all the torque
    ////////////////////////////////////////////////////////////////////////////////
    void TorqueOff();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Send whohle set of position data
    ////////////////////////////////////////////////////////////////////////////////
    void sendServoPositions();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Read whole set of position data
    ////////////////////////////////////////////////////////////////////////////////
    void readServoPositions();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Read whole set of position data(bad version)
    ////////////////////////////////////////////////////////////////////////////////
    void readServoPositionsBad();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the profile speed of all the servo in m_joints
    ///        default means the max speed
    /// @param speed Profile speed of the servo | range:0 ~ 32767
    ////////////////////////////////////////////////////////////////////////////////
    void setAllServoSpeed(const int speed = 0);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the profile speed of a single servo based on name
    ///        default means the max speed
    /// @param name       name of the servo
    /// @param speed      Profile speed of the servo | range:0 ~ 32767 | unit:0.229 [rev/min]
    ////////////////////////////////////////////////////////////////////////////////
    void setSingleServoSpeed(std::string name, int speed);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the profile speed of a single servo based on id
    ///        default means the max speed
    /// @param servo_id   id of the servo
    /// @param speed      Profile speed of the servo | range:0 ~ 32767 | unit:0.229 [rev/min]
    ////////////////////////////////////////////////////////////////////////////////
    void setSingleServoSpeed(int servo_id, int speed);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the profile speed of a single servo based on name
    ///        default means the max speed
    /// @param name       name of the servo
    /// @param position   set goal position of the servo | range:0 ~ 360°
    ////////////////////////////////////////////////////////////////////////////////
    void setSingleServoPosition(std::string name, double position);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the PI parameters of the servo controller
    /// @param servo_id       id of the servos
    /// @param P              P of the PID kick_controller | range:0 ~ 16,383 |equation:KPP = KPP(TBL) / 128
    /// @param I              I of the PID kick_controller | range:0 ~ 16,383 |equation:KPI = KPI(TBL) / 65,536
    ////////////////////////////////////////////////////////////////////////////////
    void setServoPIMode(std::vector<int> servo_id, const int P, const int I);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief      check power by read servo(not good,it takes about 1ms, which is unacctable)
    ///             now it's used to check power berfore ini the servo for the first time(and only first time)
    /// @return     return true when power is on
    ///             or false when power is off
    ////////////////////////////////////////////////////////////////////////////////
    bool checkPower();

private:
    bool m_writer_inited, m_reader_inited, m_servo_inited;

    dynamixel::PortHandler* m_servo_port;
    dynamixel::PacketHandler* m_servo_protocol;

    dynamixel::GroupSyncWrite * m_pos_writer;
    dynamixel::GroupSyncRead * m_pos_reader;
    dynamixel::GroupSyncRead * m_pos_power;//use the servo to check power for the first time


protected:
    // data
    std::vector<int> m_servo_desired_pos;
    std::vector<int> m_servo_present_pos;
    std::map<std::string, Joint> m_joints;//all the joints data

};

}
