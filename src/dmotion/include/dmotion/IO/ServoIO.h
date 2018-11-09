#pragma once

#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "dynamixel_sdk/group_sync_read.h"

#include "dmotion/Common/Type.h"

namespace Motion
{

class ServoIO
{
public:
    friend class IOManager2;
    friend class IOManager3;

    enum Addr{
        ADDR_TORQUE_ENABLE = 64,
        ADDR_LED = 65,
        ADDR_RETURN_LEVEL = 68,
        ADDR_GOAL_POSITION = 116,
        ADDR_PROFILE_VELOCITY = 112,
        ADDR_CURR_POSITION = 132,
        ADDR_PI = 82
    };

    enum Length{
        LENGTH_POSITION = 4,
        LENGTH_VELOCITY = 4,
        LENGTH_PI = 4
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
    ////////////////////////////////////////////////////////////////////////////////
    void addJoint(std::string name, Joint Joints_cfg);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief init Servo to init position
    ////////////////////////////////////////////////////////////////////////////////
    void initServoPositions();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Send whohle set of position data
    ////////////////////////////////////////////////////////////////////////////////
    void sendServoPositions();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Read whole set of position data
    ////////////////////////////////////////////////////////////////////////////////
    void readServoPositions();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the profile speed of all the servo in m_joints
    ///        default means the max speed
    /// @param speed Profile speed of the servo | range:0 ~ 32767
    ////////////////////////////////////////////////////////////////////////////////
    void setAllServoSpeed(const int speed = 0);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the profile speed of a single servo based on name
    ///        default means the max speed
    /// @param speed      Profile speed of the servo | range:0 ~ 32767 | unit:0.229 [rev/min]
    /// @param name       name of the servo
    ////////////////////////////////////////////////////////////////////////////////
    void setSingleServoSpeed(int speed, std::string name);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the profile speed of a single servo based on id
    ///        default means the max speed
    /// @param speed      Profile speed of the servo | range:0 ~ 32767 | unit:0.229 [rev/min]
    /// @param servo_id   id of the servo
    ////////////////////////////////////////////////////////////////////////////////
    void setSingleServoSpeed(int speed, int servo_id);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the PI parameters of the servo controller
    /// @param servo_id       id of the servos
    /// @param P              P of the PID kick_controller | range:0 ~ 16,383 |equation:KPP = KPP(TBL) / 128
    /// @param I              I of the PID kick_controller | range:0 ~ 16,383 |equation:KPI = KPI(TBL) / 65,536
    ////////////////////////////////////////////////////////////////////////////////
    void setServoPIMode(std::vector<int> servo_id, const int P, const int I);

private:
    bool m_writer_inited, m_servo_inited;

    dynamixel::PortHandler* m_servo_port;
    dynamixel::PacketHandler* m_servo_protocol;

    dynamixel::GroupSyncWrite * m_pos_writer;
    dynamixel::GroupSyncRead * m_pos_reader;


protected:
    // data
    std::vector<int> m_servo_desired_pos;
    std::vector<int> m_servo_present_pos;
    std::map<std::string, Joint> m_joints;//all the joints data

};

}
