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

    void initServoPositions();

    void sendServoPositions();

    void readServoPositions();

    void setServoSpeed(const int speed = 0);

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

    std::map<std::string, Joint> m_joints;
};

}
