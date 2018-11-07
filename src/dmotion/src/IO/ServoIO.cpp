#include "dmotion/IO/ServoIO.h"

#include "dmotion/IO/IOManager2.h"
#include "dmotion/IO/IOManager3.h"

#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#define Testing_Mode 1
using namespace dynamixel;

namespace Motion
{

ServoIO::ServoIO()
    : m_writer_inited(false)
    , m_servo_inited(false)
{
    ROS_DEBUG("ServoIO::ServoIO: init ServoIO instance");

    // init port
    m_servo_port = IOManager3::initPort(parameters.io.servo_port_name,
                                        parameters.io.servo_baudrate);

    // init protocol handler
    m_servo_protocol = PacketHandler::getPacketHandler(parameters.io.servo_protocol_version);

    // construct robot joint bases
    for (auto& pair:parameters.io.joint_cfg)
    {
        m_joints.insert(
            std::make_pair(pair.first, Joint(pair.second))
        );
    }

    if (parameters.global.io_debug)
    {
        for (auto& joint:m_joints)
        {
            ROS_DEBUG_STREAM("ServoIO::ServoIO:" << std::endl
                            << joint.first << ' '
                            << joint.second.cfg.id << ' '
                            << joint.second.cfg.cw << ' '
                            << joint.second.cfg.factor << ' '
                            << joint.second.cfg.id << ' '
                            << joint.second.cfg.init << ' '
                            << joint.second.cfg.max_pos << ' '
                            << joint.second.cfg.min_pos << ' '
                            << joint.second.cfg.resolution
                            );
        }
    }

    m_pos_writer = new GroupSyncWrite(m_servo_port, m_servo_protocol, ADDR_GOAL_POSITION, LENGTH_POSITION);
    m_pos_reader = new GroupSyncRead(m_servo_port, m_servo_protocol, ADDR_CURR_POSITION, LENGTH_POSITION);
}

ServoIO::~ServoIO()
{
    ROS_DEBUG("ServoIO:~ServoIO: destruct ServoIO instance");
    m_servo_port->closePort();
}

void ServoIO::initServoPositions()
{
    ROS_DEBUG("ServoIO::initServoPositions: init all servos");
    for (auto& joint:m_joints)
    {
        static uint8_t goal_position_[4];
        const JointConfig& _cfg = joint.second.cfg;

        // servo action: enable torque && turn on led && set velocity
        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_RETURN_LEVEL, 1);
        timer::delay_ms(20);    // no idea why here should be some lantency...
        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_TORQUE_ENABLE, 1);
        timer::delay_ms(20);
        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_LED, 1);
        timer::delay_ms(20);
        m_servo_protocol->write4ByteTxOnly(m_servo_port, _cfg.id, ADDR_PROFILE_VELOCITY, parameters.io.servo_init_speed);

        goal_position_[0] = DXL_LOBYTE(DXL_LOWORD(_cfg.init));
        goal_position_[1] = DXL_HIBYTE(DXL_LOWORD(_cfg.init));
        goal_position_[2] = DXL_LOBYTE(DXL_HIWORD(_cfg.init));
        goal_position_[3] = DXL_HIBYTE(DXL_HIWORD(_cfg.init));

        if (!m_writer_inited)
        {
            bool state = m_pos_writer->addParam(_cfg.id, goal_position_);
            if (!state)
            {
                ROS_ERROR_STREAM("ServoIO::initServoPositions: id " << _cfg.id << " add goal position param error");
            }

            // arm not included
            if (parameters.global.using_head_data)
            {
                if (joint.first.find("head") != std::string::npos)
                {
                    state = m_pos_reader->addParam(_cfg.id);
                }
                if (!state)
                {
                    ROS_ERROR_STREAM("ServoIO::initServoPositions: id " << _cfg.id << " add present position param error");
                }
            }

            if (parameters.global.using_odometry)
            {
                if (joint.first.find("hip") != std::string::npos
                    || joint.first.find("ankle") != std::string::npos
                    || joint.first.find("knee") != std::string::npos)
                {
                    state = m_pos_reader->addParam(_cfg.id);
                }
                if (!state)
                {
                    ROS_ERROR_STREAM("ServoIO::initServoPositions: id " << _cfg.id << " add present position param error");
                }
            }
        }
        else
        {
            bool state = m_pos_writer->changeParam(_cfg.id, goal_position_);
            if (!state)
            {
                ROS_ERROR_STREAM("ServoIO::initServoPositions: id " << _cfg.id << " change goal position param error");
            }
        }
    }

    // set velocity to max
    for (auto& joint:m_joints)
    {
        const JointConfig& _cfg = joint.second.cfg;

        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_LED, 0);
    }

    m_writer_inited = true;
    m_servo_inited = false;
}

void ServoIO::sendServoPositions()
{
    static uint8_t goal_position_buffer[4];
    static int goal_position_;
    static int init_ticks = 0;

    ROS_DEBUG("ServoIO::sendServoPositions: send servo positions");

    if (!m_servo_inited)
    {
        // if servo init done, just reset speed
        if (init_ticks > parameters.io.servo_init_ticks)
        {
            if (Testing_Mode)// TODO pyx 测试模式
              setServoSpeed(50);
            else
              setServoSpeed();

            init_ticks = 0;
            m_servo_inited = true;
        }
        else
        {
            init_ticks++;
        }
    }

    for (auto& joint:m_joints)
    {
        const Joint& _j = joint.second;
        const JointConfig& _cfg = _j.cfg;

        goal_position_ = _cfg.init + static_cast<int>(_j.goal_pos*_cfg.factor);

        goal_position_buffer[0] = DXL_LOBYTE(DXL_LOWORD(goal_position_));
        goal_position_buffer[1] = DXL_HIBYTE(DXL_LOWORD(goal_position_));
        goal_position_buffer[2] = DXL_LOBYTE(DXL_HIWORD(goal_position_));
        goal_position_buffer[3] = DXL_HIBYTE(DXL_HIWORD(goal_position_));

        m_pos_writer->changeParam(_cfg.id, goal_position_buffer);
    }

    m_pos_writer->txPacket();
}

void ServoIO::readServoPositions()
{
    ROS_DEBUG("ServoIO::readServoPositions: read servo positions");
    m_pos_reader->txRxPacket();

    for (auto& joint:m_joints)
    {
        if (parameters.global.using_head_data
            && joint.first.find("head") != std::string::npos)
        {
            Joint& _j = joint.second;
            const JointConfig& _cfg = _j.cfg;

            if (!m_pos_reader->isAvailable(_cfg.id, ADDR_CURR_POSITION, LENGTH_POSITION))
            {
                ROS_WARN("ServoIO::readServoPositions: get current joint value error");
            }
            else
            {
                _j.real_pos = (static_cast<int>(m_pos_reader->getData(_cfg.id, ADDR_CURR_POSITION, LENGTH_POSITION))-_cfg.init)/_cfg.factor;
                std::cout << _cfg.id << ' ' << _j.real_pos << std::endl;
            }
        }
        if (parameters.global.using_odometry &&
            (joint.first.find("hip") != std::string::npos
            || joint.first.find("knee") != std::string::npos
            || joint.first.find("ankle") != std::string::npos
            ))
        {
            Joint& _j = joint.second;
            const JointConfig& _cfg = _j.cfg;

            if (!m_pos_reader->isAvailable(_cfg.id, ADDR_CURR_POSITION, LENGTH_POSITION))
            {
                ROS_WARN("ServoIO::readServoPositions: get current joint value error");
            }
            else
            {
                _j.real_pos = (static_cast<int>(m_pos_reader->getData(_cfg.id, ADDR_CURR_POSITION, LENGTH_POSITION))-_cfg.init)/_cfg.factor;
                std::cout << _cfg.id << ' ' << _j.real_pos << std::endl;
            }
        }

    }
        // if (joint.first.find("arm") != std::string::npos)
        //     continue;
}

void ServoIO::setServoSpeed(const int speed)
{
    ROS_DEBUG_STREAM("ServoIO::setServoSpeed: set servo speed to " << speed);
    for (auto& joint:m_joints)
    {
        const JointConfig& _cfg = joint.second.cfg;

        m_servo_protocol->write4ByteTxOnly(m_servo_port, _cfg.id, ADDR_PROFILE_VELOCITY, speed);
    }
}

void ServoIO::setServoPIMode(std::vector<int> servo_id, const int P, const int I)
{
    static uint8_t pi_buffer[4];

    ROS_DEBUG_STREAM("ServoIO::setServoPIMode: set servo P : " << P << " set servo I " << I);

    dynamixel::GroupSyncWrite writer_(m_servo_port, m_servo_protocol, ADDR_PI, LENGTH_PI);

    pi_buffer[0] = DXL_LOBYTE(I);
    pi_buffer[1] = DXL_HIBYTE(I);
    pi_buffer[2] = DXL_LOBYTE(P);
    pi_buffer[3] = DXL_HIBYTE(P);

    for(int i = 0; i < servo_id.size(); i++)
    {
        writer_.addParam(servo_id[i], pi_buffer);
    }

    writer_.txPacket();
}

}
