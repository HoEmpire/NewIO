#include "dmotion/IO/IOManager3.h"

#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"

#include <chrono>

using namespace dynamixel;

// #define WAIT_MODE 1
// TODO(hhy) some hard code...

#define DATA_FREQUENCY 10.0    // 100hz data stream

namespace Motion
{

IOManager3::IOManager3()
    : m_power_state(OFF)
{
    ROS_DEBUG("IOManager3::IOManager3: Construct IOManager3 instance");
    initZJUJoint();
    m_servo_io.initServoPositions();
}

IOManager3::~IOManager3() = default;

dynamixel::PortHandler* IOManager3::initPort(const std::string portname, const int baudrate, const bool block)
{
    ROS_INFO_STREAM("IOManager3::initPort: init " << portname << " baudrate: " << baudrate );
    dynamixel::PortHandler* port_ = PortHandler::getPortHandler(portname.c_str());

    // Set port baudrate
    if (!port_->setBaudRate(baudrate,block))
    {
        ROS_FATAL("IOManager3::_initPort: could not change baudrate");
        ROS_FATAL("IOManager3::_initPort: are you stupid enough, dass du unfaehig zu eroeffnenung des Port bist?");
        std::abort();
    }
    return port_;
}

void IOManager3::initZJUJoint()
{
    // construct robot joint bases
    for (auto& pair:parameters.io.joint_cfg)
    {
        m_servo_io.m_joints.insert(
            std::make_pair(pair.first, Joint(pair.second))
        );
    }

    if (parameters.global.io_debug)
    {
        for (auto& joint:m_servo_io.m_joints)
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
}

void IOManager3::spinOnce()
{
      // use internal clock to calculate waiting time
      std::chrono::duration<double> duration_ = (timer::getCurrentSystemTime() - m_sync_time);
      double ticks = duration_.count()*1000;

      if (ticks > DATA_FREQUENCY)
      {
          ROS_WARN_STREAM("IOManager3::spinOnce:  motion ticks overflow..." << ticks);

      }
      else
      {
          timer::delay_ms(DATA_FREQUENCY - ticks - 0.1);
      m_sync_time = std::chrono::system_clock::now();//这句话的位置 TODO pyx before
      }
      m_servo_io.sendServoPositions();
      m_sync_time = std::chrono::system_clock::now();//这句话的位置 TODO pyx after
}

void IOManager3::setAllJointValue(const std::vector<double>& values_)
{
    m_servo_io.setSingleServoPosition("right_hip_yaw", values_[0]);
    m_servo_io.setSingleServoPosition("right_hip_roll", values_[1]);
    m_servo_io.setSingleServoPosition("right_hip_pitch", values_[2]);
    m_servo_io.setSingleServoPosition("right_knee", values_[3]);
    m_servo_io.setSingleServoPosition("right_ankle_pitch", values_[4]);
    m_servo_io.setSingleServoPosition("right_ankle_roll", values_[5]);

    m_servo_io.setSingleServoPosition("left_hip_yaw", values_[6]);
    m_servo_io.setSingleServoPosition("left_hip_roll", values_[7]);
    m_servo_io.setSingleServoPosition("left_hip_pitch", values_[8]);
    m_servo_io.setSingleServoPosition("left_knee", values_[9]);
    m_servo_io.setSingleServoPosition("left_ankle_pitch", values_[10]);
    m_servo_io.setSingleServoPosition("left_ankle_roll", values_[11]);

    m_servo_io.setSingleServoPosition("right_arm_upper", values_[12]);
    m_servo_io.setSingleServoPosition("right_arm_lower", values_[13]);
    m_servo_io.setSingleServoPosition("left_arm_upper", values_[14]);
    m_servo_io.setSingleServoPosition("left_arm_lower", values_[15]);
}

void IOManager3::setSingleJointValue(const std::string name, const double values_)
{
    m_servo_io.setSingleServoPosition(name, values_);
}

void IOManager3::readJointValue()
{
    m_servo_io.readServoPositions();
}

void IOManager3::reverseMotion()
{
    m_servo_io.TorqueOff();
    while(ros::ok()){
        std::cout << "Press any key to get a set of current joint value!(or press ESC to quit!)" << std::endl;
        char input;
        std::cin >> input;
        if (input == 27)
          break;
        readJointValue();
    }
}

void IOManager3::setServoPI(const std::vector<int> servo_id, const int p, const int i)
{
    m_servo_io.setServoPIMode(servo_id, p, i);
}

}
