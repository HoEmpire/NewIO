#include "dmotion/IO/IOManager3.h"

#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"

#include <chrono>
#include <thread>

using namespace dynamixel;

#define LEG_ONLY false //ONLY USE LEGS OF ZJU DANCER

#define DATA_FREQUENCY 10.0    // 100hz data stream
#define IMU_FREQUENCY 10.0    // 100hz data stream
#define POWER_DETECTER true   //whether open check power mode or not
                              //only used in a complete Robot
//#define old 1

namespace Motion
{
IOManager3::IOManager3()
    : m_power_state(OFF)
{
    ROS_DEBUG("IOManager3::IOManager3: Construct IOManager3 instance");
    initZJUJoint();
    std::thread t(&IOManager3::readIMU,this);
    t.detach();

    if(POWER_DETECTER)
        checkIOPower();
    else
        m_power_state = ON;

    if(m_power_state == ON)
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
      if(LEG_ONLY){
        if(!(pair.first ==  "right_arm_upper" || pair.first ==  "right_arm_lower" ||
             pair.first ==  "left_arm_upper" || pair.first ==  "left_arm_lower"||
             pair.first ==  "head_pitch" || pair.first ==  "head_yaw"))
                 m_servo_io.m_joints.insert(std::make_pair(pair.first, Joint(pair.second)));
      }
      else
          m_servo_io.m_joints.insert(std::make_pair(pair.first, Joint(pair.second)));

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

#ifdef old
void IOManager3::spinOnce()
{
    static int imu_failures = 0;
    if (ON == m_power_state)
    {
        // read imu
        if(POWER_DETECTER)
        {
            if (!m_imu_reader.readIMUData())
            {
                if (imu_failures++ > 10)
                {
                    m_power_state = OFF;
                    ROS_WARN("IOManager3::spinOnce: power off detected...");
                    return;
                }
            }
            else
            {
                imu_failures = 0;
            }
        }

        // read pressure data
        if (parameters.global.using_pressure)
        {
            if (!m_feet_io.readPressureData())
            {
                ROS_WARN("IOManager3::spinOnce: read feet pressure data error");
            }
        }
        a.toc();

        // use internal clock to calculate waiting time
        std::chrono::duration<double> duration_ = (timer::getCurrentSystemTime() - m_sync_time);
        double ticks = duration_.count()*1000;

        //set delay
        if (ticks > DATA_FREQUENCY)
        {
            ROS_WARN_STREAM("IOManager3::spinOnce:  motion ticks overflow..." << ticks);
        }
        else
        {
            timer::delay_ms(DATA_FREQUENCY - ticks - 0.1);
        }

        m_sync_time = timer::getCurrentSystemTime();//这句话的位置 TODO pyx after
        m_servo_io.sendServoPositions();

    }

    else if (OFF == m_power_state)
    {
        checkIOPower();

        if (ON == m_power_state)
        {
            m_servo_io.initServoPositions();
            m_power_state = ON;
            return;
        }
        timer::delay_ms(500);
    }
    else
    {
        ROS_ERROR("IOManager3::Surprise mother fucker....");
    }

}
#else
void IOManager3::spinOnce()
{
    //static int imu_failures = 0;
    if (ON == m_power_state)
    {

        // read pressure data
        if (parameters.global.using_pressure)
        {
            if (!m_feet_io.readPressureData())
            {
                ROS_WARN("IOManager3::spinOnce: read feet pressure data error");
            }
        }

        //
        // use internal clock to calculate waiting time
        std::chrono::duration<double> duration_ = (timer::getCurrentSystemTime() - m_sync_time);
        double ticks = duration_.count()*1000;

        //set delay
        if (ticks > DATA_FREQUENCY)
        {
            ROS_WARN_STREAM("IOManager3::spinOnce:  motion ticks overflow..." << ticks);
        }
        else
        {
            timer::delay_ms(DATA_FREQUENCY - ticks - 0.1);
        }

        m_sync_time = timer::getCurrentSystemTime();//这句话的位置 TODO pyx after
        m_servo_io.sendServoPositions();

    }
    else if (OFF == m_power_state)
    {
        timer::delay_ms(500);
    }
    else if (REOPEN == m_power_state)
    {
        m_servo_io.initServoPositions();
        m_power_state = ON;
        return;
    }
    else
    {
        ROS_ERROR("IOManager3::Surprise mother fucker....");
    }

}
#endif

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

    if(!LEG_ONLY){
      m_servo_io.setSingleServoPosition("right_arm_upper", values_[12]);
      m_servo_io.setSingleServoPosition("right_arm_lower", values_[13]);
      m_servo_io.setSingleServoPosition("left_arm_upper", values_[14]);
      m_servo_io.setSingleServoPosition("left_arm_lower", values_[15]);
    }
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
        std::cout << "Press F to get a set of current joint value!(or press OTHERS to quit!)" << std::endl;
        char input;
        std::cin >> input;
        if (input != 'f')
          break;
        readJointValue();
    }
}

void IOManager3::setServoPI(const std::vector<int> servo_id, const int p, const int i)
{
    m_servo_io.setServoPIMode(servo_id, p, i);
}

void IOManager3::ServoPowerOff()
{
    m_servo_io.TorqueOff();
}

#ifdef old
void IOManager3::checkIOPower()
{
    ROS_INFO("IOManager3::checkIOPower: checking whether power is on by imu");
    if (m_imu_reader.checkPower())
    {
        m_power_state = ON;
    }
    else
    {
        m_power_state = OFF;
    }
}
#else
void IOManager3::checkIOPower()
{
    ROS_INFO("IOManager3::checkIOPower: checking whether power is on by servo");
    if (m_servo_io.checkPower())
    {
        m_power_state = ON;
    }
    else
    {
        m_power_state = OFF;
    }
}
#endif

void IOManager3::readIMU(){
  while(ros::ok())
  {
      std::chrono::duration<double> duration_ = (timer::getCurrentSystemTime() - m_imu_sync_time);
      double ticks = duration_.count()*1000;

      //set delay
      if (ticks > IMU_FREQUENCY)
      {
          ROS_WARN_STREAM("IOManager3::readIMU:  motion ticks overflow..." << ticks);
      }
      else
      {
          timer::delay_ms(IMU_FREQUENCY - ticks - 0.1);
      }

      m_imu_sync_time = timer::getCurrentSystemTime();

      if(m_power_state == ON){
          //INFO("*****POWER ON*****");
          if (!m_imu_reader.readIMUData())
          {
              if (m_imu_failures++ > 10)
              {
                  m_power_state = OFF;
                  ROS_WARN("IOManager3::readIMU: power off detected...");
              }
          }
          else
          {
              m_imu_failures = 0;
          }
      }
      else if(m_power_state == OFF){
          //INFO("*****POWER OFF*****");
          if (m_imu_reader.checkPower())
          {
              m_power_state = REOPEN;
          }
          else
          {
              m_power_state = OFF;
          }
      }
      else if(m_power_state == REOPEN){
      }
      else
      {
          ROS_ERROR("IOManager3::Surprise mother fucker....");
      }
  }

}


}
