#include "dmotion/IO/IOManager2.h"

#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"

#include <chrono>

using namespace dynamixel;

// #define WAIT_MODE 1

// TODO(hhy) some hard code...

#define DATA_FREQUENCY 10.0    // 100hz data stream

namespace Motion
{

IOManager2::IOManager2()
    : m_power_state(OFF)
{
    ROS_DEBUG("IOManager2::IOManager2: Construct IOManager2 instance");

    // set port pointer to feet io
    m_feet_io.setPort(m_servo_io.getPortHandler());

    // check power && init if power is on
    _checkPower();

    if (ON == m_power_state)
    {
       m_servo_io.initServoPositions();

       // cuz _initServoPosition needs some time
       // so we have to call _checkPower() again to
       // synchronize time_stamp
       m_imu_reader.clearPort();

       // recheck power state
       _checkPower();
    }
}

IOManager2::~IOManager2() = default;

dynamixel::PortHandler* IOManager2::initPort(const std::string portname, const int baudrate, const bool block)
{
    ROS_INFO_STREAM("IOManager2::initPort: init " << portname << " baudrate: " << baudrate );
    dynamixel::PortHandler* port_ = PortHandler::getPortHandler(portname.c_str());

    // Set port baudrate
    if (!port_->setBaudRate(baudrate,block))
    {
        ROS_FATAL("IOManager2::_initPort: could not change baudrate");
        std::abort();
    }

    // open imu port
    if (!port_->openPort(block))
    {
        ROS_FATAL("IOManager2::_initPort: could not open IMU port");
        std::abort();
    }

    return port_;
}

void IOManager2::_checkPower()
{
    ROS_INFO("IOManager2::_checkPower: checking whether power is on");
    if (m_imu_reader.checkPower())
    {
        m_power_state = ON;
        //m_sync_time = m_imu_reader.getSyncTimePoint(); TODO
    }
    else
    {
        m_power_state = OFF;
    }
}

void IOManager2::spinOnce()
{
    static int imu_failures = 0;

    // timer t;
    // we only need this flag for one frame
    // to initialize other modules (e.g. actionManager)
    if (m_power_state == REOPEN)
    {
        m_power_state = ON;
    }

    // loading imu data
    if (m_power_state == ON)
    {
        // use internal clock to calculate waiting time
        std::chrono::duration<double> duration_ = (timer::getCurrentSystemTime() - m_sync_time);
        double ticks = duration_.count()*1000;

        // here we determine whether processing time
        // within pre-set prequency, and just sleep
        // if there is any rest time
        if (ticks > DATA_FREQUENCY)
        {
            ROS_WARN_STREAM("IOManager2::spinOnce:  motion ticks overflow..." << ticks);

            // _checkPower();
            // if (OFF == m_power_state)
            // {
            //     ROS_WARN("IOManager2::spinOnce:  power off detected...");
            //     return;
            // }
        }
        // else
        {
            timer::delay_ms(DATA_FREQUENCY - ticks - 0.1);

            // t.tic();
            // if not true means lower board power off
            if (!m_imu_reader.readIMUData())
            {
                if (imu_failures++ > 10)
                {
                    m_power_state = OFF;
                    ROS_WARN("IOManager2::spinOnce: power off detected...");
                    return;
                }
            }
            else
            {
                imu_failures = 0;
              //  m_sync_time = m_imu_reader.getSyncTimePoint(); TODO
             }
        }
        // double duration1 = t.toc();
        // std::cout << "imu reader takes time " << duration1 << std::endl;
        // t.tic();
        m_servo_io.sendServoPositions();

        // read pressure data
        if (parameters.global.using_pressure)
        {
            if (!m_feet_io.readPressureData())
            {
                ROS_WARN("IOManager2::spinOnce: read feet pressure data error");
            }
        }

        // PressureData data = m_feet_io.getPressureData();

        // double duration2 = t.toc();
        // std::cout << "send position takes time " << duration2 << std::endl;
        // t.tic();
        // m_servo_io.readServoPositions();
        // double duration3 = t.toc();
        // std::cout << "read position takes time " << duration3 << std::endl;

        // ROS_INFO_STREAM("io takes time " << duration1+duration2+duration3);
    }
    else if (m_power_state == OFF)
    {
        _checkPower();

        if (ON == m_power_state)
        {
            m_servo_io.initServoPositions();

            m_power_state = REOPEN;

            return;
        }
        timer::delay_ms(500);
    }
    else
    {
        ROS_ERROR("IOManager2::spinOnce: what the fuck....");
    }
}

// void IOManager2::setJointValue(const std::string name, const float value)
// {
//     auto _joint = m_servo_io.m_joints.find(name);
//     if (_joint == m_servo_io.m_joints.end())
//     {
//         ROS_FATAL("joint not existing... check joint name");
//         return;
//     }
//
//     _joint->second.goal_pos = value;
//
//     ROS_DEBUG_STREAM("IOManager2::setJointValue: joint name: " << name << " values: " << value);
// }
//
// const bool IOManager2::getJointValue(const std::string joint_name, float& value)
// {
//     auto _joint = m_servo_io.m_joints.find(joint_name);
//     if (_joint == m_servo_io.m_joints.end())
//     {
//         ROS_FATAL("joint not existing... check joint name");
//         return false;
//     }
//     value = _joint->second.real_pos;
//
//     return true;
// }

void IOManager2::remapJointValues()
{
    for (int i = 1; i != parameters.io.joint_number; i ++)
    {
        _mapOneJoint(i);
    }
}

inline void IOManager2::_mapOneJoint(const int id)
{
    // m_servo_io.m_servo_desired_pos[id] = parameters.io.joint_init_values[id] + parameters.io.joint_cw[id]*static_cast<int>(m_joint_values[id]/M_PI/2.0*parameters.io.joint_resolution[id]);
}

void IOManager2::remapServoValues()
{
    // static int count = 0;
    m_curr_joints[2] = (m_servo_io.m_servo_present_pos[0] - parameters.io.joint_init_values[0])*parameters.io.joint_cw[0]*2.0/parameters.io.joint_resolution[0]*M_PI;
    m_curr_joints[1] = (m_servo_io.m_servo_present_pos[1] - parameters.io.joint_init_values[1])*parameters.io.joint_cw[1]*2.0/parameters.io.joint_resolution[1]*M_PI;
    m_curr_joints[0] = (m_servo_io.m_servo_present_pos[2] - parameters.io.joint_init_values[2])*parameters.io.joint_cw[2]*2.0/parameters.io.joint_resolution[2]*M_PI;
    m_curr_joints[3] = (m_servo_io.m_servo_present_pos[3] - parameters.io.joint_init_values[3])*parameters.io.joint_cw[3]*2.0/parameters.io.joint_resolution[3]*M_PI;
    m_curr_joints[4] = (m_servo_io.m_servo_present_pos[4] - parameters.io.joint_init_values[4])*parameters.io.joint_cw[4]*2.0/parameters.io.joint_resolution[4]*M_PI;
    m_curr_joints[5] = (m_servo_io.m_servo_present_pos[5] - parameters.io.joint_init_values[5])*parameters.io.joint_cw[5]*2.0/parameters.io.joint_resolution[5]*M_PI;

    m_curr_joints[8] = (m_servo_io.m_servo_present_pos[6] - parameters.io.joint_init_values[6])*parameters.io.joint_cw[6]*2.0/parameters.io.joint_resolution[6]*M_PI;
    m_curr_joints[7] = (m_servo_io.m_servo_present_pos[7] - parameters.io.joint_init_values[7])*parameters.io.joint_cw[7]*2.0/parameters.io.joint_resolution[7]*M_PI;
    m_curr_joints[6] = (m_servo_io.m_servo_present_pos[8] - parameters.io.joint_init_values[8])*parameters.io.joint_cw[8]*2.0/parameters.io.joint_resolution[8]*M_PI;
    m_curr_joints[9] = (m_servo_io.m_servo_present_pos[9] - parameters.io.joint_init_values[9])*parameters.io.joint_cw[9]*2.0/parameters.io.joint_resolution[9]*M_PI;
    m_curr_joints[10] = (m_servo_io.m_servo_present_pos[10] - parameters.io.joint_init_values[10])*parameters.io.joint_cw[10]*2.0/parameters.io.joint_resolution[10]*M_PI;
    m_curr_joints[11] = (m_servo_io.m_servo_present_pos[11] - parameters.io.joint_init_values[11])*parameters.io.joint_cw[11]*2.0/parameters.io.joint_resolution[11]*M_PI;

}

void IOManager2::setServoPI(const std::vector<int> servo_id, const int p, const int i)
{
    m_servo_io.setServoPIMode(servo_id, p, i);
}

}
