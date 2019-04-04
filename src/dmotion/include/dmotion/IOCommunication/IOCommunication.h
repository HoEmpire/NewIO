#ifndef IOCOMMUNICATION_H
#define IOCOMMUNICATION_H

#include <thread>
#include <iostream>
#include <string>

#include "dmotion/IO/IOManager3.h"
#include "dmotion/State/StateManager.hpp"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"
#include "dmotion/PendulumWalk/PendulumWalk.h"

#include <std_msgs/Float64MultiArray.h>
#include "dmsgs/MotionInfo.h"
#include "dmsgs/ActionCommand.h"
#include "dmsgs/VisionInfo.h"
#include "dmsgs/SetInitOrientation.h"

namespace Motion
{

class IOCommunication
{
public:
     IOCommunication(ros::NodeHandle* nh);

     virtual ~IOCommunication();

     void IniIO();//初始化舵机

     void StateManagerLoop();//状态管理节点

     void *IOLoop();//IO节点

    // void IMULoop();//读IMU节点，考虑到其周期和IO可能不一致

     void Publisher();

     void Subscriber();

     void SetJointValue(const std_msgs::Float64MultiArray & msg);

     void ReadHeadServoValue(const dmsgs::ActionCommand & msg);

     void ReadVisionYaw(const dmsgs::VisionInfo & msg);

     void SetHeadServoValue();

     bool setFieldYaw(dmsgs::SetInitOrientation::Request &req,
                      dmsgs::SetInitOrientation::Response &res);

     dmsgs::MotionInfo m_motion_info;

     std_msgs::Float64MultiArray m_motion_hub;

private:
     ros::NodeHandle* m_nh;
     IOManager3 io;
     StateManager sm;
     std::vector<double> m_joint_value;
     std::string m_status;

     Motion::IMUData imu_data;
     Motion::PowerState power_data;
     Motion::PressureData pressure_data;
     std::vector<double> read_pos;
     std::vector<double> read_vel;
     float target_pitch, target_yaw;
     float desire_pitch, desire_yaw;
     float pitch_speed, yaw_speed;

     int lower_board_success_flag;

     ros::Publisher m_pub_motion_info;
     ros::Subscriber m_sub_motion_hub, m_sub_vision, m_sub_action_command;
     ros::ServiceServer m_motion_server;

     pthread_t tidIO;
     pthread_attr_t attr;
};

}
#endif
