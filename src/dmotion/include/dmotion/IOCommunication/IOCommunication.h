#include <thread>
#include <iostream>

#include "dmotion/IO/IOManager3.h"
#include "dmotion/State/StateManager.hpp"
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"
#include "dmotion/PendulumWalk/PendulumWalk.h"

namespace Motion
{

public:
     IOCommunication();

     default ~IOCommunication();

     void IniIO();//初始化舵机

     void StateManagerLoop();//状态管理节点

     void IOLoop();//IO节点

     void IMULoop();//读IMU节点，考虑到其周期和IO可能不一致

     void MotionInfoPublisher();

     void MotionHubPublisher();

     void SetJointValue(const std_msgs::Float64MultiArray & msg);

     dmsgs::MotionInfo m_motion_info;

     std_msgs::Float64MultiArray m_motion_hub;

private:
     ros::NodeHandle m_nh;
     IOManager3 io;
     StateManager sm;
     std::vector<double> m_joint_value;

     Motion::IMUData imu_data;
     Motion::PowerState power_data;
     Motion::PressureData pressure_data;
     std::vector<double> read_pos(12,0);
     std::vector<double> read_vel(12,0);

     ros::Publisher m_pub_motion_info, m_pub_motion_hub;
     ros::Subscriber m_sub_motion_hub, m_sub_vision;

}
