#include "dmotion/IOCommunication/IOCommunication.h"
#define STATE_MANAGER_LOOP_TIME 10.0
#define IMU_LOOP_TIME 10.0

namespace Motion
{

IOCommunication::IOCommunication(ros::NodeHandle* nh)
  : m_nh(nh)
{
    parameters.init(m_nh);
    IniIO();
    std::thread IO_thread(IOLoop);
    IO_thread.detach();
    timer::delay_ms(100.0);
    std::thread IMU_thread(IMULoop);
    IMU_thread.detach();
    timer::delay_ms(100.0);
    std::thread StateManager_thread(StateManager_threadLoop);
    StateManager_thread.detach();

    m_sub_motion_hub = m_nh->subscribe("/humanoid/ReloadMotionConfig", 1, &IOCommunication::SetJointValue, this);//TODO
    m_sub_vision = m_nh->subscribe("/humanoid/ReloadMotionConfig", 1, &IOCommunication::SetJointValue, this);//TODO
    m_pub_motion_info = m_nh->advertise<dmsgs::MotionInfo>("MotionInfo", 1);
    m_pub_motion_hub = m_nh->advertise<dmsgs::MotionDebugInfo>("MotionHub", 1);
}


void IOCommunication::IniIO()
{
    while(!io.m_servo_inited)
        timer::delay_ms(10.0);
    PendulumWalk pen;
    pen.GiveAStep(0,0,0);
    m_joint_value = pen.GiveATick();
    io.setAllJointValue(m_joint_value);
    io.spinOnce();
    sleep(2);
}


void IOCommunication::StateManagerLoop()
{
    timer a;
    while(ros::ok()){
      a.tic();
      sm.imu_data = imu_data;
      sm.m_power_state = power_data;
      sm.servo_initialized = servo_state;
      sm.pressure_data = pressure_data;
      sm.working();
      a.SmartDelayMs(STATE_MANAGER_LOOP_TIME);//TODO
    }
}


void IOCommunication::IOLoop()
{
  while(ros::ok())
  {
    if(!io.m_servo_inited)
        IniIO();
    else
    {
      io.spinOnce();
      io.readPosVel();
      power_data = io.getPowerState();
      pressure_data = io.getPressureData();
      read_pos = io.readAllPosition();
      read_vel = io.readAllVel();
    }
  }
}


void IOCommunication::IMULoop()
{
  timer b;
  while(ros::ok())
  {
     b.tic();
     if(io.m_servo_inited)
        imu_data = io.getIMUData();
     b.SmartDelayMs(IMU_LOOP_TIME);//TODO
  }
}

void IOCommunication::SetJointValue(const std_msgs::Float64MultiArray & msg)
{
   m_joint_value = msg.data;
}

void IOCommunication::MotionInfoPublisher()
{
    ros::Rate loop_rata(100.0)
    m_motion_info.imuRPY.x = sm.roll;
    m_motion_info.imuRPY.y = sm.pitch;
    m_motion_info.imuRPY.z = sm.yaw;
    while(ros::ok())
    {
      m_pub_motion_info.publish(m_motion_info);
      loop_rata.sleep();
    }
}

void IOCommunication::MotionHubPublisher() //TODO
{
    ros::Rate loop_rata(100.0)
    while(ros::ok())
    {
      m_pub_motion_hub.publish(msg)
      loop_rata.sleep();
    }
}

void IOCommunication::MotionHubSubscriber() //TODO
{
    ros::Rate loop_rata(10.0)
    while(ros::ok())
    {
      m_pub_motion_hub.publish(msg)
      loop_rata.sleep();
    }
}

void IOCommunication::VisionSubscriber() //TODO
{
    ros::Rate loop_rata(10.0)
    while(ros::ok())
    {
      m_pub_motion_hub.publish(msg)
      loop_rata.sleep();
    }
}


}
