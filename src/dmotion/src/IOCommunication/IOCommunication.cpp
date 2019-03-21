#include "dmotion/IOCommunication/IOCommunication.h"
#define STATE_MANAGER_LOOP_TIME 10.0
#define IMU_LOOP_TIME 10.0

namespace Motion
{

IOCommunication::IOCommunication(ros::NodeHandle* nh)
  : m_nh(nh)
{
//    parameters.init(m_nh);
    //INFO("wait for ini io to be done");
    IniIO();
    //INFO("ini io done");
    std::thread IO_thread(&IOCommunication::IOLoop, this);
    IO_thread.detach();
    timer::delay_ms(100.0);
    //INFO("io loop done");
    std::thread IMU_thread(&IOCommunication::IMULoop, this);
    IMU_thread.detach();
    timer::delay_ms(100.0);
    //INFO("imu loop done");
    std::thread StateManager_thread(&IOCommunication::StateManagerLoop, this);
    StateManager_thread.detach();
    timer::delay_ms(100.0);
    //INFO("sm loop done");

    m_sub_motion_hub = m_nh->subscribe("/ServoInfo", 1, &IOCommunication::SetJointValue, this);//TODO
   // m_sub_vision = m_nh->subscribe("/humanoid/ReloadMotionConfig", 1, &IOCommunication::SetJointValue, this);//TODO
    m_pub_motion_info = m_nh->advertise<dmsgs::MotionInfo>("MotionInfo", 1);
    // m_pub_motion_hub = m_nh->advertise<dmsgs::MotionDebugInfo>("MotionHub", 1);

    std::thread pub_thread(&IOCommunication::Publisher, this);
    pub_thread.detach();
    timer::delay_ms(100.0);
    std::thread sub_thread(&IOCommunication::Subscriber, this);
    sub_thread.detach();
    timer::delay_ms(100.0);
}

IOCommunication::~IOCommunication() = default;

void IOCommunication::IniIO()
{
    while(!io.m_servo_inited)
       io.spinOnce();
    PendulumWalk pen;
    pen.GiveAStep(0,0,0);
    m_joint_value = pen.GiveATick();
    io.setAllJointValue(m_joint_value);
    io.spinOnce();
    INFO("ini io done again");
    sleep(2);
}


void IOCommunication::StateManagerLoop()
{
    timer a;
    while(ros::ok()){
      a.tic();
      sm.imu_data = imu_data;
      sm.m_power_state = power_data;
      sm.servo_initialized = io.m_servo_inited;
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
    {
        IniIO();
    }
    else
    {
      io.setAllJointValue(m_joint_value);
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
   if(sm.imu_initialized == INITED)
   {
     m_joint_value = msg.data;
     PrintVector(m_joint_value);
   }
}

void IOCommunication::Publisher()
{
    ros::Rate loop_rata(100.0);
    while(ros::ok())
    {
      m_motion_info.imuRPY.x = sm.roll;
      m_motion_info.imuRPY.y = sm.pitch;
      m_motion_info.imuRPY.z = sm.yaw;
      m_pub_motion_info.publish(m_motion_info);
    //  m_pub_motion_hub.publish(m_motion_hub);
      loop_rata.sleep();
    }
}



void IOCommunication::Subscriber() //TODO
{
    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
    ros::waitForShutdown();
}

}
