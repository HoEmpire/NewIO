#include "dmotion/IOCommunication/IOCommunication.h"
#define STATE_MANAGER_LOOP_TIME 10.0
#define IMU_LOOP_TIME 10.0
const float  MAX_PLAT_YAW  = 135;
const float MAX_PLAT_PITCH  = 50;
const float  MIN_PLAT_PITCH  = 0;

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
    m_sub_action_command = m_nh->subscribe("/ActionCommand", 1, &IOCommunication::ReadHeadServoValue, this);//TODO
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
    m_joint_value.push_back(0);
    m_joint_value.push_back(30);
    m_joint_value.push_back(0);
    m_joint_value.push_back(30);
    m_joint_value.push_back(0);
    m_joint_value.push_back(0);
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
      SetHeadServoValue();
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
     for(int i = 0; i < 16; i++)
         m_joint_value[i] = msg.data[i];
     m_status = msg.layout.dim[0].label;
     //PrintVector(m_joint_value);
   }
}

void IOCommunication::ReadHeadServoValue(const dmsgs::ActionCommand & msg)
{
    desire_pitch = msg.headCmd.pitch;
    desire_yaw = msg.headCmd.yaw;

    pitch_speed = msg.headCmd.pitchSpeed;
    yaw_speed = msg.headCmd.yawSpeed;
}

void IOCommunication::SetHeadServoValue()
{


    desire_yaw = min(desire_yaw, MAX_PLAT_YAW);
    desire_yaw = max(desire_yaw, -MAX_PLAT_YAW);

    desire_pitch = max(desire_pitch, MIN_PLAT_PITCH);
    desire_pitch = min(desire_pitch, MAX_PLAT_PITCH);

    //set head yaw
    if(fabs(desire_yaw - target_yaw) < yaw_speed)
    {
        target_yaw = desire_yaw;
    }
    else
    {
        if(desire_yaw > target_yaw)
        {
            target_yaw += yaw_speed;
        }
        else
        {
            target_yaw -= yaw_speed;
        }
    }

    //set head pitch
    if(fabs(desire_pitch - target_pitch) < pitch_speed || target_pitch < 0)
    {
        target_pitch = desire_pitch;
    }
    else
    {
        if(desire_pitch > target_pitch)
        {
            target_pitch += pitch_speed;
        }
        else
        {
            target_pitch -= pitch_speed;
        }
    }

    //head protecter
    if(sm.m_stable_state == BACKDOWN)
    {
        target_yaw = 0;
        target_pitch = 30;
    }
    else if(sm.m_stable_state == FRONTDOWN)
    {
        target_yaw = 0;
        target_pitch = -30;
    }
    m_joint_value[16] = target_pitch;
    m_joint_value[17] = target_yaw;
}

void IOCommunication::Publisher()
{
    ros::Rate loop_rata(100.0);
    int lower_board_success_flag = 0;
    while(ros::ok())
    {
      if(sm.imu_initialized == INITED)
      {
        if(lower_board_success_flag > 5)
        {
          //stable
          if(sm.m_stable_state == STABLE)
              m_motion_info.stable = true;
          else
              m_motion_info.stable = false;

          //timestamp
          m_motion_info.timestamp = ros::Time::now();

          //lower_board_connected
          m_motion_info.lower_board_connected = true;

          //curPlat
          m_motion_info.curPlat.pitch = target_pitch;
          m_motion_info.curPlat.yaw = target_yaw;

          //imuRPY
          m_motion_info.imuRPY.x = sm.roll;
          m_motion_info.imuRPY.y = sm.pitch;
          m_motion_info.imuRPY.z = sm.yaw;

          //status
          if(m_status == "STANDBY")
              m_motion_info.status = m_motion_info.STANDBY;
          else if(m_status == "WALK" || m_status == "GETUP")
              m_motion_info.status = m_motion_info.WALKING;
          else if(m_status == "KICK" )
              m_motion_info.status = m_motion_info.KICKING;
        }
        else
        {
          m_motion_info.lower_board_connected = false;
          lower_board_success_flag++;
        }
      }
      else
      {
        m_motion_info.lower_board_connected = false;
        lower_board_success_flag = 0;
      }
      m_pub_motion_info.publish(m_motion_info);
    //  m_pub_motion_hub.publish(m_motion_hub);
      loop_rata.sleep();
    }
}



void IOCommunication::Subscriber() //TODO
{
    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();
    ros::waitForShutdown();
}

}
