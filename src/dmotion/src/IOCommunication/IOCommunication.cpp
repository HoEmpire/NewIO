#include "dmotion/IOCommunication/IOCommunication.h"
#include "dmotion/Common/DThread.hpp"
#define STATE_MANAGER_LOOP_TIME 10.0
#define IMU_LOOP_TIME 10.0
#define ROS_TOPIC_FREQ 100.0
const float  MAX_PLAT_YAW  = 135;
const float MAX_PLAT_PITCH  = 50;
const float  MIN_PLAT_PITCH  = 0;

namespace Motion
{
static void* pthreadIOLoop(void* arg);

IOCommunication::IOCommunication(ros::NodeHandle* nh)
  : m_nh(nh),
    target_pitch(0.0),
    target_yaw(0.0),
    desire_pitch(0.0),
    desire_yaw(0.0),
    lower_board_success_flag(0)
{
//    parameters.init(m_nh);
    //INFO("wait for ini io to be done");
    IniIO();

    int robotId;
    if(!m_nh->getParam("RobotId", robotId))
        throw std::runtime_error("Can't get robot id");

    int rs = pthread_attr_init(&attr);
    assert(rs==0);

    printf("set thread: SCHED_RR policy\n");
    set_thread_policy(&attr,SCHED_RR);
    int priority = 99;
    printf("set thread: %d policy\n", priority);
    set_thread_priority(&attr, priority);
    printf("show priority of current thread: ");
    priority = get_thread_priority(&attr);
    //pthread_setschedprio(tidIO, 99);
    if (pthread_create(&tidIO, &attr, &pthreadIOLoop,  (void *)this)!=0)
    {
       printf("create error!\n");
    }

    m_sub_motion_hub = m_nh->subscribe("/ServoInfo", 1, &IOCommunication::SetJointValue, this);//TODO
    m_sub_action_command = m_nh->subscribe("/dbehavior_" + std::to_string(robotId) + "/ActionCommand", 1, &IOCommunication::ReadHeadServoValue, this);//TODO
    m_sub_vision = m_nh->subscribe("/division_" + std::to_string(robotId) + "/VisionInfo", 1, &IOCommunication::ReadVisionYaw, this);//TODO
    m_pub_motion_info = m_nh->advertise<dmsgs::MotionInfo>("/dmotion_" + std::to_string(robotId) + "/MotionInfo", 1);
    m_motion_server = m_nh->advertiseService("/dmotion_" + std::to_string(robotId) + "/set_motion_yaw", &IOCommunication::setFieldYaw, this); //这个服务暂时没什么用，纯属适应老体系,具体实现应该在IO实现
    // m_pub_motion_hub = m_nh->advertise<dmsgs::MotionDebugInfo>("MotionHub", 1);

}

IOCommunication::~IOCommunication()
{
  pthread_join(tidIO,NULL);
  pthread_attr_destroy(&attr);
}

void IOCommunication::IniIO()
{
    while(!io.m_servo_inited && ros::ok())
       io.spinOnce();
    PendulumWalk pen;
    pen.GiveAStep(0,0,0);
    m_joint_value = pen.GiveATick();
    m_joint_value.push_back(0.0);
    m_joint_value.push_back(30.0);
    m_joint_value.push_back(0.0);
    m_joint_value.push_back(30.0);
    m_joint_value.push_back(0.0);
    m_joint_value.push_back(0.0);
    io.setAllJointValue(m_joint_value);
    io.spinOnce();
    INFO("ini io done again");
    sleep(2);
}


// void IOCommunication::StateManagerLoop()
// {
//     timer a;
//     while(ros::ok()){
//       a.tic();
//       sm.imu_data = imu_data;
//       sm.m_power_state = power_data;
//       sm.servo_initialized = io.m_servo_inited;
//       sm.pressure_data = pressure_data;
//       sm.servo_pos = read_pos;
//       sm.servo_vel = read_vel;
//       sm.working();
//       a.SmartDelayMs(STATE_MANAGER_LOOP_TIME);//TODO
//     }
// }

static void* pthreadIOLoop(void* arg){
  return static_cast<IOCommunication*>(arg)->IOLoop();
}
void* IOCommunication::IOLoop()
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
      //if(sm.m_stable_state == STABLE || m_status != "WALK")//TODO 去除跌倒不動

      io.setAllJointValue(m_joint_value);
      io.spinOnce();
      //timer a;
      //a.tic();
      //io.readPosVel();
      //INFO("read servo");
      //a.toc();
      // power_data = io.getPowerState();
      // pressure_data = io.getPressureData();
      // read_pos = io.readAllPosition();
      // read_vel = io.readAllVel();
      // imu_data = io.getIMUData();

      sm.imu_data = io.getIMUData();
      sm.m_power_state = io.getPowerState();
      sm.servo_initialized = io.m_servo_inited;
      sm.pressure_data = io.getPressureData();
      sm.servo_pos = io.readAllPosition();
      sm.servo_vel = io.readAllVel();
      sm.working();
    }
    Publisher();
  }
    return NULL;
}


// void IOCommunication::IMULoop()
// {
//   timer b;
//   while(ros::ok())
//   {
//      b.tic();
//      if(io.m_servo_inited)
//         imu_data = io.getIMUData();
//      b.SmartDelayMs(IMU_LOOP_TIME);//TODO
//   }
// }

void IOCommunication::SetJointValue(const std_msgs::Float64MultiArray & msg)
{
   if(sm.imu_initialized == INITED)
   {
     for (int i = 0; i < 16; i++)
     {
         if(isnan(msg.data[i]))
         {
           ROS_WARN("NaN from MotionHub!!");
           return;
         }
     }
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
    if(sm.pitch * 180 / M_PI <= -50)//back down
    {
        target_yaw = 0;
        target_pitch = 30;
    }
    else if(sm.pitch * 180 / M_PI >= 50)//front down
    {
        target_yaw = 0;
        target_pitch = -30;
    }
    m_joint_value[16] = target_pitch;
    m_joint_value[17] = target_yaw;
}

void IOCommunication::Publisher()
{
    if(sm.imu_initialized == INITED)
    {
      if(lower_board_success_flag > 5)
      {
        //stable & forward_or_backward
        if(sm.m_stable_state == STABLE)
            m_motion_info.stable = true;
        else
        {
          m_motion_info.stable = false;
          if(sm.m_stable_state == FRONTDOWN)
             m_motion_info.forward_or_backward = true;
          else
             m_motion_info.forward_or_backward = false;
        }

        //timestamp
        m_motion_info.timestamp = ros::Time::now();

        //lower_board_connected
        m_motion_info.lower_board_connected = true;

        //curPlat
        m_motion_info.curPlat.pitch = target_pitch;
        m_motion_info.curPlat.yaw = target_yaw;

        //odometry
        m_motion_info.odometry.x = sm.x_now;
        m_motion_info.odometry.y = sm.y_now;
        m_motion_info.odometry.z = sm.yaw * 180 / M_PI;

        //imuRPY
        m_motion_info.imuRPY.x = sm.roll * 180 / M_PI;
        m_motion_info.imuRPY.y = sm.pitch * 180 / M_PI;
        m_motion_info.imuRPY.z = sm.yaw * 180 / M_PI;

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
    ros::spinOnce();
}

void IOCommunication::ReadVisionYaw(const dmsgs::VisionInfo & msg)
{
    sm.vision_yaw = msg.robot_pos.z / 180.0 * M_PI;
}

bool IOCommunication::setFieldYaw(dmsgs::SetInitOrientation::Request &req,
                 dmsgs::SetInitOrientation::Response &res)
{
    return true;
}

}
