#include "dmotion/State/StateManager.hpp"

#include <ros/ros.h>
#include "dmotion/Common/Parameters.h"
#include "dmotion/Common/Utility/Utility.h"
#define IMUFILTER_INITICKS 1000 //Unit:5ms
#define PRESSURE_INITICKS 200 //Unit:5ms
#define STABLE_COUNT 10
#define PRESSURE_THRESHOLD 1.0

using namespace std;
namespace Motion{

StateManager::StateManager()
  : m_support_state(SUPPORT_BOTH)
  , imu_initialized(WAIT)
  , pressure_initialized(WAIT)
  , m_power_state(OFF)
  , m_stable_state(STABLE)
{
    ROS_DEBUG("RobotStatus::RobotStatus: init RobotStatus instance");
}

StateManager::~StateManager() = default;

void StateManager::iniIMUFilter()
{
    float lin_acc_x;
    float lin_acc_y;
    float lin_acc_z;
    float ang_vel_x;
    float ang_vel_y;
    float ang_vel_z;
    if(ini_ticks == 1)
    {
      INFO("***********************");
      INFO("StateManager::initIMUFilter:IMU Ini begins!!!");
      INFO("***********************");
    }

    if(ini_ticks <= IMUFILTER_INITICKS)
    {
      lin_acc_x = imu_data.accl.x;
      lin_acc_y = imu_data.accl.y;
      lin_acc_z = imu_data.accl.z;
      ang_vel_x = imu_data.gypo.x;
      ang_vel_y = imu_data.gypo.y;
      ang_vel_z = imu_data.gypo.z;
      if(ini_ticks == 1)
      {
        m_imu_filter.iniAcclast(lin_acc_x , lin_acc_y, lin_acc_z);
      }
      else
      {
        m_imu_filter.iniIMU(ang_vel_x, ang_vel_y, ang_vel_z,
                    lin_acc_x, lin_acc_y, lin_acc_z,
                    ini_ticks);
      }
      ini_ticks++;
    }
    else{
      m_imu_filter.iniGravity();
      m_imu_filter.iniQuaternion();
      INFO("***********************");
      INFO("StateManager::initIMUFilter:IMU Ini finished!!!");
      INFO("***********************");
      imu_initialized = INITED;
    }
    // float x1,y1,z1;
    // QuaternionToEulerAngles(q0,q1,q2,q3,x1,y1,z1);
    // cout << "x1: "<< x1 << " ,y1: " << y1 << ",z1: " << z1 << endl;
}

void StateManager::calIMUFilter()
{
  float lin_acc_x;
  float lin_acc_y;
  float lin_acc_z;
  float ang_vel_x;
  float ang_vel_y;
  float ang_vel_z;

  lin_acc_x = imu_data.accl.x;
  lin_acc_y = imu_data.accl.y;
  lin_acc_z = imu_data.accl.z;
  ang_vel_x = imu_data.gypo.x;
  ang_vel_y = imu_data.gypo.y;
  ang_vel_z = imu_data.gypo.z;

  m_imu_filter.Fusing(ang_vel_x, ang_vel_y, ang_vel_z,
              lin_acc_x, lin_acc_y, lin_acc_z);
  m_imu_filter.getRPY(roll,pitch,yaw);
  m_imu_filter.calAccWog();
  ax_wog = m_imu_filter.ax_wog;
  ay_wog = m_imu_filter.ay_wog;
  az_wog = m_imu_filter.az_wog;


  if(0)
  {
      std::cout << "x = " << roll << " ,y = " << pitch << " ,z = " << yaw << std::endl;
  }
}

void StateManager::checkStableState()
{

    float deg_roll = RAD2DEG(roll);
    float deg_pitch = RAD2DEG(pitch);
    static int frontcount = 0, backcount = 0, leftcount = 0, rightcount = 0, stablecount = 0;

    if (m_power_state == OFF || !imu_initialized || m_power_state == REOPEN)
    {
        frontcount = 0;
        backcount = 0;
        leftcount = 0;
        rightcount = 0;
        stablecount = 0;
        ROS_WARN("check stable state failed");
    }

    if (abs(deg_pitch) > 25|| abs(deg_roll) > 25) {

        stablecount = 0;

        // TODO hard code ....
        if (deg_pitch > 60.0 && deg_pitch < 120.0)
        {
            // std::cout << "fuck" << std::endl;
            frontcount++;
            m_stable_state = UNSTABLE;
        }
        else
        {
            frontcount = 0;
        }

        if (deg_pitch < -60.0 && deg_pitch > -120.0) {
            backcount++;
            m_stable_state = UNSTABLE;
        }
        else
        {
            backcount = 0;
        }

        if (deg_roll > 60 && deg_roll < 120)
        {
            rightcount++;
            m_stable_state = UNSTABLE;
        }
        else
        {
            rightcount = 0;
        }

        if (deg_roll < -60 && deg_roll > -120 )
        {
            leftcount++;
            m_stable_state = UNSTABLE;
        }
        else
        {
            leftcount = 0;
        }
    }
    else
    {
        stablecount++;
        frontcount = backcount = leftcount = rightcount = 0;
    }

    if (frontcount > STABLE_COUNT)
    {
        m_stable_state = FRONTDOWN;
        //ROS_INFO("front down...");
    }
    else if (backcount > STABLE_COUNT)
    {
        m_stable_state = BACKDOWN;
        //ROS_INFO("back down...");
    }
    else if (leftcount > STABLE_COUNT)
    {
        m_stable_state = LEFTDOWN;
        //ROS_INFO("left down...");
    }
    else if (rightcount > STABLE_COUNT)
    {
        m_stable_state = RIGHTDOWN;
        //ROS_INFO("right down...");
    }
    else if (stablecount > STABLE_COUNT)
    {
        m_stable_state = STABLE;
        //ROS_INFO("stable...");
    }

}

void StateManager::checkIniState()
{
  if(servo_initialized)
  {
    if(pressure_initialized == INITED && imu_initialized == WAIT)
    {
        if(parameters.global.using_pressure)
        {
          if(m_support_state == SUPPORT_BOTH)
          {
              imu_initialized = INITING;
              m_imu_filter.clearData();
              ini_ticks = 1;
          }
        }
       else
       {
         imu_initialized = INITING;
         m_imu_filter.clearData();
         ini_ticks = 1;
       }
    }
    else if(pressure_initialized == WAIT)
    {
          pressure_initialized = INITING;
          ini_ticks = 1;
    }
  }
  else
  {
      pressure_initialized = WAIT;
      imu_initialized = WAIT;
  }
}

void StateManager::iniPressureSensor()
{
  float left_sum_ = 0, right_sum_ = 0;

  // sum pressure data of each feet
  for(int i = 0; i < 4; i++)
  {
      left_sum_ += pressure_data.left[i];
  }
  for(int i = 0; i < 4; i++)
  {
      right_sum_ += pressure_data.right[i];
  }


  if(ini_ticks == 1)
  {
    INFO("***********************");
    INFO("StateManager::iniPressureSensor:Pressure sensors Ini begins!!!");
    INFO("***********************");
  }

  if(ini_ticks <= PRESSURE_INITICKS)
  {
    PressureBiasLeft = 1.0 * (ini_ticks - 1) / ini_ticks * PressureBiasLeft + 1.0 / ini_ticks * left_sum_;
    PressureBiasRight = 1.0 * (ini_ticks - 1) / ini_ticks * PressureBiasRight + 1.0 / ini_ticks * right_sum_;
    ini_ticks++;
  }
  else{
    INFO("***********************");
    INFO("StateManager::initPressureSensor:Pressure sensors ini finished!!!");
    INFO("StateManager::initPressureSensor:Now you have 3 seconds to put the robot on the ground");
    sleep(3);
    INFO("***********************");
    pressure_initialized = INITED;
  }
}

void StateManager::checkSupportState()
{
  float left_sum_ = 0, right_sum_ = 0;
  float support_sum_ = 0;
  static float sum_, average_;

  // sum pressure data of each feet
  for(int i = 0; i < 4; i++)
  {
      left_sum_ += pressure_data.left[i];
      //std::cout << "left_sum：" << left_sum_ << std::endl;
  }
  for(int i = 0; i < 4; i++)
  {
      right_sum_ += pressure_data.right[i];
      //std::cout << "right_sum：" << right_sum_ << std::endl;
  }
//  std::cout << "left：" << left_sum_ - PressureBiasLeft << std::endl;
//  std::cout << "right：" << right_sum_ - PressureBiasRight << std::endl;

  if((left_sum_ - PressureBiasLeft > PRESSURE_THRESHOLD) && (right_sum_ - PressureBiasRight > PRESSURE_THRESHOLD))
  {
      m_support_state = SUPPORT_BOTH;
    //  ROS_INFO("support both");
  }
  else if(left_sum_ - PressureBiasLeft > PRESSURE_THRESHOLD)
  {
      m_support_state = SUPPORT_LEFT;
    //  ROS_INFO("support left");
  }
  else if(right_sum_ - PressureBiasRight > PRESSURE_THRESHOLD)
  {
      m_support_state = SUPPORT_RIGHT;
    //  ROS_INFO("support right");
  }
  else
  {
      ROS_INFO("unkown support");
  }
}

void StateManager::working()
{
  checkIniState();
  if(pressure_initialized == INITED)
  {
    checkSupportState();
  }
  else if(pressure_initialized == INITING)
  {
    iniPressureSensor();
  }

  if(imu_initialized == INITED)
  {
    calIMUFilter();
    checkStableState();
  }
  else if(imu_initialized == INITING)
  {
    iniIMUFilter();
  }
}

}
