#include "dmotion/State/StateManager.hpp"

#include <ros/ros.h>
#include "dmotion/Common/Parameters.h"
#include "dmotion/Common/Utility/Utility.h"
#define IMUFILTER_INITICKS 1000 //Unit:5ms

using namespace std;
namespace Motion{

StateManager::StateManager()
  : m_support(SUPPORT_BOTH)
  , imu_initialized(false)
  , m_power_state(Motion::OFF)
{
    //RobotPara::update(m_nh); // FUCK(MWX): MUST call before everyone
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
      a.tic();
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
      a.smartDelay_ms(5.0);
      a.tic();
      ini_ticks++;
    }
    else{
      m_imu_filter.iniGravity();
      m_imu_filter.iniQuaternion();
      INFO("***********************");
      INFO("StateManager::initIMUFilter:IMU Ini finished!!!");
      INFO("***********************");
      imu_initialized = true;
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

  if(DEBUG_OUTPUT)
  {
      std::cout << "x = " << roll << " ,y = " << pitch << " ,z = " << yaw << std::endl;
  }
  a.smartDelay_ms(5.0);
  a.tic();
}

}
