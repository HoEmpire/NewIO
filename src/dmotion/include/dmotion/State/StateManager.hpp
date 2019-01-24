#pragma once
//#include "dmotion/Common/Common.h"

#include "dmotion/State/imu_filter_new.hpp"
#include <ros/ros.h>
#include "dmotion/Common/Type.h"
namespace Motion{

class StateManager
{
  public:
    StateManager();

    ~StateManager();

    void iniIMUFilter();

    void calIMUFilter();

    Motion::IMUData imu_data;
    float roll, pitch, yaw;
    bool imu_initialized;



  private:
    SupportPhase m_support;
    ImuFilter m_imu_filter;
    Motion::PowerState m_power_state;
    timer a;
    int ini_ticks = 1;
};

}
