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

    void checkStableState();

    void checkIniState();

    void checkSupportState();

    void iniPressureSensor();

    void working();

    IMUData imu_data;
    PowerState m_power_state;
    PressureData pressure_data;
    float roll, pitch, yaw;
    float ax_wog, ay_wog, az_wog;
    IniState imu_initialized;
    IniState pressure_initialized;
    bool servo_initialized;
    SupportState m_support_state;


  private:

    ImuFilter m_imu_filter;
    StableState m_stable_state;
    float PressureBiasLeft = 0.0;
    float PressureBiasRight = 0.0;
    timer a;
    int ini_ticks = 1;
};

}
