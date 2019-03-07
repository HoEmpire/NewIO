#pragma once
//#include "dmotion/Common/Common.h"

#include "IMUFilter.hpp"
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



    float roll, pitch, yaw;
    float ax_wog, ay_wog, az_wog;
    SupportState m_support_state;
    IniState imu_initialized;
    PressureData pressure_data;
    IniState pressure_initialized;
    PowerState m_power_state;
    bool servo_initialized;



  private:

    ImuFilter m_imu_filter;
    StableState m_stable_state;
    float PressureBiasLeft = 0.0;
    float PressureBiasRight = 0.0;
    timer a;
    int ini_ticks = 1;
};

}
