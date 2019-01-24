#pragma once
//#include "dmotion/Common/Common.h"

#include "dmotion/State/imu_filter.hpp"
#include "dmotion/State/world_frame.hpp"
#include "dmotion/State/stateless_orientation.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <deque>
#include <ros/ros.h>
//#include "dcommon/dcommon.hpp"

#include "dmotion/Common/Type.h"
namespace Motion{

class RobotStatus
{
  public:
    explicit RobotStatus(ros::NodeHandle* nh);

    ~RobotStatus();

    /* get infromation about robot status */
  public:
    void setIMUData(const Motion::IMUData& imu_data);

    void setPressureData(const Motion::PressureData& pressure_data);

    void setDesiredPhase(const SUPPORT_STATUS support_);

    const bool getPhaseChangingState() const
    {
      return m_isPhaseChanging;
    }

    GyroData getGdata();

    void resetIMUFilter();

    void updateDeltaDist(float dx, float dy, float dt);

    void updateEularAngle();

    Motion::Vec3d getEularAngle(); // 弧度制

    stabilityStatus checkStableState();

    stabilityStatus getStableState()
    {
      return m_bStable;
    }

    deltadataDebug checkDeltaDist();

    deltadataDebug getDelta();

    void setPowerState(const Motion::PowerState power_state_)
    {
      m_power_state = power_state_;
    }

    void setPhaseChaningState(const bool is_changing)
    {
      m_isPhaseChanging = is_changing;
    }

  public:
    stabilityStatus m_bStable;

    // support phase && pressure info
    SUPPORT_STATUS m_support;
    SUPPORT_STATUS m_desired_support;
    int m_walking_tick;
    bool m_isPhaseChanging;

    // imu relevant
    angledataDebug m_angle_rpy;
    tf2::Matrix3x3 m_imu_offset;
    // geometry_msgs::Vector3 m_lin_acc, m_ang_vel;
    Motion::Vec3d m_euler;

    GyroData m_gypdata;
    offsetDebug m_offset;

    // robot odometry
    deltadataDebug m_deltaDist;

  private:
    void _initIMUFilter();

  private:
    ros::NodeHandle* m_nh;

    int robotnumber;
    ddouble_t gyro_x_avg;
    ddouble_t gyro_y_avg;
    ddouble_t gyro_z_avg;
    std::deque<ddouble_t> gyro_x_sum;
    std::deque<ddouble_t> gyro_y_sum;
    std::deque<ddouble_t> gyro_z_sum;
    int gyro_cnt;

    std::deque<Motion::Vec3d> m_pose_deque;

    bool imu_initialized;
    int imu_hardware_init_ticks;
    int imu_init_cnt;
    int imu_prepare_time;

    ddouble_t imu_roll_bias;
    ddouble_t imu_pitch_bias;
    ddouble_t imu_yaw_bias;

    WorldFrame::WorldFrame m_frame;
    ImuFilter m_imu_filter;

    Motion::PowerState m_power_state;
};

}
