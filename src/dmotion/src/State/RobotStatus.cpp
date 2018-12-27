#include "dmotion/State/RobotStatus.hpp"

#include <fstream>
#include <ros/ros.h>

#include "dmotion/Common/Parameters.h"
#include "dmotion/Common/Utility/Utility.h"

using namespace std;
RobotStatus::RobotStatus(ros::NodeHandle* nh)
  : m_support(DOUBLE_BASED)
  , m_desired_support(RIGHT_BASED)
  , imu_initialized(false)
  , imu_hardware_init_ticks(0)
  , m_nh(nh)
  , m_power_state(Motion::OFF)
{
    RobotPara::update(m_nh); // FUCK(MWX): MUST call before everyone

    _initIMUFilter();

    ROS_DEBUG("RobotStatus::RobotStatus: init RobotStatus instance");
}

RobotStatus::~RobotStatus() = default;

void
RobotStatus::_initIMUFilter()
{
    m_frame = WorldFrame::NWU;
    m_imu_filter.setWorldFrame(m_frame);
    m_imu_filter.setDriftBiasGain(parameters.state.imu_bias_gain);
    m_imu_filter.setAlgorithmGain(parameters.state.imu_algorithm_gain);

    m_imu_offset.setEulerYPR(0,
                             DEG2RAD(-parameters.state.imu_pitch_bias),
                             DEG2RAD(-parameters.state.imu_roll_bias));
    gyro_x_avg = 0;
    gyro_y_avg = 0;
    gyro_z_avg = 0;
    imu_roll_bias = 0;
    imu_pitch_bias = 0;
    imu_yaw_bias = 0;
    imu_init_cnt = 0;
    imu_hardware_init_ticks = 0;

    m_euler.x = 0;
    m_euler.y = 0;
    m_euler.z = M_PI/2;
}

void
RobotStatus::resetIMUFilter()
{
    ROS_WARN("RobotStatus::resetIMUFilter: IMU filter reseted, must be power off");
    imu_initialized = false;
    imu_init_cnt = 0;
    imu_hardware_init_ticks = 0;

    m_euler.x = 0;
    m_euler.y = 0;
    m_euler.z = M_PI/2;
}

void
RobotStatus::setIMUData(const Motion::IMUData& imu_data)
{
    static float q0_, q1_, q2_, q3_;
    static geometry_msgs::Vector3 lin_acc, ang_vel;
    static int prepare_ticks;

    lin_acc.x = -imu_data.accl.x;
    lin_acc.y = -imu_data.accl.y;
    lin_acc.z = -imu_data.accl.z;
    ang_vel.x = imu_data.gypo.x;
    ang_vel.y = imu_data.gypo.y;
    ang_vel.z = imu_data.gypo.z+parameters.state.imu_yaw_vel_correction;

    ROS_DEBUG_STREAM(std::endl << "RobotStatus::setIMUData" << std::endl
                    << " IMU raw data accl: "
                    << lin_acc.x << ' ' << lin_acc.y << ' ' << lin_acc.z << std::endl
                    << " IMU raw data gyro: "
                    << ang_vel.x << ' ' << ang_vel.y << ' ' << ang_vel.z);

    // std::cout <<std::setw(15) << lin_acc.x << std::endl;
    // std::cout <<std::setw(15) << lin_acc.y << std::endl;
    // std::cout <<std::setw(15) << lin_acc.z << std::endl;

    if (!imu_initialized)
    {
        if( imu_hardware_init_ticks++ < 10)
        {
            return;
        }
        geometry_msgs::Quaternion init_q;
        StatelessOrientation::computeOrientation(m_frame, lin_acc, init_q);
        m_imu_filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

        std::cout << init_q.x << ' ' << init_q.y << ' ' << init_q.z << ' ' << init_q.w << std::endl;
        tf2::Quaternion q(init_q.x, init_q.y, init_q.z, init_q.w);

        double x, y, z;
        tf2::Matrix3x3(q).getRPY(x, y, z);

        imu_initialized = true;
        imu_init_cnt = 0;

        ROS_INFO_STREAM("RobotStatus::setIMUData: IMU filter first frame" << std::endl
                        << " IMU raw data accl: "
                        << lin_acc.x << ' ' << lin_acc.y << ' ' << lin_acc.z << std::endl
                        << " IMU raw data gyro: "
                        << ang_vel.x << ' ' << ang_vel.y << ' ' << ang_vel.z << std::endl
                        << "Quaternion: "
                        << init_q.x << ' ' << init_q.y << ' ' << init_q.z << ' ' << init_q.w << std::endl
                        << "Euler: "
                        << x << ' ' << y << ' ' << z << std::endl
                        );
    }
    else
    {
        // TODO hardcode for motion sample time
        m_imu_filter.madgwickAHRSupdateIMU(ang_vel.x, ang_vel.y, ang_vel.z,
                                           lin_acc.x, lin_acc.y, lin_acc.z,
                                           0.01);
        m_imu_filter.getOrientation(q0_, q1_, q2_, q3_);

        (tf2::Matrix3x3(tf2::Quaternion(q1_, q2_, q3_, q0_))
        *m_imu_offset)
        .getRPY(m_euler.x, m_euler.y, m_euler.z, 1);

        ROS_DEBUG_STREAM("RobotStatus::setIMUData: current frame pose "
                        << m_euler.x << ' ' << m_euler.y << ' ' << m_euler.z);

        // if (parameters.global.debug)
        // {
        //     m_euler_debug.x = m_euler.x;
        //     m_euler_debug.y = m_euler.y;
        //     m_euler_debug.z = m_euler.z;
        // }

    }
    imu_init_cnt ++;

    if (RobotPara::gyroscope_debug && imu_init_cnt % 10 == 0) {
        std::cout << "imu_init_cnt    :" << imu_init_cnt << std::endl;
        std::cout << "gdata.GYPO[0] rX:" << ang_vel.x << std::endl;
        std::cout << "gdata.GYPO[1] pY:" << ang_vel.y << std::endl;
        std::cout << "gdata.GYPO[2] yZ:" << ang_vel.z << std::endl;
        std::cout << "gdata.ACCL[0] aX:" << lin_acc.x << std::endl;
        std::cout << "gdata.ACCL[1] aY:" << lin_acc.y << std::endl;
        std::cout << "gdata.ACCL[2] aZ:" << lin_acc.z << std::endl;
        std::cout << "angle          X:" << m_euler.x / M_PI * 180 << std::endl;
        std::cout << "angle          Y:" << m_euler.y / M_PI * 180 << std::endl;
        std::cout << "angle          Z:" << m_euler.z / M_PI * 180 << std::endl;
    }

    prepare_ticks = static_cast<int>(parameters.state.imu_prepare_time*100);
    if (imu_init_cnt < prepare_ticks )
    {
        m_angle_rpy.angleX = m_euler.x;
        m_angle_rpy.angleY = m_euler.y;
        m_angle_rpy.angleZ = parameters.global.yaw_offset;
        // imu_init_cnt++;
    }
    else if (imu_init_cnt == prepare_ticks)
    {
        ROS_WARN("RobotStatus::setIMUData: Imu init done...");
        m_angle_rpy.angleX = m_euler.x;
        m_angle_rpy.angleY = m_euler.y;
        m_angle_rpy.angleZ = parameters.global.yaw_offset;
        // imu_roll_bias = m_euler.angleX;
        // imu_pitch_bias = m_euler.angleY;
        imu_yaw_bias = m_euler.z;

        ROS_INFO_STREAM("RobotStatus::setIMUData: robot pose yaw bias will be set to "
                        << imu_yaw_bias << " rad" << RAD2DEG(imu_yaw_bias) << " deg");
    }
    else
    {
        m_angle_rpy.angleX = m_euler.x;
        m_angle_rpy.angleY = m_euler.y;
        m_angle_rpy.angleZ = m_euler.z - imu_yaw_bias + parameters.global.yaw_offset;
    }

    updateEularAngle();

    ROS_DEBUG_STREAM("RobotStatus::setIMUData: current frame pose "
                    << RAD2DEG(m_euler.x) << ' ' << RAD2DEG(m_euler.y) << ' ' << RAD2DEG(m_euler.z));
}

void RobotStatus::setDesiredPhase(const SUPPORT_STATUS support_)
{
    m_desired_support = support_;
    m_walking_tick = 0;
}

void
RobotStatus::setPressureData(const Motion::PressureData& pressure_data)
{
    float left_sum_ = 0, right_sum_ = 0;
    float support_sum_ = 0;
    static float sum_, average_;

    // sum pressure data of each feet
    for(int i = 0; i < 4; i++)
    {
        left_sum_ += pressure_data.left[i];
    }
    for(int i = 0; i < 4; i++)
    {
        right_sum_ += pressure_data.right[i];
    }

    // choose sum according to support phase
    if (LEFT_BASED == m_desired_support)
    {
        support_sum_ = right_sum_;
    }
    else if (m_desired_support == RIGHT_BASED)
    {
        support_sum_ = left_sum_;
    }
    else
    {
        ROS_FATAL("error supporting error");
    }

    // calculate average value && determine support phase changing
    m_walking_tick++;
    if (m_walking_tick == parameters.state.pressure_counting_start)
    {
        sum_ = 0;
        // ROS_INFO_STREAM( "tick " << m_walking_tick << " start counting pressure data");
    }
    else if( m_walking_tick > parameters.state.pressure_counting_start
        && m_walking_tick <= parameters.state.pressure_counting_end)
    {
        sum_ += support_sum_;
        if (m_walking_tick == parameters.state.pressure_counting_end)
        {
            average_ = sum_/(parameters.state.pressure_counting_end- parameters.state.pressure_counting_start);
            m_isPhaseChanging = false;
            // ROS_INFO_STREAM("tick " << m_walking_tick << "calculate average: " << average_);
        }
    }
    else if(m_walking_tick > parameters.state.pressure_counting_end)
    {
        if (support_sum_ - average_ > parameters.state.security_threshold)
        {
            m_isPhaseChanging = true;
            // ROS_INFO_STREAM("tick " << m_walking_tick << "calculate average: " << average_);
        }
    }
}


void
RobotStatus::updateEularAngle()
{
    m_pose_deque.push_front(m_euler);
    if (m_pose_deque.size() > 100) // 1s
    {
        m_pose_deque.pop_back();
    }
    return;
}

Motion::Vec3d
RobotStatus::getEularAngle()
{
    Motion::Vec3d retv;
    if (m_pose_deque.size() > 0)
    {
        retv = m_pose_deque.front();
    }

    return retv;
}

stabilityStatus
RobotStatus::checkStableState()
{
    Motion::Vec3d eular = m_pose_deque.front();

    eular.x = eular.x * 180.0 / M_PI;
    eular.y = eular.y * 180.0 / M_PI;
    static int frontcount = 0, backcount = 0, leftcount = 0, rightcount = 0, stablecount = 0;

    if (Motion::OFF == m_power_state && imu_init_cnt < static_cast<int>(parameters.state.imu_prepare_time*100))
    {
        frontcount = 0;
        backcount = 0;
        leftcount = 0;
        rightcount = 0;
        stablecount = 0;
        return stable;
    }

    if (eular.y < -5 || eular.y > 25 || abs(eular.x) > 28) {

        stablecount = 0;

        // TODO hard code ....
        if (eular.y > parameters.state.front_down_threshold)
        {
            // std::cout << "fuck" << std::endl;
            frontcount++;
            m_bStable = unstable;
        }
        else
        {
            frontcount = 0;
        }

        if (eular.y < parameters.state.back_down_threshold) {
            backcount++;
            m_bStable = unstable;
        }
        else
        {
            backcount = 0;
        }

        if (eular.x > parameters.state.side_down_threshold)
        {
            rightcount++;
            m_bStable = unstable;
        }
        else
        {
            rightcount = 0;
        }

        if (eular.x < -parameters.state.side_down_threshold)
        {
            leftcount++;
            m_bStable = unstable;
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

    if (frontcount > parameters.state.front_assertion_ticks)
    {
        m_bStable = frontdown;
        // ROS_WARN("front down...");
    }
    else if (backcount > parameters.state.back_assertion_ticks)
    {
        m_bStable = backdown;
        // ROS_WARN("back down...");
    }
    else if (leftcount > parameters.state.side_assertion_ticks)
    {
        m_bStable = leftdown;
        // ROS_WARN("left down...");
    }
    else if (rightcount > parameters.state.side_assertion_ticks)
    {
        m_bStable = rightdown;
        // ROS_WARN("right down...");
    }
    else if (stablecount > parameters.state.stable_assertion_ticks)
    {
        m_bStable = stable;
        // ROS_WARN("stable...");
    }

    // FIXME(hhy)
    // m_bStable = stable;
    return m_bStable;
}

void
RobotStatus::updateDeltaDist(float dx, float dy, float dt)
{
    auto t = m_deltaDist.m_angle;
    auto sin_ = sin(t / 180.0 * M_PI);
    auto cos_ = cos(t / 180.0 * M_PI);

    auto xp = dx * cos_ - dy * sin_;
    auto yp = dx * sin_ + dy * cos_;

    m_deltaDist.m_x += xp;
    m_deltaDist.m_y += yp;
    m_deltaDist.m_angle = AngleNormalization(m_deltaDist.m_angle + dt);
}

// clean delta data & get last delta data
deltadataDebug
RobotStatus::checkDeltaDist()
{
    deltadataDebug deltaDist = m_deltaDist;

    m_deltaDist.m_angle = 0;
    m_deltaDist.m_x = 0;
    m_deltaDist.m_y = 0;

    return deltaDist;
}

deltadataDebug
RobotStatus::getDelta()
{
    return m_deltaDist;
}
