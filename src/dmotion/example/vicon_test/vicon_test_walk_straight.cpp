#include "dmotion/Common/Utility/Utility.h"
#include <Eigen/Geometry>
#include "dmotion/Common/Parameters.h"
#include "dmotion/IO/IMUReader.h"
#include "dmotion/IO/IOManager3.h"
#include "dmotion/State/StateManager.hpp"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"
#include "PendulumWalk.h"

#include <thread>
#include <fstream>
#include <iostream>

#define workplace "/home/ubuntu/test/NewIO/test_data/test1"

#define TEST_MODE 1 // 1为慢速测试，2为中速测试，3为快速测试，4为正弦，5为圆圈

#define GPARAM(x, y)                                                                                                                                                                                   \
    do {                                                                                                                                                                                               \
        if (!m_nh->getParam(x, y)) {                                                                                                                                                                   \
            MOTION_WARN("Motion get pararm " #x " error!");                                                                                                                                          \
        }                                                                                                                                                                                              \
    } while (0)

using namespace std;
using namespace Eigen;

Motion::IMUData imu_data;
Motion::PowerState power_data;
Motion::PressureData pressure_data;
bool servo_state;
std::vector<double> read_pos(12,0);
std::vector<double> read_vel(12,0);

using namespace dmotion;
using namespace Motion;
int flag = 0;

static inline void cross(
  Eigen::Matrix<double,3,1> x,
  Eigen::Matrix<double,3,1> y,
  Eigen::Matrix<double,3,1> z)
{
  z(0) = x(1) * y(2) - x(2) * y(1);
  z(1) = x(2) * y(0) - x(0) * y(2);
  z(2) = x(0) * y(1) - x(1) * y(0);
}

template <typename T>
void AddElements(std::vector<T> &master, std::vector<T> slave)
{
    for (unsigned int i = 0; i < slave.size(); i++)
    {
        master.emplace_back(slave[i]);
    }
}


void tt()
{
    Motion::IOManager3 io;
    io.setAllspeed(30);
    std::vector<double> fucking;
    PendulumWalk pen;
    pen.GiveAStep(0,0,0);
    fucking = pen.GiveATick();
    io.setAllJointValue(fucking);
    io.spinOnce();
    sleep(2);

    int ticks = 0;
    int step = 0;
    int mode = TEST_MODE;

    GPARAM("test_mode", mode);

    while(ros::ok()){
      if(flag >= 500)
      {
      switch(mode)
      {
        case 1:
                if(step < 40)
                {
                  if(ticks == 0)
                    pen.GiveAStep(3,0,0);
                  if(ticks < 35)
                  {
                    fucking = pen.GiveATick();
                    fucking.push_back(0);
                    fucking.push_back(30);
                    fucking.push_back(0);
                    fucking.push_back(30);
                    fucking.push_back(0);
                    fucking.push_back(0);
                    io.setAllJointValue(fucking);
                    ticks++;
                  }
                  if(ticks >= 35)
                  {
                    ticks = 0;
                    step++;
                  }
                }
                else
                {
                  if(ticks < 35)
                  {
                    ticks++;
                  }
                  if(ticks >= 35)
                  {
                    ticks = 0;
                    step++;
                  }
                }
               break;
        case 2:
                if(step < 40)
                {
                  if(ticks == 0)
                    pen.GiveAStep(5,0,0);
                  if(ticks < 35)
                  {
                    fucking = pen.GiveATick();
                    fucking.push_back(0);
                    fucking.push_back(30);
                    fucking.push_back(0);
                    fucking.push_back(30);
                    fucking.push_back(0);
                    fucking.push_back(0);
                    io.setAllJointValue(fucking);
                    ticks++;
                  }
                  if(ticks >= 35)
                  {
                    ticks = 0;
                    step++;
                  }
                }
                else
                {
                  if(ticks < 35)
                  {
                    ticks++;
                  }
                  if(ticks >= 35)
                  {
                    ticks = 0;
                    step++;
                  }
                }
               break;
        case 3:
                if(step < 40)
                {
                  if(ticks == 0)
                    pen.GiveAStep(7,0,0);
                  if(ticks < 35)
                  {
                    fucking = pen.GiveATick();
                    fucking.push_back(0);
                    fucking.push_back(30);
                    fucking.push_back(0);
                    fucking.push_back(30);
                    fucking.push_back(0);
                    fucking.push_back(0);
                    io.setAllJointValue(fucking);
                    ticks++;
                  }
                  if(ticks >= 35)
                  {
                    ticks = 0;
                    step++;
                  }
                }
                else
                {
                  if(ticks < 35)
                  {
                    ticks++;
                  }
                  if(ticks >= 35)
                  {
                    ticks = 0;
                    step++;
                  }
                }
               break;
        case 4:
        case 5:
      }

      }


      io.spinOnce();
      io.readPosVel();
      imu_data = io.getIMUData();
      power_data = io.getPowerState();
      pressure_data = io.getPressureData();
      servo_state = io.m_servo_inited;
      read_pos = io.readAllPosition();
      read_vel = io.readAllVel();
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");
    parameters.init(&nh);

    if(chdir(workplace))
        exit(0);  //设置工作路径，也就是B显示的路径

    //IMU data
    ofstream ax("ax.txt", ios::out|ios::trunc);
    ofstream ay("ay.txt", ios::out|ios::trunc);
    ofstream az("az.txt", ios::out|ios::trunc);
    ofstream wx("wx.txt", ios::out|ios::trunc);
    ofstream wy("wy.txt", ios::out|ios::trunc);
    ofstream wz("wz.txt", ios::out|ios::trunc);

    //right data
    ofstream right_x("right_x.txt", ios::out|ios::trunc);
    ofstream right_y("right_y.txt", ios::out|ios::trunc);
    ofstream right_z("right_z.txt", ios::out|ios::trunc);
    ofstream right_roll("right_roll.txt", ios::out|ios::trunc);
    ofstream right_pitch("right_pitch.txt", ios::out|ios::trunc);
    ofstream right_yaw("right_yaw.txt", ios::out|ios::trunc);
    ofstream right_vx("right_vx.txt", ios::out|ios::trunc);
    ofstream right_vy("right_vy.txt", ios::out|ios::trunc);
    ofstream right_vy("right_vz.txt", ios::out|ios::trunc);

    //left data
    ofstream left_x("left_x.txt", ios::out|ios::trunc);
    ofstream left_y("left_y.txt", ios::out|ios::trunc);
    ofstream left_z("left_z.txt", ios::out|ios::trunc);
    ofstream left_roll("left_roll.txt", ios::out|ios::trunc);
    ofstream left_pitch("left_pitch.txt", ios::out|ios::trunc);
    ofstream left_yaw("left_yaw.txt", ios::out|ios::trunc);
    ofstream left_vx("left_vx.txt", ios::out|ios::trunc);
    ofstream left_vy("left_vy.txt", ios::out|ios::trunc);
    ofstream left_vz("left_vz.txt", ios::out|ios::trunc);

    //suupport state
    ofstream support("support.txt", ios::out|ios::trunc);

    //feet attitude
    ofstream feet_roll("feet_roll.txt", ios::out|ios::trunc);
    ofstream feet_pitch("feet_pitch.txt", ios::out|ios::trunc);
    ofstream feet_yaw("feet_yaw.txt", ios::out|ios::trunc);

    //feet pos
    ofstream x("x.txt", ios::out|ios::trunc);
    ofstream y("y.txt", ios::out|ios::trunc);
    ofstream z("z.txt", ios::out|ios::trunc);

    //pressure data
    ofstream left_pressure("left_pressure.txt", ios::out|ios::trunc);
    ofstream right_pressure("right_pressure.txt", ios::out|ios::trunc);

    std::vector<float> data_ax, data_ay, data_az;
    std::vector<float> data_wx, data_wy, data_wz;

    std::vector<double> data_right_x, data_right_y, data_right_z;
    std::vector<double> data_right_roll, data_right_pitch, data_right_yaw;
    std::vector<double> data_right_vx, data_right_vy, data_right_vz;

    std::vector<double> data_left_x, data_left_y, data_left_z;
    std::vector<double> data_left_roll, data_left_pitch, data_left_yaw;
    std::vector<double> data_left_vx, data_left_vy, data_left_vz;

    std::vector<int> support_now;

    std::vector<double> data_feet_roll, data_feet_pitch, data_feet_yaw;

    std::vector<double> data_x, data_y, data_z;

    std::vector<double> data_left_pressure, data_right_pressure;

    double support_flag = 1;

    std::thread t1(tt);
    t1.detach();
    Motion::StateManager sm;
    timer a;
    while(ros::ok()){
      a.tic();
      sm.imu_data = imu_data;
      sm.m_power_state = power_data;
      sm.servo_initialized = servo_state;
      sm.pressure_data = pressure_data;
      sm.servo_pos = read_pos;
      sm.servo_vel = read_vel;
      sm.working();
      a.SmartDelayMs(10.0);//暂时回到10ms
      if(sm.imu_initialized == Motion::INITED && sm.pressure_initialized == Motion::INITED)
      {
         data_ax.push_back(imu_data.accl.x);
         data_ay.push_back(imu_data.accl.y);
         data_az.push_back(imu_data.accl.z);
         data_wx.push_back(imu_data.gypo.x);
         data_wy.push_back(imu_data.gypo.y);
         data_wz.push_back(imu_data.gypo.z);

         data_left_pressure.push_back(sm.left_pressure);
         data_right_pressure.push_back(sm.right_pressure);

         std::vector<double> right_p(read_pos.begin(), read_pos.begin() + 6);
         std::vector<double> right_v(read_vel.begin(), read_vel.begin() + 6);
         ForKin  leg_right(right_p,true);
         data_right_x.push_back(leg_right.x_result);
         data_right_y.push_back(leg_right.y_result);
         data_right_z.push_back(leg_right.z_result);
         data_right_roll.push_back(leg_right.roll_result);
         data_right_pitch.push_back(leg_right.pitch_result);
         data_right_yaw.push_back(leg_right.yaw_result);
         leg_right.calVelocity(right_v);
         data_right_vx.push_back(leg_right.vx_result);
         data_right_vy.push_back(leg_right.vy_result);
         data_right_vz.push_back(leg_right.vz_result);

         std::vector<double> left_p(read_pos.begin() + 6, read_pos.begin() + 12);
         std::vector<double> left_v(read_vel.begin() + 6, read_vel.begin() + 12);
         ForKin  leg_left(left_p,false);
         data_left_x.push_back(leg_left.x_result);
         data_left_y.push_back(leg_left.y_result);
         data_left_z.push_back(leg_left.z_result);
         data_left_roll.push_back(leg_left.roll_result);
         data_left_pitch.push_back(leg_left.pitch_result);
         data_left_yaw.push_back(leg_left.yaw_result);
         leg_left.calVelocity(left_v);
         data_left_vx.push_back(leg_left.vx_result);
         data_left_vy.push_back(leg_left.vy_result);
         data_left_vz.push_back(leg_left.vz_result);

         if(sm.m_support_state == SUPPORT_RIGHT)
         {
            support_now.push_back(1);
            support_flag = 1;
         }
         else if(sm.m_support_state == SUPPORT_LEFT)
         {
            support_now.push_back(0);
            support_flag = 0;
         }
         else if(sm.m_support_state == SUPPORT_BOTH)
            support_now.push_back(2);
         else
            support_now.push_back(3);

        flag++;// start walking ticks

        if(support_flag == 0)
        {
            data_feet_roll.push_back(leg_left.roll_result);
            data_feet_pitch.push_back(leg_left.pitch_result);
            data_feet_yaw.push_back(leg_left.yaw_result);

            data_x.push_back(leg_left.x_result);
            data_y.push_back(leg_left.y_result);
            data_z.push_back(leg_left.z_result);
        }
        else
        {
            data_feet_roll.push_back(leg_right.roll_result);
            data_feet_pitch.push_back(leg_right.pitch_result);
            data_feet_yaw.push_back(leg_right.yaw_result);

            data_x.push_back(leg_right.x_result);
            data_y.push_back(leg_right.y_result);
            data_z.push_back(leg_right.z_result);
        }
      }
    }

    INFO("***********************");
    INFO("get shit done");
    INFO("***********************");

    cout << int(roll.size()) << endl;
    int i;
    for(i = 0;i < int(roll.size()); i++)
    {
      data_ax << ax[i] << " ";
      data_ay << ay[i] << " ";
      data_az << az[i] << " ";
      data_wx << wx[i] << " ";
      data_wy << wy[i] << " ";
      data_wz << wz[i] << " ";

      right_x << data_right_x[i] << " ";
      right_y << data_right_y[i] << " ";
      right_z << data_right_z[i] << " ";
      right_roll << data_right_roll[i] << " ";
      right_pitch << data_right_pitch[i] << " ";
      right_yaw << data_right_yaw[i] << " ";
      right_vx << data_right_vx[i] << " ";
      right_vy << data_right_vy[i] << " ";
      right_vz << data_right_vz[i] << " ";

      left_x << data_left_x[i] << " ";
      left_y << data_left_y[i] << " ";
      left_z << data_left_z[i] << " ";
      left_roll << data_left_roll[i] << " ";
      left_pitch << data_left_pitch[i] << " ";
      left_yaw << data_left_yaw[i] << " ";
      left_vx << data_left_vx[i] << " ";
      left_vy << data_left_vy[i] << " ";
      left_vz << data_left_vz[i] << " ";

      support << support_now[i] << " ";

      feet_roll << data_feet_roll[i] << " ";
      feet_pitch << data_feet_pitch[i] << " ";
      feet_yaw << data_feet_yaw[i] << " ";

      feet_rollv << data_feet_rollv[i] << " ";
      feet_pitchv << data_feet_pitchv[i] << " ";
      feet_yawv << data_feet_yawv[i] << " ";

      x << data_x[i] << " ";
      y << data_y[i] << " ";
      z << data_z[i] << " ";

      left_pressure << data_left_pressure[i] << " ";
      right_pressure << data_right_pressure[i] << " ";
    }
    cout << i << endl;

    cout << "writing datas done" << endl;

      ax.close();
      ay.close();
      az.close();
      wx.close();
      wy.close();
      wz.close();

      right_x.close();
      right_y.close();
      right_z.close();
      right_roll.close();
      right_pitch.close();
      right_yaw.close();
      right_vx.close();
      right_vy.close();
      right_vz.close();

      left_x.close();
      left_y.close();
      left_z.close();
      left_roll.close();
      left_pitch.close();
      left_yaw.close();
      left_vx.close();
      left_vy.close();
      left_vz.close();

      support.close();

      feet_yaw.close();
      feet_pitch.close();
      feet_roll.close();

      x.close();
      y.close();
      z.close();

      left_pressure.close();
      right_pressure.close();
}
