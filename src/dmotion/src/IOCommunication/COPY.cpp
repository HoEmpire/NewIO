#include "dmotion/Common/Utility/Utility.h"

#include <Eigen/Geometry>
//
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

using namespace std;
using namespace Eigen;
Motion::IMUData imu_data;
Motion::PowerState power_data;
Motion::PressureData pressure_data;
double vx_real;
double x_real;
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
    std::vector<double> fucking(18,0);
    PendulumWalk pen;
    pen.GiveAStep(0,0,0);
    //fucking = pen.GiveATick();
    io.setAllJointValue(fucking);
    io.spinOnce();
    sleep(2);
    io.setAllspeed(0);//after 4 seconds, servo ini finished

    double E;
    int E_flag = 0;
    bool is_stable = true;
    int ticks = 0;
    int ticks_push_recovery = 0;
    std::vector<double> y(35,0);
    double T = 35.0;
    for(int i = 0; i < 35; i++)
    {
      y[i] = cos(2*M_PI/T*i);
    }

    InvKin left(false);
    InvKin right(true);
    std::vector<double> left_pos = {2, 4.5, -27, 0, 0, 0};
    std::vector<double> right_pos = {-2, -4.5, -27, 0, 0, 0};

    std::vector<double> left_servo(6,0);
    std::vector<double> right_servo(6,0);
    left_servo = left.LegInvKin(left_pos);
    right_servo = right.LegInvKin(right_pos);

    for(int i = 0; i < 6; i++)
        fucking[i] = right_servo[i];

    for(int i = 6; i < 12; i++)
        fucking[i] = left_servo[i-6];


    io.setAllJointValue(fucking);
    io.setAllspeed(30);
    io.spinOnce();
    sleep(3);
    io.setAllspeed(0);

    while(ros::ok()){
      if(flag >= 2000)
      {
          if(ticks < 35)
          {
            fucking[2] = y[ticks];
            fucking[8] = y[ticks];
            io.setAllJointValue(fucking);
            ticks++;
          }
        else
              ticks = 0;
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



    std::thread t1(tt);
    sleep(4);
    t1.detach();

    Motion::StateManager sm;
    timer a;
    while(ros::ok()){
      a.tic();
      sm.imu_data = imu_data;
      sm.m_power_state = power_data;
      sm.servo_initialized = servo_state;
      sm.pressure_data = pressure_data;
      sm.working();
      a.SmartDelayMs(10.0);//暂时回到10ms
    }



}
