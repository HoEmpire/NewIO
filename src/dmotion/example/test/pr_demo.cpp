#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/State/imu_filter_new.hpp"

#include <Eigen/Geometry>
//
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/IMUReader.h"
#include "../../include/dmotion/IO/IOManager3.h"
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
    std::vector<double> fucking;
    PendulumWalk pen;
    pen.GiveAStep(0,0,0);
    fucking = pen.GiveATick();
    io.setAllJointValue(fucking);
    io.spinOnce();
    sleep(2);
    io.setAllspeed(0);//after 4 seconds, servo ini finished

    double E;
    int E_flag = 0;
    int ticks = 0;
    while(ros::ok()){


        E = 0.0001*(0.5*vx_real*vx_real-14*x_real*x_real);
        if(E < -0.1)
        {
          INFO("too slow");
          E_flag = 1;
        }


        if(E_flag)
        {
          if(ticks == 0)
            pen.GiveAStep(5,0,0);
          if(ticks < 35)
          {
            fucking = pen.GiveATick();
            io.setAllJointValue(fucking);
            ticks++;
          }
          else
          {
            ticks = 0;
            E_flag = 0;
          }
        }

        if(E > 0.8)
            INFO("too fast");
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

 //open files
    ofstream out1("roll.txt", ios::out|ios::trunc);
    ofstream out2("pitch.txt", ios::out|ios::trunc);
    ofstream out3("yaw.txt", ios::out|ios::trunc);
    ofstream out4("ax.txt", ios::out|ios::trunc);
    ofstream out5("ay.txt", ios::out|ios::trunc);
    ofstream out6("az.txt", ios::out|ios::trunc);
    ofstream out7("wx.txt", ios::out|ios::trunc);
    ofstream out8("wy.txt", ios::out|ios::trunc);
    ofstream out9("wz.txt", ios::out|ios::trunc);
    ofstream out10("ax_wog.txt", ios::out|ios::trunc);
    ofstream out11("ay_wog.txt", ios::out|ios::trunc);
    ofstream out12("az_wog.txt", ios::out|ios::trunc);

    ofstream right_x("right_x.txt", ios::out|ios::trunc);
    ofstream right_y("right_y.txt", ios::out|ios::trunc);
    ofstream right_yaw("right_yaw.txt", ios::out|ios::trunc);
    ofstream right_vx("right_vx.txt", ios::out|ios::trunc);
    ofstream right_vy("right_vy.txt", ios::out|ios::trunc);
    ofstream right_vyaw("right_vyaw.txt", ios::out|ios::trunc);

    ofstream left_x("left_x.txt", ios::out|ios::trunc);
    ofstream left_y("left_y.txt", ios::out|ios::trunc);
    ofstream left_yaw("left_yaw.txt", ios::out|ios::trunc);
    ofstream left_vx("left_vx.txt", ios::out|ios::trunc);
    ofstream left_vy("left_vy.txt", ios::out|ios::trunc);
    ofstream left_vyaw("left_vyaw.txt", ios::out|ios::trunc);

    ofstream support("support.txt", ios::out|ios::trunc);

    ofstream vx("vx.txt", ios::out|ios::trunc);
    ofstream vy("vy.txt", ios::out|ios::trunc);

    ofstream vx_w("vx_w.txt", ios::out|ios::trunc);
    ofstream vy_w("vy_w.txt", ios::out|ios::trunc);

    ofstream vx_pos("vx_pos.txt", ios::out|ios::trunc);
    ofstream vy_pos("vy_pos.txt", ios::out|ios::trunc);

    ofstream feet_roll("feet_roll.txt", ios::out|ios::trunc);
    ofstream feet_pitch("feet_pitch.txt", ios::out|ios::trunc);
    ofstream feet_yaw("feet_yaw.txt", ios::out|ios::trunc);

    ofstream x("x.txt", ios::out|ios::trunc);
    ofstream y("y.txt", ios::out|ios::trunc);
    ofstream z("z.txt", ios::out|ios::trunc);
    ofstream feet_rollv("feet_rollv.txt", ios::out|ios::trunc);
    ofstream feet_pitchv("feet_pitchv.txt", ios::out|ios::trunc);
    ofstream feet_yawv("feet_yawv.txt", ios::out|ios::trunc);

    std::vector<float> roll,pitch,yaw;
    std::vector<float> ax,ay,az;
    std::vector<float> wx,wy,wz;
    std::vector<float> ax_wog,ay_wog,az_wog;
    std::vector<double> data_right_x, data_right_y, data_right_yaw;
    std::vector<double> data_right_vx, data_right_vy, data_right_vyaw;
    std::vector<double> data_left_x, data_left_y, data_left_yaw;
    std::vector<double> data_left_vx, data_left_vy, data_left_vyaw;
    std::vector<double> data_vx, data_vy, data_feet_roll, data_feet_pitch, data_feet_yaw;
    std::vector<double> data_feet_rollv, data_feet_pitchv, data_feet_yawv;
    std::vector<double> data_vx_pos, data_vy_pos;
    std::vector<double> data_vx_w, data_vy_w;
    std::vector<double> data_x, data_y, data_z;

    std::vector<int> support_now;
    double x_old = 0, y_old = 0, z_old = 0;
    double x_new = 0, y_new = 0, z_new = 0;
    Eigen::Matrix3d rotation_matrix_OA, rotation_matrix_BA, rotation_matrix_OB, rotation_matrix_body;
    bool isRight;
    double temp_vx, temp_vy, temp_wx, temp_wy, temp_wz, temp_roll, temp_pitch, temp_yaw;
    Eigen::Matrix<double,3,1> temp_vel;
    Eigen::Matrix<double,3,1> vel_ref, vel_co;
    Eigen::Matrix<double,3,1> temp_pos, pos_real;
    Eigen::Matrix<double,3,1> pos_old, pos_new;
    Eigen::Matrix<double,3,1> w1, w2, w3;
    Eigen::Matrix<double,3,1> Pd;
    Eigen::Matrix<double,3,3> R_new, dR, R_old;
    R_old.setIdentity();

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
      if(sm.imu_initialized == Motion::INITED && sm.pressure_initialized == Motion::INITED)
      {
         roll.push_back(sm.roll);
         pitch.push_back(sm.pitch);
         yaw.push_back(sm.yaw);
         ax.push_back(imu_data.accl.x);
         ay.push_back(imu_data.accl.y);
         az.push_back(imu_data.accl.z);
         wx.push_back(imu_data.gypo.x);
         wy.push_back(imu_data.gypo.y);
         wz.push_back(imu_data.gypo.z);
         ax_wog.push_back(imu_data.gypo.x);
         ay_wog.push_back(imu_data.gypo.y);
         az_wog.push_back(imu_data.gypo.z);

         std::vector<double> right_p(read_pos.begin(), read_pos.begin() + 6);
         std::vector<double> right_v(read_vel.begin(), read_vel.begin() + 6);
         ForKin  leg_right(right_p,true);
         data_right_x.push_back(leg_right.x_result);
         data_right_y.push_back(leg_right.y_result);
         data_right_yaw.push_back(leg_right.yaw_result);
         leg_right.calVelocity(right_v);
         data_right_vx.push_back(leg_right.vx_result);
         data_right_vy.push_back(leg_right.vy_result);
         data_right_vyaw.push_back(leg_right.vyaw_result);

         std::vector<double> left_p(read_pos.begin() + 6, read_pos.begin() + 12);
         std::vector<double> left_v(read_vel.begin() + 6, read_vel.begin() + 12);
         double support_flag = 1;
         ForKin  leg_left(left_p,false);
         data_left_x.push_back(leg_left.x_result);
         data_left_y.push_back(leg_left.y_result);
         data_left_yaw.push_back(leg_left.yaw_result);
         leg_left.calVelocity(left_v);
         data_left_vx.push_back(leg_left.vx_result);
         data_left_vy.push_back(leg_left.vy_result);
         data_left_vyaw.push_back(leg_left.vyaw_result);
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


        // if(leg_left.vx_result > leg_right.vx_result)
        if(support_flag == 0)
        {
            isRight = false;
            temp_vx = leg_left.vx_result;
            temp_vy = leg_left.vy_result;
            temp_wx = leg_left.vroll_result;
            temp_wy = leg_left.vpitch_result;
            temp_wz = leg_left.vyaw_result;
            temp_roll = leg_left.roll_result;
            temp_pitch = leg_left.pitch_result;
            temp_yaw = leg_left.yaw_result;
            Eigen::Vector3d eulerAngle_BA(leg_left.yaw_result/180*M_PI,
                                          leg_left.pitch_result/180*M_PI,
                                          leg_left.roll_result/180*M_PI);
            Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle_BA(2),Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle_BA(1),Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle_BA(0),Vector3d::UnitZ()));
            rotation_matrix_BA = yawAngle*pitchAngle*rollAngle;

            data_feet_roll.push_back(leg_left.roll_result);
            data_feet_pitch.push_back(leg_left.pitch_result);
            data_feet_yaw.push_back(leg_left.yaw_result);

            data_feet_rollv.push_back(leg_left.vroll_result);
            data_feet_pitchv.push_back(leg_left.vpitch_result);
            data_feet_yawv.push_back(leg_left.vyaw_result);

            x_new = leg_left.x_result;
            y_new = leg_left.y_result;
            z_new = leg_left.z_result;
        }
        else
        {
            isRight = true;
            temp_vx = leg_right.vx_result;
            temp_vy = leg_right.vy_result;
            temp_wx = leg_right.vroll_result;
            temp_wy = leg_right.vpitch_result;
            temp_wz = leg_right.vyaw_result;
            temp_roll = leg_right.roll_result;
            temp_pitch = leg_right.pitch_result;
            temp_yaw = leg_right.yaw_result;
            Eigen::Vector3d eulerAngle_BA(leg_right.yaw_result/180*M_PI,
                                          leg_right.pitch_result/180*M_PI,
                                          leg_right.roll_result/180*M_PI);
            Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle_BA(2),Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle_BA(1),Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle_BA(0),Vector3d::UnitZ()));
            rotation_matrix_BA = yawAngle*pitchAngle*rollAngle;

            data_feet_roll.push_back(leg_right.roll_result);
            data_feet_pitch.push_back(leg_right.pitch_result);
            data_feet_yaw.push_back(leg_right.yaw_result);

            data_feet_rollv.push_back(leg_right.vroll_result);
            data_feet_pitchv.push_back(leg_right.vpitch_result);
            data_feet_yawv.push_back(leg_right.vyaw_result);

            x_new = leg_right.x_result;
            y_new = leg_right.y_result;
            z_new = leg_right.z_result;
        }


        Eigen::Vector3d eulerAngle_OA(sm.yaw, sm.pitch, sm.roll);
        Eigen::AngleAxisd rollAngle2(AngleAxisd(eulerAngle_OA(2),Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle2(AngleAxisd(eulerAngle_OA(1),Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle2(AngleAxisd(eulerAngle_OA(0),Vector3d::UnitZ()));
        rotation_matrix_OA = yawAngle2*pitchAngle2*rollAngle2;

        rotation_matrix_OB = rotation_matrix_OA * rotation_matrix_BA.transpose();
        temp_vel(0) = temp_vx;
        temp_vel(1) = temp_vy;
        temp_vel(2) = 0;
        temp_vel = rotation_matrix_OB * temp_vel;
        data_vx.push_back(temp_vel(0));
        data_vy.push_back(temp_vel(1));
        vel_ref = temp_vel;

        pos_new(0) = x_new;
        pos_new(1) = y_new;
        pos_new(2) = z_new;
        pos_new = rotation_matrix_OB * pos_new;

        pos_old(0) = x_old;
        pos_old(1) = y_old;
        pos_old(2) = z_old;
        pos_old = rotation_matrix_OB * pos_old;

        temp_vel = (pos_new - pos_old)/0.01;
        data_vx_pos.push_back(temp_vel(0));
        data_vy_pos.push_back(temp_vel(1));

        x_old = x_new;
        y_old = y_new;
        z_old = z_new;

        w2(0) = -temp_wy*sin(temp_yaw) + temp_wx*cos(temp_yaw)*cos(temp_pitch);
        w2(1) =  temp_wy*cos(temp_yaw) + temp_wx*sin(temp_yaw)*cos(temp_pitch);
        w2(2) =  temp_wz - temp_wx*sin(temp_pitch);

        w2 = rotation_matrix_OB * w2;
        w3(0) = imu_data.gypo.x;
        w3(1) = imu_data.gypo.y;
        w3(2) = imu_data.gypo.z;
        w3 = rotation_matrix_OA * w3;

        w1 = w3 - w2;

        cross(w1, rotation_matrix_OB * pos_new, temp_vel);
        data_vx_w.push_back(temp_vel(0));
        data_vy_w.push_back(temp_vel(1));
        vel_co = temp_vel;

        data_x.push_back(x_new);
        data_y.push_back(y_new);
        data_z.push_back(z_new);

        rotation_matrix_body = yawAngle2;
        temp_vel = rotation_matrix_body.transpose() * (vel_co + vel_ref);
        vx_real = temp_vel(0);

        temp_pos(0) = x_new;
        temp_pos(1) = y_new;
        temp_pos(2) = z_new;
        pos_real = rotation_matrix_body.transpose() * (rotation_matrix_OB * temp_pos);
        x_real =  pos_real(0); //update x

        R_old = rotation_matrix_BA;
      }
    }

    INFO("***********************");
    INFO("get shit done");
    INFO("***********************");

    cout << int(roll.size()) << endl;
    int i;
    for(i = 0;i < int(roll.size()); i++)
    {
      out1 << roll[i] << " ";
      out2 << pitch[i] << " ";
      out3 << yaw[i] << " ";
      out4 << ax[i] << " ";
      out5 << ay[i] << " ";
      out6 << az[i] << " ";
      out7 << wx[i] << " ";
      out8 << wy[i] << " ";
      out9 << wz[i] << " ";
      out10 << ax_wog[i] << " ";
      out11 << ay_wog[i] << " ";
      out12 << az_wog[i] << " ";

      right_x << data_right_x[i] << " ";
      right_y << data_right_y[i] << " ";
      right_yaw << data_right_yaw[i] << " ";
      right_vx << data_right_vx[i] << " ";
      right_vy << data_right_vy[i] << " ";
      right_vyaw << data_right_vyaw[i] << " ";

      left_x << data_left_x[i] << " ";
      left_y << data_left_y[i] << " ";
      left_yaw << data_left_yaw[i] << " ";
      left_vx << data_left_vx[i] << " ";
      left_vy << data_left_vy[i] << " ";
      left_vyaw << data_left_yaw[i] << " ";

      support << support_now[i] << " ";

      vx << data_vx[i] << " ";
      vy << data_vy[i] << " ";

      vx_w << data_vx_w[i] << " ";
      vy_w << data_vy_w[i] << " ";

      vx_pos << data_vx_pos[i] << " ";
      vy_pos << data_vy_pos[i] << " ";

      feet_roll << data_feet_roll[i] << " ";
      feet_pitch << data_feet_pitch[i] << " ";
      feet_yaw << data_feet_yaw[i] << " ";

      feet_rollv << data_feet_rollv[i] << " ";
      feet_pitchv << data_feet_pitchv[i] << " ";
      feet_yawv << data_feet_yawv[i] << " ";

      x << data_x[i] << " ";
      y << data_y[i] << " ";
      z << data_z[i] << " ";
    }
    cout << i << endl;

    cout << "writing datas done" << endl;


      out1.close();
      out2.close();
      out3.close();
      out4.close();
      out5.close();
      out6.close();
      out7.close();
      out8.close();
      out9.close();
      out10.close();
      out11.close();
      out12.close();

      right_x.close();
      right_y.close();
      right_yaw.close();
      right_vx.close();
      right_vy.close();
      right_vyaw.close();

      left_x.close();
      left_y.close();
      left_yaw.close();
      left_vx.close();
      left_vy.close();
      left_vyaw.close();

      support.close();

      vx.close();
      vy.close();

      vx_w.close();
      vy_w.close();

      vx_pos.close();
      vy_pos.close();

      feet_yaw.close();
      feet_pitch.close();
      feet_roll.close();

      feet_yawv.close();
      feet_pitchv.close();
      feet_rollv.close();

      x.close();
      y.close();
      z.close();
}
