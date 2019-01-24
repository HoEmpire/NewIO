#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/State/imu_filter_new.hpp"

#include <Eigen/Geometry>
//
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/IMUReader.h"
#include "../../include/dmotion/IO/IOManager3.h"
#include "dmotion/State/StateManager.hpp"

#include <thread>
#include <fstream>
#include <iostream>

#define workplace "/home/ubuntu/NewIO/test_data/test1"

using namespace std;
using namespace Eigen;
Motion::IMUData imu_data;

// void tt()
// {
//     Motion::IMUReader imu;
//     while(ros::ok()){
//         imu.readIMUData();
//         imu_data = imu.getIMUData();
//     }
//
// }
//
// static inline void QuaternionToEulerAngles(float qw, float qx, float qy, float qz,
//                                            float& roll, float& pitch, float& yaw)
// {
//     roll = atan2f(2.f * (qz*qy + qw*qx), 1-2*(qx*qx+qy*qy)); //x
//     pitch =  asinf(2.f * (qw*qy - qx*qz)); //y
//     yaw = atan2f(2.f * (qx*qy + qw*qz), 1-2*(qy*qy+qz*qz));//z
// }
//
// int main(int argc, char ** argv)
// {
//     if(chdir(workplace))
//         exit(0);  //设置工作路径，也就是B显示的路径
//
//     ros::init(argc, argv, "testing");
//     ros::NodeHandle nh("~");
//     parameters.init(&nh);
//
//     ofstream out1("roll.txt", ios::out|ios::trunc);
//     ofstream out2("pitch.txt", ios::out|ios::trunc);
//     ofstream out3("yaw.txt", ios::out|ios::trunc);
//     ofstream out4("ax.txt", ios::out|ios::trunc);
//     ofstream out5("ay.txt", ios::out|ios::trunc);
//     ofstream out6("az.txt", ios::out|ios::trunc);
//     ofstream out7("wx.txt", ios::out|ios::trunc);
//     ofstream out8("wy.txt", ios::out|ios::trunc);
//     ofstream out9("wz.txt", ios::out|ios::trunc);
//
//     std::vector<float> roll,pitch,yaw;
//     std::vector<float> ax,ay,az;
//     std::vector<float> wx,wy,wz;
//
//     timer::time_point start,now;
//     std::chrono::duration<double> elapsed_seconds;
//
//     ImuFilter dick;
//     std::thread t1(tt);
//     t1.detach();
//     sleep(1);
//
//     long int count = 0;
//     int ini_ticks = 1;
//     timer a;
//     float lin_acc_x;
//     float lin_acc_y;
//     float lin_acc_z;
//     float ang_vel_x;
//     float ang_vel_y;
//     float ang_vel_z;
//     float x,y,z;
//     float q0,q1,q2,q3;
//     a.tic();
//     for(ini_ticks = 1; ini_ticks <= 1000; ini_ticks++)
//     {
//       lin_acc_x = imu_data.accl.x;
//       lin_acc_y = imu_data.accl.y;
//       lin_acc_z = imu_data.accl.z;
//       ang_vel_x = imu_data.gypo.x;
//       ang_vel_y = imu_data.gypo.y;
//       ang_vel_z = imu_data.gypo.z;
//       if(ini_ticks == 1)
//       {
//         dick.iniAcclast(lin_acc_x , lin_acc_y, lin_acc_z);
//       }
//       else
//       {
//         dick.iniIMU(ang_vel_x, ang_vel_y, ang_vel_z,
//                     lin_acc_x, lin_acc_y, lin_acc_z,
//                     ini_ticks);
//       }
//       start = timer::getCurrentSystemTime();
//       a.smartDelay_ms(5.0);
//       a.tic();
//     }
//
//     dick.iniGravity();
//     dick.iniQuaternion();
//     dick.getOrientation(q0, q1, q2, q3);
//     INFO("***********************");
//     INFO("Ini finished!!!");
//     INFO("***********************");
//     // Quaterniond Q1(q0, q1, q2, q3);
//     // Matrix3d R1;
//     // R1 = Q1.matrix();
//     // Vector3d Euler1 = R1.eulerAngles(2,1,0);
//     float x1,y1,z1;
//     // x1 = Euler1(2);
//     // y1 = Euler1(1);
//     // z1 = Euler1(0);
//     QuaternionToEulerAngles(q0,q1,q2,q3,x1,y1,z1);
//     cout << "x1: "<< x1 << " ,y1: " << y1 << ",z1: " << z1 << endl;
//
//     while(ros::ok())
//     {
//
//       lin_acc_x = imu_data.accl.x;
//       lin_acc_y = imu_data.accl.y;
//       lin_acc_z = imu_data.accl.z;
//       ang_vel_x = imu_data.gypo.x;
//       ang_vel_y = imu_data.gypo.y;
//       ang_vel_z = imu_data.gypo.z;
//
//       dick.Fusing(ang_vel_x, ang_vel_y, ang_vel_z,
//                   lin_acc_x, lin_acc_y, lin_acc_z);
//       dick.getOrientation(q0, q1, q2, q3);
//
//       // Quaterniond Q(q0, q1, q2, q3);
//       // Matrix3d R;
//       // R = Q.matrix();
//       // Vector3d Euler = R.eulerAngles(2,1,0);
//       // x = Euler(2);
//       // y = Euler(1);
//       // z = Euler(0);
//       //QuaternionToEulerAngles(q0,q1,q2,q3,x,y,z);
//       dick.getRPY(x,y,z);
//       roll.push_back(x);
//       pitch.push_back(y);
//       yaw.push_back(z);
//       ax.push_back(lin_acc_x);
//       ay.push_back(lin_acc_y);
//       az.push_back(lin_acc_x);
//       wx.push_back(ang_vel_x);
//       wy.push_back(ang_vel_y);
//       wz.push_back(ang_vel_z);
//
//       if(count % 1 == 0)
//       {
//           std::cout << "x = " << x << " ,y = " << y << " ,z = " << z << std::endl;
//       }
//
//       count++;
//
//       a.smartDelay_ms(5.0);
//       a.tic();
//     }
//
//     sleep(2);
//     INFO("***********************");
//     INFO("get shit done");
//     INFO("***********************");
//     cout << int(roll.size()) << endl;
//     int i;
//     for(i = 0;i < int(roll.size()); i++)
//     {
//       out1 << roll[i] << " ";
//       out2 << pitch[i] << " ";
//       out3 << yaw[i] << " ";
//       out4 << ax[i] << " ";
//       out5 << ay[i] << " ";
//       out6 << az[i] << " ";
//       out7 << wx[i] << " ";
//       out8 << wy[i] << " ";
//       out9 << wz[i] << " ";
//     }
//     cout << i << endl;
//
//
//     out1.close();
//     out2.close();
//     out3.close();
//     out4.close();
//     out5.close();
//     out6.close();
//     out7.close();
//     out8.close();
//     out9.close();
//
// }


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");
    parameters.init(&nh);


    Motion::IOManager3 io;
    Motion::StateManager sm;
    while(sm.imu_initialized) {
      sm.imu_data = io.getIMUData();
      sm.iniIMUFilter();
    }

    while(ros::ok()){
      sm.imu_data = io.getIMUData();
      sm.calIMUFilter();
    }


    INFO("***********************");
    INFO("get shit done");
    INFO("***********************");

}
