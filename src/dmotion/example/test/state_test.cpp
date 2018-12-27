#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Type.h"
#include "dmotion/State/imu_filter.hpp"
#include "dmotion/State/stateless_orientation.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/IMUReader.h"

#include <thread>

#include <fstream>
#include <iostream>

#define workplace "/home/tim/NewIO/test_data/test1"

using namespace std;

#define imu_bias_gain 0.1
#define imu_algorithm_gain 0.8

Motion::IMUData imu_data;

void tt()
{
    Motion::IMUReader imu;
    while(ros::ok()){
        imu.readIMUData();
        imu_data = imu.getIMUData();
    }

}

int main(int argc, char ** argv)
{
    if(chdir(workplace))
        exit(0);  //设置工作路径，也就是B显示的路径

    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");
    parameters.init(&nh);

    ofstream out1("roll.txt", ios::out|ios::trunc);
    ofstream out2("pitch.txt", ios::out|ios::trunc);
    ofstream out3("yaw.txt", ios::out|ios::trunc);
    ofstream out4("ax.txt", ios::out|ios::trunc);
    ofstream out5("ay.txt", ios::out|ios::trunc);
    ofstream out6("az.txt", ios::out|ios::trunc);
    ofstream out7("wx.txt", ios::out|ios::trunc);
    ofstream out8("wy.txt", ios::out|ios::trunc);
    ofstream out9("wz.txt", ios::out|ios::trunc);
    ofstream out10("time.txt", ios::out|ios::trunc);

    std::vector<float> roll,pitch,yaw;
    std::vector<float> ax,ay,az;
    std::vector<float> wx,wy,wz;
    std::vector<double> t;

    timer::time_point start,now;
    std::chrono::duration<double> elapsed_seconds;
    double time_past;

    auto m_frame = WorldFrame::ENU;
    ImuFilter m_imu_filter;
    m_imu_filter.setWorldFrame(m_frame);
    m_imu_filter.setDriftBiasGain(imu_bias_gain);
    m_imu_filter.setAlgorithmGain(imu_algorithm_gain);

    geometry_msgs::Vector3 lin_acc, ang_vel;
    geometry_msgs::Quaternion init_q;
    // lin_acc.x = 0;
    // lin_acc.y = 0;
    // lin_acc.z = -9.8;
    // ang_vel.x = 0;
    // ang_vel.y = 0;
    // ang_vel.z = 0;
    //
    //
    // StatelessOrientation::computeOrientation(m_frame, lin_acc, init_q);
    // m_imu_filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

    tf2::Matrix3x3 R;
    float q0, q1, q2, q3;
    double x,y,z;
    m_imu_filter.getOrientation(q0, q1, q2, q3);

    tf2::Quaternion q(q1, q2, q3, q0);
    tf2::Matrix3x3(q).getRPY(x, y, z);
    std::cout << "x = " << x << " ,y = " << y << " ,z = " << z << std::endl;

    std::thread t1(tt);
    t1.detach();

    sleep(5);

    long int count = 0;
    int ini_ticks = 0;
    timer a;

    tf2::Matrix3x3(q).getRPY(x, y, z);
    while(ros::ok())
    {
      a.tic();
      lin_acc.x = imu_data.accl.x;
      lin_acc.y = imu_data.accl.y;
      lin_acc.z = imu_data.accl.z;
      ang_vel.x = imu_data.gypo.x;
      ang_vel.y = imu_data.gypo.y;
      ang_vel.z = imu_data.gypo.z;

      if(ini_ticks < 100)
      {
          StatelessOrientation::computeOrientation(m_frame, lin_acc, init_q);
          m_imu_filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
          start = timer::getCurrentSystemTime();
      }
      else
      {
          m_imu_filter.madgwickAHRSupdateIMU(ang_vel.x, ang_vel.y, ang_vel.z,
                                           lin_acc.x, lin_acc.y, lin_acc.z,
                                           0.005);
          m_imu_filter.getOrientation(q0, q1, q2, q3);
          tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0)).getRPY(x, y, z);

          now = timer::getCurrentSystemTime();//TODO be careful about the time
          elapsed_seconds = now -start;
          time_past = elapsed_seconds.count() * 1000;

          roll.push_back(x);
          pitch.push_back(y);
          yaw.push_back(z);
          ax.push_back(lin_acc.x);
          ay.push_back(lin_acc.y);
          az.push_back(lin_acc.z);
          wx.push_back(ang_vel.x);
          wy.push_back(ang_vel.y);
          wz.push_back(ang_vel.z);
          t.push_back(time_past);
      }

      if(count % 100 == 0)
      {
          std::cout << "x = " << x << " ,y = " << y << " ,z = " << z << std::endl;
      }

      ini_ticks++;
      count++;



      a.smartDelay_ms(5.0);
    }

    sleep(2);

    INFO("get shit done");
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
      out10 << t[i] << " ";
    }
    cout << i << endl;


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

}
