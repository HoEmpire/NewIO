#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "dmotion/InverseKinematics/InverseKinematics.h"
#include "dmotion/ThreeInterpolation/ThreeInterpolation.h"
#include "dmotion/Utility/dmotion_math.hpp"
#include "../../include/dmotion/IO/ServoIO.h"
#include "../../include/dmotion/IO/IOManager3.h"

#define LEG_ONLY false

using namespace std;
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "testing");
  ros::NodeHandle nh("~");
  parameters.init(&nh);
  Motion::IOManager3 io;
  ofstream outfile;
  outfile.open("/home/ubuntu/climb.txt", std::ios::trunc);
  std::vector<double> SingleInstantPosition;
  double zeit = 0;
  io.ServoPowerOff();
  //设置按键控制开始读值
  char input;             
  std::cin >> input;
  if(input == 'b')
  {
    auto start = std::chrono::system_clock::now();
    while(ros::ok())
    {
      timer a;
      io.readJointValue();
      timer::delay_ms(2);    //控制每次读值时间在5ms
      SingleInstantPosition = io.readAllPosition();
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start;    //记录每次读值的时间戳
      SingleInstantPosition.push_back(elapsed_seconds.count() * 1000);
      zeit = a.toc();
      cout << "[read time]:" << zeit << endl << endl;
      //对称的腿和手的rpy值取平均使其动作呈镜像状态
      double hip_yaw = (SingleInstantPosition[0] + SingleInstantPosition[6]) / 2;
      double hip_roll = (SingleInstantPosition[1] + SingleInstantPosition[7]) / 2;
      double hip_pitch = (SingleInstantPosition[2] + SingleInstantPosition[8]) / 2;
      double knee = (SingleInstantPosition[3] + SingleInstantPosition[9]) / 2;
      double ankle_pitch = (SingleInstantPosition[4] + SingleInstantPosition[10]) /2;
      double ankle_roll = (SingleInstantPosition[5] + SingleInstantPosition[11]) / 2;
      SingleInstantPosition[0] = SingleInstantPosition[6] = hip_yaw;
      SingleInstantPosition[1] = SingleInstantPosition[7] = hip_roll;
      SingleInstantPosition[2] = SingleInstantPosition[8] = hip_pitch;
      SingleInstantPosition[3] = SingleInstantPosition[9] = knee;
      SingleInstantPosition[4] = SingleInstantPosition[10] = ankle_pitch;
      SingleInstantPosition[5] = SingleInstantPosition[11] = ankle_roll;
      if(!LEG_ONLY)
      {
        double arm_upper = (SingleInstantPosition[12] + SingleInstantPosition[14]) / 2;
        double arm_lower = (SingleInstantPosition[13] + SingleInstantPosition[15]) / 2;
        SingleInstantPosition[12] = SingleInstantPosition[14] = arm_upper;
        SingleInstantPosition[13] = SingleInstantPosition[15] = arm_lower;
      }
      //将每次读出的值写入文件
      for (unsigned i = 0; i < SingleInstantPosition.size(); i ++)
      {
        outfile << SingleInstantPosition[i] << ' ';
      }
      outfile << std::endl;
    }
  }
}