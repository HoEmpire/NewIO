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
#define LEG_ONLY_NUMBER 13
#define NOT_LEG_ONLY_NUMBER 17

using namespace std;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "testing");
  ros::NodeHandle nh("~");
  parameters.init(&nh);
  Motion::IOManager3 io;
  std::vector<std::vector<double> > AllPosition_time;
  std::vector<std::vector<double> > AllPosition; 
  ifstream infile;
  infile.open("/home/ubuntu/climb.txt", std::ios::in|std::ios::out);
  //将文件中的值读入二维数组中
  while(!infile.eof())
  {
    double buffer;
    std::vector<double> SinglePosition;
    //选择腿部还是全身
    for(int i = 0; i < NOT_LEG_ONLY_NUMBER; i ++)
    {
      infile >> buffer;
      SinglePosition.push_back(buffer);
    }
    AllPosition_time.push_back(SinglePosition);
    SinglePosition.clear();
  }
  //复制一个二维数组，去掉时间戳给舵机发值
  AllPosition = AllPosition_time;
  for (unsigned i = 0; i < AllPosition.size(); i++)
  {
    AllPosition[i].pop_back();
  }
  int flag = 0;
  while(ros::ok())
  {
    if(flag == 0)
    {
      sleep(5);
      flag = 1;
    }
    //每两组值中选取一组发给舵机，保证每次写入动作是10ms
    for(unsigned i = 0; i < AllPosition.size(); i += 2)
    {
      //避免一系列动作完成后出现抖动
      if(i == 0)
      {
        io.setAllspeed(30);
        io.setAllJointValue(AllPosition[i]);
        io.spinOnce();
        sleep(3);
      }
      else
      {
        io.setAllspeed(0);
        io.setAllJointValue(AllPosition[i]);
        io.spinOnce();
      }
      std::cout << i << std::endl;
    }
  }
}