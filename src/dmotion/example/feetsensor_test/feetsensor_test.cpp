
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "dmotion/IO/FeetSensorIO.h"
#include <thread>
#include "dmotion/IO/IOManager3.h"
#define workplace "/home/ubuntu/test/NewIO/test_data/test1"

using namespace std;
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");
    parameters.init(&nh);
    Motion::FeetSensorIO fsio;
    timer b;
    if(chdir(workplace))
        exit(0);  //设置工作路径，也就是B显示的路径

    while(ros::ok()){
      b.tic();
      fsio.readPressureData();
      b.toc();
      b.SmartDelayMs(10.0);
    }

}
