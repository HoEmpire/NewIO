#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/IMUReader.h"

using namespace std;
int main(int argc, char ** argv)
{
      ros::init(argc, argv, "testing");
      ros::NodeHandle nh("~");
      parameters.init(&nh);
      Motion::IOManager3 io;

      while(ros::ok()){
        io.spinOnce();
        //io.readIMU();
        //io.getIMUData();
      }
}
