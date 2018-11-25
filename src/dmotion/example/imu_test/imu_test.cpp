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
      Motion::IMUReader imu;

      while(ros::ok()){
        imu.readIMUData();
        imu.getIMUData();
        timer::delay_ms(10);
      }
}
