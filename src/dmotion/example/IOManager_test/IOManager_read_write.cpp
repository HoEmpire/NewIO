#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/ServoIO.h"

using namespace std;
int main(int argc, char ** argv)
{
      ros::init(argc, argv, "testing");
      ros::NodeHandle nh("~");
      parameters.init(&nh);
      Motion::IOManager3 io;
      std::vector<double> fucking(16);
      ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
      int cunt = 0;
      //write
      while(ros::ok() && cunt < 10){

        for(int i = 0; i < 12; i++){
          fucking[11] = fucking[11] - 1;
          fucking[4] = fucking[4] - 1;
          io.setAllJointValue(fucking);
          io.spinOnce();
        }

        for(int i = 0; i < 12; i++){
          fucking[11] = fucking[11] + 1;
          fucking[4] = fucking[4] + 1;
          io.setAllJointValue(fucking);
          io.spinOnce();
        }
        cunt++;

      }

      //read constantly
      while(ros::ok() && cunt < 20){
        io.readPosVel();
        sleep(1);
        cunt++;
      }

      //read after type f
      io.reverseMotion();
}
