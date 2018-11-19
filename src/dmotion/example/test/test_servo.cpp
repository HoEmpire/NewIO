#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "dmotion/InverseKinematics/InverseKinematics.h"
#include "dmotion/ThreeInterpolation/ThreeInterpolation.h"
#include "dmotion/Utility/dmotion_math.hpp"
#include "../../include/dmotion/IO/ServoIO.h"

#include <iostream>
#include <unistd.h>
using namespace std;
// int main(int argc, char ** argv)
// {
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     ros::init(argc, argv, "testing");
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     ros::NodeHandle nh("~");
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     parameters.init(&nh);
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     //Motion::IOManager3 io;
//     //Motion::ServoIO m_servo_io;
//   //   Motion::FeetSensorIO m_feet_io;
//   // //  m_feet_io.setPort(m_servo_io.getPortHandler());
//   //   while(1){
//   //   //  m_servo_io.sendServoPositions();
//   //     if (!m_feet_io.readPressureData())
//   //     {
//   //         ROS_WARN("IOManager3::spinOnce: read feet pressure data error");
//   //     }
//   //     sleep(1);
//   //   }
//
// }

int main(int argc, char ** argv)
{
      ros::init(argc, argv, "testing");
      ros::NodeHandle nh("~");
      parameters.init(&nh);
      Motion::IOManager3 io;
      std::vector<double> fucking(16);
      ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
      int cunt = 0;
      double zeit = 0;
      //read constantly
      while(ros::ok()){
        timer a;
        io.readJointValue();
        zeit = a.toc();
        cout << "***********************************" << endl;
        cout << "[read time]:" << zeit << endl << endl;
        cout << "***********************************" << endl;
        sleep(1);
      }

}
