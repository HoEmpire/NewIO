#include <ros/ros.h>
#include "include/dmotion/PendulumWalk/PendulumWalk.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/uint8.h>
#include ""
// uint8 status
// stop = 0
// front = 8
// back = 2
// left = 4
// right = 6
// turn = 6


void Command(const std_msgs::uint8 & msg)
{
  static int ticks = 0;
  int com = msg.data;
  if(ticks == 0)
  {
    
  }
  else
  {

  }
}

int main(int argc, char const *argv[])
{
  ros::init(argc, argv, "testing");
  ros::NodeHandle nh("~");
  parameters.init(&nh);
  sub_command = m_nh->subscribe("PTest", 1, &Command, this);
  pub_servo = m_nh->advertise<std_msgs::Float64MultiArray>("/ServoInfo", 1);
  while(ros::ok())
  {

  }
  return 0;
}
