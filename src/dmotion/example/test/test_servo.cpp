#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/IMUReader.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <pthread.h>
// #include <sched.h>
#define PORT_NAME "/dev/IMU"
#define BAUDRATE  576000


using namespace std;
int main(int argc, char ** argv)
{
  dynamixel::PortHandler* imu_port = dynamixel::PortHandler::getPortHandler(PORT_NAME);

  if (!imu_port->setBaudRate(BAUDRATE,false))
  {
      ROS_FATAL("IOManager3::_initPort: could not change baudrate");
      ROS_FATAL("IOManager3::_initPort: are you stupid enough, dass du unfaehig zu eroeffnenung des Port bist?");
      std::abort();
  }

  uint8_t byte_buffer;
  timer a;
  a.tic();
  int count;
  while(1)
  {
    INFO("start reading!");
    count = 0;
    while(!imu_port->readPort(&byte_buffer, 1))
    {
      timer::delay_us(100);
      count++;
    }
    INFO("read success");
    std::cout << "read times:" << count << std::endl;
    timer::delay_ms(2);
    INFO("delay 2ms");
    imu_port->clearPort();
    INFO("clear port");
    a.toc();
    a.tic();
  }

}
