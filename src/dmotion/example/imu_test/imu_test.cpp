#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "dmotion/IO/IMUReader.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sched.h>

using namespace std;
int main(int argc, char ** argv)
{


    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");
    parameters.init(&nh);
    Motion::IMUReader imu;

#if 1
    pid_t pid = getpid();
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_RR);   // 也可用SCHED_RR
    sched_setscheduler(pid, SCHED_RR, &param);                   // 设置当前进程
    pthread_setschedparam(pthread_self(), SCHED_RR, &param);   // 设置当前线程
#endif
    timer a;
    while(ros::ok()){
      //a.tic();
      imu.readIMUData();
      //a.toc();
      //imu.test_imu();
      imu.getIMUData();
      //timer::delay_ms(6);

    }
}
