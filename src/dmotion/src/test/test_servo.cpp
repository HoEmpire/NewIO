#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include <iostream>
#include <unistd.h>
static void updateJointValuesIO(Motion::IOManager3& io,const std::vector<double>& values_);

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");
    parameters.init(&nh);
    Motion::IOManager3 io;

    //以下代码作为测试使用
    //功能为旋转来回旋转末端
    ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
    std::vector<double> fucking(12,0);
    fucking[11] = 0.75;
    for(int i = 0; i < 12; i++ ){
      std::cout << "第" << i << "个:" << fucking[i] << std::endl;
    }
    sleep(1);

    while(ros::ok()){
      for(int i = 0; i < 75; i++){
        fucking[11] = fucking[11] - 0.01;
        updateJointValuesIO(io,fucking);
      }

      for(int i = 0; i < 75; i++){
        fucking[11] = fucking[11] + 0.01;
        updateJointValuesIO(io,fucking);
      }
    }
    // ServoIO m_servo_io;
    //
    // while(ros::ok()){
    //   for(int i = 0; i < 75; i++){
    //     fucking[11] = fucking[11] - 0.01;
    //     updateJointValuesIO(io,fucking);
    //   }
    //
    //   for(int i = 0; i < 75; i++){
    //     fucking[11] = fucking[11] + 0.01;
    //     updateJointValuesIO(io,fucking);
    //   }
    // }

}


static void updateJointValuesIO(Motion::IOManager3& io,
                                const std::vector<double>& values_)
{
    io.setJointValue("right_arm_upper", DEG2RAD(-40.0));
    io.setJointValue("right_arm_lower", DEG2RAD(60.0));
    io.setJointValue("left_arm_upper", DEG2RAD(-40.0));
    io.setJointValue("left_arm_lower", DEG2RAD(60.0));

    io.setJointValue("right_hip_yaw", values_[0]);
    io.setJointValue("right_hip_roll", values_[1]);
    io.setJointValue("right_hip_pitch", values_[2]);
    io.setJointValue("right_knee", values_[3]);
    io.setJointValue("right_ankle_pitch", values_[4]);
    io.setJointValue("right_ankle_roll", values_[5]);

    io.setJointValue("left_hip_yaw", values_[6]);
    io.setJointValue("left_hip_roll", values_[7]);
    io.setJointValue("left_hip_pitch", values_[8]);
    io.setJointValue("left_knee", values_[9]);
    io.setJointValue("left_ankle_pitch", values_[10]);
    io.setJointValue("left_ankle_roll", values_[11]);

    io.spinOnce();
}
