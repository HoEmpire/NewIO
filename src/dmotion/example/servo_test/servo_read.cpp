#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "../../include/dmotion/IO/ServoIO.h"
#include <iostream>

using namespace std;
int main(int argc, char ** argv)
{
    Motion::ServoIO servo_test;

    INFO("FUCKTION WELL!");

    struct Motion::JointConfig _cfg12(14,-1,4096,203,4096,0);
    struct Motion::Joint joints_cfg12(_cfg12);
    servo_test.addJoint("left_ankle_roll", joints_cfg12);

    struct Motion::JointConfig _cfg11(13,-1,4096,156,4096,0);
    struct Motion::Joint joints_cfg11(_cfg11);
    servo_test.addJoint("left_ankle_pitch", joints_cfg11);

    struct Motion::JointConfig _cfg10(12,1,4096,157,4096,0);
    struct Motion::Joint joints_cfg10(_cfg10);
    servo_test.addJoint("left_knee", joints_cfg10);

    struct Motion::JointConfig _cfg9(11,-1,4096,242,4096,0);
    struct Motion::Joint joints_cfg9(_cfg9);
    servo_test.addJoint("left_hip_yaw", joints_cfg9);

    struct Motion::JointConfig _cfg8(10,-1,4096,202,4096,0);
    struct Motion::Joint joints_cfg8(_cfg8);
    servo_test.addJoint("left_hip_roll", joints_cfg8);

    struct Motion::JointConfig _cfg7(9,1,4096,202,4096,0);
    struct Motion::Joint joints_cfg7(_cfg7);
    servo_test.addJoint("left_hip_pitch", joints_cfg7);

    struct Motion::JointConfig _cfg6(7,-1,4096,158,4096,0);
    struct Motion::Joint joints_cfg6(_cfg6);
    servo_test.addJoint("right_ankle_roll", joints_cfg6);

    struct Motion::JointConfig _cfg5(6,-1,4096,201,4096,0);
    struct Motion::Joint joints_cfg5(_cfg5);
    servo_test.addJoint("right_ankle_pitch", joints_cfg5);

    struct Motion::JointConfig _cfg4(5,-1,4096,202,4096,0);
    struct Motion::Joint joints_cfg4(_cfg4);
    servo_test.addJoint("right_knee", joints_cfg4);

    struct Motion::JointConfig _cfg3(4,1,4096,181,4096,0);
    struct Motion::Joint joints_cfg3(_cfg3);
    servo_test.addJoint("right_hip_yaw", joints_cfg3);

    struct Motion::JointConfig _cfg2(3,1,4096,158,4096,0);
    struct Motion::Joint joints_cfg2(_cfg2);
    servo_test.addJoint("right_hip_roll", joints_cfg2);

    struct Motion::JointConfig _cfg1(2,1,4096,157,4096,0);
    struct Motion::Joint joints_cfg1(_cfg1);
    servo_test.addJoint("right_hip_pitch", joints_cfg1);

    servo_test.initServoPositions();
    sleep(2);
    servo_test.TorqueOff();
    timer a,b;
    while(1){
      a.tic();
      b.tic();
      servo_test.readServoVelocity();
      servo_test.readServoPositions_test();
      INFO("*****************************");
      a.toc();
      INFO("*****************************");
      b.smartDelay_ms(10.0);
    }

}
