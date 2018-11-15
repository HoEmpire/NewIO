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
    struct Motion::JointConfig _cfg6(7,-1,4096,158,4096,0);
    struct Motion::Joint joints_cfg6(_cfg6);
    servo_test.addJoint("right_ankle_roll", joints_cfg6);

    struct Motion::JointConfig _cfg5(6,-1,4096,201,4096,0);
    struct Motion::Joint joints_cfg5(_cfg5);
    servo_test.addJoint("right_ankle_pitch", joints_cfg5);

    servo_test.initServoPositions();

    while(1){
        for(int i = 0; i < 23; i++){
           servo_test.setSingleServoPosition("right_ankle_roll", i);
           servo_test.setSingleServoPosition("right_ankle_pitch", i);
           servo_test.sendServoPositions();
           timer::delay_ms(10);
        }

        for(int i = 23; i > 0; i--){
           servo_test.setSingleServoPosition("right_ankle_roll", i);
           servo_test.setSingleServoPosition("right_ankle_pitch", i);
           servo_test.sendServoPositions();
           timer::delay_ms(10);
        }
    }

}
