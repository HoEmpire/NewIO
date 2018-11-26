// #include "dmotion/IO/IOManager3.h"
// #include <iostream>
// #include <ros/ros.h>
// #include "dmotion/Common/Utility/Utility.h"
// #include "dmotion/Common/Parameters.h"
// #include "dmotion/InverseKinematics/InverseKinematics.h"
// #include "dmotion/ThreeInterpolation/ThreeInterpolation.h"
// #include "dmotion/Utility/dmotion_math.hpp"
// #include "../../include/dmotion/IO/ServoIO.h"
//
// #include <iostream>
// #include <unistd.h>
// using namespace std;
// #include <iostream>
// #include <ros/ros.h>
// #include "dmotion/Common/Utility/Utility.h"
// #include "../../include/dmotion/IO/ServoIO.h"
// #include <iostream>
//
// using namespace std;
// int main(int argc, char ** argv)
// {
//       ros::init(argc, argv, "testing");
//       ros::NodeHandle nh("~");
//       nh.setParam("PowerState", false);
//       while(ros::ok()){
//         timer::delay_ms(10);
//       }
// }

#include <iostream>
#include <thread>

using namespace std;

void output(int i)
{
    cout << i << endl;
}

int main(int argc, char ** argv)
{

    for (uint8_t i = 0; i < 4; i++)
    {
        thread t(output, i);
        t.detach();
    }

    getchar();
    return 0;
}



// using namespace std;
// int main(int argc, char ** argv)
// {
//     Motion::ServoIO servo_test;
//
//     INFO("FUCKTION WELL!");
//
//     struct Motion::JointConfig _cfg12(14,-1,4096,203,4096,0);
//     struct Motion::Joint joints_cfg12(_cfg12);
//     servo_test.addJoint("left_ankle_roll", joints_cfg12);
//
//     struct Motion::JointConfig _cfg11(13,-1,4096,156,4096,0);
//     struct Motion::Joint joints_cfg11(_cfg11);
//     servo_test.addJoint("left_ankle_pitch", joints_cfg11);
//
//     struct Motion::JointConfig _cfg10(12,1,4096,157,4096,0);
//     struct Motion::Joint joints_cfg10(_cfg10);
//     servo_test.addJoint("left_knee", joints_cfg10);
//
//     struct Motion::JointConfig _cfg9(11,-1,4096,242,4096,0);
//     struct Motion::Joint joints_cfg9(_cfg9);
//     servo_test.addJoint("left_hip_yaw", joints_cfg9);
//
//     struct Motion::JointConfig _cfg8(10,-1,4096,202,4096,0);
//     struct Motion::Joint joints_cfg8(_cfg8);
//     servo_test.addJoint("left_hip_roll", joints_cfg8);
//
//     struct Motion::JointConfig _cfg7(9,1,4096,202,4096,0);
//     struct Motion::Joint joints_cfg7(_cfg7);
//     servo_test.addJoint("left_hip_pitch", joints_cfg7);
//
//     struct Motion::JointConfig _cfg6(7,-1,4096,158,4096,0);
//     struct Motion::Joint joints_cfg6(_cfg6);
//     servo_test.addJoint("right_ankle_roll", joints_cfg6);
//
//     struct Motion::JointConfig _cfg5(6,-1,4096,201,4096,0);
//     struct Motion::Joint joints_cfg5(_cfg5);
//     servo_test.addJoint("right_ankle_pitch", joints_cfg5);
//
//     struct Motion::JointConfig _cfg4(5,-1,4096,202,4096,0);
//     struct Motion::Joint joints_cfg4(_cfg4);
//     servo_test.addJoint("right_knee", joints_cfg4);
//
//     struct Motion::JointConfig _cfg3(4,1,4096,181,4096,0);
//     struct Motion::Joint joints_cfg3(_cfg3);
//     servo_test.addJoint("right_hip_yaw", joints_cfg3);
//
//     struct Motion::JointConfig _cfg2(3,1,4096,158,4096,0);
//     struct Motion::Joint joints_cfg2(_cfg2);
//     servo_test.addJoint("right_hip_roll", joints_cfg2);
//
//     struct Motion::JointConfig _cfg1(2,1,4096,157,4096,0);
//     struct Motion::Joint joints_cfg1(_cfg1);
//     servo_test.addJoint("right_hip_pitch", joints_cfg1);
//
//     servo_test.initServoPositions();
//     sleep(2);
//     servo_test.TorqueOff();
//     static int PowerOffCount = 0;
//     timer a;
//     while(PowerOffCount <= 10){
//
//       a.tic();
//       if(!servo_test.checkPower()){
//           PowerOffCount++;
//           continue;
//       }
//       INFO("*****************************");
//       a.toc();
//       INFO("*****************************");
//       timer::delay_ms(1000);
//
//     }
//
//     INFO("FUCK YOU BRO");
//
// }

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
//     Motion::FeetSensorIO m_feet_io;
//   //  m_feet_io.setPort(m_servo_io.getPortHandler());
//     while(ros::ok()){
//     //  m_servo_io.sendServoPositions();
//       if (!m_feet_io.readPressureData())
//       {
//           ROS_WARN("IOManager3::spinOnce: read feet pressure data error");
//       }
//       sleep(1);
//     }
//
// }

// int main(int argc, char ** argv)
// {
//       ros::init(argc, argv, "testing");
//       ros::NodeHandle nh("~");
//       parameters.init(&nh);
//       Motion::IOManager3 io;
//       std::vector<double> fucking(16);
//       ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//       int cunt = 0;
//       double zeit = 0;
//       //read constantly
//       io.ServoPowerOff();
//       while(ros::ok()){
//         timer a;
//         io.readJointValue();
//         zeit = a.toc();
//         cout << "***********************************" << endl;
//         cout << "[read time]:" << zeit << endl << endl;
//         cout << "***********************************" << endl;
//         sleep(1);
//       }
//
// }


// int main(int argc, char ** argv)
// {
//     Motion::ServoIO servo_test;
//
//     INFO("FUCKTION WELL!");
//
//     struct Motion::JointConfig _cfg12(14,-1,4096,203,4096,0);
//     struct Motion::Joint joints_cfg12(_cfg12);
//     servo_test.addJoint("left_ankle_roll", joints_cfg12);
//
//     struct Motion::JointConfig _cfg11(13,-1,4096,156,4096,0);
//     struct Motion::Joint joints_cfg11(_cfg11);
//     servo_test.addJoint("left_ankle_pitch", joints_cfg11);
//
//     struct Motion::JointConfig _cfg10(12,1,4096,157,4096,0);
//     struct Motion::Joint joints_cfg10(_cfg10);
//     servo_test.addJoint("left_knee", joints_cfg10);
//
//     struct Motion::JointConfig _cfg9(11,-1,4096,242,4096,0);
//     struct Motion::Joint joints_cfg9(_cfg9);
//     servo_test.addJoint("left_hip_yaw", joints_cfg9);
//
//     struct Motion::JointConfig _cfg8(10,-1,4096,202,4096,0);
//     struct Motion::Joint joints_cfg8(_cfg8);
//     servo_test.addJoint("left_hip_roll", joints_cfg8);
//
//     struct Motion::JointConfig _cfg7(9,1,4096,202,4096,0);
//     struct Motion::Joint joints_cfg7(_cfg7);
//     servo_test.addJoint("left_hip_pitch", joints_cfg7);
//
//     struct Motion::JointConfig _cfg6(7,-1,4096,158,4096,0);
//     struct Motion::Joint joints_cfg6(_cfg6);
//     servo_test.addJoint("right_ankle_roll", joints_cfg6);
//
//     struct Motion::JointConfig _cfg5(6,-1,4096,201,4096,0);
//     struct Motion::Joint joints_cfg5(_cfg5);
//     servo_test.addJoint("right_ankle_pitch", joints_cfg5);
//
//     struct Motion::JointConfig _cfg4(5,-1,4096,202,4096,0);
//     struct Motion::Joint joints_cfg4(_cfg4);
//     servo_test.addJoint("right_knee", joints_cfg4);
//
//     struct Motion::JointConfig _cfg3(4,1,4096,181,4096,0);
//     struct Motion::Joint joints_cfg3(_cfg3);
//     servo_test.addJoint("right_hip_yaw", joints_cfg3);
//
//     struct Motion::JointConfig _cfg2(3,1,4096,158,4096,0);
//     struct Motion::Joint joints_cfg2(_cfg2);
//     servo_test.addJoint("right_hip_roll", joints_cfg2);
//
//     struct Motion::JointConfig _cfg1(2,1,4096,157,4096,0);
//     struct Motion::Joint joints_cfg1(_cfg1);
//     servo_test.addJoint("right_hip_pitch", joints_cfg1);
//
//     servo_test.initServoPositions();
//     sleep(2);
//     servo_test.TorqueOff();
//     double zeit;
//     while(1){
//       timer a;
//       servo_test.readServoPositions();
//       zeit = a.toc();
//       cout << "***********************************" << endl;
//       cout << "[read time]:" << zeit << endl << endl;
//       cout << "***********************************" << endl;
//       //sleep(1);
//     }
// }

// int main(int argc, char ** argv)
// {
//       ros::init(argc, argv, "testing");
//       ros::NodeHandle nh("~");
//       parameters.init(&nh);
//       Motion::IOManager3 io;
//       std::vector<double> fucking(16,0);
//       ROS_INFO("!!!!!!!!!!!!!!!!！FUCK!!!!!!!!!!!!!!!!");
//       timer a;
//       double zeit;
//       while(ros::ok()){
//         ROS_INFO("gg!!!!!!!");
//         for(int i = 0; i < 50; i++){
//            for(int j = 0; j < 12; j++)
//                 fucking[j] = fucking[j] - 0.1;
//           io.setAllJointValue(fucking);
//           a.tic();
//           io.spinOnce();
//           io.readJointValue();
//           zeit = a.toc();
//           cout << "***********************************" << endl;
//           cout << "[read time]:" << zeit << endl << endl;
//           cout << "***********************************" << endl;
//         }
//
//         for(int i = 0; i < 50; i++){
//           for(int j = 0; j < 12; j++)
//                fucking[j] = fucking[j] + 0.1;
//           io.setAllJointValue(fucking);
//           a.tic();
//           io.spinOnce();
//           io.readJointValue();
//           zeit = a.toc();
//           cout << "***********************************" << endl;
//           cout << "[read time]:" << zeit << endl << endl;
//           cout << "***********************************" << endl;
//         }
//
//       }
//
// }
