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
int main(int argc, char ** argv)
{
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     ros::init(argc, argv, "testing");
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     ros::NodeHandle nh("~");
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     parameters.init(&nh);
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     Motion::IOManager3 io;
       Motion::ServoIO servo_test;
//
// /////////////////////////////////////////////////////////////////////////
// ///test fot IOManager3
// /////////////////////////////////////////////////////////////////////////
//     ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
//     // std::vector<double> fucking(16,0);
//     // for(int i = 0; i < 12; i++ ){
//     //   std::cout << "第" << i << "个:" << fucking[i] << std::endl;
//     // }
//     // sleep(1);
//     io.reverseMotion();

    // std::vector<double> tit(10,0);
    // std::vector<double> valuex(10,0);
    // std::vector<double> valuez(10,0);
    // std::vector<double> valuexd(10,0);
    // std::vector<double> valuezd(10,0);
    // for (int i=0; i<10;i++)
    // {
    //   tit[i] = 3*i/300;
    //   valuex[i] = 2*std::sin(i*2*M_PI/10);
    //   valuez[i] = 2*std::cos(i*2*M_PI/10);
    //   valuexd[i] = 2*std::cos(i*2*M_PI/10);
    //   valuezd[i] = -2*std::sin(i*2*M_PI/10);
    // }
    //
    // // ThreeInterpolation x_offset(tit,valuex,valuexd);
    // // ThreeInterpolation z_offset(tit,valuez,valuezd);
    // // x_offset.CalculatePoints(11);
    // // z_offset.CalculatePoints(11);
    //
    // std::vector<double>  servo_offsetx =x_offset.GetPoints();
    // std::vector<double> servo_offsetz  = z_offset.GetPoints();
    // std::vector<double> servo_times = x_offset.GetTimes();
    // //dmotion::PrintVector(servo_times);
    // //sleep(100);
    // // dmotion::PrintVector(servo_offsetx);
    // // sleep(100);
    // InvKin leg(true);
    // cout << servo_offsetx.size() << endl<< endl<< endl<< endl<< endl;
    // sleep(3);
    // for (unsigned i = 0 ;i<servo_times.size();i++)
    // {
    //   double tmp_x = servo_offsetx[i];
    //   double tmp_z = servo_offsetz[i]-28;
    //   std::vector<double> manipulate_points  = {tmp_x,-4.5,tmp_z,0,0,0};
    //   std::vector<double> servo_points = leg.LegInvKin(manipulate_points);
    //
    //
    //   fucking[0] = servo_points[0];
    //   fucking[1] = servo_points[1];
    //   fucking[2] = servo_points[2];
    //   fucking[3] = servo_points[3];
    //   fucking[4] = servo_points[4];
    //   fucking[5] = servo_points[5];
    //   io.setAllJointValue(fucking);
    //   io.spinOnce();
    //   if(i == 0)
    //     sleep(5);
    //
    //   cout << i << endl<< endl<< endl<< endl;
    //
    // }




    // io.reverseMotion();
    // while(ros::ok()){
    //   for(int i = 0; i < 12; i++){
    //     fucking[11] = fucking[11] - 1;
    //     fucking[4] = fucking[4] - 1;
    //     io.setAllJointValue(fucking);
    //     io.spinOnce();
    //   }
    //
    //   for(int i = 0; i < 12; i++){
    //     fucking[11] = fucking[11] + 1;
    //     fucking[4] = fucking[4] + 1;
    //     io.setAllJointValue(fucking);
    //     io.spinOnce();
    //   }
    //}
/////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////
///test fot ServoIO
////////////////////////////////////////////////////////////////////////

    // struct Motion::JointConfig _cfg1(14,-1,4096,203,4096,0);
    // struct Motion::Joint joints_cfg1(_cfg1);
    // servo_test.addJoint("gg", joints_cfg1);
    // struct Motion::JointConfig _cfg2(13,-1,4096,156,4096,0);
    // struct Motion::Joint joints_cfg2(_cfg2);
    // servo_test.addJoint("tt", joints_cfg2);

    // Motion::ServoIO servo_test;

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



//right feet
//ankel_roll and knee is the same
//others are not
//the following has changed to right cw
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

    // struct Motion::JointConfig _cfg1(2,1,4096,157,4096,0);
    // struct Motion::Joint joints_cfg1(_cfg1);
    // servo_test.addJoint("right_hip_pitch", joints_cfg1);

    //
    //
    servo_test.initServoPositions();
    sleep(2);
    servo_test.TorqueOff();
    while(1){
    // //   for(int i = 0; i < 23; i++){
    // //      servo_test.setSingleServoPosition("right_knee", i);
    // //      servo_test.sendServoPositions();
    // //      timer::delay_ms(20);
    // //      servo_test.readServoPositions();
    // //   }
    // //
    // //   for(int i = 23; i > 0; i--){
    // //      servo_test.setSingleServoPosition("right_knee", i);
    // //      servo_test.sendServoPositions();
    // //      timer::delay_ms(20);
    // //      servo_test.readServoPositions();
    // //   }
    //   INFO("fuck!!!!!!");
      servo_test.readServoPositions();
      timer::delay_ms(1000);
    }


    INFO("FUCKING NEXT!");
    // servo_test.setSingleServoPosition("gg", -1);
    // servo_test.sendServoPositions();
    //
    sleep(5);
    INFO("FUCKING!");

}
