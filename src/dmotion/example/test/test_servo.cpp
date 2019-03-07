// #include "dmotion/IO/IOManager3.h"
// #include <iostream>
// #include <ros/ros.h>
// #include "dmotion/Common/Utility/Utility.h"
// #include "dmotion/Common/Parameters.h"
// #include "../../include/dmotion/IO/IMUReader.h"
// // #include <stdio.h>
// // #include <stdlib.h>
// // #include <pthread.h>
// // #include <sched.h>
// #define PORT_NAME "/dev/IMU"
// #define BAUDRATE  576000
//
//
// using namespace std;
// int main(int argc, char ** argv)
// {
//   dynamixel::PortHandler* imu_port = dynamixel::PortHandler::getPortHandler(PORT_NAME);
//
//   if (!imu_port->setBaudRate(BAUDRATE,false))
//   {
//       ROS_FATAL("IOManager3::_initPort: could not change baudrate");
//       ROS_FATAL("IOManager3::_initPort: are you stupid enough, dass du unfaehig zu eroeffnenung des Port bist?");
//       std::abort();
//   }
//
//   uint8_t byte_buffer;
//   timer a;
//   a.tic();
//   int count;
//   while(1)
//   {
//     INFO("start reading!");
//     count = 0;
//     while(!imu_port->readPort(&byte_buffer, 1))
//     {
//       timer::delay_us(100);
//       count++;
//     }
//     INFO("read success");
//     std::cout << "read times:" << count << std::endl;
//     timer::delay_ms(2);
//     INFO("delay 2ms");
//     imu_port->clearPort();
//     INFO("clear port");
//     a.toc();
//     a.tic();
//   }
//
// }

// #include <iostream>
// #include <vector>
// #include <fstream>
// #include "dmotion/Common/Utility/Utility.h"
// #include <unistd.h>
// #define workplace "/home/tim/NewIO/test_data/test1"
//
// using namespace std;
// int main(int argc, char **argv) {
//
//   if(chdir(workplace))
//         exit(0);  //设置工作路径，也就是B显示的路径
//
//   std::vector<double> v;
//
//   v.push_back(1.2);
//   v.push_back(2.2);
//   int i = 1;
//   std::cout << int(v.size()) << std::endl;
//   std::cout << (i < int(v.size())) << std::endl;
//
//   ofstream out1;
//   out1.open("roll.txt", ios::out|ios::trunc);
//   if(!out1)
//       INFO("FUCK!!!!!!!!");
//   out1 << "fuck!!" << endl;
//   out1.close();
//
//
//
//   return 0;
// }

// #include <Eigen/Core>
// #include <iostream>
// #include "dmotion/State/imu_filter_new.hpp"
// using namespace std;
// using namespace Eigen;
// static inline void QuaternionToAngleAxis(const float * quaternion,
//                                          float* angle_axis) {
//   const float q1 = quaternion[1];
//   const float q2 = quaternion[2];
//   const float q3 = quaternion[3];
//   const float sin_squared_theta = q1 * q1 + q2 * q2 + q3 * q3;
//
//   // For quaternions representing non-zero rotation, the conversion
//   // is numerically stable.
//   if (sin_squared_theta > 0.0f) {
//     const float sin_theta = sqrt(sin_squared_theta);
//     const float cos_theta = quaternion[0];
//
//
//     const float two_theta =
//         2.0 * ((cos_theta < 0.0)
//                ? atan2(-sin_theta, -cos_theta)
//                : atan2(sin_theta, cos_theta));
//     const float k = two_theta / sin_theta;
//     angle_axis[0] = q1 * k;
//     angle_axis[1] = q2 * k;
//     angle_axis[2] = q3 * k;
//   } else {
//     // For zero rotation, sqrt() will produce NaN in the derivative since
//     // the argument is zero. By approximating with a Taylor series,
//     // and truncating at one term, the value and first derivatives will be
//     // computed correctly when Jets are used.
//     angle_axis[0] = q1 * 2.0f;
//     angle_axis[1] = q2 * 2.0f;
//     angle_axis[2] = q3 * 2.0f;
//   }
// }
//
// void QuaternionToEulerAngles(double qw, double qx, double qy, double qz)
// {
//     double roll, yaw, pitch;
//     roll = atan2f(2.f * (qz*qy + qw*qx), 1-2*(qx*qx+qy*qy)); //Z
//     pitch =  asinf(2.f * (qw*qy - qx*qz)); //Y
//     yaw = atan2f(2.f * (qx*qy + qw*qz), 1-2*(qy*qy+qz*qz));//X
//
//     cout << "roll = " << roll << endl;
//     cout << "pitch = " << pitch << endl;
//     cout << "yaw = " << yaw << endl;
// }
//
// //0.998183,0.0148911,-0.00673367,-1.26106e-11
// //0.7746,0.5164,0.2582,0.2582
// //0.7378    0.1337    1.2278
// int main(int argc, char **argv) {
//   float q1[4],rpy[3];
//
//   q1[0] = 0.7746;
//   q1[1] = 0.5164;
//   q1[2] = 0.2582;
//   q1[3] = 0.2582;
//   QuaternionToAngleAxis(q1,rpy);
//   cout << rpy[0] << endl << rpy[1] << endl << rpy[2] << endl;
//   Eigen::Quaterniond q;
//     q.x() = 0.0148911;
//     q.y() = 0.00673367;
//     q.z() = 0;
//     q.w() = 0.998183;
//     Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
//     cout << "Quaterniond2Euler result is:" <<endl;
//     cout << "x = "<< euler[2] << endl ;
//     cout << "y = "<< euler[1] << endl ;
//     cout << "z = "<< euler[0] << endl << endl;
//       q.x() = 0;
//       q.y() = 0;
//       q.z() = 0;
//       q.w() = 1;
//       euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
//       cout << "Quaterniond2Euler result is:" <<endl;
//       cout << "x = "<< euler[2] << endl ;
//       cout << "y = "<< euler[1] << endl ;
//       cout << "z = "<< euler[0] << endl << endl;
//       QuaternionToEulerAngles(0.998183,0.0148911,-0.00673367,-1.26106e-11);
//       QuaternionToEulerAngles(0.7746,0.5164,0.2582,0.2582);
// }
//

#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/IMUReader.h"
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
    Motion::FeetSensorIO Feet;

    timer a;
    while(ros::ok()){
      a.tic();
      Feet.readPressureData();
      a.SmartDelayMs(10.0);
    }
}
