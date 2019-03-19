//
//这个例程是用来使得机器人右腿进行画圆运动的
//Auther: Yuxiang Pen & Wu Fan
//E-mail: zjufanwu@zju.edu.cn
//
#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "dmotion/InverseKinematics/InverseKinematics.h"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"
#include "dmotion/ThreeInterpolation/ThreeInterpolation.h"
#include "dmotion/Utility/dmotion_math.hpp"

#include <iostream>
#include <unistd.h>
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");

    parameters.init(&nh);
    Motion::IOManager3 io;

    std::vector<double> fucking(16, 0);
    std::vector<double> dx(16, 0);
    for (int i = 0; i < 12; i++)
    {
        std::cout << "第" << i << "个:" << fucking[i] << std::endl;
    }
    sleep(1);

    //第一步，生成样本点的序列（把一个圆圈分为10份）
    int sample_num = 11;     //从始至终需要11个点
    double whole_time = 1.0; //完成整个动作需要3秒
    double radio = 4.0;
    std::vector<double> tit(sample_num, 0);
    std::vector<double> valuex(sample_num, 0);
    std::vector<double> valuez(sample_num, 0);
    std::vector<double> valuexd(sample_num, 0);
    std::vector<double> valuezd(sample_num, 0);
    for (int i = 0; i < sample_num; i++)
    {
        tit[i] = whole_time * i / (sample_num - 1);
        valuex[i] = radio * std::sin(i * 2.0 * M_PI / (sample_num - 1));
        valuez[i] = radio * std::cos(i * 2.0 * M_PI / (sample_num - 1));
        valuexd[i] = 2.0 * M_PI * radio / whole_time * std::cos(i * 2.0 * M_PI / (sample_num - 1));
        valuezd[i] = -2.0 * M_PI * radio / whole_time * std::sin(i * 2.0 * M_PI / (sample_num - 1));
    }

    ThreeInterpolation x_offset(tit, valuex, valuexd);
    ThreeInterpolation z_offset(tit, valuez, valuezd);
    x_offset.CalculatePoints(10);
    z_offset.CalculatePoints(10);//TODO 小心坑

    std::vector<double> servo_offsetx = x_offset.GetPoints();
    std::vector<double> servo_offsetz = z_offset.GetPoints();
    std::vector<double> servo_times = x_offset.GetTimes();
    //dmotion::PrintVector(servo_times);
    //sleep(100);
    // dmotion::PrintVector(servo_offsetx);
    // sleep(100);
    InvKin leg(true);

    sleep(3);

    int flag = 0;
    // io.setAllTimeBase();
    // sleep(1);
    // io.setAllspeed(10);
    timer a;
    std::vector<double> vel(16,0);
    std::vector<double> pos(16,0);
    double tmp_x,tmp_z,tmp_vx,tmp_vz;
    std::vector<double> manipulate_points, manipulate_vel;
    std::vector<double> servo_points;

    tmp_x = servo_offsetx[0];
    tmp_z = servo_offsetz[0] - 33;
    manipulate_points = {tmp_x, -4.5, tmp_z, 0, 0, 0};
    servo_points = leg.LegInvKin(manipulate_points);
    for(int j = 0;j <= 5; j++)
    {
      fucking[j] = servo_points[j];
      pos[j] = servo_points[j];
    }
    io.SetAllJointValueTimeBase(fucking, false);
    io.spinOnce();

    sleep(5);

    while (ros::ok())
    {
        for (unsigned i = 0; i < servo_times.size(); i++)
        {
            tmp_x = servo_offsetx[i];
            tmp_z = servo_offsetz[i] - 33;
            manipulate_points = {tmp_x, -4.5, tmp_z, 0, 0, 0};
            servo_points = leg.LegInvKin(manipulate_points);

            // for(int j = 0;j <= 5; j++)
            // {
            //   fucking[j] = 2 * servo_points[j] - pos[j] - vel[j] * 0.005;
            //   vel[j] = 2 * (servo_points[j] - pos[j]) / 0.01 - vel[j];
            //   pos[j] = servo_points[j];
            // }

            for(int j = 0;j <= 5; j++)
            {
              fucking[j] = servo_points[j];
            }

            //io.SetAllJointValueTimeBase(fucking, true);
            io.SetAllJointValue(fucking);
            io.spinOnce();
            io.readPosVel();
            // heihei = io.readAllVel();
            // gg = io.readAllPosition();
            for(int j = 0;j <= 5;j++)
              cout << fucking[j] << "," << servo_points[j] <<endl;
            if(!ros::ok())
              break;
        }
    }
}
