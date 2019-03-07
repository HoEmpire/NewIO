#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "dmotion/InverseKinematics/InverseKinematics.h"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"
#include "dmotion/Utility/dmotion_math.hpp"
#include "PendulumWalk.h"
#include <iostream>
using namespace std;
using namespace dmotion;
template <class T1>
std::vector<T1> twointerpolation(std::vector<T1> points, T1 v0, int time_points,int time_io)
{
  int i;
  T1 a0, a1, a2, v;
  v = v0;
  int n = time_points / time_io;
  std::vector<T1> x;
  int t;

  for(unsigned j = 0; j < points.size() - 1; j++)
  {

    a0 = points[j];
    a1 = v;
    a2 = (points[j+1] - a0 - a1 * time_points) / (time_points * time_points);
    for(i = 0; i < n; i++)
    {
      t = i * time_io;
      x.push_back(a0 + a1 * t + a2 * t * t);
    }
    v = a1 + 2 * a2 * time_points;
  }
  t = i * time_io;
  x.push_back(a0 + a1 * t + a2 * t * t);
  //cout <<  "x = " << x.size() << endl;
  return x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gg");
    ros::NodeHandle nh("~");

    parameters.init(&nh);
    Motion::IOManager3 io;
    PendulumWalk pen;
    OneFootLanding a(false);

    std::vector<int> servo_id = {6,7};
    io.setServoPI(servo_id, 800, 0);

    INFO("FUCK");
    std::vector<double> fucking(16, 0);
    //第一步，生成样本点的序列（把一个圆圈分为10份）
    int sample_num = 100;     //从始至终需要11个点
    double whole_time = 1000; //完成整个动作需要1000  ms
    int points_time = whole_time/sample_num; //完成整个动作需要3秒
    double R = 4.0;
    double tmp_x,tmp_z;
    std::vector<double> tit(sample_num, 0);
    std::vector<double> valuex(sample_num, 0);
    std::vector<double> valuez(sample_num, 0);
    std::vector<double> valuedx(sample_num, 0);
    std::vector<double> valuedz(sample_num, 0);

    std::vector<double> tmp,servo_points,dx(16,0),dv(16,0),da(16,0),servo_vel;

    Eigen::Matrix<double,6,1> temp_acc;
    Eigen::Matrix<double,6,6> Jb;

    for (int i = 0; i <= sample_num; i++)
    {
        valuex[i] = R * sin(i * 2.0 * M_PI / sample_num);
        valuez[i] = R * cos(i * 2.0 * M_PI / sample_num) - 33;//TODO
        valuedx[i] = 2.0 * M_PI * R / whole_time * cos(i * 2.0 * M_PI / (sample_num - 1));
        valuedz[i] = -2.0 * M_PI * R / whole_time * sin(i * 2.0 * M_PI / (sample_num - 1));
    }

    std::vector<std::vector<double> > joint_value, joint_vel;
    dmotion::InvKin leg(true);

    std::vector<double> manipulate_points,manipulate_vel;
    std::vector<double> servo1(sample_num + 1, 0),servo2(sample_num + 1, 0),servo3(sample_num + 1, 0),
                        servo4(sample_num + 1, 0),servo5(sample_num + 1, 0),servo0(sample_num + 1, 0);

    std::vector<double> servo1v(sample_num + 1, 0),servo2v(sample_num + 1, 0),servo3v(sample_num + 1, 0),
                        servo4v(sample_num + 1, 0),servo5v(sample_num + 1, 0),servo0v(sample_num + 1, 0);

    //std::vector<double> out0,out1,out2,out3,out4,out5;

    for(int j = 0; j <= sample_num; j++)
    {
        tmp_x = valuex[j];
        tmp_z = valuez[j];
        manipulate_points = {tmp_x, -4.5, tmp_z, 0, 0, 0};
        servo_points = leg.LegInvKin(manipulate_points);

        tmp_x = valuedx[j];
        tmp_z = valuedz[j];
        manipulate_vel = {tmp_x, -4.5, tmp_z, 0, 0, 0};
        ForKin right(manipulate_points, true);
        right.calVelocity(manipulate_vel);
        servo_vel =  right.resultv_vector;

        servo0[j] = servo_points[0];
        servo1[j] = servo_points[1];
        servo2[j] = servo_points[2];
        servo3[j] = servo_points[3];
        servo4[j] = servo_points[4];
        servo5[j] = servo_points[5];

        servo0v[j] = servo_vel[0];
        servo1v[j] = servo_vel[1];
        servo2v[j] = servo_vel[2];
        servo3v[j] = servo_vel[3];
        servo4v[j] = servo_vel[4];
        servo5v[j] = servo_vel[5];
    }

        INFO("FUCK");
        //joint_value.push_back(tmp);
        tmp = twointerpolation(servo0, 0.0, points_time,10);
        joint_value.push_back(tmp);
        tmp = twointerpolation(servo1, 0.0, points_time,10);
        joint_value.push_back(tmp);
        tmp = twointerpolation(servo2, 0.0, points_time,10);
        joint_value.push_back(tmp);
        tmp = twointerpolation(servo3, 0.0, points_time,10);
        joint_value.push_back(tmp);
        tmp = twointerpolation(servo4, 0.0, points_time,10);
        joint_value.push_back(tmp);
        tmp = twointerpolation(servo5, 0.0, points_time,10);
        joint_value.push_back(tmp);

        tmp = twointerpolation(servo0v, 0.0, points_time,10);
        joint_vel.push_back(tmp);
        tmp = twointerpolation(servo1v, 0.0, points_time,10);
        joint_vel.push_back(tmp);
        tmp = twointerpolation(servo2v, 0.0, points_time,10);
        joint_vel.push_back(tmp);
        tmp = twointerpolation(servo3v, 0.0, points_time,10);
        joint_vel.push_back(tmp);
        tmp = twointerpolation(servo4v, 0.0, points_time,10);
        joint_vel.push_back(tmp);
        tmp = twointerpolation(servo5v, 0.0, points_time,10);
        joint_vel.push_back(tmp);
    // for(int i = 0; i < joint_value[0].size(); i++)
    // {
    //   cout << joint_value[3][i] << endl;
    // }

    std::vector<double> vel(16,0);
    std::vector<double> pos(16,0);
    std::vector<double> read_pos(16,0);
    std::vector<double> read_vel(16,0);
    std::vector<double> servo_real(6,0);
    std::vector<double> servo_desire(6,0);
    std::vector<double> pos_real(6,0);
    std::vector<double> pos_desire(6,0);
    std::vector<double> vel_real;
    std::vector<double>  vel_desire;
    std::vector<double> servo_vel_real(6,0);
    std::vector<double> servo_vel_desire(6,0);


    for(int j = 0;j <= 5; j++)
    {
      fucking[j] = joint_value[j][0];
      pos[j] = joint_value[j][0];
      vel[j] = joint_vel[j][0];
    //  cout << vel[j] << endl;
    }
    //abort();

    io.setAllJointValue(fucking);
    io.spinOnce();
    sleep(5);
    double Kp = 600,Kd = 100,Kpv = 5000;
    int flag = 0;

     while (ros::ok())
     {
        for (unsigned i = 0; i < joint_value[0].size(); i++)
        {

            for(int j = 0;j <= 5; j++)
            {
              fucking[j] = joint_value[j][i];
            }
            tmp =  fucking;
            //
            for(int j = 0;j <= 5; j++)
            {
              fucking[j] = 2 * tmp[j] - pos[j] - vel[j] * 0.005;
              //vel[j] = 2 * (tmp[j] - pos[j]) / 0.01 - vel[j];
              pos[j] = tmp[j];
              if(flag == 1)
              {
                fucking[j] = fucking[j] - da[j] * 0.01* 0.01;
                cout << "fuck" << endl;
              }
            }
            if(flag == 1)
                flag = 0;

            //
            io.setAllJointValue(fucking);
            io.spinOnce();
            io.readPosVel();
            read_pos = io.readAllPosition();
            read_vel = io.readAllVel();


            for(int m = 0; m < 6; m++ )
            {
              servo_real[m] = read_pos[m];
              servo_desire[m] = pos[m];
              servo_vel_real[m] = read_vel[m];
              servo_vel_desire[m] = vel[m];
            }

            ForKin leg_real(servo_real,true);
            ForKin leg_desire(servo_desire,true);

            //**旧版读速度
            // Eigen::Matrix<double,6,6> J_real = leg_real.Jacobian();
            // Eigen::Matrix<double,6,6> J_desire = leg_desire.Jacobian();
            //
            // Eigen::Matrix<double,6,1> temp;
            //
            // for(int i = 0; i < 6; i++)
            // {
            //   temp(i) = servo_vel_real[i] / 180 * M_PI;
            // }
            // vel_real = J_real * temp;
            //
            // for(int i = 0; i < 6; i++)
            // {
            //   temp(i) = servo_vel_desire[i];
            // }
            // vel_desire = J_desire * temp;

            //**新版读速度
            leg_real.calVelocity(servo_vel_real);
            vel_real = leg_real.resultv_vector;
            pos_real = leg_real.result_vector;
            Jb = leg_real.Jacobian();

            leg_desire.calVelocity(servo_vel_desire);
            vel_desire = leg_desire.resultv_vector;
            pos_desire = leg_desire.result_vector;
            // cout << "理论位置" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << pos_desire[m] << ",";
            // }
            // cout << endl;
            // cout << "实际位置" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << pos_real[m] << ",";
            // }
            // cout << endl;

            /////**输出质心情况
            // cout << "理论位置" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << pos_desire[m] << ",";
            // }
            // cout << endl;
            //
            // cout << "实际位置" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << pos_real[m] << ",";
            // }
            // cout << endl;
            // cout << "偏差" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << pos_desire[m] - pos_real[m] << ",";
            // }
            // cout << endl;


            /////**输出速度误差
            // cout << std::setprecision (2);
            // cout << "理论速度" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << vel_desire(m) << ",";
            // }
            // cout << endl;
            // cout << "实际速度" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << vel_real[m] << ",";
            // }
            // cout << endl;

            /////**输出位置误差
            // cout << "理论位置" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << pos_desire[m] << ",";
            // }
            // cout << endl;
            // cout << "实际位置" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << pos_real[m] << ",";
            // }
            // cout << endl;
            // cout << "偏差" << endl;
            // for(int m = 0; m < 6; m++ )
            // {
            //   cout << pos_desire[m] - pos_real[m] << ",";
            // }
            // cout << endl;
            //


            // for(int k = 0; k < 6; k++)
            // {
            //   dx[k] = pos[k] - read_pos[k];
            //   dv[k] = vel[k] - read_vel[k];
            //   if(abs(dx[k]) > 6)
            //   {
            //     cout << "第" << k << "个超了, dx = " << dx[k] <<endl;
            //     cout << "理论位置：" << pos[k] << "应到位置：" << read_pos[k] <<endl;
            //     da[k] = Kp * dx[k] + Kd* dv[k];
            //     cout << "修正加速度：" << da[k] << "位置修正量：" << da[k] * 0.01* 0.01 <<endl;
            //     flag = 1;
            //   }
            // }
            temp_acc << 0,0,0,0,0,0;
            for(int k = 0; k < 2; k++)
            {
              dx[k] = pos_desire[k] - pos_real[k];
              dv[k] = vel_desire[k] - vel_real[k];

              if(abs(dx[k]) > 2)
              {
                cout << "第" << k << "个超了, dx = " << dx[k] <<endl;
                cout << "理论位置：" << pos[k] << "应到位置：" << read_pos[k] <<endl;
                temp_acc(k) = Kp * dx[k]; //+ Kd * dv[k];
                // da[k] = Kp * dx[k] + Kd * dv[k];

                // cout << "修正加速度：" << da[k] << "位置修正量：" << da[k] * 0.01* 0.01 <<endl;
                flag = 1;
              }
            }

            for(int k = 2; k < 3; k++)
            {
              dx[k] = pos_desire[k] - pos_real[k];
              dv[k] = vel_desire[k] - vel_real[k];

              if(abs(dx[k]) > 1)
              {
                cout << "第" << k << "个超了, dx = " << dx[k] <<endl;
                cout << "理论位置：" << pos[k] << "应到位置：" << read_pos[k] <<endl;
                temp_acc(k) = Kp * dx[k]; //+ Kd * dv[k];
                // da[k] = Kp * dx[k] + Kd * dv[k];

                // cout << "修正加速度：" << da[k] << "位置修正量：" << da[k] * 0.01* 0.01 <<endl;
                flag = 1;
              }
            }

            for(int k = 3; k < 6; k++)
            {
              dx[k] = (pos_desire[k] - pos_real[k]) / 180 * M_PI;
              dv[k] = (vel_desire[k] - vel_real[k]) / 180 * M_PI;

              if(abs(dx[k]) > 2.0 / 180 * M_PI)
              {
                cout << "第" << k << "个超了, dx = " << dx[k] * 180 / M_PI << endl;
                // cout << "理论位置：" << pos[k] << "应到位置：" << read_pos[k] <<endl;
                temp_acc(k) = Kpv * dx[k] ;// + Kd * dv[k];
                // da[k] = Kp * dx[k] + Kd * dv[k];

                // cout << "修正加速度：" << da[k] << "位置修正量：" << da[k] * 0.01* 0.01 <<endl;
                flag = 1;
              }
            }

              temp_acc = Jb.transpose() * temp_acc;
            if(flag == 1)
            {

              for(int k = 0; k < 6; k++)
              {
                da[k] = temp_acc(k);
                cout << da[k] * 0.01 * 0.01 << endl;
              }
            }
            if(!ros::ok())
                break;

        }
     }
}
