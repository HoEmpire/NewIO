#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/State/imu_filter_new.hpp"

#include <Eigen/Geometry>
//
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/IMUReader.h"
#include "../../include/dmotion/IO/IOManager3.h"
#include "dmotion/State/StateManager.hpp"
#include "dmotion/ForwardKinematics/ForwardKinematics.h"

#include <thread>
#include <fstream>
#include <iostream>

#define workplace "/home/ubuntu/test/NewIO/test_data/test1"

using namespace std;
using namespace Eigen;
Motion::IMUData imu_data;
Motion::PowerState power_data;
Motion::PressureData pressure_data;
bool servo_state;
std::vector<double> read_pos(12,0);
std::vector<double> read_vel(12,0);

using namespace dmotion;
using namespace Motion;
int flag = 0;
// void tt()
// {
//     Motion::IMUReader imu;
//     while(ros::ok()){
//         imu.readIMUData();
//         imu_data = imu.getIMUData();
//     }
//
// }
//
// static inline void QuaternionToEulerAngles(float qw, float qx, float qy, float qz,
//                                            float& roll, float& pitch, float& yaw)
// {
//     roll = atan2f(2.f * (qz*qy + qw*qx), 1-2*(qx*qx+qy*qy)); //x
//     pitch =  asinf(2.f * (qw*qy - qx*qz)); //y
//     yaw = atan2f(2.f * (qx*qy + qw*qz), 1-2*(qy*qy+qz*qz));//z
// }
//
// int main(int argc, char ** argv)
// {
//     if(chdir(workplace))
//         exit(0);  //设置工作路径，也就是B显示的路径
//
//     ros::init(argc, argv, "testing");
//     ros::NodeHandle nh("~");
//     parameters.init(&nh);
//
//     ofstream out1("roll.txt", ios::out|ios::trunc);
//     ofstream out2("pitch.txt", ios::out|ios::trunc);
//     ofstream out3("yaw.txt", ios::out|ios::trunc);
//     ofstream out4("ax.txt", ios::out|ios::trunc);
//     ofstream out5("ay.txt", ios::out|ios::trunc);
//     ofstream out6("az.txt", ios::out|ios::trunc);
//     ofstream out7("wx.txt", ios::out|ios::trunc);
//     ofstream out8("wy.txt", ios::out|ios::trunc);
//     ofstream out9("wz.txt", ios::out|ios::trunc);
//
//     std::vector<float> roll,pitch,yaw;
//     std::vector<float> ax,ay,az;
//     std::vector<float> wx,wy,wz;
//
//     timer::time_point start,now;
//     std::chrono::duration<double> elapsed_seconds;
//
//     ImuFilter dick;
//     std::thread t1(tt);
//     t1.detach();
//     sleep(1);
//
//     long int count = 0;
//     int ini_ticks = 1;
//     timer a;
//     float lin_acc_x;
//     float lin_acc_y;
//     float lin_acc_z;
//     float ang_vel_x;
//     float ang_vel_y;
//     float ang_vel_z;
//     float x,y,z;
//     float q0,q1,q2,q3;
//     a.tic();
//     for(ini_ticks = 1; ini_ticks <= 1000; ini_ticks++)
//     {
//       lin_acc_x = imu_data.accl.x;
//       lin_acc_y = imu_data.accl.y;
//       lin_acc_z = imu_data.accl.z;
//       ang_vel_x = imu_data.gypo.x;
//       ang_vel_y = imu_data.gypo.y;
//       ang_vel_z = imu_data.gypo.z;
//       if(ini_ticks == 1)
//       {
//         dick.iniAcclast(lin_acc_x , lin_acc_y, lin_acc_z);
//       }
//       else
//       {
//         dick.iniIMU(ang_vel_x, ang_vel_y, ang_vel_z,
//                     lin_acc_x, lin_acc_y, lin_acc_z,
//                     ini_ticks);
//       }
//       start = timer::getCurrentSystemTime();
//       a.smartDelay_ms(2.0);
//       a.tic();
//     }
//
//     dick.iniGravity();
//     dick.iniQuaternion();
//     dick.getOrientation(q0, q1, q2, q3);
//     INFO("***********************");
//     INFO("Ini finished!!!");
//     INFO("***********************");
//     // Quaterniond Q1(q0, q1, q2, q3);
//     // Matrix3d R1;
//     // R1 = Q1.matrix();
//     // Vector3d Euler1 = R1.eulerAngles(2,1,0);
//     float x1,y1,z1;
//     // x1 = Euler1(2);
//     // y1 = Euler1(1);
//     // z1 = Euler1(0);
//     QuaternionToEulerAngles(q0,q1,q2,q3,x1,y1,z1);
//     cout << "x1: "<< x1 << " ,y1: " << y1 << ",z1: " << z1 << endl;
//
//     while(ros::ok())
//     {
//
//       lin_acc_x = imu_data.accl.x;
//       lin_acc_y = imu_data.accl.y;
//       lin_acc_z = imu_data.accl.z;
//       ang_vel_x = imu_data.gypo.x;
//       ang_vel_y = imu_data.gypo.y;
//       ang_vel_z = imu_data.gypo.z;
//
//       dick.Fusing(ang_vel_x, ang_vel_y, ang_vel_z,
//                   lin_acc_x, lin_acc_y, lin_acc_z);
//       dick.getOrientation(q0, q1, q2, q3);
//
//       // Quaterniond Q(q0, q1, q2, q3);
//       // Matrix3d R;
//       // R = Q.matrix();
//       // Vector3d Euler = R.eulerAngles(2,1,0);
//       // x = Euler(2);
//       // y = Euler(1);
//       // z = Euler(0);
//       //QuaternionToEulerAngles(q0,q1,q2,q3,x,y,z);
//       dick.getRPY(x,y,z);
//       roll.push_back(x);
//       pitch.push_back(y);
//       yaw.push_back(z);
//       ax.push_back(lin_acc_x);
//       ay.push_back(lin_acc_y);
//       az.push_back(lin_acc_x);
//       wx.push_back(ang_vel_x);
//       wy.push_back(ang_vel_y);
//       wz.push_back(ang_vel_z);
//
//       if(count % 1 == 0)
//       {
//           std::cout << "x = " << x << " ,y = " << y << " ,z = " << z << std::endl;
//       }
//
//       count++;
//
//       a.smartDelay_ms(2.0);
//       a.tic();
//     }
//
//     sleep(2);
//     INFO("***********************");
//     INFO("get shit done");
//     INFO("***********************");
//     cout << int(roll.size()) << endl;
//     int i;
//     for(i = 0;i < int(roll.size()); i++)
//     {
//       out1 << roll[i] << " ";
//       out2 << pitch[i] << " ";
//       out3 << yaw[i] << " ";
//       out4 << ax[i] << " ";
//       out5 << ay[i] << " ";
//       out6 << az[i] << " ";
//       out7 << wx[i] << " ";
//       out8 << wy[i] << " ";
//       out9 << wz[i] << " ";
//     }
//     cout << i << endl;
//
//
//     out1.close();
//     out2.close();
//     out3.close();
//     out4.close();
//     out5.close();
//     out6.close();
//     out7.close();
//     out8.close();
//     out9.close();
//
// }

static inline void cross(
  Eigen::Matrix<double,3,1> x,
  Eigen::Matrix<double,3,1> y,
  Eigen::Matrix<double,3,1> z)
{
  z(0) = x(1) * y(2) - x(2) * y(1);
  z(1) = x(2) * y(0) - x(0) * y(2);
  z(2) = x(0) * y(1) - x(1) * y(0);
}

// template <typename T>
// // void AddElements(std::vector<T> &master, std::vector<T> slave)
// // {
// //     for (unsigned int i = 0; i < slave.size(); i++)
// //     {
// //         master.emplace_back(slave[i]);
// //     }
// // }

void tt()
{
    Motion::IOManager3 io;

    std::vector<double> fucking(16, 0);

    ifstream af("/home/ubuntu/walk_c.txt");
    std::vector<double> left;
    std::vector<double> right;
    std::vector<std::vector<double>> rc_left;
    std::vector<std::vector<double>> rc_right;
    int cunt = 0;
    while (!af.eof())
    {
        double tmp;
        af >> tmp;
        //cout <<fixed << setprecision(6) << tmp  <<endl;
        if (6 == cunt)
        {
            rc_right.push_back(right);
            right.clear();
            left.push_back(tmp);
            cunt++;
        }
        else if (12 == cunt)
        {
            rc_left.push_back(left);
            left.clear();
            right.push_back(tmp);
            cunt = 1;
        }
        else if (cunt >= 0 && cunt <= 5)
        {
            right.push_back(tmp);
            cunt++;
        }
        else if (cunt >= 7 && cunt <= 11)
        {
            left.push_back(tmp);
            cunt++;
        }
    }

    int num = rc_left.size();
    cout << "num: " << num << endl;

    std::vector<double> arm = {0, 30, 0, 30};
    std::vector<int> changeP = {6,7,13,14};
    io.setServoPI(changeP,700,0);
    sleep(2);

    fucking.clear();
    AddElements(fucking, rc_right[0]);
    AddElements(fucking, rc_left[0]);
    //AddElements(fucking, arm);
    io.setAllspeed(30);
    io.setAllJointValue(fucking);
    io.spinOnce();
    sleep(2);
    io.setAllspeed(0);


    cunt = 0;
    while(ros::ok()){
        if(flag >= 1000)
        {
            if(cunt == 2*num)
            {
                cunt = 0;
            }

            if(cunt < num)
            {
                fucking.clear();
                AddElements(fucking, rc_right[cunt]);
                AddElements(fucking, rc_left[cunt]);
                //AddElements(fucking, arm);
                io.setAllJointValue(fucking);
            }
            else if(cunt < 2 * num && cunt >= num)
            {
                fucking.clear();
                AddElements(fucking, rc_left[cunt - num]);
                AddElements(fucking, rc_right[cunt - num]);
                //AddElements(fucking, arm);
                io.setAllJointValue(fucking);
            }
            cunt++;
        }

        io.spinOnce();
        io.readPosVel();
        power_data = io.getPowerState();
        imu_data = io.getIMUData();
        pressure_data = io.getPressureData();
        servo_state = io.m_servo_inited;
        read_pos = io.readAllPosition();
        read_vel = io.readAllVel();
        //read_vel = io.readAllVel();
    }

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");
    parameters.init(&nh);

    if(chdir(workplace))
        exit(0);  //设置工作路径，也就是B显示的路径

    ofstream out1("roll.txt", ios::out|ios::trunc);
    ofstream out2("pitch.txt", ios::out|ios::trunc);
    ofstream out3("yaw.txt", ios::out|ios::trunc);
    ofstream out4("ax.txt", ios::out|ios::trunc);
    ofstream out5("ay.txt", ios::out|ios::trunc);
    ofstream out6("az.txt", ios::out|ios::trunc);
    ofstream out7("wx.txt", ios::out|ios::trunc);
    ofstream out8("wy.txt", ios::out|ios::trunc);
    ofstream out9("wz.txt", ios::out|ios::trunc);
    ofstream out10("ax_wog.txt", ios::out|ios::trunc);
    ofstream out11("ay_wog.txt", ios::out|ios::trunc);
    ofstream out12("az_wog.txt", ios::out|ios::trunc);

    ofstream right_x("right_x.txt", ios::out|ios::trunc);
    ofstream right_y("right_y.txt", ios::out|ios::trunc);
    ofstream right_yaw("right_yaw.txt", ios::out|ios::trunc);
    ofstream right_vx("right_vx.txt", ios::out|ios::trunc);
    ofstream right_vy("right_vy.txt", ios::out|ios::trunc);
    ofstream right_vyaw("right_vyaw.txt", ios::out|ios::trunc);

    ofstream left_x("left_x.txt", ios::out|ios::trunc);
    ofstream left_y("left_y.txt", ios::out|ios::trunc);
    ofstream left_yaw("left_yaw.txt", ios::out|ios::trunc);
    ofstream left_vx("left_vx.txt", ios::out|ios::trunc);
    ofstream left_vy("left_vy.txt", ios::out|ios::trunc);
    ofstream left_vyaw("left_vyaw.txt", ios::out|ios::trunc);

    ofstream support("support.txt", ios::out|ios::trunc);

    ofstream vx("vx.txt", ios::out|ios::trunc);
    ofstream vy("vy.txt", ios::out|ios::trunc);

    ofstream vx_w("vx_w.txt", ios::out|ios::trunc);
    ofstream vy_w("vy_w.txt", ios::out|ios::trunc);

    ofstream vx_pos("vx_pos.txt", ios::out|ios::trunc);
    ofstream vy_pos("vy_pos.txt", ios::out|ios::trunc);

    ofstream feet_roll("feet_roll.txt", ios::out|ios::trunc);
    ofstream feet_pitch("feet_pitch.txt", ios::out|ios::trunc);
    ofstream feet_yaw("feet_yaw.txt", ios::out|ios::trunc);

    ofstream x("x.txt", ios::out|ios::trunc);
    ofstream y("y.txt", ios::out|ios::trunc);
    ofstream z("z.txt", ios::out|ios::trunc);
    ofstream feet_rollv("feet_rollv.txt", ios::out|ios::trunc);
    ofstream feet_pitchv("feet_pitchv.txt", ios::out|ios::trunc);
    ofstream feet_yawv("feet_yawv.txt", ios::out|ios::trunc);




    std::vector<float> roll,pitch,yaw;
    std::vector<float> ax,ay,az;
    std::vector<float> wx,wy,wz;
    std::vector<float> ax_wog,ay_wog,az_wog;
    std::vector<double> data_right_x, data_right_y, data_right_yaw;
    std::vector<double> data_right_vx, data_right_vy, data_right_vyaw;
    std::vector<double> data_left_x, data_left_y, data_left_yaw;
    std::vector<double> data_left_vx, data_left_vy, data_left_vyaw;
    std::vector<double> data_vx, data_vy, data_feet_roll, data_feet_pitch, data_feet_yaw;
    std::vector<double> data_feet_rollv, data_feet_pitchv, data_feet_yawv;
    std::vector<double> data_vx_pos, data_vy_pos;
    std::vector<double> data_vx_w, data_vy_w;
    std::vector<double> data_x, data_y, data_z;

    std::vector<int> support_now;
    double x_old = 0, y_old = 0, z_old = 0;
    double x_new = 0, y_new = 0, z_new = 0;
    Eigen::Matrix3d rotation_matrix_OA, rotation_matrix_BA, rotation_matrix_OB;
    bool isRight;
    double temp_vx, temp_vy, temp_wx, temp_wy, temp_wz;
    Eigen::Matrix<double,3,1> temp_vel;
    Eigen::Matrix<double,3,1> pos_old, pos_new;
    Eigen::Matrix<double,3,1> w1, w2, w3;
    Eigen::Matrix<double,3,1> Pd;
    Eigen::Matrix<double,3,3> R_new, dR, R_old;
    R_old.setIdentity();


    std::thread t1(tt);
    sleep(6);
    t1.detach();

    Motion::StateManager sm;
    timer a;
    while(ros::ok()){
      a.tic();
      sm.imu_data = imu_data;
      sm.m_power_state = power_data;
      sm.servo_initialized = servo_state;
      sm.pressure_data = pressure_data;
      sm.working();
      a.smartDelay_ms(10.0);//暂时回到10ms
      if(sm.imu_initialized == Motion::INITED && sm.pressure_initialized == Motion::INITED)
      {
         roll.push_back(sm.roll);
         pitch.push_back(sm.pitch);
         yaw.push_back(sm.yaw);
         ax.push_back(imu_data.accl.x);
         ay.push_back(imu_data.accl.y);
         az.push_back(imu_data.accl.z);
         wx.push_back(imu_data.gypo.x);
         wy.push_back(imu_data.gypo.y);
         wz.push_back(imu_data.gypo.z);
         ax_wog.push_back(imu_data.gypo.x);
         ay_wog.push_back(imu_data.gypo.y);
         az_wog.push_back(imu_data.gypo.z);

         std::vector<double> right_p(read_pos.begin(), read_pos.begin() + 6);
         std::vector<double> right_v(read_vel.begin(), read_vel.begin() + 6);
         ForKin  leg_right(right_p,true);
         data_right_x.push_back(leg_right.x_result);
         data_right_y.push_back(leg_right.y_result);
         data_right_yaw.push_back(leg_right.yaw_result);
         leg_right.calVelocity(right_v);
         data_right_vx.push_back(leg_right.vx_result);
         data_right_vy.push_back(leg_right.vy_result);
         data_right_vyaw.push_back(leg_right.vyaw_result);

         std::vector<double> left_p(read_pos.begin() + 6, read_pos.begin() + 12);
         std::vector<double> left_v(read_vel.begin() + 6, read_vel.begin() + 12);
         double support_flag = 1;
         ForKin  leg_left(left_p,false);
         data_left_x.push_back(leg_left.x_result);
         data_left_y.push_back(leg_left.y_result);
         data_left_yaw.push_back(leg_left.yaw_result);
         leg_left.calVelocity(left_v);
         data_left_vx.push_back(leg_left.vx_result);
         data_left_vy.push_back(leg_left.vy_result);
         data_left_vyaw.push_back(leg_left.vyaw_result);
         if(sm.m_support_state == SUPPORT_RIGHT)
         {
            support_now.push_back(1);
            support_flag = 1;
         }
         else if(sm.m_support_state == SUPPORT_LEFT)
         {
            support_now.push_back(0);
            support_flag = 0;
         }
         else if(sm.m_support_state == SUPPORT_BOTH)
            support_now.push_back(2);
         else
            support_now.push_back(3);

        flag++;// start walking ticks


        // if(leg_left.vx_result > leg_right.vx_result)
        if(support_flag == 0)
        {
            isRight = false;
            temp_vx = leg_left.vx_result;
            temp_vy = leg_left.vy_result;
              temp_wx = leg_left.vroll_result;
            temp_wy = leg_left.vpitch_result;
            temp_wz = leg_left.vyaw_result;
            Eigen::Vector3d eulerAngle_BA(leg_left.yaw_result/180*M_PI,
                                          leg_left.pitch_result/180*M_PI,
                                          leg_left.roll_result/180*M_PI);
            Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle_BA(2),Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle_BA(1),Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle_BA(0),Vector3d::UnitZ()));
            rotation_matrix_BA = yawAngle*pitchAngle*rollAngle;

            data_feet_roll.push_back(leg_left.roll_result);
            data_feet_pitch.push_back(leg_left.pitch_result);
            data_feet_yaw.push_back(leg_left.yaw_result);

            data_feet_rollv.push_back(leg_left.vroll_result);
            data_feet_pitchv.push_back(leg_left.vpitch_result);
            data_feet_yawv.push_back(leg_left.vyaw_result);

            x_new = leg_left.x_result;
            y_new = leg_left.y_result;
            z_new = leg_left.z_result;
        }
        else
        {
            isRight = true;
            temp_vx = leg_right.vx_result;
            temp_vy = leg_right.vy_result;
            temp_wx = leg_right.vroll_result;
            temp_wy = leg_right.vpitch_result;
            temp_wz = leg_right.vyaw_result;
            Eigen::Vector3d eulerAngle_BA(leg_right.yaw_result/180*M_PI,
                                          leg_right.pitch_result/180*M_PI,
                                          leg_right.roll_result/180*M_PI);
            Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle_BA(2),Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle_BA(1),Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle_BA(0),Vector3d::UnitZ()));
            rotation_matrix_BA = yawAngle*pitchAngle*rollAngle;

            data_feet_roll.push_back(leg_right.roll_result);
            data_feet_pitch.push_back(leg_right.pitch_result);
            data_feet_yaw.push_back(leg_right.yaw_result);

            data_feet_rollv.push_back(leg_right.vroll_result);
            data_feet_pitchv.push_back(leg_right.vpitch_result);
            data_feet_yawv.push_back(leg_right.vyaw_result);

            x_new = leg_right.x_result;
            y_new = leg_right.y_result;
            z_new = leg_right.z_result;
        }


        Eigen::Vector3d eulerAngle_OA(sm.yaw, sm.pitch, sm.roll);
        Eigen::AngleAxisd rollAngle2(AngleAxisd(eulerAngle_OA(2),Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle2(AngleAxisd(eulerAngle_OA(1),Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle2(AngleAxisd(eulerAngle_OA(0),Vector3d::UnitZ()));
        rotation_matrix_OA = yawAngle2*pitchAngle2*rollAngle2;

        rotation_matrix_OB = rotation_matrix_OA * rotation_matrix_BA.transpose();
        temp_vel(0) = temp_vx;
        temp_vel(1) = temp_vy;
        temp_vel(2) = 0;
        temp_vel = rotation_matrix_OB * temp_vel;
        data_vx.push_back(temp_vel(0));
        data_vy.push_back(temp_vel(1));

        pos_new(0) = x_new;
        pos_new(1) = y_new;
        pos_new(2) = z_new;
        pos_new = rotation_matrix_OB * pos_new;

        pos_old(0) = x_old;
        pos_old(1) = y_old;
        pos_old(2) = z_old;
        pos_old = rotation_matrix_OB * pos_old;

        temp_vel = (pos_new - pos_old)/0.01;
        data_vx_pos.push_back(temp_vel(0));
        data_vy_pos.push_back(temp_vel(1));

        x_old = x_new;
        y_old = y_new;
        z_old = z_new;

        R_new = rotation_matrix_BA;
        dR = R_new * R_old.transpose();
        double theta;
        theta = acos(0.5*(dR(0,0) + dR(1,1) + dR(2,2) - 1));
        if(theta < 1e-5)
        {
          w2(0) = 0;
          w2(1) = 0;
          w2(2) = 0;
        }
        else
        {
          double factor = theta / 2 * sin(theta);
          w2(0) = dR(2,1) - dR(1,2);
          w2(1) = dR(0,2) - dR(2,0);
          w2(2) = dR(1,0) - dR(0,1);
          w2 = w2 * factor;
        }
        w3(0) = imu_data.gypo.x;
        w3(1) = imu_data.gypo.y;
        w3(2) = imu_data.gypo.z;
        w3 = rotation_matrix_OA * w3;

        w2(0) = temp_wx;
        w2(1) = temp_wy;
        w2(2) = temp_wz;
        w2 = rotation_matrix_OB * w2;

        w1 = w3 - w2;
        cross(w1, rotation_matrix_OB * pos_new, temp_vel);
        data_vx_w.push_back(temp_vel(0));
        data_vy_w.push_back(temp_vel(1));

        data_x.push_back(x_new);
        data_y.push_back(y_new);
        data_z.push_back(z_new);

        R_old = rotation_matrix_BA;



      }

    }

    INFO("***********************");
    INFO("get shit done");
    INFO("***********************");

    cout << int(roll.size()) << endl;
    int i;
    for(i = 0;i < int(roll.size()); i++)
    {
      out1 << roll[i] << " ";
      out2 << pitch[i] << " ";
      out3 << yaw[i] << " ";
      out4 << ax[i] << " ";
      out5 << ay[i] << " ";
      out6 << az[i] << " ";
      out7 << wx[i] << " ";
      out8 << wy[i] << " ";
      out9 << wz[i] << " ";
      out10 << ax_wog[i] << " ";
      out11 << ay_wog[i] << " ";
      out12 << az_wog[i] << " ";

      right_x << data_right_x[i] << " ";
      right_y << data_right_y[i] << " ";
      right_yaw << data_right_yaw[i] << " ";
      right_vx << data_right_vx[i] << " ";
      right_vy << data_right_vy[i] << " ";
      right_vyaw << data_right_vyaw[i] << " ";

      left_x << data_left_x[i] << " ";
      left_y << data_left_y[i] << " ";
      left_yaw << data_left_yaw[i] << " ";
      left_vx << data_left_vx[i] << " ";
      left_vy << data_left_vy[i] << " ";
      left_vyaw << data_left_yaw[i] << " ";

      support << support_now[i] << " ";

      vx << data_vx[i] << " ";
      vy << data_vy[i] << " ";

      vx_w << data_vx_w[i] << " ";
      vy_w << data_vy_w[i] << " ";

      vx_pos << data_vx_pos[i] << " ";
      vy_pos << data_vy_pos[i] << " ";

      feet_roll << data_feet_roll[i] << " ";
      feet_pitch << data_feet_pitch[i] << " ";
      feet_yaw << data_feet_yaw[i] << " ";

      feet_rollv << data_feet_rollv[i] << " ";
      feet_pitchv << data_feet_pitchv[i] << " ";
      feet_yawv << data_feet_yawv[i] << " ";

      x << data_x[i] << " ";
      y << data_y[i] << " ";
      z << data_z[i] << " ";
    }
    cout << i << endl;

    cout << "writing datas done" << endl;


      out1.close();
      out2.close();
      out3.close();
      out4.close();
      out5.close();
      out6.close();
      out7.close();
      out8.close();
      out9.close();
      out10.close();
      out11.close();
      out12.close();

      right_x.close();
      right_y.close();
      right_yaw.close();
      right_vx.close();
      right_vy.close();
      right_vyaw.close();

      left_x.close();
      left_y.close();
      left_yaw.close();
      left_vx.close();
      left_vy.close();
      left_vyaw.close();

      support.close();

      vx.close();
      vy.close();

      vx_w.close();
      vy_w.close();

      vx_pos.close();
      vy_pos.close();

      feet_yaw.close();
      feet_pitch.close();
      feet_roll.close();

      feet_yawv.close();
      feet_pitchv.close();
      feet_rollv.close();

      x.close();
      y.close();
      z.close();
}
