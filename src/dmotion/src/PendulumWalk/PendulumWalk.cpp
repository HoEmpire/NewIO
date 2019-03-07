//
// Created by zjudancer on 18-12-18.
//

#include "PendulumWalk.h"

using namespace dmotion;
namespace dmotion {


    PendulumWalk::PendulumWalk() {
        com_ac_x = 0;
        com_ac_y = 0;
        support_is_right = false;
        hang_foot = {0, ANKLE_DIS, 0};
        com_pos = {0, ANKLE_DIS / 2.0, 0};
        com_x_changed = 0;
        com_y_changed = 0;
        Support = new OneFootLanding(false);
    }

    /**
     * 生成下一步的动作数据
     * @param dx 下一步质心的x变化，相对于上半身，单位是cm
     * @param dy 下一步执行的y变化，相对于上半身，单位是cm
     * @param d_yaw 下一步执行的yaw变化，相对于上半身，角度制
     */
    void PendulumWalk::GiveAStep(double dx, double dy, double d_yaw) {


        cout << endl;

        delete Support;
        Support = new OneFootLanding(support_is_right);

        x0 = com_pos[0] * cos(hang_foot[2]) - hang_foot[0] * cos(hang_foot[2]) + com_pos[1] * sin(hang_foot[2]) -
             hang_foot[1] * sin(hang_foot[2]);

        xt = (dx - ((support_is_right) ? (ANKLE_DIS / 2.0) : (-ANKLE_DIS / 2.0)) * sin(Deg2Rad(d_yaw)))/2.0;

//        std::cout << "x0 :" << x0 << std::endl;
//        std::cout << "xt :" << xt << std::endl;


        tao = TAO;

        //算出摆的周期常数，这里的com_h暂时是由机器人crouch姿态下倒挂着摆动测量得出的
        com_h = COM_H;
        Tc = std::sqrt(com_h / 980);
//        std::cout << "Tc  :" << Tc << " " << std::endl;

        //算出来这个步态单元的初速度vx

        vx = (xt - x0 * cosh(tao / Tc)) / (Tc * sinh(tao / Tc));
//        std::cout << "vx :" << vx << std::endl;

        //y方向的研究

        y00 = com_pos[1] * cos(hang_foot[2]) - hang_foot[1] * cos(hang_foot[2]) - com_pos[0] * sin(hang_foot[2]) +
              hang_foot[0] * sin(hang_foot[2]);
        ytt = (dy + ((support_is_right) ? (ANKLE_DIS / 2.0) : (-ANKLE_DIS / 2.0)) +
              ((support_is_right) ? (ANKLE_DIS / 2.0) : (-ANKLE_DIS / 2.0)) * cos(Deg2Rad(d_yaw)))/2;


//        std::cout << "y00 :" << y00 << std::endl;
//        std::cout << "ytt :" << ytt << std::endl;

//这里实际计算质心轨迹的是y0和yt，防止∆y太大。
        y0 = support_is_right ? (Y_HALF_AMPLITUDE) : (-Y_HALF_AMPLITUDE);
        yt = support_is_right ? (Y_HALF_AMPLITUDE) : (-Y_HALF_AMPLITUDE);

        //计算过程中换元得到的一个临时变量m
        m = exp(tao / Tc);
//        std::cout << "m  :" << m << " " << std::endl;

        //步行单元的周期定了后y方向的最大速度是确定的
        vy = (yt - m * y0) / ((m + 1) * Tc);
//        std::cout << "vy  :" << vy << " " << std::endl;


        //使用插值算法定义抬脚的时间位移曲线
        std::vector<double> akZ_t = {0, 0.1043925386769408, 0.2158391257720493, 0.35};
        std::vector<double> akZ_p = {0, 0.8621589623943856, 3.173077504386064, 0};
        std::vector<double> akZ_s = {0, 23.85927719624021, 0, 0};
        ThreeInterpolation ankle_z(akZ_t, akZ_p, akZ_s);
        ankle_z.CalculatePoints(10);
        akZ = ankle_z.GetPoints();

        //TODO consider if we are going to plan the ankle pitch

        //线性规划上半身yaw
        std::vector<double> comYaw_t = {0, 0.35};
        std::vector<double> comYaw_p = {Rad2Deg(com_pos[2] - hang_foot[2]), d_yaw / 2.0};
        double slope_comYaw = (d_yaw / 2.0 - Rad2Deg(com_pos[2] - hang_foot[2])) / 0.35;
        std::vector<double> comYaw_s = {slope_comYaw, slope_comYaw};
        ThreeInterpolation com_yaw(comYaw_t, comYaw_p, comYaw_s);
        com_yaw.CalculatePoints(10);
        comYaw = com_yaw.GetPoints();
//        std::cout << "comYaw :" << std::endl;
//        PrintVector(comYaw);

        //线性规划质心的y基础位置
        std::vector<double> comY_t = {0, 0.35};
        std::vector<double> comY_p = {y00, ytt};
        double slope_comY = (ytt - y00) / 0.35;
        std::vector<double> comY_s = {slope_comY, slope_comY};
        ThreeInterpolation com_y(comY_t, comY_p, comY_s);
        com_y.CalculatePoints(10);
        comY = com_y.GetPoints();
//        std::cout << "comY :" << std::endl;
//        PrintVector(comY);


        //线性的质心加速度x偏移
        double ac_x = (dx - com_x_changed) * ACC_COEF_X;
        std::vector<double> accX_t = {0, 0.35};
        std::vector<double> accX_p = {com_ac_x, ac_x};
        double slope_accX = (ac_x - com_ac_x) / 0.35;
        std::vector<double> accX_s = {slope_accX, slope_accX};
        ThreeInterpolation acc_x(accX_t, accX_p, accX_s);
        acc_x.CalculatePoints(10);
        accX = acc_x.GetPoints();
//        std::cout << "accX :" << std::endl;
//        PrintVector(accX);


        //线性的质心加速度y偏移
        double ac_y = (dy - com_y_changed) * ACC_COEF_Y;
        std::vector<double> accY_t = {0, 0.35};
        std::vector<double> accY_p = {com_ac_y, ac_y};
        double slope_accY = (ac_y - com_ac_y) / 0.35;
        std::vector<double> accY_s = {slope_accY, slope_accY};
        ThreeInterpolation acc_y(accY_t, accY_p, accY_s);
//        cout << "com_ac_y :" <<com_ac_y << endl;
//        cout << "ac_y :" <<ac_y << endl;
        acc_y.CalculatePoints(10);
        accY = acc_y.GetPoints();
//        std::cout << "accY :" << std::endl;
//        PrintVector(accY);



        //脚踝的x方向起点终点的插值
        double ak_x_0 = -hang_foot[0] * cos(hang_foot[2]) - hang_foot[1] * sin(hang_foot[2]);
        double ak_x_t = 2 * xt;
        std::vector<double> akX_t = {0, 0.35};
        std::vector<double> akX_p = {ak_x_0, ak_x_t};
//        cout << "x0 :" << x0 << endl;
//        cout << "xt :" << xt << endl;
//        cout << "ak_x_0 :" << ak_x_0 << endl;
//        cout << "ak_x_t :" << ak_x_t << endl;
        double slope_akX = (ak_x_t - ak_x_0) / 0.35;
        std::vector<double> akX_s = {slope_akX, slope_akX};
        ThreeInterpolation ak_x(akX_t, akX_p, akX_s);
        ak_x.CalculatePoints(10);
        akX = ak_x.GetPoints();
//        std::cout << "akX :" << std::endl;
//        PrintVector(akX);

        //脚踝的y方向起点终点的插值
        double ak_y_0 = hang_foot[0] * sin(hang_foot[2]) - hang_foot[1] * cos(hang_foot[2]);
        double ak_y_t = 2 * ytt;
        std::vector<double> akY_t = {0, 0.35};
        std::vector<double> akY_p = {ak_y_0, ak_y_t};
        double slope_akY = (ak_y_t - ak_y_0) / 0.35;
        std::vector<double> akY_s = {slope_akY, slope_akY};
        ThreeInterpolation ak_y(akY_t, akY_p, akY_s);
        ak_y.CalculatePoints(10);
        akY = ak_y.GetPoints();
//        std::cout << "akY :" << std::endl;
//        PrintVector(akY);


        //脚踝的yaw起点终点的插值
        std::vector<double> akYaw_t = {0,0.35};
        std::vector<double> akYaw_p = {Rad2Deg( - hang_foot[2]), d_yaw};
        double slope_akYaw = (ak_y_t - ak_y_0) / 0.35;
        std::vector<double> akYaw_s = {slope_akYaw, slope_akYaw};
        ThreeInterpolation ak_yaw(akYaw_t, akYaw_p, akYaw_s);
        ak_yaw.CalculatePoints(10);
        akYaw = ak_yaw.GetPoints();



        //为下一步的数据做准备
        support_is_right = !support_is_right;
        com_x_changed = dx;
        com_y_changed = dy;

        com_pos.clear();
        com_pos.push_back(xt);
        com_pos.push_back(ytt);
        com_pos.push_back(Deg2Rad(d_yaw) / 2.0);
//        cout << "com_pos :" << endl;
//        PrintVector(com_pos);
        hang_foot.clear();
        hang_foot.push_back(ak_x_t);
        hang_foot.push_back(ak_y_t);
        hang_foot.push_back(2.0 * com_pos[2]);//这里使得落脚点的yaw和质心一致了
//        cout << "hang_foot :" << endl;
//        PrintVector(hang_foot);

        com_ac_x = ac_x;
        com_ac_y = ac_y;

        tick_num = 0;


    }


    std::vector<double> PendulumWalk::GiveATick() {


        motion_tick tmptick;
        tmptick.time_stamp = 10000000 * (tick_num); //10毫秒
        x = x0 * std::cosh(0.01 * (tick_num) / Tc) + Tc * vx * std::sinh(0.01 * (tick_num) / Tc);
        y = y0 * std::cosh(0.01 * (tick_num) / Tc) + Tc * vy * std::sinh(0.01 * (tick_num) / Tc);
        tmptick.yaw = comYaw[tick_num];
        tmptick.whole_com.emplace_back(accX[tick_num] + x);//(x * 100 +1.5);
        tmptick.whole_com.emplace_back(y - y0 + comY[tick_num]+accY[tick_num]);
        tmptick.whole_com.emplace_back(COM_HEIGHT);  //0.308637

        tmptick.hang_foot.emplace_back(akX[tick_num]); //(200 * x);
        tmptick.hang_foot.emplace_back(akY[tick_num]);
        tmptick.hang_foot.emplace_back(akZ[tick_num]);
        tmptick.hang_foot.emplace_back(0);
        tmptick.hang_foot.emplace_back(0);
        tmptick.hang_foot.emplace_back(akYaw[tick_num]);

//        if(tick_num == 0)
//        {
//            cout << "comY[tick_num] :" << comY[tick_num] <<endl;
//            dmotion::PrintVector(tmptick.whole_com);
//            dmotion::PrintVector(tmptick.hang_foot);
//        }
        tick_num++;


      return (Support->GetOneStep(tmptick.hang_foot, tmptick.whole_com, tmptick.yaw, false));

    }


}
