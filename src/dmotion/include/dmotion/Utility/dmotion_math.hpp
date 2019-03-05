//
// Created by fw on 18/10/17.
// E-mail: zjufanwu@zju.edu.cn
//

#ifndef DMOTION_MATH_HPP
#define DMOTION_MATH_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <iomanip>


namespace dmotion {


    /**
     * 把曲线斜率化为倾斜角（角度制）的函数
     */
    template<class T>
    double Slope_to_AngleDeg(T s) {
        return atan(s) * 57.3;
    }

    /**
     * 角度制到弧度值的转换
     */
    template<class T>
    inline T Deg2Rad(T deg) {
        return deg * M_PI / 180;
    }

    /**
     * 弧度值到角度值的转换
     */
    template<class T>
    inline T Rad2Deg(T rad) {
        return rad * 180 / M_PI;
    }

    /**
     * vector的角度值到弧度值的转换
     */
    template<typename Scalar>
    inline std::vector<Scalar> &Deg2Rad(std::vector<Scalar> &deg) {
        int size = deg.size();
        for (int i = 0; i < size; i++) {
            deg.push_back(Deg2Rad(deg[i]));
        }
        deg.erase(deg.begin(), deg.begin() + size);
        return deg;
    }

    /**
     * vector的角度值到弧度值的转换
     */
    template<typename Scalar>
    inline std::vector<Scalar> &Rad2Deg(std::vector<Scalar> &rad) {
        int size = rad.size();
        for (int i = 0; i < size; i++) {
            rad.push_back(Rad2Deg(rad[i]));
        }
        rad.erase(rad.begin(), rad.begin() + size);
        return rad;
    }

    /**
     * vector模版类的全局输出
     */
    template<class T>
    inline void PrintVector(const std::vector<T> &vectorOb) {
        for (unsigned i = 0; i < vectorOb.size(); i++)
            std::cout <<std::fixed << std::setprecision(7) << vectorOb[i]<< " ";
        std::cout << std::endl;
    }

    /**
     * 判断输入参数的符号的模板函数
     */
    template<typename T>
    inline T sign(T val) {
        return val > 0 ? 1 : (val < 0 ? -1 : 0);
    }


    /**
     * 使用三角形的余弦定理求某个角度,这种运算为了保证正确性和精度,用double比较好,所以这里不使用模板
     * @param edge_1 临边1长度
     * @param edge_2 临边2长度
     * @param edge_opposite 对边长度
     * @return 要求的角的角度值
     */
    inline double CosineTheorem(const double &edge_1,
                                const double &edge_2,
                                const double &edge_opposite) {
        double cosine = (edge_1 * edge_1 + edge_2 * edge_2 - edge_opposite * edge_opposite) / (2 * edge_1 * edge_2);
        return std::acos(cosine);
    }

    /**
     * 获得两个三维向量之间夹角的函数
     * @param x_1
     * @param y_1
     * @param z_1
     * @param x_2
     * @param y_2
     * @param z_2
     * @return
     */
    inline double GetDelta(const double &x_1, const double &y_1, const double &z_1, const double &x_2,
                           const double &y_2, const double &z_2) {
        double tmpcos = (x_1 * x_2 + y_1 * y_2 + z_1 * z_2) /
                        sqrt((x_1 * x_1 + y_1 * y_1 + z_1 * z_1) * (x_2 * x_2 + y_2 * y_2 + z_2 * z_2));
        return std::acos(tmpcos);
    }

    /**
     * 获得两个二维向量之间夹角的函数
     * @param x_1
     * @param y_1
     * @param x_2
     * @param y_2
     * @return
     */
    inline double GetDelta(const double &x_1, const double &y_1, const double &x_2, const double &y_2) {
        double tmpcos = (x_1 * x_2 + y_1 * y_2) / sqrt((x_1 * x_1 + y_1 * y_1) * (x_2 * x_2 + y_2 * y_2));
        return std::acos(tmpcos);
    }

    /**
     * 使用Eigen的计算方法重载求两向量夹角的函数
     * @tparam order
     * @tparam T
     * @param v1
     * @param v2
     * @return
     */
    template<int order, typename T>
    inline double GetDelta(const Eigen::Matrix<T, order, 1> &v1, const Eigen::Matrix<T, order, 1> &v2) {
        double tmp = v1.dot(v2) / (v1.norm() * v2.norm());
        std::cout << "tmp: " << tmp << std::endl << std::endl;
        if (1 < tmp)
            tmp = 1.0;
        else if (-1 > tmp)
            tmp = -1.0;
        return std::acos(1);
    }

    /**
     * 双变量反正切函数，用于准确求取一个角度，定义域是[0,2pi)
     * @tparam T
     * @param opposite 对边长度（直角边）
     * @param neighbor 临边长度 (直角边)
     * @return
     */
    inline double Atan(const double opposite, const double neighbor) {
        if (0 < neighbor)
            return std::atan(opposite / neighbor);
        else if (0 == neighbor) {
            if (0 > opposite)
                return -M_PI / 2;
            else if (0 < opposite)
                return M_PI / 2;
            else
                return 0;
        } else {
            if (0 <= opposite)
                return std::atan(opposite / neighbor) + M_PI;
            else
                return std::atan(opposite / neighbor) - M_PI;
        }
    }

    /**
     * 用于计算三维空间中两点之间的距离
     * @tparam T
     * @param x_1
     * @param y_1
     * @param z_1
     * @param x_2
     * @param y_2
     * @param z_2
     * @return
     */
    template<typename T>
    inline double getDistance(const T &x_1, const T &y_1, const T &z_1, const T &x_2, const T &y_2, const T &z_2) {
        return std::sqrt(std::pow(x_1 - x_2, 2) + std::pow(y_1 - y_2, 2) + std::pow(z_1 - z_2, 2));

    }

    /**
     * 把slave vector里面的元素追加到master后面
     * @tparam T
     * @param master
     * @param slave
     * @return
     */
    template <typename T>
    void AddElements(std::vector<T> &master, std::vector<T> slave)
    {
        for (unsigned int i = 0; i< slave.size() ; i++)
        {
            master.emplace_back(slave[i]);
        }
    }

}


#endif //DMOTION_MATH_HPP
