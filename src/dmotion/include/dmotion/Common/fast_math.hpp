#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "dcommon/dcommon.hpp"

#define UNKNOW_DOUBLE_VALUE (-9999)
#define INF_INT 99999
#define EPSILN 0.0001;

inline ddouble_t
AngleNormalization(ddouble_t angle)
{
    ddouble_t a = angle;
    while (a > 180) {
        a -= 360;
    }
    while (a <= -180) {
        a += 360;
    }
    return a;
}

inline ddouble_t
degrees(ddouble_t rad)
{
    return rad * 180 / M_PI;
}

inline ddouble_t
radians(ddouble_t deg)
{
    return deg * M_PI / 180;
}

/**
 * safe way to judge aangle equal
 **/
inline bool
angleEqual(ddouble_t a, ddouble_t b)
{
    return std::abs(a - b) < EPSILN;
}

inline bool
double_equal(ddouble_t a, ddouble_t b)
{
    auto diff = a - b;
    return diff < 0.0001 && diff > -0.0001;
}

inline ddouble_t
getPieceWise(const std::vector<ddouble_t>& x, const std::vector<ddouble_t>& y, const ddouble_t wx)
{
    if (x.size() != y.size())
        throw std::runtime_error("getPieceWise function , the vector size not same ");
    if (!x.size())
        throw std::runtime_error("getPieceWise function , the vector size empty ");
    int size = x.size();
    ddouble_t returnValue = y[1];

    if (wx < x[0])
        return y[0];
    if (wx > x[size - 1])
        return y[size - 1];

    // first calculate k
    // ddouble_t k[size - 1];
    ddouble_t* k = new ddouble_t[size - 1];
    for (int i = 0; i < size - 1; i++) {
        if (std::abs(x[i + 1] - x[i]) < 0.0001)
            k[i] = 0;
        else
            k[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
        //std::cout << k[i] << std::endl;
    }
    for (int i = 0; i < size; i++) {
        if (wx < x[i]) // i = 0 will never happan
        {
            returnValue = y[i - 1] + k[i - 1] * (wx - x[i - 1]); // RobotPara::mid_theta_amend + k*(tsx - RobotPara::mid_x_max);
            break;
        }
    }
    delete[] k;
    return returnValue;
}
