#pragma once

#include <Eigen/Dense>

// namespace Motion
// {
inline float DEG2RAD(float deg)
{
    return deg*M_PI/180.0f;
}

inline float RAD2DEG(float rad)
{
    return rad*180.0f/M_PI;
}

// }