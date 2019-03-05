#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
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

// template <class T1>
// std::vector<T1> twointerpolation(std::vector<T1> points, T1 v0, int time_points,int time_io)
// {
//   int i;
//   T1 a0, a1, a2, v;
//   v = v0;
//   int n = time_points / time_io;
//   std::vector<T1> x;
//   int t;
//   for(unsigned j = 0; j < points.size(); j++)
//   {
//     a0 = points[j];
//     a1 = v;
//     a2 = (points[j+1] - a0 - a1 * time_points) / (time_points * time_points);
//     for(i = 0; i <= n; i++)
//     {
//       t = i * time_io;
//       x[i + n * j] = a0 + a1 * t + a2 * pow(t, 2);
//     }
//     v = a1 + 2 * a2 * time_points;
//     std::cout <<  x[i + n * j] << std::endl;
//   }
//   return x;
// }

// }
