/**
 * \file: motioninfo.hpp
 * \bref: type definitions of motion
 **/
#pragma once

#include <ostream>
#include "dcommon/dcommon.hpp"

enum stabilityStatus
{
    stable,
    rightdown, // 1
    leftdown, // 2
    frontdown, // 3
    backdown, //  4
    unstable,

    null_status
};

/**
 * Gyroscope data
 **/
struct GyroData
{
    GyroData()
      : corresCycle(0)
    {
        for (int i = 0; i < 3; i++) {
            ACCL[i] = 0;
            GYPO_a[i] = 0;
            GYPO[i] = 0;
        }
    }
    ~GyroData()
    {
    }

    ddouble_t GYPO[3];
    ddouble_t GYPO_a[3];
    ddouble_t ACCL[3];
    long corresCycle;
};

/**
 * Compass data
 **/
struct CompassData
{
    CompassData()
      : x(0)
      , y(0)
      , z(0)
      , temperature(0)
      , corresCycle(0)
    {
    }
    ~CompassData()
    {
    }

    long x;
    long y;
    long z;
    ddouble_t temperature;
    long corresCycle;
};

/**
 * Body angle
 **/
struct AngleData
{
    AngleData()
      : angleX(0)
      , angleY(0)
      , angleZ(0)
      , corresCycle(0)
    {
    }
    ~AngleData()
    {
    }

    ddouble_t angleX;
    ddouble_t angleY;
    ddouble_t angleZ;
    long corresCycle;
};

/**
 *body delta distance
 **/
struct deltadataDebug
{
    ddouble_t m_x;
    ddouble_t m_y;
    ddouble_t m_angle;

    deltadataDebug()
      : m_x(0)
      , m_y(0)
      , m_angle(0)
    {
    }

    friend std::ostream& operator<<(std::ostream& oss, const deltadataDebug& obj)
    {
        return oss << "m_x: " << obj.m_x << " m_y: " << obj.m_y << " m_angle: " << obj.m_angle;
    }

    std::string toString()
    {
        return "deltadata [" + std::to_string(m_x) + ", " + std::to_string(m_y) + ", " + std::to_string(m_angle) + "]";
    }
};
