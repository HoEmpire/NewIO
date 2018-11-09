#pragma once

#include <ros/ros.h>

namespace Motion
{
    #define MOTION_WARN(...) ROS_WARN(__VA_ARGS__);
    #define MOTION_INFO(...) ROS_INFO(__VA_ARGS__);
    #define MOTION_ERROR(...) ROS_ERROR(__VA_ARGS__);
    #define MOTION_DEBUG(...) ROS_DEBUG(__VA_ARGS__);
    #define MOTION_FATAL(...) ROS_FATAL(__VA_ARGS__);

    #define MOTION_WARN_STREAM(args) ROS_WARN_STREAM(args);
    #define MOTION_INFO_STREAM(args) ROS_INFO_STREAM(args);
    #define MOTION_ERROR_STREAM(args) ROS_ERROR_STREAM(args);
    #define MOTION_DEBUG_STREAM(args) ROS_DEBUG_STREAM(args);
    #define MOTION_FATAL_STREAM(args) ROS_FATAL_STREAM(args);

    
}
