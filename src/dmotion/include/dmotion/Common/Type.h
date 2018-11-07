#pragma once

#include "EigenTypes.h"

#include <string>
#include <vector>

namespace Motion
{
typedef struct
{
  float x, y, z;
} Vec3f;

struct  Vec3d
{
  Vec3d()
    : x(0), y(0), z(0)
  {}
  double x, y, z;
};

typedef struct
{
  Vec3f position;
  Vec3f rotation;
} Pose6Dof;

struct IMUData
{
  Vec3f gypo;
  Vec3f accl;
};

struct PressureData
{
  PressureData()
  {
    left.resize(4);
    right.resize(4);
  }

  std::vector<float> left, right;
};

struct JointConfig
{
  JointConfig(const int id_, 
              const int cw_, 
              const int resolution_, 
              const int init_, 
              const int max_pos_,
              const int min_pos_)
    : id(id_), cw(cw_), resolution(resolution_),
     init(init_), max_pos(max_pos_), min_pos(min_pos_)
  {
      factor = static_cast<float>(resolution)/M_PI/2.0*cw;
  }

  void setFactor()
  {
    factor = static_cast<float>(resolution)/M_PI/2.0*cw;
  }

  int id;
  int cw;
  int resolution;
  int init;
  int max_pos;
  int min_pos;

  float factor;
};

struct Joint
{
    explicit Joint(const JointConfig& cfg_)
      : cfg(cfg_), goal_pos(0), real_pos(0)
    {
        
    }

    void setGoalPos(const float position)
    {
        goal_pos = position;
    }

    void setRealPos(const float position)
    {
        real_pos = position;
    }
    
    const JointConfig& cfg;
    float goal_pos;
    float real_pos;
};

struct WholeBodyTransform
{
  Isometry3<float> right_leg = Isometry3<float>::Identity();
  Isometry3<float> left_leg = Isometry3<float>::Identity();
  Isometry3<float> right_hand, left_hand;
  Isometry3<float> head;
};

enum ActionCommand
{
  STANDUP,
  CROUNCH,
  WALK,
  KICK,
  SETUPBACK,
  SETUPFRONT,
  GOALIE
};

enum PowerState
{
  ON,
  OFF,
  REOPEN
};

// static const std::string
// getActionCommandName(const ActionCommand cmd)
// {
//     switch (cmd)
//     {
//         case STANDUP:
//             return "standup";
//         case CROUNCH:
//             return "crounch";
//         case KICK:
//             return "kick";
//         case WALK:
//             return "walk";
//         case SETUPBACK:
//             return "setupback";
//         case SETUPFRONT:
//             return "setupfront";
//         case GOALIE:
//             return "goalie";
//         default:
//             return "nontype";
//     }
// }

enum RoboState
{
  STABLE,
  UNSTABLE,
  FALLING
};

enum SupportPhase
{
  SUPPORT_BOTH,
  SUPPORT_LEFT,
  SUPPORT_RIGHT
};

typedef std::vector<float> JointValues;
}