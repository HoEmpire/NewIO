#pragma once

#include <string>
#include <vector>
//#define M_PI 3.1415926
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
      factor = static_cast<float>(resolution)/360.0*cw;
      init = init * factor * cw;
  }

  void setFactor()
  {
    factor = static_cast<float>(resolution)/360.0*cw;
      init = init * factor * cw;
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

    void setGoalPos(const double position)
    {
        goal_pos = position;
    }

    void setRealPos(const double position)
    {
        real_pos = position;
    }

    const JointConfig& cfg;
    double goal_pos;
    double real_pos;
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
