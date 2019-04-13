#include "dmotion/IO/IOManager3.h"

#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"

#include <chrono>
#include <thread>

using namespace dynamixel;

#define LEG_ONLY false //ONLY USE LEGS OF ZJU DANCER

#define DATA_FREQUENCY 10.0    // 100hz data stream
#define POWER_DETECTER true   //whether open check power mode or not
                              //only used in a complete Robot
//#define old 1

namespace Motion
{
static void* pthreadIMU(void* arg);
static void* pthreadPow(void* arg);

static int get_thread_policy(pthread_attr_t *attr)
{
  int policy;
  int rs = pthread_attr_getschedpolicy(attr,&policy);
  assert(rs==0);
  switch(policy)
  {
  case SCHED_FIFO:
    printf("policy= SCHED_FIFO\n");
    break;
  case SCHED_RR:
    printf("policy= SCHED_RR\n");
    break;
  case SCHED_OTHER:
    printf("policy=SCHED_OTHER\n");
    break;
  default:
    printf("policy=UNKNOWN\n");
    break;
  }
  return policy;
}

// static void show_thread_priority(pthread_attr_t *attr,int policy)
// {
//   int priority = sched_get_priority_max(policy);
//   assert(priority!=-1);
//   printf("max_priority=%d\n",priority);
//   priority= sched_get_priority_min(policy);
//   assert(priority!=-1);
//   printf("min_priority=%d\n",priority);
// }

static int get_thread_priority(pthread_attr_t *attr)
{
  struct sched_param param;
  int rs = pthread_attr_getschedparam(attr,&param);
  assert(rs==0);
  printf("priority=%d\n",param.__sched_priority);
  return param.__sched_priority;
}

static int set_thread_priority(pthread_attr_t *attr, int priority)
{
  struct sched_param param;
  param.sched_priority = priority;
  pthread_attr_setschedparam(attr,&param);
  //pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
  return param.__sched_priority;
}

static void set_thread_policy(pthread_attr_t *attr,int policy)
{
  int rs = pthread_attr_setschedpolicy(attr,policy);
  assert(rs==0);
  get_thread_policy(attr);
}

IOManager3::IOManager3()
    : m_power_state(OFF),
      m_servo_inited(false)
{
    //nice(0);
    ROS_DEBUG("IOManager3::IOManager3: Construct IOManager3 instance");
    initZJUJoint();
    int rs = pthread_attr_init(&attr);
    assert(rs==0);

    printf("show priority of current thread: ");
    int priority = get_thread_priority(&attr);

    printf("set thread: SCHED_RR policy\n");
    set_thread_policy(&attr,SCHED_RR);
    priority = 95;
    printf("set thread: %d policy\n", priority);
    set_thread_priority(&attr, priority);
    printf("show priority of current thread: ");
    priority = get_thread_priority(&attr);
    assert(rs==0);

    if(POWER_DETECTER)
    {
        //if ((pthread_create(&tidIMU,NULL,IOManager3::readIMU, NULL)!=0)
        if (pthread_create(&tidIMU,NULL, &pthreadIMU,  (void *)this)!=0)
        {
           printf("create error!\n");
        }
        timer::delay_ms(200);

        if (pthread_create(&tidPow,NULL, &pthreadPow,  (void *)this)!=0)
        {
               printf("create error!\n");
        }
        checkIOPower();
    }
    else
        m_power_state = ON;

    if(m_power_state == ON)
    {
      m_servo_io.initServoPositions();
      m_servo_inited = true;
    }

}

IOManager3::~IOManager3()
{
    pthread_join(tidIMU,NULL);
    pthread_join(tidPow,NULL);
    pthread_attr_destroy(&attr);
}

dynamixel::PortHandler* IOManager3::initPort(const std::string portname, const int baudrate, const bool block)
{
    ROS_INFO_STREAM("IOManager3::initPort: init " << portname << " baudrate: " << baudrate );
    dynamixel::PortHandler* port_ = PortHandler::getPortHandler(portname.c_str());

    // Set port baudrate
    if (!port_->setBaudRate(baudrate,block))
    {
        ROS_FATAL("IOManager3::_initPort: could not change baudrate");
        ROS_FATAL("IOManager3::_initPort: are you stupid enough, dass du unfaehig zu eroeffnenung des Port bist?");
        std::abort();
    }
    return port_;
}

void IOManager3::initZJUJoint()
{
    // construct robot joint bases
    for (auto& pair:parameters.io.joint_cfg)
    {
      if(LEG_ONLY){
        if(!(pair.first ==  "right_arm_upper" || pair.first ==  "right_arm_lower" ||
             pair.first ==  "left_arm_upper" || pair.first ==  "left_arm_lower"||
             pair.first ==  "head_pitch" || pair.first ==  "head_yaw"))
                 m_servo_io.m_joints.insert(std::make_pair(pair.first, Joint(pair.second)));
      }
      else
          m_servo_io.m_joints.insert(std::make_pair(pair.first, Joint(pair.second)));

    }

    for (auto& joint:m_servo_io.m_joints)
    {
        ROS_DEBUG_STREAM("ServoIO::ServoIO:" << std::endl
                        << joint.first << ' '
                        << joint.second.cfg.id << ' '
                        << joint.second.cfg.cw << ' '
                        << joint.second.cfg.factor << ' '
                        << joint.second.cfg.id << ' '
                        << joint.second.cfg.init << ' '
                        << joint.second.cfg.max_pos << ' '
                        << joint.second.cfg.min_pos << ' '
                        << joint.second.cfg.resolution
                        );
    }

}

void IOManager3::spinOnce()
{
    //static int imu_failures = 0;
    if (ON == m_power_state)
    {
        //
        // use internal clock to calculate waiting time
        std::chrono::duration<double> duration_ = (timer::getCurrentSystemTime() - m_sync_time);
        double ticks = duration_.count()*1000;

        //set delay
        if (ticks > DATA_FREQUENCY)
        {
            ROS_WARN_STREAM("IOManager3::spinOnce:  motion ticks overflow..." << ticks);
            timer::delay_ms(100);//TODO
        }
        else
        {
            timer::delay_us((DATA_FREQUENCY - ticks)*1000);
        }

        m_sync_time = timer::getCurrentSystemTime();//这句话的位置 TODO pyx after

        m_servo_io.sendServoPositions();
        // read pressure data
        if (parameters.global.using_pressure)
        {
            if (!m_feet_io.readPressureData())
            {
                ROS_WARN("IOManager3::spinOnce: read feet pressure data error");
            }
        }

    }
    else if (OFF == m_power_state)
    {
        m_servo_inited = false;
        timer::delay_ms(500);
    }
    else if (REOPEN == m_power_state)
    {
        m_servo_io.initServoPositions();
        m_servo_inited = true;
        m_power_state = ON;
        return;
    }
    else
    {
        ROS_ERROR("IOManager3::Surprise mother fucker....");
    }

}

void IOManager3::setAllJointValue(const std::vector<double>& values_)
{
    m_servo_io.setSingleServoPosition("right_hip_yaw", values_[0]);
    m_servo_io.setSingleServoPosition("right_hip_roll", values_[1]);
    m_servo_io.setSingleServoPosition("right_hip_pitch", values_[2]);
    m_servo_io.setSingleServoPosition("right_knee", values_[3]);
    m_servo_io.setSingleServoPosition("right_ankle_pitch", values_[4]);
    m_servo_io.setSingleServoPosition("right_ankle_roll", values_[5]);

    m_servo_io.setSingleServoPosition("left_hip_yaw", values_[6]);
    m_servo_io.setSingleServoPosition("left_hip_roll", values_[7]);
    m_servo_io.setSingleServoPosition("left_hip_pitch", values_[8]);
    m_servo_io.setSingleServoPosition("left_knee", values_[9]);
    m_servo_io.setSingleServoPosition("left_ankle_pitch", values_[10]);
    m_servo_io.setSingleServoPosition("left_ankle_roll", values_[11]);

    if(!LEG_ONLY){
      m_servo_io.setSingleServoPosition("right_arm_upper", values_[12]);
      m_servo_io.setSingleServoPosition("right_arm_lower", values_[13]);
      m_servo_io.setSingleServoPosition("left_arm_upper", values_[14]);
      m_servo_io.setSingleServoPosition("left_arm_lower", values_[15]);
      m_servo_io.setSingleServoPosition("head_pitch", values_[16]);
      m_servo_io.setSingleServoPosition("head_yaw", values_[17]);
    }
}

void IOManager3::SetAllJointValueTimeBase(const std::vector<double>& values_, bool is_time_base_on)
{
    m_servo_io.SetSingleServoPositionTimeBase("right_hip_yaw", values_[0], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("right_hip_roll", values_[1], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("right_hip_pitch", values_[2], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("right_knee", values_[3], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("right_ankle_pitch", values_[4], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("right_ankle_roll", values_[5], is_time_base_on);

    m_servo_io.SetSingleServoPositionTimeBase("left_hip_yaw", values_[6], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("left_hip_roll", values_[7], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("left_hip_pitch", values_[8], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("left_knee", values_[9], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("left_ankle_pitch", values_[10], is_time_base_on);
    m_servo_io.SetSingleServoPositionTimeBase("left_ankle_roll", values_[11], is_time_base_on);

    if(!LEG_ONLY){
      m_servo_io.SetSingleServoPositionTimeBase("right_arm_upper", values_[12], is_time_base_on);
      m_servo_io.SetSingleServoPositionTimeBase("right_arm_lower", values_[13], is_time_base_on);
      m_servo_io.SetSingleServoPositionTimeBase("left_arm_upper", values_[14], is_time_base_on);
      m_servo_io.SetSingleServoPositionTimeBase("left_arm_lower", values_[15], is_time_base_on);
      m_servo_io.SetSingleServoPositionTimeBase("head_pitch", values_[16], is_time_base_on);
      m_servo_io.SetSingleServoPositionTimeBase("head_yaw", values_[17], is_time_base_on);
    }
}

void IOManager3::setJointValue(const std::string name, const double values_)
{
    m_servo_io.setSingleServoPosition(name, values_);
}

void IOManager3::readJointValue()
{
    m_servo_io.readServoPositions();
}

void IOManager3::reverseMotion()
{
    m_servo_io.TorqueOff();
    while(ros::ok()){
        std::cout << "Press f to get a set of current joint value!(or press OTHERS to quit!)" << std::endl;
        char input;
        std::cin >> input;
        if (input != 'f')
          break;
        readJointValue();
    }
}

std::vector<double> IOManager3::readAllPosition()
{
  std::vector<double> data;
  data.push_back(m_servo_io.getReadPos("right_hip_yaw"));
  data.push_back(m_servo_io.getReadPos("right_hip_roll"));
  data.push_back(m_servo_io.getReadPos("right_hip_pitch"));
  data.push_back(m_servo_io.getReadPos("right_knee"));
  data.push_back(m_servo_io.getReadPos("right_ankle_pitch"));
  data.push_back(m_servo_io.getReadPos("right_ankle_roll"));

  data.push_back(m_servo_io.getReadPos("left_hip_yaw"));
  data.push_back(m_servo_io.getReadPos("left_hip_roll"));
  data.push_back(m_servo_io.getReadPos("left_hip_pitch"));
  data.push_back(m_servo_io.getReadPos("left_knee"));
  data.push_back(m_servo_io.getReadPos("left_ankle_pitch"));
  data.push_back(m_servo_io.getReadPos("left_ankle_roll"));

  if(!LEG_ONLY){
    data.push_back(m_servo_io.getReadPos("right_arm_upper"));
    data.push_back(m_servo_io.getReadPos("right_arm_lower"));
    data.push_back(m_servo_io.getReadPos("left_arm_upper"));
    data.push_back(m_servo_io.getReadPos("left_arm_lower"));
  }

  return data;
}

std::vector<double> IOManager3::readAllVel()
{
  std::vector<double> data;
  data.push_back(m_servo_io.getReadVel("right_hip_yaw"));
  data.push_back(m_servo_io.getReadVel("right_hip_roll"));
  data.push_back(m_servo_io.getReadVel("right_hip_pitch"));
  data.push_back(m_servo_io.getReadVel("right_knee"));
  data.push_back(m_servo_io.getReadVel("right_ankle_pitch"));
  data.push_back(m_servo_io.getReadVel("right_ankle_roll"));

  data.push_back(m_servo_io.getReadVel("left_hip_yaw"));
  data.push_back(m_servo_io.getReadVel("left_hip_roll"));
  data.push_back(m_servo_io.getReadVel("left_hip_pitch"));
  data.push_back(m_servo_io.getReadVel("left_knee"));
  data.push_back(m_servo_io.getReadVel("left_ankle_pitch"));
  data.push_back(m_servo_io.getReadVel("left_ankle_roll"));

  if(!LEG_ONLY){
    data.push_back(m_servo_io.getReadVel("right_arm_upper"));
    data.push_back(m_servo_io.getReadVel("right_arm_lower"));
    data.push_back(m_servo_io.getReadVel("left_arm_upper"));
    data.push_back(m_servo_io.getReadVel("left_arm_lower"));
  }

  return data;
}

bool IOManager3::getJointValue(const std::string joint_name, float& value)
{
    auto _joint = m_servo_io.m_joints.find(joint_name);
    if (_joint == m_servo_io.m_joints.end())
    {
        ROS_FATAL("joint not existing... check joint name");
        return false;
    }
    value = _joint->second.real_pos;

    return true;
}

void IOManager3::setServoPI(const std::vector<int> servo_id, const int p, const int i)
{
    m_servo_io.setServoPIMode(servo_id, p, i);
}

void IOManager3::ServoPowerOff()
{
    m_servo_io.TorqueOff();
}

void IOManager3::setAllspeed(int v)
{
    m_servo_io.setAllServoSpeed(v);
}

void IOManager3::readPosVel()
{
  m_servo_io.readServoPosVel();
}

// void IOManager3::setAllTimeBase()
// {
//     m_servo_io.setAllServoTimeBase(true);
// }

void IOManager3::checkIOPower()
{
    ROS_INFO("IOManager3::checkIOPower: checking whether power is on by servo");
    if (m_servo_io.checkPower())
    {
        m_power_state = ON;
    }
    else
    {
        m_power_state = OFF;
    }
}

static void* pthreadIMU(void* arg){
  return static_cast<IOManager3*>(arg)->readIMU();
}
void* IOManager3::readIMU(){
    while(ros::ok())
    {

        if (!m_imu_reader.readIMUData())
        {
            if (m_imu_failures++ > 10)//200ms read fail
            {
                INFO("read IMU failure once");
            }
        }
        else
        {
            m_imu_failures = 0;
            // std::chrono::duration<double> duration_ = (m_imu_reader.m_imu_readBegin - m_imu_sync_time);
            // double ticks = duration_.count()*1000;
            // INFO("*********************************");
            // std::cout << "time: " << ticks << std::endl;
            m_imu_sync_time = m_imu_reader.m_imu_readBegin;
        }
          //  timer::delay_us(6000);
    }
    return NULL;
}

static void* pthreadPow(void* arg){
  return static_cast<IOManager3*>(arg)->checkPower();
}

void* IOManager3::checkPower(){
    int PowerChangeTick = 0;
    while(ros::ok()){
      if(m_power_state == ON)
      {
        //INFO("POWER ON");
        if(m_imu_failures > 10)
        {
          if(PowerChangeTick++ >= 2)
               m_power_state = OFF;
               m_servo_inited = false;
        }
        else
          PowerChangeTick = 0;
      }
     else if(m_power_state == OFF)
     {
       //INFO("POWER OFF");
       if(m_imu_failures == 0)
       {
         if(PowerChangeTick++ >= 2)
              m_power_state = REOPEN;
       }
       else
         PowerChangeTick = 0;
     }
     else
      // INFO("REOPENING...");
    timer::delay_ms(250);
    }
    return NULL;
}

void IOManager3::ResetIniPosRead()
{
    m_servo_io.ResetIniPosRead();
}


}
