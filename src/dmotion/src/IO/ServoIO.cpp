#include "dmotion/IO/ServoIO.h"

//#include "dmotion/IO/IOManager2.h"
#include "dmotion/IO/IOManager3.h"

#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
using namespace dynamixel;
#define SAFE_MODE false //if true, then if will move slower to avoid unexpected damadges
#define PORT_NAME "/dev/ttyUSB0"
#define BAUDRATE  1000000
#define PROTOCOL_VERSION 2.0
#define INIT_TICKS 10 //the total time that keep low speed | unit:10ms
#define INIT_VELLOCITY 30
#define SAFE_MODE_SPEED 50

namespace Motion
{

ServoIO::ServoIO()
    : m_writer_inited(false)
    , m_servo_inited(false)
{
    INFO("ServoIO::ServoIO: init ServoIO instance");

    // init port
    m_servo_port = IOManager3::initPort(PORT_NAME, BAUDRATE);

    // init protocol handler
    m_servo_protocol = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    m_pos_writer = new GroupSyncWrite(m_servo_port, m_servo_protocol, ADDR_GOAL_POSITION, LENGTH_POSITION);
    m_pos_reader = new GroupSyncRead(m_servo_port, m_servo_protocol, ADDR_CURR_POSITION, LENGTH_POSITION);
    //m_pos_reader = new GroupBulkRead(m_servo_port, m_servo_protocol);
}

ServoIO::~ServoIO()
{
    INFO("ServoIO:~ServoIO: destruct ServoIO instance");
    m_servo_port->closePort();
}

void ServoIO::addJoint(std::string name, Joint Joints_cfg)
{
    std::map<std::string, Joint>::iterator it = m_joints.begin();
    it = m_joints.find(name);
    if(it == m_joints.end()){
      m_joints.insert(std::pair<std::string, Joint>(name, Joints_cfg));
      std::cout << "INFO:add " << name << " success!" <<std::endl;
    }
    else{
      std::cout << "WARNING:" << name << " already exists" << std::endl;
      std::cout << "WARNING:ADD NEW JOINT FAILED!" << std::endl;
    }

}

void ServoIO::initServoPositions()
{
    INFO("ServoIO::initServoPositions: init all servos");
    for (auto& joint:m_joints)
    {
        static uint8_t goal_position_[4];
        const JointConfig& _cfg = joint.second.cfg;

        // servo action: enable torque && turn on led && set velocity
        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_RETURN_LEVEL, 1);
        timer::delay_ms(20);    // no idea why here should be some lantency...
        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_TORQUE_ENABLE, 1);
        timer::delay_ms(20);
        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_LED, 1);
        timer::delay_ms(20);
        // m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_RETURN_DELAY, 10);
        // timer::delay_ms(20);
        // m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_RETURN_LEVEL, 1);
        // timer::delay_ms(20);
        m_servo_protocol->write4ByteTxOnly(m_servo_port, _cfg.id, ADDR_PROFILE_VELOCITY, INIT_VELLOCITY);//init safety

        goal_position_[0] = DXL_LOBYTE(DXL_LOWORD(_cfg.init));
        goal_position_[1] = DXL_HIBYTE(DXL_LOWORD(_cfg.init));
        goal_position_[2] = DXL_LOBYTE(DXL_HIWORD(_cfg.init));
        goal_position_[3] = DXL_HIBYTE(DXL_HIWORD(_cfg.init));

        if (!m_writer_inited)
        {
            bool state = m_pos_writer->addParam(_cfg.id, goal_position_);
            if (!state)
            {
                ROS_ERROR_STREAM("ServoIO::initServoPositions: id " << _cfg.id << " add goal position param error");
            }
        }
        else
        {
            bool state = m_pos_writer->changeParam(_cfg.id, goal_position_);
            if (!state)
            {
                ROS_ERROR_STREAM("ServoIO::initServoPositions: id " << _cfg.id << " change goal position param error");
            }
        }

        bool state = m_pos_reader->addParam(_cfg.id);
        //bool state = m_pos_reader->addParam(_cfg.id, ADDR_CURR_POSITION, LENGTH_POSITION);
        if (!state)
        {
            ROS_ERROR_STREAM("ServoIO::initServoPositions: id " << _cfg.id << " add read position param error");
        }


    }

    m_pos_writer->txPacket();

    // turn off led
    for (auto& joint:m_joints)
    {
        const JointConfig& _cfg = joint.second.cfg;

        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_LED, 0);
    }

//    m_pos_writer->txPacket();

    m_writer_inited = true;
    m_servo_inited = false;
    sleep(1);
    INFO("Servo ini success!!!!!");
}

void ServoIO::TorqueOff()
{
    INFO("turn off the torque of the servos!");
    for (auto& joint:m_joints)
    {
        const JointConfig& _cfg = joint.second.cfg;
        m_servo_protocol->write1ByteTxOnly(m_servo_port, _cfg.id, ADDR_TORQUE_ENABLE, 0);
    }
    sleep(3);
    INFO("torque off ist schon gemacht!");

}

void ServoIO::sendServoPositions()
{
    static uint8_t goal_position_buffer[4];
    static int goal_position_;
    static int init_ticks = 0;

    //INFO("ServoIO::sendServoPositions: send servo positions");

    if (!m_servo_inited)
    {
        // if servo init done, just reset speed
        // power safety
        if (init_ticks > INIT_TICKS)
        {
            if (SAFE_MODE)// TODO pyx 测试模式
              setAllServoSpeed(SAFE_MODE_SPEED);
            else
              setAllServoSpeed();

            init_ticks = 0;
            m_servo_inited = true;
        }
        else
        {
            init_ticks++;
        }
    }

    for (auto& joint:m_joints)
    {
        const Joint& _j = joint.second;
        const JointConfig& _cfg = _j.cfg;

        goal_position_ = _cfg.init + static_cast<int>(_j.goal_pos*_cfg.factor);
        //std::cout << "goal:" << goal_position_ << std::endl;

        goal_position_buffer[0] = DXL_LOBYTE(DXL_LOWORD(goal_position_));
        goal_position_buffer[1] = DXL_HIBYTE(DXL_LOWORD(goal_position_));
        goal_position_buffer[2] = DXL_LOBYTE(DXL_HIWORD(goal_position_));
        goal_position_buffer[3] = DXL_HIBYTE(DXL_HIWORD(goal_position_));

        m_pos_writer->changeParam(_cfg.id, goal_position_buffer);
    }

    m_pos_writer->txPacket();
}

void ServoIO::readServoPositions()
{
    INFO("ServoIO::readServoPositions: read servo positions");
    //m_pos_reader->txRxPacket();
    //timer::delay_ms(100);
    //
    auto dxl_comm_result = m_pos_reader->txRxPacket();

    int cunt = 0;
    while (dxl_comm_result != COMM_SUCCESS && cunt < 100)
    {
      INFO("fucking reading error");
      //INFO(dpacketHandler->getTxRxResult(dxl_comm_result
      dxl_comm_result = m_pos_reader->txRxPacket();
      cunt ++;
    }


    for (auto& joint:m_joints)
    {
        const JointConfig& _cfg = joint.second.cfg;
        if (!m_pos_reader->isAvailable(_cfg.id, ADDR_CURR_POSITION, LENGTH_POSITION))
        {
           INFO ("ServoIO::readServoPositions: get current joint value error");
        }
        else
        {
            joint.second.real_pos = (static_cast<int>(m_pos_reader->getData(_cfg.id, ADDR_CURR_POSITION, LENGTH_POSITION))-_cfg.init)/_cfg.factor;
            std::cout.setf(std::ios::left);
            std::cout << std::setprecision (2);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);
            std::cout << "ServoName：" << std::setfill(' ') << std::setw(17) << joint.first.c_str()
                      << " | " << "pos:" << joint.second.real_pos << std::endl;
        }
    }
}

void ServoIO::readServoPositionsBad()
{
    INFO("ServoIO::readServoPositions: read servo positions");
    m_pos_reader->txRxPacket();
    //timer::delay_ms(100);
    for (auto& joint:m_joints)
    {
        const JointConfig& _cfg = joint.second.cfg;
        uint8_t dxl_error = 0;                          // Dynamixel error
        uint32_t dxl_present_position = 0;               // Present position
        bool state = m_servo_protocol->read4ByteTxRx(m_servo_port, _cfg.id, ADDR_CURR_POSITION, &dxl_present_position, &dxl_error);
        if (state)
        {
           INFO ("ServoIO::readServoPositions: get current joint value error");
        }
        else
        {
            joint.second.real_pos = (static_cast<int>(dxl_present_position)-_cfg.init)/_cfg.factor;
            std::cout << "ServoName：" << joint.first.c_str() << " | "  << "pos:"<< joint.second.real_pos << std::endl;
        }
        //timer::delay_ms(10);
    }
}

void ServoIO::setAllServoSpeed(const int speed)
{
    std::cout << "ServoIO::setAllServoSpeed: set servo speed to " << speed << std::endl;
    for (auto& joint:m_joints)
    {
        const JointConfig& _cfg = joint.second.cfg;

        m_servo_protocol->write4ByteTxOnly(m_servo_port, _cfg.id, ADDR_PROFILE_VELOCITY, speed);
    }
}

void ServoIO::setSingleServoSpeed(std::string name, int speed)
{
    //std::cout << "ServoIO::setSingleServoSpeed: set servo speed to " << speed << std::endl;

    std::map<std::string, Joint>::iterator it = m_joints.begin();
    it = m_joints.find(name);
    if(it != m_joints.end())
    {
        const JointConfig& _cfg = (*it).second.cfg;
        std::cout << "INFO:find " << name << " success!" << " | id::" << _cfg.id <<std::endl;
        m_servo_protocol->write4ByteTxOnly(m_servo_port, _cfg.id, ADDR_PROFILE_VELOCITY, speed);
    }
    else
    {
        std::cout << "WARNING:" << name << " not found!" << std::endl;
    }
}

void ServoIO::setSingleServoSpeed(int servo_id,  int speed)
{
  //  std::cout << "ServoIO::setSingleServoSpeed: set servo speed to " << speed << std::endl;
    m_servo_protocol->write4ByteTxOnly(m_servo_port, servo_id, ADDR_PROFILE_VELOCITY, speed);
}

void ServoIO::setSingleServoPosition(std::string name, double position)
{
    //std::cout << "ServoIO::setSingleServoSpeed: set goal postion of | " << name << " | " << std::endl;

    std::map<std::string, Joint>::iterator it = m_joints.begin();
    it = m_joints.find(name);
    if(it != m_joints.end())
    {
        //const JointConfig& _cfg = (*it).second.cfg;
        //std::cout << "INFO:find " << name << " success!" << " | id::" << _cfg.id <<std::endl;
        (*it).second.goal_pos = position;
    }
    else
    {
        std::cout << "WARNING:" << name << " not found!" << std::endl;
    }
}


void ServoIO::setServoPIMode(std::vector<int> servo_id, const int P, const int I)
{
    static uint8_t pi_buffer[4];

    std::cout << "ServoIO::setServoPIMode: set servo P : " << P << " set servo I " << I << std::endl;

    dynamixel::GroupSyncWrite writer_(m_servo_port, m_servo_protocol, ADDR_PI, LENGTH_PI);

    pi_buffer[0] = DXL_LOBYTE(I);
    pi_buffer[1] = DXL_HIBYTE(I);
    pi_buffer[2] = DXL_LOBYTE(P);
    pi_buffer[3] = DXL_HIBYTE(P);

    for(int i = 0; i < int(servo_id.size()); i++)
    {
        writer_.addParam(servo_id[i], pi_buffer);
    }

    writer_.txPacket();
}

}
