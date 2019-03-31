#include <stdio.h>
#include "dmotion/IO/IMUReader.h"
//#include "dmotion/IO/IOManager2.h"
#include "dmotion/IO/IOManager3.h"

#include "dmotion/Common/Parameters.h"
#include "dmotion/Common/Utility/Utility.h"

#define WaitingTicks 20000//the max time of read failure
#define PORT_NAME "/dev/IMU"
#define BAUDRATE  576000
// #define PORT_NAME "/dev/ttyUSB0"
// #define BAUDRATE  3000000


namespace Motion
{

IMUReader::IMUReader()
    : m_failures(0)
{
    ROS_DEBUG("IMUReader::IMUReader: init ServoIO instance");
    // init IMU port
    m_imu_port =  IOManager3::initPort(PORT_NAME, BAUDRATE);
}

IMUReader::~IMUReader()
{
    m_imu_port->clearPort();
    m_imu_port->closePort();
    INFO("IMU failures");
}

bool IMUReader::checkPower()
{
    int m_imu_success = 0;
    int ticks = 0;
    while(ticks++ < 10){
      if (readIMUData())
      {
          if (m_imu_success++ > 2)
          {
            return true;
          }
      }
      else
      {
          m_imu_success = 0;
      }
      timer::delay_ms(10.0);
    }
    return false;
}

void IMUReader::clearPort()
{
    m_imu_port->clearPort();
}

bool IMUReader::readIMUData()
{

      typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> microClock_type;
      microClock_type tp;
      //获取当前时间点，windows system_clock是100纳秒级别的(不同系统不一样，自己按照介绍的方法测试)，所以要转换

      /**************** buffer initial ****************/
      const unsigned char gyro_preamble[3] = {0xEE, 0xEE, 0x50};

      uint8_t byte_buffer;
      uint8_t datas_buffer[28];
      //header:{0xEE, 0xEE, 0x50} (1Byte x3)
      //accl_x | accl_y | accl_z | grpo_x | grpo_y | grpo_z | checksum (2Byte x6 +1)
      //magn | checksum (8Byte + 1)
      //packet_end (1Byte)

      int state = 0;
      int cnt_read_num = 0;
      int same_cnt = 0;
      bool read_flag = 0;
      int power_tick = 0;

      std::chrono::time_point<std::chrono::steady_clock> m_imu_readBegin_tmp;
      timer a;
      while(true)
      {
        while(!m_imu_port->readPort(&byte_buffer, 1))
        {
          timer::delay_us(10);
          power_tick++;
          if(power_tick > 2000){
              INFO("tick overflow");
              //a.toc();
              return false;// 20ms read failed
          }
        }
        power_tick = 0;
        switch(state)
        {
          case 0:
          {
            if(byte_buffer == gyro_preamble[0])
            {
              //INFO("CASE 0");
              state = 1;
              m_imu_readBegin_tmp = timer::getCurrentSystemTime();
              //a.tic();
            }
            else
            {
              //printf("Gyro:Read Byte %2x, number: %d, state: %d \n",byte_buffer,same_cnt,state);
              //std::cout << "Gyro:Read Byte " << std::hex << byte_buffer << std::dec <<",number "<< same_cnt << ",state "<< state << std::endl;
              m_imu_port->clearPort();
            }
          }
          break;
          case 1:
          {
            if(byte_buffer == gyro_preamble[1])
            {
              //INFO("CASE 1");
              state = 2;
            }
            else
            {
              printf("Gyro:Read Byte %2x, number: %d, state: %d \n",byte_buffer,same_cnt,state);
              //std::cout << "Gyro:Read Byte " << std::hex << byte_buffer << std::dec <<",number "<< same_cnt << ",state "<< state << std::endl;
              state = 0;
            }
          }
          break;
          case 2:
          {
            if(byte_buffer == gyro_preamble[2])
            {
              //INFO("CASE 2");
              state = 3;
              cnt_read_num = 0;
            }
            else
            {
              printf("Gyro:Read Byte %2x, number: %d, state: %d \n",byte_buffer,same_cnt,state);
              //std::cout << "Gyro:Read Byte " << std::hex << byte_buffer << std::dec <<",number "<< same_cnt << ",state "<< state << std::endl;
              state = 0;
            }
          }
          break;
          case 3:
          {
            //INFO("CASE 3");
            datas_buffer[cnt_read_num++] = byte_buffer;
            if(cnt_read_num == 25)
            {
              read_flag = 1;
              state = 0;
            }
          }
        }

      if(read_flag == 0)
        continue;
      read_flag = 0;


       #if 0
         for(i = 0;i<46;i++){
           printf("%02x,",datas_buffer[i]);
         }
         printf("\n");
       #endif
       int i = 0;
       uint8_t sum_acc = 0;
       uint8_t sum_magn = 0;
       for(i = 0;i < 13;i++){
         sum_acc += datas_buffer[i];
       }

       for(i = 13;i < 22;i++){
         sum_magn += datas_buffer[i];
       }
       if(((sum_acc & 0xff) == 0xff) && ((sum_magn & 0xff) == 0xff)){
         remapIMUData(datas_buffer);
         //INFO("remap true.");
       }
       else{
         INFO("check sum failed.");
         return false;
       }
       m_imu_readBegin = m_imu_readBegin_tmp;
       m_imu_port->clearPort();
      // a.toc();
       return true;
      }

}


inline bool IMUReader::remapIMUData(const uint8_t* buffer)
{
    unsigned short int tmp1[12];
    for (int i = 0; i < 12; i++)
    {
        tmp1[i] = buffer[i];
    }

    unsigned short int tmp2[6];
    for (int i = 0; i < 6; i++)
    {
        tmp2[i] = ((tmp1[2 * i] << 8) & 0xff00) + (tmp1[2 * i + 1] & 0x00ff);
    }

    short int tmp[6];
    for (int i = 0; i < 6; i++)
    {
        if (tmp2[i] > 0xb000)
        {
            tmp[i] = (tmp2[i] | 0xc000);
        }
        else
        {
            tmp[i] = (tmp2[i] & 0x3fff);
        }
    }

    if (tmp[0] == 0 && tmp[1] == 0 && tmp[2] == 0 && tmp[3] == 0 && tmp[4] == 0 && tmp[5] == 0) // if error, just be equal to the former
    {
        ROS_ERROR("IMUReader::_remapIMUData: IMU Data all zero...");
        return false;
    }

    if (parameters.io.imu_version == "ADIS16365BMLZ") //  ADIS 16365BMLZ
    {
        m_data.gypo.x = tmp[0] * 0.05 / 180 * M_PI;
        m_data.gypo.y = tmp[1] * 0.05 / 180 * M_PI;
        m_data.gypo.z = tmp[2] * 0.05 / 180 * M_PI;
        m_data.accl.x = tmp[3] * 0.00333 * 9.8;
        m_data.accl.y = tmp[4] * 0.00333 * 9.8;
        m_data.accl.z = tmp[5] * 0.00333 * 9.8;
    }
    else if (parameters.io.imu_version == "ADIS16355AMLZ") // ADIS 16355AMLZ default
    {
        m_data.gypo.x = tmp[0] * 0.07326 / 180 * M_PI;
        m_data.gypo.y = tmp[1] * 0.07326 / 180 * M_PI;
        m_data.gypo.z = tmp[2] * 0.07326 / 180 * M_PI;
        m_data.accl.x = tmp[3] * 0.002522 * 9.8;
        m_data.accl.y = tmp[4] * 0.002522 * 9.8;
        m_data.accl.z = tmp[5] * 0.002522 * 9.8;
    }
    else if (parameters.io.imu_version == "ADIS16405BMLZ") // ADIS 16355AMLZ default
    {
        m_data.gypo.x = tmp[0] * 0.05 / 180 * M_PI;
        m_data.gypo.y = tmp[1] * 0.05 / 180 * M_PI;
        m_data.gypo.z = tmp[2] * 0.05 / 180 * M_PI;
        m_data.accl.x = tmp[3] * 0.00333 * 9.8;
        m_data.accl.y = tmp[4] * 0.00333 * 9.8;
        m_data.accl.z = tmp[5] * 0.00333 * 9.8;
    }
    else
    {
        ROS_ERROR("IMUReader::_remapIMUData: ERROR IMU TYPE");
    }

    return true;
}

void IMUReader::test_imu()
{
  uint8_t byte_buffer;
  timer a;
  while(ros::ok())
  {
    a.tic();
    while(!m_imu_port->readPort(&byte_buffer, 1))
    {
      timer::delay_us(10);
    }
    m_imu_port->clearPort();
    a.toc();
  }
}


}
