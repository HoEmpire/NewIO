#include "dmotion/IO/FeetSensorIO.h"
#include "dmotion/IO/IOManager3.h"

#define PORT_NAME "/dev/Servo"
#define BAUDRATE  1000000
// #define PORT_NAME "/dev/ttyUSB0"
// #define BAUDRATE  3000000
#define old 1

namespace Motion
{
// TODO solving hard codes...
FeetSensorIO::FeetSensorIO()
    : m_tx_len(8)
    , m_rx_len(22)
{
    m_tx_packet.resize(m_tx_len);
    m_rx_packet.resize(m_rx_len);

    m_tx_packet[0] = 0xff;
    m_tx_packet[1] = 0xff;
    m_tx_packet[2] = 0x17;
    m_tx_packet[3] = 0x04;
    m_tx_packet[4] = 0x00;
    m_tx_packet[5] = 0x01;
    m_tx_packet[6] = 0x02;
    m_tx_packet[7] = 0xE1;
    m_port = IOManager3::initPort(PORT_NAME, BAUDRATE);
}

FeetSensorIO::FeetSensorIO(dynamixel::PortHandler* port)
    : m_port(port)
    , m_tx_len(8)
    , m_rx_len(22)
{
    m_tx_packet.resize(m_tx_len);
    m_rx_packet.resize(m_rx_len);

    m_tx_packet[0] = 0xff;
    m_tx_packet[1] = 0xff;
    m_tx_packet[2] = 0x17;
    m_tx_packet[3] = 0x04;
    m_tx_packet[4] = 0x00;
    m_tx_packet[5] = 0x01;
    m_tx_packet[6] = 0x02;
    m_tx_packet[7] = 0xE1;
}

FeetSensorIO::~FeetSensorIO() = default;

inline float deserialize(uint8_t *inbuffer)
{
    int offset = 0;
    float data;
    union {
        float real;
        uint32_t base;
    } u_data;
    u_data.base = 0;
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    data = u_data.real;
    offset += sizeof(data);

    return data;
}

bool FeetSensorIO::readPressureData()
{
    // clear port and send command
    // static int cunt = 0;
    m_port->clearPort();
    m_port->writePort(m_tx_packet.data(), m_tx_len);
    // m_port->clearPort();
    //
    // uint8_t byte_buffer;
    // uint8_t datas_buffer[21] = {0};
    //     m_port->dynamixel::PortHandler::SetMultiByteBlock(21);
    //     m_port->readPort(datas_buffer, 21);
    //     std::cout << "left  foot: ";
    //     for(int i = 0; i < 21; i++){
    //       std::cout << std::dec << i << ":" << std::hex << datas_buffer[i];
    //       if(i != 20) std::cout << " ";
    //       else std::cout << std::endl;
    //     }
    //     m_port->clearPort();
    //
    //
    //     m_port->readPort(datas_buffer, 21);
    //     std::cout << "right foot: ";
    //     for(int i = 0; i < 21; i++){
    //       std::cout << std::dec << i << ":" << std::hex << datas_buffer[i];
    //       if(i != 20) std::cout << " ";
    //       else std::cout << std::endl;
    //     }
    //     m_port->clearPort();

    //reading left data
    if ( !readSinglePackage(true) )
    {
      //if(DEBUG_OUTPUT)
      //  INFO("左脚读值失败");
        return false;
    }

    //m_port->writePort(m_tx_packet.data(), m_tx_len);
    // reading right data
    if ( !readSinglePackage(false) )
    {
      //if(DEBUG_OUTPUT)
      //  INFO("右脚读值失败");
        return false;
    }

    // if(cunt < 100)
    //    cunt++;
    // else
    // {
    //    cunt = 0;
    //    ROS_INFO_STREAM("FeetSensorIO::readPressureData: data reading success" << std::endl
    //                    << "Left Feet: " << m_data.left[0] << ' ' << m_data.left[1] << ' ' << m_data.left[2] << ' ' << m_data.left[3]
    //                    << "Right Feet: " << m_data.right[0] << ' ' << m_data.right[1] << ' ' << m_data.right[2] << ' ' << m_data.right[3]
    //                    );
    // }


    return true;
}

#ifdef old
bool FeetSensorIO::readSinglePackage(const bool isLeft)
{
    static int count = 0;
    assert(static_cast<int>(m_tx_packet.size()) == m_tx_len);

    bool com_res_ = m_port->readData1Byte(m_rx_packet.data(), m_rx_len, 4.0);//TODO
    //bool com_res_ = m_port->readPort(m_rx_packet.data(), m_rx_len);

    if (!com_res_)
    {
      INFO("脚底读值失败!");
    //com_res_ = m_port->readPort(m_rx_packet.data(), m_rx_len);
    //std::abort();
    }

    if (com_res_)
    {
        uint8_t check_sum_ = 0;

        for (int i = 2; i != 22; i++)
            check_sum_ += m_rx_packet[i];

        if (m_rx_packet[0] != 0xff ||
            m_rx_packet[1] != 0xff ||
            m_rx_packet[3] != 0x12 ||
            m_rx_packet[4] != 0x00
            || check_sum_ != 0xff
            || (isLeft && m_rx_packet[2] != 0x18)
            || (!isLeft && m_rx_packet[2] != 0x19)
            )
        {
            count ++;
          //  ROS_WARN("FeetSensorIO::readPressureData: header error!!!");
            // exit(0);
            m_port->clearPort();
            return false;
        }

        remapPressureData(isLeft);
        return true;
    }
    else
    {
        m_port->clearPort();
      //  ROS_WARN("FeetSensorIO::readPressureData: incomplete packet");
        return false;
    }
}

#else
// bool FeetSensorIO::readSinglePackage(const bool isLeft){
//
//
//       /**************** buffer initial ****************/
//       uint8_t byte_buffer;
//       uint8_t datas_buffer[22];
//       //header:{0xEE, 0xEE, 0x50} (1Byte x3)
//       //accl_x | accl_y | accl_z | grpo_x | grpo_y | grpo_z | checksum (2Byte x6 +1)
//       //magn | checksum (8Byte + 1)
//       //packet_end (1Byte)
//
//       int state = 0;
//       int cnt_read_num = 0;
//       int same_cnt = 0;
//       bool read_flag = 0;
//       //std::chrono::time_point<std::chrono::steady_clock> StartTime;
//       //std::chrono::time_point<std::chrono::steady_clock> EndTime;
//       //std::chrono::duration<double> WaitTime;
//
//     //  StartTime = timer::getCurrentSystemTime();
//       timer a, b;int cnt1 = 0;
//       while(ros::ok())
//       {
//         a.tic();b.tic();
//         while(!m_port->readPort(&byte_buffer, 1))
//         {
//           timer::delay_us(25);
//         //  EndTime = timer::getCurrentSystemTime();
//         //  WaitTime = EndTime - StartTime;
//           // if(a.toc_no_output() > 4.0){
//           //     INFO("pressure sensor:after 4ms read failed");
//           //     return false;// 2ms read failed
//           // }
//         }
//         a.toc();cnt1++;
//         std::cout << a.toc_no_output() << " ***** "<< std::hex << unsigned(byte_buffer) << std::endl;
//         if(cnt1 == 44) {
//           b.toc();
//           cnt1 = 0;
//         }
//       //   switch(state)
//       //   {
//       //     case 0:
//       //     {
//       //       if(byte_buffer == 0xff)
//       //       {
//       //         //INFO("CASE 0");
//       //         state = 1;
//       //         datas_buffer[cnt_read_num++] = byte_buffer;
//       //
//       //         //a.tic();
//       //       }
//       //       else
//       //       {
//       //         cnt_read_num = 0;
//       //         std::cout << "pressure sensor:Read Byte " << std::hex << byte_buffer;
//       //         std::cout << std::dec <<",number "<< same_cnt << ",state "<< state << std::endl;
//       //         //m_port->clearPort();//TODO 2019.3.31修改
//       //       }
//       //     }
//       //     break;
//       //     case 1:
//       //     {
//       //       if(byte_buffer == 0xff)
//       //       {
//       //         //INFO("CASE 1");
//       //         state = 2;
//       //         datas_buffer[cnt_read_num++] = byte_buffer;
//       //       }
//       //       else
//       //       {
//       //         std::cout << "pressure sensor:Read Byte " << std::hex << byte_buffer << std::dec <<",number "<< same_cnt << ",state "<< state << std::endl;
//       //         state = 0;
//       //         cnt_read_num = 0;
//       //       }
//       //     }
//       //     break;
//       //     case 2:
//       //     {
//       //       if((isLeft && byte_buffer == 0x18) || (!isLeft && byte_buffer == 0x19))
//       //       {
//       //         //INFO("CASE 2");
//       //         state = 3;
//       //         datas_buffer[cnt_read_num++] = byte_buffer;
//       //       }
//       //       else
//       //       {
//       //         std::cout << "pressure sensor:Read Byte " << std::hex << byte_buffer << std::dec <<",number "<< same_cnt << ",state "<< state << std::endl;
//       //         state = 0;
//       //         cnt_read_num = 0;
//       //       }
//       //     }
//       //     break;
//       //     case 3:
//       //     {
//       //       if(byte_buffer == 0x12)
//       //       {
//       //         //INFO("CASE 3");
//       //         state = 4;
//       //         datas_buffer[cnt_read_num++] = byte_buffer;
//       //       }
//       //       else
//       //       {
//       //         std::cout << "pressure sensor:Read Byte " << std::hex << byte_buffer << std::dec <<",number "<< same_cnt << ",state "<< state << std::endl;
//       //         state = 0;
//       //         cnt_read_num = 0;
//       //       }
//       //     }
//       //     break;
//       //     case 4:
//       //     {
//       //       if(byte_buffer == 0x00)
//       //       {
//       //         //INFO("CASE 4");
//       //         state = 5;
//       //         datas_buffer[cnt_read_num++] = byte_buffer;
//       //       }
//       //       else
//       //       {
//       //         std::cout << "pressure sensor:Read Byte " << std::hex << byte_buffer << std::dec <<",number "<< same_cnt << ",state "<< state << std::endl;
//       //         state = 0;
//       //         cnt_read_num = 0;
//       //       }
//       //     }
//       //     break;
//       //     case 5:
//       //     {
//       //       //INFO("CASE 5");
//       //       datas_buffer[cnt_read_num++] = byte_buffer;
//       //       if(cnt_read_num == 22)
//       //       {
//       //         read_flag = 1;
//       //         state = 0;
//       //       }
//       //     }
//       //   }
//       //
//       // if(read_flag == 0)
//       //   continue;
//       // read_flag = 0;
//       //
//       //  int i = 0;
//       //  uint8_t sum = 0;
//       //
//       //  for(i = 2;i < 22;i++){
//       //    sum += datas_buffer[i];
//       //  }
//       //
//       //  if(sum == 0xff){
//       //    for(i = 0;i< 22;i++)
//       //        m_rx_packet[i] = datas_buffer[i];
//       //
//       //    remapPressureData(isLeft);
//       //    //INFO("pressure sensor:remap true.");
//       //  }
//       //  else{
//       //    INFO("pressure sensor:check sum failed.");
//       //    m_port->clearPort();
//       //    return false;
//       //  }
//       //  m_port->clearPort();
//        // return true;
//       }
//       return false;
// }
bool FeetSensorIO::readSinglePackage(const bool isLeft){
      uint8_t byte_buffer = 0x00;
      uint8_t datas_buffer[22] = {0};

      int state = 0;
      int cnt_read_num = 0;
      int same_cnt = 0;
      bool read_flag = 0;
      int length;
      //timer a;
      while(ros::ok())
      {
        //a.tic();
        m_port->dynamixel::PortHandler::SetSingleByteBlock();
        m_port->readPort(&byte_buffer, 1);
        if(byte_buffer == 0xff)
        {
            datas_buffer[0] = byte_buffer;
            m_port->dynamixel::PortHandler::SetMultiByteBlock(21);
            length = m_port->readPort(datas_buffer + 1, 21);

            if(length == 21)
            {
              for(int i = 0; i < 22; i++){
                std::cout << std::dec << i << ":" << std::hex << datas_buffer[i];
                if(i != 21) std::cout << " ";
                else std::cout << std::endl;
              }
              return true;
            }
        }
        else
          continue;
      }
      return false;
}
#endif




void FeetSensorIO::remapPressureData(const bool isLeft)
{
    if ( isLeft )
    {
        for (int i = 0; i != 4; ++i)
        {
            m_data.left[i] = deserialize(m_rx_packet.data()+5+i*4);
        }
    }
    else
    {
        for (int i = 0; i != 4; ++i)
        {
            m_data.right[i] = deserialize(m_rx_packet.data()+5+i*4);
        }
    }
}

}
