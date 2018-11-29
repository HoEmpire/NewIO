#include "dmotion/IO/FeetSensorIO.h"

#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/IO/IOManager3.h"

#define PORT_NAME "/dev/Servo"
//#define PORT_NAME "/dev/ttyUSB0"
#define BAUDRATE  1000000


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
    m_port->clearPort();
    m_port->writePort(m_tx_packet.data(), m_tx_len);

    // reading left data
    if ( !readSinglePackage(true) )
    {
      if(DEBUG_OUTPUT)
        //INFO("左脚读值失败");
        return false;
    }

    // reading right data
    if ( !readSinglePackage(false) )
    {
      if(DEBUG_OUTPUT)
        //INFO("右脚读值失败");
        return false;
    }
    ROS_DEBUG_STREAM("FeetSensorIO::readPressureData: data reading success" << std::endl
                    << "Left Feet " << m_data.left[0] << ' ' << m_data.left[1] << ' ' << m_data.left[2] << ' ' << m_data.left[3]
                    << "Right Feet " << m_data.right[0] << ' ' << m_data.right[1] << ' ' << m_data.right[2] << ' ' << m_data.right[3]
                    );
    return true;
}

bool FeetSensorIO::readSinglePackage(const bool isLeft)
{
    static int count = 0;
    assert(static_cast<int>(m_tx_packet.size()) == m_tx_len);

    bool com_res_ = m_port->readData1Byte(m_rx_packet.data(), m_rx_len, 5.0);
    //bool com_res_ = m_port->readPort(m_rx_packet.data(), m_rx_len);

    if (!com_res_){
      INFO("脚底读值失败!");
      com_res_ = m_port->readPort(m_rx_packet.data(), m_rx_len);
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
            ROS_WARN("FeetSensorIO::readPressureData: header error!!!");
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
        ROS_WARN("FeetSensorIO::readPressureData: incomplete packet");
        return false;
    }
}

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
