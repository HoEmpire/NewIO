#include "dmotion/IO/IMUReader.h"
#include "dmotion/IO/IOManager2.h"

#include "dmotion/Common/Parameters.h"
#include "dmotion/Common/Utility/Utility.h"

// #define IMU_BLOCK 1

namespace Motion
{

IMUReader::IMUReader()
    : m_failures(0)
{
    ROS_DEBUG("IMUReader::IMUReader: init ServoIO instance");
    // init IMU port
    m_imu_port =  IOManager2::initPort(parameters.io.imu_port_name,
                                       parameters.io.imu_baudrate);
}

IMUReader::~IMUReader()
{
    m_imu_port->clearPort();
    m_imu_port->closePort();
    std::cout << "IMU failures " << std::endl;
}

bool IMUReader::checkPower()
{
    ROS_DEBUG("IMUReader::checkPower: entering checkpower");
    if (m_imu_port->waitUntilReadable(100))
    {
        ROS_DEBUG("IMUReader::checkPower: first buffer reading passed, going to double check");
        timer::delay_ms(2.0);

        m_imu_port->clearPort();
        if (m_imu_port->waitUntilReadable(100))
        {
            ROS_DEBUG("IMUReader::checkPower: second buffer received");
            if (readIMUData())
            {
                ROS_DEBUG("IMUReader::checkPower: read imu data success, power is on");
                return true;
            }
        }
    }

    return false;
}

void IMUReader::clearPort()
{
    m_imu_port->clearPort();    
}

std::chrono::time_point<std::chrono::system_clock>
IMUReader::getSyncTimePoint() const
{
    return m_sync_time;
}

bool IMUReader::readIMUData(int waiting_ticks)
{
    static int failure_ = 0;
    uint8_t recBuffer[28];
    char type;
    bool readRes_;
    int ticks_failing_ = 0;

    while (true)
    {
        // if cannot get complete packet for some time 
        // lower board might not on power...
        if (ticks_failing_++ > waiting_ticks)    //TODO() hardcode
        {
            // m_isPowerOn = false;
            m_failures++;
            ROS_WARN("IMUReader::readIMUData: IMU Reading failure...going to exit....");
            return false;
        }
      
        // step 1. receive head
 #ifdef IMU_BLOCK
        // std::cout << "fuck imu block" << std::endl;
        readRes_ = m_imu_port->readPort(recBuffer, 1);
 #else
        readRes_ = m_imu_port->readData1Byte(recBuffer, 1, 3.0);
 #endif 
        if ( !readRes_ || recBuffer[0] != (uint8_t)0xee)
        {
            // ROS_INFO("not cool...");
            continue;
        }

        // try set time point here
        m_sync_time = timer::getCurrentSystemTime();

#ifdef IMU_BLOCK
        readRes_ = m_imu_port->readPort(recBuffer, 1);
 #else
        readRes_ = m_imu_port->readData1Byte(recBuffer, 1, 3.0);
 #endif 
        if ( !readRes_ || recBuffer[0] != (uint8_t)0xee)
        {
            // ROS_INFO_STREAM("not head " << int(recBuffer[0]));
            ROS_DEBUG("IMUReader::readIMUData: not head 2");
            continue;
        }

        // step 2. receiveype
#ifdef IMU_BLOCK
        readRes_ = m_imu_port->readPort(recBuffer, 1);
 #else
        readRes_ = m_imu_port->readData1Byte(recBuffer, 1, 3.0);
 #endif 

        type = recBuffer[0];
        if ((type & 0x00) != 0)
        {
            ROS_DEBUG("IMUReader::readIMUData: type error");
        }
        if (!readRes_ || (type & 0x10) == 0 || (type & 0x40) == 0)
            continue;
        
        // step 3. gypo type
        if ((type & 0x10) != 0)
        {
            uint8_t checksum = 0;
#ifdef IMU_BLOCK
            if(!m_imu_port->readPort(recBuffer, 13))
            {
                ROS_WARN("IMUReader::readIMUData: read imu packet incomplete...");
                continue;
            }
#else
            if(!m_imu_port->readData1Byte(recBuffer, 13, 3.0))
            {
                ROS_WARN("IMUReader::readIMUData: read imu packet incomplete...");
                continue;
            }  
#endif
            // checksum
            for (int i = 0; i < 13; i++)
            {
                checksum += recBuffer[i];
            }
            if ((checksum & 0xff) != 0xff)
            {
                ROS_WARN("IMUReader::readIMUData: imu checksum error!");
                continue;
            }
            _remapIMUData(recBuffer);
        }

        // step 4. extra data
        if ((type & 0x40) != 0)
        {
            uint8_t checksum = 0;
#ifdef IMU_BLOCK            
            if(!m_imu_port->readPort(recBuffer, 9))
            {
                ROS_WARN("IMUReader::readIMUData: read end buffer incomplete...");
                continue;                
            }
#else
            if(!m_imu_port->readData1Byte(recBuffer, 9, 3.0))
            {
                ROS_WARN("IMUReader::readIMUData: read end buffer incomplete...");
                continue;                
            }
#endif
            // checksum
            for (int i = 0; i < 9; i++)
            {
                checksum += recBuffer[i];
            }
            if ((checksum & 0xff) != 0xff)
            {
                ROS_WARN("IMUReader::readIMUData: packet end checksum error");
                continue;
            }
        }

        // step 5. packet end
#ifdef IMU_BLOCK
        readRes_ = m_imu_port->readPort(recBuffer, 1);
 #else
        readRes_ = m_imu_port->readData1Byte(recBuffer, 1, 3.0);
 #endif 
            
        break;
    }

    return true;
}

inline bool IMUReader::_remapIMUData(const uint8_t* buffer)
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

}