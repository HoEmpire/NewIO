#pragma once

#include "../Common/Type.h"

#include "dynamixel_sdk/port_handler.h"
#include "dmotion/Common/Utility/Utility.h"

#include <chrono>


namespace Motion
{

class IMUReader
{
public:
    friend class IOManager3;

    IMUReader();
    ~IMUReader();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        check time by checking whether there is data coming from imu
    /// @return       true if power is on;false if power is offf
    ////////////////////////////////////////////////////////////////////////////////
    bool checkPower();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        clear the port
    ////////////////////////////////////////////////////////////////////////////////
    void clearPort();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        read imu
    /// @return       true if reading data successed;false if reading data failed(power off or something else )
    ////////////////////////////////////////////////////////////////////////////////
    bool readIMUData();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        get the private imu data
    /// @return       imu data
    ////////////////////////////////////////////////////////////////////////////////
    const IMUData& getIMUData() const
    {
        if(DEBUG_OUTPUT){
          std::cout << std::setprecision (3);
          std::cout << "grpo_x: " << m_data.gypo.x <<std::endl;
          std::cout << "grpo_y: " << m_data.gypo.y <<std::endl;
          std::cout << "grpo_z: " << m_data.gypo.z <<std::endl;
          std::cout << "accl_x: " << m_data.accl.x <<std::endl;
          std::cout << "accl_y: " << m_data.accl.y <<std::endl;
          std::cout << "accl_z: " << m_data.accl.z <<std::endl;
        }
        return m_data;
    }

    void test_imu();

public:
    std::chrono::time_point<std::chrono::steady_clock> m_imu_readBegin;
    // std::chrono::time_point<std::chrono::steady_clock> m_sync_time;

private:
    inline bool remapIMUData(const uint8_t* buffer);//converse data to the form that we can read from binary

private:
    int m_failures;//read failure time
    dynamixel::PortHandler* m_imu_port;

    std::chrono::time_point<std::chrono::steady_clock> m_sync_time;
    IMUData m_data;//imu data
};

}
