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
    friend class IOManager2;
    friend class IOManager3;

    IMUReader();
    ~IMUReader();

    bool checkPower();

    void clearPort();

    // std::chrono::time_point<std::chrono::system_clock>
    // getSyncTimePoint() const;

    bool readIMUData();

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

private:
    inline bool remapIMUData(const uint8_t* buffer);

private:
    int m_failures;
    dynamixel::PortHandler* m_imu_port;

    std::chrono::time_point<std::chrono::system_clock> m_sync_time;
    IMUData m_data;
};

}
