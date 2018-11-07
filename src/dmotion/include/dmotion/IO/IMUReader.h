#pragma once

#include "../Common/Type.h"

#include "dynamixel_sdk/port_handler.h"

#include <chrono>

namespace Motion
{

class IMUReader
{
public:
    friend class IOManager2;
    
    IMUReader();    
    ~IMUReader();

    bool checkPower();

    void clearPort();

    std::chrono::time_point<std::chrono::system_clock>
    getSyncTimePoint() const;

    bool readIMUData(int waiting_ticks = 10);

    const IMUData& getIMUData() const
    {
        return m_data;
    }

private:
    inline bool _remapIMUData(const uint8_t* buffer);

private:
    int m_failures;
    dynamixel::PortHandler* m_imu_port;
    
    std::chrono::time_point<std::chrono::system_clock> m_sync_time;
    IMUData m_data;
};

}