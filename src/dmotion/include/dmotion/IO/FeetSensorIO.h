#pragma once

#include "dmotion/Common/Type.h"

#include "dynamixel_sdk/port_handler.h"

// TODO function for enabling pressure reading
namespace Motion
{

class FeetSensorIO
{
public:
    friend class IOManager3;
    friend class IOManager2;

    FeetSensorIO();

    explicit FeetSensorIO(dynamixel::PortHandler* port);

    ~FeetSensorIO();

    void setPort(dynamixel::PortHandler* port)
    {
        m_port = port;
    }

    bool readPressureData();

    bool readSinglePackage(const bool isLeft);

    const PressureData& getPressureData() const
    {
        return m_data;
    }

    void remapPressureData(const bool isLeft);

private:
    dynamixel::PortHandler* m_port;

    PressureData m_data;

    // tx data
    std::vector<uint8_t> m_tx_packet, m_rx_packet;
    int m_tx_len, m_rx_len;
};

}
