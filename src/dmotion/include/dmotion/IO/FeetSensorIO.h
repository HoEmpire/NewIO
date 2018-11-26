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

    FeetSensorIO();

    explicit FeetSensorIO(dynamixel::PortHandler* port);

    ~FeetSensorIO();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        set the port of foot sensor
    ////////////////////////////////////////////////////////////////////////////////
    void setPort(dynamixel::PortHandler* port)
    {
        m_port = port;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        read data from both foot(level hight than readSinglePackage)
    /// @return       true if read successed;false if not
    ////////////////////////////////////////////////////////////////////////////////
    bool readPressureData();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        read data from one foot sensor
    /// @return       true if read successed;false if not
    ////////////////////////////////////////////////////////////////////////////////
    bool readSinglePackage(const bool isLeft);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        get the private foot sensor data
    /// @return       sensor data
    ////////////////////////////////////////////////////////////////////////////////
    const PressureData& getPressureData() const
    {
        return m_data;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief        converse data to the form that we can read from binary
    /// @param        isLeft:true if it's left foot;false if it's right
    ////////////////////////////////////////////////////////////////////////////////
    void remapPressureData(const bool isLeft);

private:
    dynamixel::PortHandler* m_port;

    PressureData m_data;

    std::vector<uint8_t> m_tx_packet, m_rx_packet;
    int m_tx_len, m_rx_len;//length of send and receive packet
};

}
