#pragma once

#include "../Common/Type.h"
#include "../Common/Parameters.h"

#include <vector>
#include <iostream>

namespace Motion
{

class IOManagerBase
{
public:
    virtual ~IOManagerBase() = default;

    virtual void spinOnce() = 0;

    virtual void remapJointValues() = 0;

    void setJointvalues(const std::vector<float>& joint_values)
    {
        assert(static_cast<int>(joint_values.size()) == parameters.io.joint_number);
        assert(static_cast<int>(m_joint_values.size()) == parameters.io.joint_number);
        
        std::memcpy(m_joint_values.data(), joint_values.data(), sizeof(float)*parameters.io.joint_number);
    }

    virtual const IMUData& getIMUData()
    {
        std::cout << "getIMUData not override.." << std::endl;
        IMUData tmp;
        return tmp;
    }

    virtual const std::vector<float>& getJointValues()
    {
        std::cout << "getJointValues not override.." << std::endl;
        std::vector<float> values;
        return values;
    }

    virtual const PowerState getPowerState() const
    {
        return PowerState::ON;        
    }

protected:
    IOManagerBase()
    {
        m_joint_values.resize(parameters.io.joint_number);
        
        for(int i = 0; i < parameters.io.joint_number; i++)
        {
            m_joint_values[i] = 0;
        }
        
    }

    std::vector<float> m_joint_values;
};

}
