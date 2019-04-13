#include "dmotion/Common/Parameters.h"

#include "dmotion/Common/Utility/Utility.h"


#define GPARAM(x, y)                                                                                                                                                                                   \
    do {                                                                                                                                                                                               \
        if (!m_nh->getParam(x, y)) {                                                                                                                                                                   \
            MOTION_WARN("Motion get pararm " #x " error!");                                                                                                                                          \
        }                                                                                                                                                                                              \
    } while (0)

void Parameters::init(ros::NodeHandle* nh)
{
    m_nh = nh;
    update();
}

void Parameters::update()
{
    ROS_WARN("update parameters...");

    GPARAM("dmotion/global/using_lateral_stablizer", global.using_lateral_stablizer);
    GPARAM("dmotion/global/using_push_recovery", global.using_push_recovery);
    GPARAM("dmotion/state/using_pressure", global.using_pressure);

    // load io parameters
    GPARAM("dmotion/hardware/imu_version", io.imu_version);

    GPARAM("dmotion/motor/num", io.joint_number);
    std::vector<float> joint_init_float;
    GPARAM("dmotion/motor/init", joint_init_float);
    GPARAM("dmotion/motor/zf", io.joint_cw);
    GPARAM("dmotion/motor/k", io.joint_resolution);
    GPARAM("dmotion/motor/id", io.joint_index);
    GPARAM("dmotion/motor/ub", io.joint_upper_bound);
    GPARAM("dmotion/motor/lb", io.joint_lower_bound);
    GPARAM("dmotion/motor/name", io.joint_name);

    assert(static_cast<int>(io.joint_index.size()) == io.joint_number);
    assert(static_cast<int>(io.joint_resolution.size()) == io.joint_number);
    assert(static_cast<int>(io.joint_cw.size()) == io.joint_number);
    assert(static_cast<int>(joint_init_float.size()) == io.joint_number);
    assert(static_cast<int>(io.joint_upper_bound.size()) == io.joint_number);
    assert(static_cast<int>(io.joint_lower_bound.size()) == io.joint_number);

    io.joint_init_values.resize(io.joint_number);

    if (static_cast<int>(io.joint_cfg.size()) == io.joint_number)
    {
        for(int i = 0; i < io.joint_number; i++)
        {
            io.joint_init_values[i] = std::round(joint_init_float[i]);
            auto iter = io.joint_cfg.find(io.joint_name[i]);
            if (iter == io.joint_cfg.end())
            {
                ROS_WARN("joint not found");
            }
            else
            {
                iter->second.id = io.joint_index[i];
                iter->second.cw = io.joint_cw[i];
                iter->second.resolution = io.joint_resolution[i];
                iter->second.init = io.joint_init_values[i];
                iter->second.max_pos = io.joint_upper_bound[i];
                iter->second.min_pos = io.joint_lower_bound[i];
                iter->second.setFactor();
            }
        }
    }
    else
    {
        for(int i = 0; i < io.joint_number; i++)
        {
            io.joint_init_values[i] = std::round(joint_init_float[i]);
            io.joint_cfg.insert(
                std::make_pair(
                    io.joint_name[i],
                    Motion::JointConfig(io.joint_index[i],
                                        io.joint_cw[i],
                                        io.joint_resolution[i],
                                        io.joint_init_values[i],
                                        io.joint_upper_bound[i],
                                        io.joint_lower_bound[i])
                )
            );
        }
    }


    GPARAM("dmotion/state/imu_prepare_time", state.imu_prepare_time);

}

Parameters parameters;
