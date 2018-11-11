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

void Parameters::setFieldYaw(const double yaw)
{
    global.yaw_offset = DEG2RAD(yaw);
    ROS_WARN_STREAM("##########   FIELD YAW SET TO " << RAD2DEG(global.yaw_offset) << " ##############");
}

void Parameters::update()
{
    ROS_WARN("update parameters...");
    // load global parameters
    GPARAM("dmotion/global/debug", global.debug);
    GPARAM("dmotion/global/debug_io", global.io_debug);
    GPARAM("dmotion/global/debug_stablizer", global.stablizer_debug);
    GPARAM("dmotion/global/debug_imu", global.imu_debug);

    GPARAM("dmotion/global/using_odometry", global.using_odometry);
    GPARAM("dmotion/global/using_head_data", global.using_head_data);
    GPARAM("dmotion/global/using_lateral_stablizer", global.using_lateral_stablizer);
    GPARAM("dmotion/global/using_push_recovery", global.using_push_recovery);
    GPARAM("dmotion/state/using_pressure", global.using_pressure);

    // GPARAM("dmotion/simulation", global.simulation);
    // GPARAM("dmotion/enable_ankle", global.enable_ankle);

    // load io parameters
    GPARAM("dmotion/hardware/imu_port_name", io.imu_port_name);
    GPARAM("dmotion/hardware/imu_baudrate", io.imu_baudrate);
    GPARAM("dmotion/hardware/imu_version", io.imu_version);

    GPARAM("dmotion/hardware/servo_port_name", io.servo_port_name);
    GPARAM("dmotion/hardware/servo_baudrate", io.servo_baudrate);
    GPARAM("dmotion/hardware/servo_protocol_version", io.servo_protocol_version);
    GPARAM("dmotion/hardware/servo_init_ticks", io.servo_init_ticks);
    GPARAM("dmotion/hardware/servo_init_speed", io.servo_init_speed);

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
            // io.joint_cfg.insert(
            //     std::make_pair(
            //         io.joint_name[i],
            //         Motion::JointConfig(io.joint_index[i],
            //                             io.joint_cw[i],
            //                             io.joint_resolution[i],
            //                             io.joint_init_values[i],
            //                             io.joint_upper_bound[i],
            //                             io.joint_lower_bound[i])
            //     )
            // );
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

    // // load motion parameters
    // GPARAM("dmotion/length_tibia", motion.length_tibia);
    // GPARAM("dmotion/length_thigh", motion.length_thigh);
    // GPARAM("dmotion/length_hip", motion.length_hip);

    // GPARAM("dmotion/crounch_ticks", motion.crounch_ticks);
    // GPARAM("dmotion/crounch_com_height", motion.crounch_com_height);
    // GPARAM("dmotion/crounch_ankle_distance", motion.crounch_ankle_distance);

    // GPARAM("dmotion/com_offset_x", motion.com_offset_x);
    // GPARAM("dmotion/com_offset_y", motion.com_offset_y);
    // GPARAM("dmotion/com_offset_roll", motion.com_offset_roll);
    // motion.com_offset_roll = DEG2RAD(motion.com_offset_roll);
    // GPARAM("dmotion/com_offset_pitch", motion.com_offset_pitch);
    // motion.com_offset_pitch = DEG2RAD(motion.com_offset_pitch);

    // GPARAM("dmotion/ankle_roll_p", motion.ankle_roll_p);
    // GPARAM("dmotion/ankle_roll_i", motion.ankle_roll_i);
    // GPARAM("dmotion/ankle_roll_d", motion.ankle_roll_d);
    // GPARAM("dmotion/ankle_roll_min", motion.ankle_roll_min);
    // GPARAM("dmotion/ankle_roll_max", motion.ankle_roll_max);
    // GPARAM("dmotion/ankle_ramp_rate", motion.ankle_ramp_rate);

    // // load state parameters
    GPARAM("dmotion/state/imu_algorithm_gain", state.imu_algorithm_gain);
    GPARAM("dmotion/state/imu_bias_gain", state.imu_bias_gain);
    GPARAM("dmotion/state/imu_pitch_bias", state.imu_pitch_bias);
    GPARAM("dmotion/state/imu_roll_bias", state.imu_roll_bias);
    GPARAM("dmotion/state/imu_yaw_vel_correction", state.imu_yaw_vel_correction);

    GPARAM("dmotion/state/pressure_counting_start", state.pressure_counting_start);
    GPARAM("dmotion/state/pressure_counting_end", state.pressure_counting_end);
    GPARAM("dmotion/state/security_threshold", state.security_threshold);
    GPARAM("dmotion/state/security_delay_ticks", state.security_delay_ticks);
    GPARAM("dmotion/state/com_roll_threshold_left", state.com_roll_threshold_left);
    GPARAM("dmotion/state/com_roll_threshold_right", state.com_roll_threshold_right);
    GPARAM("dmotion/state/velocity_threshold", state.velocity_threshold);

    GPARAM("dmotion/state/imu_prepare_time", state.imu_prepare_time);

    GPARAM("dmotion/state/front_down_threshold", state.front_down_threshold);
    GPARAM("dmotion/state/back_down_threshold", state.back_down_threshold);
    GPARAM("dmotion/state/side_down_threshold", state.side_down_threshold);
    GPARAM("dmotion/state/front_assertion_ticks", state.front_assertion_ticks);
    GPARAM("dmotion/state/back_assertion_ticks", state.back_assertion_ticks);
    GPARAM("dmotion/state/side_assertion_ticks", state.side_assertion_ticks);
    GPARAM("dmotion/state/stable_assertion_ticks", state.stable_assertion_ticks);


    GPARAM("dmotion/kick/using_pi_mode", kick.using_pi_mode);
    GPARAM("dmotion/kick/kick_controller_i", kick.kick_controller_i);
    GPARAM("dmotion/kick/kick_controller_p", kick.kick_controller_p);
    GPARAM("dmotion/kick/using_support_stablizer", kick.using_support_stablizer);

    // GPARAM("dmotion/robot/lower_leg", motion.length_tibia);
    // GPARAM("dmotion/robot/upper_leg", motion.length_thigh);
    // GPARAM("dmotion/robot/hip_distance", motion.length_hip);


    motion.length_hip = 0.09;
    motion.length_thigh = 0.12;
    motion.length_tibia = 0.12;

    GPARAM("dmotion/walk/freq", walk.freq);
    GPARAM("dmotion/walk/doubleSupportRatio", walk.doubleSupportRatio);
    GPARAM("dmotion/walk/footDistance", walk.footDistance);
    GPARAM("dmotion/walk/footRise", walk.footRise);
    GPARAM("dmotion/walk/footApexPhase", walk.footApexPhase);
    GPARAM("dmotion/walk/footOvershootRatio", walk.footOvershootRatio);
    GPARAM("dmotion/walk/footOvershootPhase", walk.footOvershootPhase);
    GPARAM("dmotion/walk/trunkHeight", walk.trunkHeight);
    GPARAM("dmotion/walk/trunkPause", walk.trunkPause);
    GPARAM("dmotion/walk/trunkPhase", walk.trunkPhase);
    GPARAM("dmotion/walk/trunkSwing", walk.trunkSwing);
    GPARAM("dmotion/walk/trunkXOffset", walk.trunkXOffset);
    GPARAM("dmotion/walk/trunkYOffset", walk.trunkYOffset);
    GPARAM("dmotion/walk/trunkPitch", walk.trunkPitch);
    GPARAM("dmotion/walk/trunkXOffsetPCoefForward", walk.trunkXOffsetPCoefForward);
    GPARAM("dmotion/walk/trunkXOffsetPCoefTurn", walk.trunkXOffsetPCoefTurn);
    GPARAM("dmotion/walk/trunkPitchPCoefForward", walk.trunkPitchPCoefForward);
    GPARAM("dmotion/walk/trunkPitchPCoefTurn", walk.trunkPitchPCoefTurn);





    // double doubleSupportRatio = 0.2;
    // double footDistance = 0.13;
    // double footRise = 0.035;
    // double footApexPhase = 0.5;
    // double footOvershootRatio = 0.05;
    // double footOvershootPhase = 0.85;
    // double trunkHeight = 0.27;
    // double trunkPitch = 0.2;
    // double trunkPhase = 0.4;
    // double trunkXOffset = 0.005;
    // double trunkYOffset = 0.0;
    // double trunkSwing = 0.3;
    // double trunkPause = 0.0;
    // double trunkXOffsetPCoefForward = 0.0;
    // double trunkXOffsetPCoefTurn = 0.0;
    // double trunkPitchPCoefForward = 0.0;
    // double trunkPitchPCoefTurn = 0.0;
}

Parameters parameters;
