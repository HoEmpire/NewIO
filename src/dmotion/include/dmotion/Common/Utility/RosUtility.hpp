//
// Created by zjudancer on 17-11-22.
//

#ifndef dmotion_lib_ROSCONTROLLER_H
#define dmotion_lib_ROSCONTROLLER_H

#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace Motion
{
    class JointController
    {
    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_joint_pub;
        uint32_t m_seq;

        sensor_msgs::JointState m_joint_msg;

        std::vector<std::pair<std::string, double >> jointValues;

    public:
        JointController(ros::NodeHandle &nh_)
                : m_nh(nh_), m_seq(0)
        {
            m_joint_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

            initJointValues();

            // m_joint_msg.name.push_back("j_thigh1_r");
            // m_joint_msg.position.push_back(0.5);
        }

        ~JointController() = default;

        void mpub()
        {
            m_joint_msg.header.seq = m_seq++;
            m_joint_msg.header.stamp = ros::Time::now();
            m_joint_pub.publish(m_joint_msg);
        }

        void initJointValues()
        {
            jointValues.push_back(std::make_pair("j_pan", 0.0)); jointValues.push_back(std::make_pair("j_tilt", 0.0)); jointValues.push_back(std::make_pair("j_shoulder_l", 0.0));
            jointValues.push_back(std::make_pair("j_high_arm_l", 0.0)); jointValues.push_back(std::make_pair("j_low_arm_l", 0.0)); jointValues.push_back(std::make_pair("j_wrist_l", 0.0));
            jointValues.push_back(std::make_pair("j_gripper_l", 0.0)); jointValues.push_back(std::make_pair("j_shoulder_r", 0.0)); jointValues.push_back(std::make_pair("j_high_arm_r", 0.0));
            jointValues.push_back(std::make_pair("j_low_arm_r", 0.0)); jointValues.push_back(std::make_pair("j_wrist_r", 0.0)); jointValues.push_back(std::make_pair("j_gripper_r", 0.0));
            jointValues.push_back(std::make_pair("j_pelvis_l", 0.0)); jointValues.push_back(std::make_pair("j_thigh1_l", 0.0)); jointValues.push_back(std::make_pair("j_thigh2_l", 0.0));
            jointValues.push_back(std::make_pair("j_tibia_l", 0.0)); jointValues.push_back(std::make_pair("j_ankle1_l", 0.0)); jointValues.push_back(std::make_pair("j_ankle2_l", 0.0));
            jointValues.push_back(std::make_pair("j_pelvis_r", 0.0)); jointValues.push_back(std::make_pair("j_thigh1_r", 0.0)); jointValues.push_back(std::make_pair("j_thigh2_r", 0.0));
            jointValues.push_back(std::make_pair("j_tibia_r", 0.0)); jointValues.push_back(std::make_pair("j_ankle1_r", 0.0)); jointValues.push_back(std::make_pair("j_ankle2_r", 0.0));

            for (auto &p:jointValues)
            {
                m_joint_msg.name.push_back(p.first);
                m_joint_msg.position.push_back(p.second);
            }
        }

        void updateJointValues()
        {
            for (int i = 0, _size = (int)jointValues.size(); i != _size; ++i)
                m_joint_msg.position[i] = jointValues[i].second;
        }

        void changeJointValues(const std::string &_joint_name, const double &_value)
        {
            // string jn_ = _joint_name;
            // vector<pair<string, double >>::iterator
            auto it = std::find_if( jointValues.begin(), jointValues.end(),
                                    [&](const std::pair<std::string, double> &element){
                                        return element.first == _joint_name;
                                    } );
            it->second = _value;

            updateJointValues();
        }

    private:
        //bool isSame()
    };

    class DataVisualizer
    {
    public:
        typedef std::shared_ptr<DataVisualizer> Ptr;
        DataVisualizer(const std::string &topic_name_, ros::NodeHandle nh_)
        {
            m_pub = nh_.advertise<std_msgs::Float64>(topic_name_, 5);
        }
        ~DataVisualizer() = default;

        void update(double data_)
        {
            m_msg.data = data_;
            m_pub.publish(m_msg);
        }

        static DataVisualizer::Ptr createDataVisualizer(const std::string &topic_name_, ros::NodeHandle nh_)
        {
            return DataVisualizer::Ptr(new DataVisualizer(topic_name_, nh_));
        }

    private:
        ros::Publisher m_pub;
        std_msgs::Float64 m_msg;
    };
}

#endif //dmotion_lib_ROSCONTROLLER_H
