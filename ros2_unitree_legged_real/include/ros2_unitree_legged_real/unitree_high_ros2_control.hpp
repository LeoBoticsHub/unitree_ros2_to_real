/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#ifndef UNITREE_HIGH_ROS2_CONTROL_HPP
#define UNITREE_HIGH_ROS2_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <pthread.h>
#include <string>
#include <map>
#include <vector>
#include <algorithm>

#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "ros2_unitree_legged_msgs/srv/get_battery_state.hpp"
#include "ros2_unitree_legged_msgs/srv/get_float.hpp" 
#include "ros2_unitree_legged_msgs/srv/get_int.hpp" 
#include "ros2_unitree_legged_msgs/srv/set_body_orientation.hpp"
#include "ros2_unitree_legged_msgs/srv/set_float.hpp"
#include "ros2_unitree_legged_msgs/srv/set_int.hpp"

using namespace UNITREE_LEGGED_SDK;

/**
 * @brief Custom unitree class for connecting via UDP to the robot controller
 */
class Custom
{
public:
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    xRockerBtnDataStruct keyData;

public:

    using SharedPtr = std::shared_ptr<Custom>; 

    Custom(): 
        high_udp(8090, "192.168.123.220", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
    }

    void highUdpSend()
    {
        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void highUdpRecv()
    {
        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

/**
 * @brief Average function to compute the mean of a vector
 */
template< class T >
double avg(T* vect, unsigned int size)
{
    T res = 0.0;
    for(unsigned int i = 0; i < size; i++)
        res = res + vect[i];
    return res/size;
}

/**
 * @brief UnitreeRos2HighController ros2 node class for managing services, subscriber and publisher
 */
class UnitreeRos2HighController : public rclcpp::Node
{
private:

    // * UDP sender/receiver for robot high state
    Custom::SharedPtr custom_;

    // * Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr body_orient_sub_;

    // * Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    // * Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_button_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_down_srv_;
    rclcpp::Service<ros2_unitree_legged_msgs::srv::GetInt>::SharedPtr get_mode_srv_;
    rclcpp::Service<ros2_unitree_legged_msgs::srv::GetBatteryState>::SharedPtr battery_state_srv_;
    rclcpp::Service<ros2_unitree_legged_msgs::srv::SetFloat>::SharedPtr set_foot_height_srv_;
    rclcpp::Service<ros2_unitree_legged_msgs::srv::SetFloat>::SharedPtr set_body_height_srv_;
    rclcpp::Service<ros2_unitree_legged_msgs::srv::GetFloat>::SharedPtr get_foot_height_srv_;
    rclcpp::Service<ros2_unitree_legged_msgs::srv::GetFloat>::SharedPtr get_body_height_srv_;

    // * ROS messages
    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::JointState joint_state_msg_;
    nav_msgs::msg::Odometry odom_msg_;
    sensor_msgs::msg::BatteryState battery_msg_;
    geometry_msgs::msg::TransformStamped odom_H_trunk_;

    // * Constants
    const int N_MOTORS{12};
    const std::string BASE_LINK_NAME{"trunk"};
    const std::string IMU_NAME{"imu_link"};
    const std::string ODOM_NAME{"odom"};

    // * Joint variables
    const std::array<unsigned int, 12> b1_motor_idxs
    {{
        UNITREE_LEGGED_SDK::FL_0, UNITREE_LEGGED_SDK::FL_1, UNITREE_LEGGED_SDK::FL_2, // LF
        UNITREE_LEGGED_SDK::RL_0, UNITREE_LEGGED_SDK::RL_1, UNITREE_LEGGED_SDK::RL_2, // LH
        UNITREE_LEGGED_SDK::FR_0, UNITREE_LEGGED_SDK::FR_1, UNITREE_LEGGED_SDK::FR_2, // RF
        UNITREE_LEGGED_SDK::RR_0, UNITREE_LEGGED_SDK::RR_1, UNITREE_LEGGED_SDK::RR_2, // RH
    }};
    const std::array<std::string, 12> b1_motor_names
    {{
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
    }};

    // * Other variables
    rclcpp::Time t_, t_prev_, t_timer_, t_cmd_vel_, t_cmd_orient_;
    bool timer_on_, cmd_vel_active_, cmd_orient_active_ = false;
    std::mutex stand_mtx_;

    std::shared_ptr<LoopFunc> loop_StatePub_, loop_udpSend_, loop_udpRecv_;

    // * ROS 2 State Publisher
    /**
     * @brief This function reads the Robot high state values from an UDP message (joint states, imu, odom, wirless remote) and publish them on ros messages
     */
    void highStatePublisher();

    // * Callbacks
    /**
     *
     * @brief cmd_vel callback that receive a twist command message and send it to the robot
     * 
     * @param msg The twist velocity message to send to the robot
     * 
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback used for setting robot orientation
     * 
     * @param msg [geometry_msgs/Vector3] roll pitch yaw angle references.
     * roll range: [-0.3, 0.3], pitch range: [-0.3, 0.3], yaw range: [-0.6, 0.6]
     */
    void cmdBodyOrientationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    /**
     * @brief callback used to power off the robot when it is in crouched position
     */
    bool powerOffCallback(
        std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    );
    
    /**
     * @brief callback used to power off the robot always as an emergency button. BE CAREFUL IN USING IT.
     */
    bool emergencyStopCallback(
        std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    );

    /**
     * @brief Callback used for standing up the robot
     */
    bool standUpCallback(
        std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    );

    /**
     * @brief Callback used for standing down the robot
     */
    bool standDownCallback(
        std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    );
    
    /**
     * @brief battery state callback to check current robot battery state
     */
    bool batteryStateCallback(
        std::shared_ptr<ros2_unitree_legged_msgs::srv::GetBatteryState::Request> req, 
        std::shared_ptr<ros2_unitree_legged_msgs::srv::GetBatteryState::Response> res
    );

    /**
     * @brief Callback used for getting robot mode, e.g. stand up, stand down, walk, etc
     */
    bool getRobotModeCallback(
        std::shared_ptr<ros2_unitree_legged_msgs::srv::GetInt::Request> /* req */, 
        std::shared_ptr<ros2_unitree_legged_msgs::srv::GetInt::Response> res
    );

    /**
     * @brief Callback used for setting robot foot raise height
     */
    bool setFootHeightCallback(
        std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Request> req, 
        std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Response> res
    );
    
    /**
     * @brief Callback used for getting current robot foot raise height
     */
    bool getFootHeightCallback(
        std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Request> /* req */, 
        std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Response> res
    );

    /**
     * @brief Callback used for setting robot body height
     */
    bool setBodyHeightCallback(
        std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Request> req, 
        std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Response> res
    );

    /**
     * @brief Callback used for getting current robot body height
     */
    bool getBodyHeightCallback(
        std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Request> /* req */,
        std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Response> res
    );


public:

    void init_class();

    UnitreeRos2HighController();

};

#endif