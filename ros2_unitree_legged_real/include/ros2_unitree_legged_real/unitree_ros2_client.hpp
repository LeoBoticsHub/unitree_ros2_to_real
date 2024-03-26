/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#ifndef UNITREE_SERVICE_HANDLER_HPP
#define UNITREE_SERVICE_HANDLER_HPP

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "ros2_unitree_legged_msgs/srv/get_battery_state.hpp"
#include "ros2_unitree_legged_msgs/srv/get_float.hpp" 
#include "ros2_unitree_legged_msgs/srv/get_int.hpp" 
#include "ros2_unitree_legged_msgs/srv/set_body_orientation.hpp"
#include "ros2_unitree_legged_msgs/srv/set_float.hpp"

using namespace std::chrono_literals;

/**
 * @brief Class used to handle the unitree services for controlling the robot in a smoot way
 */
class UnitreeRos2Client
{
private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_button_srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stand_down_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetInt>::SharedPtr get_mode_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetBatteryState>::SharedPtr battery_state_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::SetFloat>::SharedPtr set_foot_height_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::SetFloat>::SharedPtr set_body_height_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetFloat>::SharedPtr get_foot_height_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetFloat>::SharedPtr get_body_height_srv_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_orient_pub_;

    std::shared_ptr<rclcpp::Node> node_;

public:

    /**
     * @brief Construct a new UnitreeRos2Client object
     * 
     * @param node the ros 2 node pointer to the ros node
     */
    UnitreeRos2Client(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief Initialize ros 2 service clients and publishers
     */
    void initRos2Interface();
    
    /**
     * @brief Power off service interface 
     * 
     * @return true if the robot power off
     * @return false otherwise
     */
    bool powerOff();

    /**
     * @brief Emergency stop service interface
     * 
     * @return true always because the robot power off in any case
     * @return false never
     */
    bool emergencyStop();

    /**
     * @brief Stand up servie interface
     * 
     * @return true if the robot stands up
     * @return false otherwise
     */
    bool standUp();

    /**
     * @brief Stand Down servie interface
     * 
     * @return true if the robot stands down
     * @return false otherwise
     */
    bool standDown();

    /**
     * @brief Get robot mode service interface
     * 
     * @return RobotMode the current modality of the robot
     */
    uint8_t getRobotMode();

    /**
     * @brief get battery state interface for retrieving state of charge percentage
     * 
     * @return float the state of charge of the battery in percentage
     */
    float getBatterySoc();

    /**
     * @brief set foot height service interface
     * 
     * @param height the heigth delta value to add to the current foot height
     * @return true if the delta height is added
     * @return false otherwise
     */
    bool setFootHeight(float height);

    /**
     * @brief the get foot height service interface
     * 
     * @return float the current foot height of the robot steps
     */
    float getFootHeight();

    /**
     * @brief the set body height service interface
     * 
     * @param height the delta body height to be added to the current one
     * @return true if the delta has been added 
     * @return false if it is not
     */
    bool setBodyHeight(float height);
 
    /**
     * @brief the get body height sensor interface
     * 
     * @return float the current body height
     */
    float getBodyHeight();
    
    /**
     * @brief To set the robot cmd vel
     * 
     * @param vx linear velocity along x axis (forward)
     * @param vy linear velocity along y axis (lateral)
     * @param w angular velocity around z axis (yaw)
     */
    void pubCmdVel(float vx, float vy, float w);

    /**
     * @brief to set the body orientation
     * 
     * @param roll the requested roll angle
     * @param pitch  the requested pitch angle
     * @param yaw  the requested yaw angle
     */
    void pubCmdOrient(float roll, float pitch, float yaw);
    
};

#endif