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

#include "ros2_unitree_legged_msgs/srv/get_battery_state.hpp"
#include "ros2_unitree_legged_msgs/srv/get_float.hpp" 
#include "ros2_unitree_legged_msgs/srv/get_int.hpp" 
#include "ros2_unitree_legged_msgs/srv/set_body_orientation.hpp"
#include "ros2_unitree_legged_msgs/srv/set_float.hpp"
#include "ros2_unitree_legged_msgs/srv/set_int.hpp"

using namespace std::chrono_literals;

/**
 * @brief Class used to handle the unitree services for controlling the robot in a smoot way
 */
class ClientServicesHandler
{
private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_button_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::SetInt>::SharedPtr set_mode_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetInt>::SharedPtr get_mode_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::SetInt>::SharedPtr set_gate_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetInt>::SharedPtr get_gate_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::SetInt>::SharedPtr set_speed_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetBatteryState>::SharedPtr battery_state_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::SetFloat>::SharedPtr set_foot_height_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::SetFloat>::SharedPtr set_body_height_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetFloat>::SharedPtr get_foot_height_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::GetFloat>::SharedPtr get_body_height_srv_;
    rclcpp::Client<ros2_unitree_legged_msgs::srv::SetBodyOrientation>::SharedPtr set_body_orientation_srv_;

    std::shared_ptr<rclcpp::Node> node_;

public:
    ClientServicesHandler(std::shared_ptr<rclcpp::Node> node);

    enum class RobotMode { 
        IDLE_STAND=0, FORCED_STAND=1, VELOCITY_WALKING=2, SPEED=3, 
        STAND_DOWN=5, STAND_UP=6, DAMPING=7, RECOVERY_STAND=9
    };

    enum class GateType { IDLE=0, TROT, TROT_RUNNING, CLIMB_STAIRS, TROT_OBSTACLES };

    enum class SpeedLevel { LOW=0, MEDIUM, FAST };
    
    // intermediate functions to call services without interfacing directly with ros services
    bool powerOff();
    bool emergencyStop();
    bool setRobotMode(RobotMode mode);
    RobotMode getRobotMode();
    bool setGateType(GateType type);
    GateType getGateType();
    bool setSpeedLevel(SpeedLevel speed);
    float getBatterySoc();
    bool setFootHeight(float height);
    float getFootHeight();
    bool setBodyHeight(float height);
    float getBodyHeight();
    bool setBodyOrientation(float roll, float pitch, float yaw);
    
};

#endif