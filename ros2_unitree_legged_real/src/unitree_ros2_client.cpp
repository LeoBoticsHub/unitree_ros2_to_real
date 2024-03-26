/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#include "ros2_unitree_legged_real/unitree_ros2_client.hpp"

UnitreeRos2Client::UnitreeRos2Client(std::shared_ptr<rclcpp::Node> node): node_{node}{}

void UnitreeRos2Client::initRos2Interface()
{
    // Services
    power_off_srv_ = node_->create_client<std_srvs::srv::Trigger>("power_off");
    emergency_button_srv_ = node_->create_client<std_srvs::srv::Trigger>("emergency_stop");
    get_mode_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetInt>("get_robot_mode");
    stand_up_srv_ = node_->create_client<std_srvs::srv::Trigger>("stand_up");
    stand_down_srv_ = node_->create_client<std_srvs::srv::Trigger>("stand_down");
    battery_state_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetBatteryState>("get_battery_state");
    set_foot_height_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::SetFloat>("set_foot_height");
    get_foot_height_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetFloat>("get_foot_height");
    set_body_height_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::SetFloat>("set_body_height");
    get_body_height_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetFloat>("get_body_height");

    // publisher
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    cmd_orient_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("cmd_orient", 1);
}

bool UnitreeRos2Client::powerOff()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = power_off_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Power off service call failed");
    }

    if (!response.get()->success)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", response.get()->message.c_str());
    }

    return response.get()->success;
}

bool UnitreeRos2Client::emergencyStop()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = emergency_button_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Emergency stop service call failed");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", response.get()->message.c_str());

    return response.get()->success;
}

bool UnitreeRos2Client::standUp()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = stand_up_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Stand up service call failed");
    }

    if (response.get()->success)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", response.get()->message.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", response.get()->message.c_str());

    return response.get()->success;
}

bool UnitreeRos2Client::standDown()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = stand_down_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Stand down service call failed");
    }

    if (response.get()->success)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", response.get()->message.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", response.get()->message.c_str());

    return response.get()->success;
}

uint8_t UnitreeRos2Client::getRobotMode()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetInt::Request>();
    auto response = get_mode_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Get robot mode service call failed");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", response.get()->message.c_str());

    return response.get()->value;
}

float UnitreeRos2Client::getBatterySoc()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetBatteryState::Request>();
    auto response = battery_state_srv_->async_send_request(request);

    rclcpp::spin_until_future_complete(node_, response);

    return response.get()->battery_state.percentage;
}

bool UnitreeRos2Client::setFootHeight(float height)
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::SetFloat::Request>();
    request.get()->set__value(height);
    auto response = set_foot_height_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Set foot height service call failed");
    }
    
    if (response.get()->success)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());

    return response.get()->success;
}

float UnitreeRos2Client::getFootHeight()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetFloat::Request>();
    auto response = get_foot_height_srv_->async_send_request(request);

    rclcpp::spin_until_future_complete(node_, response);

    return response.get()->value;
}

bool UnitreeRos2Client::setBodyHeight(float height)
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::SetFloat::Request>();
    request.get()->set__value(height);
    auto response = set_body_height_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Set body height service call failed");
    }
    
    if (response.get()->success)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());

    return response.get()->success;
}

float UnitreeRos2Client::getBodyHeight()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetFloat::Request>();
    auto response = get_body_height_srv_->async_send_request(request);

    rclcpp::spin_until_future_complete(node_, response);

    return response.get()->value;
}

void UnitreeRos2Client::pubCmdVel(float vx, float vy, float w)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = w;

    cmd_vel_pub_->publish(cmd_vel);
}

void UnitreeRos2Client::pubCmdOrient(float roll, float pitch, float yaw)
{
    geometry_msgs::msg::Vector3 cmd_orient;
    cmd_orient.x = roll;
    cmd_orient.y = pitch;
    cmd_orient.z = yaw;

    cmd_orient_pub_->publish(cmd_orient);
}