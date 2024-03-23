/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#include "ros2_unitree_legged_real/unitree_services_handler.hpp"

/**
 * @brief Construct a new Client Services Handler:: Client Services Handler object
 * 
 * @param n the ros node handle of the ros node using this class
 */
ClientServicesHandler::ClientServicesHandler(std::shared_ptr<rclcpp::Node> node): node_{node}
{
    power_off_srv_ = node_->create_client<std_srvs::srv::Trigger>("power_off");
    emergency_button_srv_ = node_->create_client<std_srvs::srv::Trigger>("emergency_stop");
    set_mode_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::SetInt>("set_robot_mode");
    get_mode_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetInt>("get_robot_mode");
    set_gate_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::SetInt>("set_gate_type");
    get_gate_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetInt>("get_gate_type");
    set_speed_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::SetInt>("set_speed_level");
    battery_state_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetBatteryState>("get_battery_state");
    set_foot_height_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::SetFloat>("set_foot_height");
    get_foot_height_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetFloat>("get_foot_height");
    set_body_height_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::SetFloat>("set_body_height");
    get_body_height_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::GetFloat>("get_body_height");
    set_body_orientation_srv_ = node_->create_client<ros2_unitree_legged_msgs::srv::SetBodyOrientation>("set_body_orientation");

}

/**
 * @brief Power off service interface 
 * 
 * @return true if the robot power off
 * @return false otherwise
 */
bool ClientServicesHandler::powerOff()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = power_off_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return response.get()->success;
}

/**
 * @brief Emergency stop service interface
 * 
 * @return true always because the robot power off in any case
 * @return false never
 */
bool ClientServicesHandler::emergencyStop()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = emergency_button_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return response.get()->success;
}

/**
 * @brief Set robot mode service interface
 * 
 * @param mode the RobotMode modality you want to set
 * @return true if the modality switching has been performed
 * @return false otherwise
 */
bool ClientServicesHandler::setRobotMode(RobotMode mode)
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::SetInt::Request>();
    request.get()->set__value(static_cast<uint8_t>(mode));
    auto response = set_mode_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return response.get()->success;
}

/**
 * @brief Get robot mode service interface
 * 
 * @return RobotMode the current modality of the robot
 */
ClientServicesHandler::RobotMode ClientServicesHandler::getRobotMode()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetInt::Request>();
    auto response = get_mode_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return static_cast<RobotMode>(response.get()->value);
}

/**
 * @brief set gate type service interface 
 * 
 * @param type the GateType requested for the robot
 * @return true if the gate type is correctly set
 * @return false otherwise
 */
bool ClientServicesHandler::setGateType(GateType type)
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::SetInt::Request>();
    request.get()->set__value(static_cast<uint8_t>(type));
    auto response = set_gate_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return response.get()->success;
}

/**
 * @brief get gate type service interface 
 * 
 * @return ClientServicesHandler::GateType the current gate type
 */
ClientServicesHandler::GateType ClientServicesHandler::getGateType()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetInt::Request>();
    auto response = get_gate_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return static_cast<GateType>(response.get()->value);
}

/**
 * @brief set speed level service interface
 * 
 * @param speed the required speed
 * @return true if speed has been correctly set
 * @return false othrewise
 */
bool ClientServicesHandler::setSpeedLevel(SpeedLevel speed)
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::SetInt::Request>();
    request.get()->set__value(static_cast<uint8_t>(speed));
    auto response = set_gate_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return response.get()->success;
}

/**
 * @brief get battery state interface for retrieving state of charge percentage
 * 
 * @return float the state of charge of the battery in percentage
 */
float ClientServicesHandler::getBatterySoc()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetBatteryState::Request>();
    auto response = battery_state_srv_->async_send_request(request);

    rclcpp::spin_until_future_complete(node_, response);

    return response.get()->battery_state.percentage;
}

/**
 * @brief set foot height service interface
 * 
 * @param height the heigth delta value to add to the current foot height
 * @return true if the delta height is added
 * @return false otherwise
 */
bool ClientServicesHandler::setFootHeight(float height)
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::SetFloat::Request>();
    request.get()->set__value(height);
    auto response = set_foot_height_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return response.get()->success;
}

/**
 * @brief the get foot height service interface
 * 
 * @return float the current foot height of the robot steps
 */
float ClientServicesHandler::getFootHeight()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetFloat::Request>();
    auto response = get_foot_height_srv_->async_send_request(request);

    rclcpp::spin_until_future_complete(node_, response);

    return response.get()->value;
}

/**
 * @brief the set body height service interface
 * 
 * @param height the delta body height to be added to the current one
 * @return true if the delta has been added 
 * @return false if it is not
 */
bool ClientServicesHandler::setBodyHeight(float height)
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::SetFloat::Request>();
    request.get()->set__value(height);
    auto response = set_body_height_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return response.get()->success;
}

/**
 * @brief the get body height sensor interface
 * 
 * @return float the current body height
 */
float ClientServicesHandler::getBodyHeight()
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::GetFloat::Request>();
    auto response = get_body_height_srv_->async_send_request(request);

    rclcpp::spin_until_future_complete(node_, response);

    return response.get()->value;
}

/**
 * @brief the set body orientation service interface
 * 
 * @param roll the requested roll angle
 * @param pitch  the requested pitch angle
 * @param yaw  the requested yaw angle
 * @return true if the orientation is correctly set
 * @return false otherwise
 */
bool ClientServicesHandler::setBodyOrientation(float roll, float pitch, float yaw)
{
    auto request = std::make_shared<ros2_unitree_legged_msgs::srv::SetBodyOrientation::Request>();
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    request.get()->set__rpy(rpy);


    auto response = set_body_orientation_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), response.get()->message.c_str());
    }

    return response.get()->success;
}