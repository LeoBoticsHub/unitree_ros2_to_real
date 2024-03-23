/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#include "ros2_unitree_legged_real/unitree_high_ros2_control.hpp"

UnitreeRos2HighController::UnitreeRos2HighController():
    Node("unitree_ros2_high_controller")
{   
    
}

void UnitreeRos2HighController::init_class()
{
    // TODO initialize custom loops and subscribers etc
}

void UnitreeRos2HighController::highStatePublisher()
{
    
}

void UnitreeRos2HighController::cmdVelCallback(const geometry_msgs::msg::Twist::ConstPtr &msg)
{

}

bool UnitreeRos2HighController::powerOffCallback(
    std_srvs::srv::Trigger::Request& req, 
    std_srvs::srv::Trigger::Response& res
)
{

}

bool UnitreeRos2HighController::emergencyStopCallback(
    std_srvs::srv::Trigger::Request& req, 
    std_srvs::srv::Trigger::Response& res
)
{
    
}

bool UnitreeRos2HighController::batteryStateCallback(
    ros2_unitree_legged_msgs::srv::GetBatteryState::Request& req, 
    ros2_unitree_legged_msgs::srv::GetBatteryState::Response& res
)
{
    
}

bool UnitreeRos2HighController::setRobotModeCallback(
    ros2_unitree_legged_msgs::srv::SetInt::Request& req, 
    ros2_unitree_legged_msgs::srv::SetInt::Response& res
)
{
    
}

bool UnitreeRos2HighController::getRobotModeCallback(
    ros2_unitree_legged_msgs::srv::GetInt::Request& /* req */, 
    ros2_unitree_legged_msgs::srv::GetInt::Response& res
)
{
    
}

bool UnitreeRos2HighController::setGateTypeCallback(
    ros2_unitree_legged_msgs::srv::SetInt::Request& req, 
    ros2_unitree_legged_msgs::srv::SetInt::Response& res
)
{
    
}

bool UnitreeRos2HighController::getGateTypeCallback(
    ros2_unitree_legged_msgs::srv::GetInt::Request& /* req */, 
    ros2_unitree_legged_msgs::srv::GetInt::Response& res
)
{
    
}

bool UnitreeRos2HighController::setSpeedLevelCallback(
    ros2_unitree_legged_msgs::srv::SetInt::Request& req, 
    ros2_unitree_legged_msgs::srv::SetInt::Response& res
)
{
    
}

bool UnitreeRos2HighController::setFootHeightCallback(
    ros2_unitree_legged_msgs::srv::SetFloat::Request& req, 
    ros2_unitree_legged_msgs::srv::SetFloat::Response& res
)
{
    
}

bool UnitreeRos2HighController::getFootHeightCallback(
    ros2_unitree_legged_msgs::srv::GetFloat::Request& /* req */, 
    ros2_unitree_legged_msgs::srv::GetFloat::Response& res
)
{
    
}

bool UnitreeRos2HighController::setBodyHeightCallback(
    ros2_unitree_legged_msgs::srv::SetFloat::Request& req, 
    ros2_unitree_legged_msgs::srv::SetFloat::Response& res
)
{
    
}

bool UnitreeRos2HighController::getBodyHeightCallback(
    ros2_unitree_legged_msgs::srv::GetFloat::Request& /* req */, 
    ros2_unitree_legged_msgs::srv::GetFloat::Response& res
)
{
    
}

bool UnitreeRos2HighController::setBodyOrientationCallback(
    ros2_unitree_legged_msgs::srv::SetBodyOrientation::Request& req, 
    ros2_unitree_legged_msgs::srv::SetBodyOrientation::Response& res
)
{

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node{std::make_shared<UnitreeRos2HighController>()};
    node->init_class();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}