/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#ifndef UNITREE_LOW_ROS2_CONTROL_HPP
#define UNITREE_LOW_ROS2_CONTROL_HPP

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
#include <trajectory_msgs/msg/joint_trajectory.hpp>
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
    UDP low_udp;

    /**
     *   @brief Struct used for low level actuator control. All the member are initialized to 0.
     *   @param head            Identifier for the data frame
     *   @param levelFlag;      Control mode (LOWLEVEL, HIGHLEVEL)     
     *   @param frameReserve;        
     *   @param SN;      
     *   @param version; 
     *   @param bandWidth;
     *   @param motorCmd;
     *   @param bms;
     *   @param wirelessRemote; 
     *   @param reserve;
     *   @param crc;
     */
    LowCmd low_cmd = {0};

    /**
     *  @brief  Struct used for low level feedback from robot's sensors, battery and other components.
     *          all the members are initialized to 0
     * 
     *  @param  head            Identifier for the data frame
     *  @param  levelFlag       Control mode (LOWLEVEL, HIGHLEVEL)
     *  @param  frameReserve       
     *  @param  SN      
     *  @param  version 
     *  @param  bandWidth              
     *  @param  imu
     *  @param  motorState
     *  @param  bms
     *  @param  footForce       Data from foot airbag sensor
     *  @param  footForceEst    Reserve，typically zero
     *  @param  tick            Reference real-time from motion controller (unit: ms)
     *  @param  wirelessRemote  Data from Unitree Joystick.
     *  @param  reserve        
     *  @param  crc  
     */
    LowState low_state = {0};

    xRockerBtnDataStruct keyData; // Useless

    Safety safe_;   // Could be useless or problematic if it sets restrictive range of motion for the legs

public:

    // Create an alias for std::shared_ptr<Custom>
    using SharedPtr = std::shared_ptr<Custom>; 

    // Constructor
    Custom():
        low_udp(LOWLEVEL, 8091, robot_ip, 8007),
    {
        // Initialization
        low_udp.InitCmdData(low_cmd);
    }

    void lowUdpSend()
    {
        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {
        low_udp.Recv();
        low_udp.GetRecv(low_state);
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
 * @brief bound a value between its min and max values
 * 
 * @param value the value to bound
 * @param min the lower bound
 * @param max the upper bound
 * @return double - the bounded value
 */
double bound(double value, double min, double max)
{
    if (value < min)
        value = min;
    else if (value > max)
        value = max;

    return value;
}

/**
 * @brief enum class for robot state modalities in high level control
 * 
 */
// enum class ROBOT_STATE
// {
//     IDLE = 0, // 0. idle, default stand
//     FORCE_STAND = 1, // 1. force stand (controlled by dBodyHeight + ypr)
//     TARGET_VEL = 2, // 2. target velocity walking (controlled by velocity + yawSpeed)
//     STAND_DOWN = 5, // 5. position stand down.
//     STAND_UP = 6, // 6. position stand up
//     DAMPING = 7 // 7. damping mode
// };

/**
 * @brief UnitreeRos2LowController ros2 node class for managing services, subscriber and publisher
 */
class UnitreeRos2LowController : public rclcpp::Node
{
private:

    // * UDP sender/receiver for robot low state
    Custom::SharedPtr custom_;

    // * Subscribers
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub;

    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr body_orient_sub_;

    // * Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    // * Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_button_srv_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_down_srv_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_srv_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr revert_odom_srv_;
    // rclcpp::Service<ros2_unitree_legged_msgs::srv::GetInt>::SharedPtr get_mode_srv_;
    // rclcpp::Service<ros2_unitree_legged_msgs::srv::GetBatteryState>::SharedPtr battery_state_srv_;
    // rclcpp::Service<ros2_unitree_legged_msgs::srv::SetFloat>::SharedPtr set_foot_height_srv_;
    // rclcpp::Service<ros2_unitree_legged_msgs::srv::SetFloat>::SharedPtr set_body_height_srv_;
    // rclcpp::Service<ros2_unitree_legged_msgs::srv::GetFloat>::SharedPtr get_foot_height_srv_;
    // rclcpp::Service<ros2_unitree_legged_msgs::srv::GetFloat>::SharedPtr get_body_height_srv_;

    // * ROS messages
    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::JointState joint_state_msg_;
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg_;
    // nav_msgs::msg::Odometry odom_msg_;
    // sensor_msgs::msg::BatteryState battery_msg_;
    // geometry_msgs::msg::TransformStamped odom_H_trunk_;

    // * Constants
    const int N_MOTORS{12};
    const std::string BASE_LINK_NAME{"trunk"};
    const std::string IMU_NAME{"imu_link"};
    // const std::string ODOM_NAME{"odom"};
    double initial_body_height_;
    std::array<float, 3UL> initial_position_;

    // * Joint variables

    // Maybe is better to reorder FR FL RR RL

    const std::array<unsigned int, 12> b1_motor_idxs
    {{
        UNITREE_LEGGED_SDK::FL_0, UNITREE_LEGGED_SDK::FL_1, UNITREE_LEGGED_SDK::FL_2, // LF
        UNITREE_LEGGED_SDK::RL_0, UNITREE_LEGGED_SDK::RL_1, UNITREE_LEGGED_SDK::RL_2, // LH
        UNITREE_LEGGED_SDK::FR_0, UNITREE_LEGGED_SDK::FR_1, UNITREE_LEGGED_SDK::FR_2, // RF
        UNITREE_LEGGED_SDK::RR_0, UNITREE_LEGGED_SDK::RR_1, UNITREE_LEGGED_SDK::RR_2, // RH
    }};

    // Maybe is better to reorder FR FL RR RL

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
    bool publish_odom_tf_;
    std::string robot_name_;
    std::string tf_namespace_;
    double udp_loop_time_, state_pub_loop_time_; 
    double euler_x_bound_, euler_y_bound_, euler_z_bound_;
    double foot_height_lower_bound_, foot_height_upper_bound_, body_height_lower_bound_, body_height_upper_bound_;
    double min_vx_, max_vx_, vy_bound_, w_bound_;

    std::shared_ptr<LoopFunc> loop_StatePub_, loop_udpSend_, loop_udpRecv_;

    // * ROS 2 State Publisher
    /**
     * @brief This function reads the Robot low state values from an UDP message (joint states, imu, odom, wirless remote) and publish them on ros messages
     */
    void LowStatePublisher();

    // * Callbacks
    /**
     *
     * @brief cmd_vel callback that receive a twist command message and send it to the robot
     * 
     * @param msg The twist velocity message to send to the robot
     * 
     */
    void trajectoryCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback used for setting robot orientation
     * 
     * @param msg [geometry_msgs/Vector3] roll pitch yaw angle references.
     * roll range: [-0.3, 0.3], pitch range: [-0.3, 0.3], yaw range: [-0.6, 0.6]
     */
    // void cmdBodyOrientationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

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
    // bool standUpCallback(
    //     std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    //     std::shared_ptr<std_srvs::srv::Trigger::Response> res
    // );

    /**
     * @brief Callback used for standing down the robot
     */
    // bool standDownCallback(
    //     std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    //     std::shared_ptr<std_srvs::srv::Trigger::Response> res
    // );

    /**
     * @brief Callback used for resetting odometry on robot base link
     */
    // bool resetOdometryCallback(
    //     std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    //     std::shared_ptr<std_srvs::srv::Trigger::Response> res
    // );

    /**
     * @brief Callback used for reverting odometry to the orginal robot one
     */
    // bool revertOdometryCallback(
    //     std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    //     std::shared_ptr<std_srvs::srv::Trigger::Response> res
    // );
    
    /**
     * @brief battery state callback to check current robot battery state
     */
    // bool batteryStateCallback(
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::GetBatteryState::Request> req, 
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::GetBatteryState::Response> res
    // );

    /**
     * @brief Callback used for getting robot mode, e.g. stand up, stand down, walk, etc
     */
    // bool getRobotModeCallback(
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::GetInt::Request> /* req */, 
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::GetInt::Response> res
    // );

    /**
     * @brief Callback used for setting robot foot raise height
     */
    // bool setFootHeightCallback(
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Request> req, 
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Response> res
    // );
    
    /**
     * @brief Callback used for getting current robot foot raise height
     */
    // bool getFootHeightCallback(
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Request> /* req */, 
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Response> res
    // );

    /**
     * @brief Callback used for setting robot body height
     */
    // bool setBodyHeightCallback(
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Request> req, 
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Response> res
    // );

    /**
     * @brief Callback used for getting current robot body height
     */
    // bool getBodyHeightCallback(
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Request> /* req */,
    //     std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Response> res
    // );


public:

    void init_class();

    UnitreeRos2LowController();

};

#endif