#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "convert.h"
#include <cmath>

using namespace UNITREE_LEGGED_SDK;

class JointTrajectoryHandler : public rclcpp::Node
{
public:
    JointTrajectoryHandler()
        : Node("joint_trajectory_handler")
    {
        // Create a subscriber for the joint trajectory topic
        joint_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "joint_group_effort_controller/joint_trajectory", 10, std::bind(&JointTrajectoryHandler::trajectoryCallback, this, std::placeholders::_1));
        joint_trajectory_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1);
        
        // LowCmd msg fields setting 
        this->low_cmd_ros.head[0] = 0xFE; // 0xFE is a common header to mark the start of a message (first byte of the header)
        this->low_cmd_ros.head[1] = 0xEF; // 0xFE is a common header to mark the start of a message (second byte of the header)
        this->low_cmd_ros.level_flag = LOWLEVEL; // Control mode set to LOWLEVEL (direct control of the joint). Is defined in common.h (unitree_legged_sdk)

        for (int i = 0; i < 12; i++)
        {
            this->low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode (desired working mode. Servo : 0x0A, Damping : 0x00)
            this->low_cmd_ros.motor_cmd[i].q = PosStopF; // Desired angle (unit: radian). In this case disable position loop (which control the position of the joint). Is defined in common.h (unitree_legged_sdk)
            this->low_cmd_ros.motor_cmd[i].kp = 0; // Desired position stiffness (unit: N.m/rad )
            this->low_cmd_ros.motor_cmd[i].dq = VelStopF; // Desired velocity (unit: radian/second). In this case disable velocity loop (which control the speed of the joint). Is defined in common.h (unitree_legged_sdk)
            this->low_cmd_ros.motor_cmd[i].kd = 0; // Desired velocity stiffness (unit: N.m/(rad/s) 
            this->low_cmd_ros.motor_cmd[i].tau = 0; // Desired output torque (unit: N.m)
        }        
        // For demonstration: log every processed command
        RCLCPP_INFO(this->get_logger(), "JointTrajectoryHandler initialized.");
    }

public:
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_;
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr joint_trajectory_pub_;

    ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        this->low_cmd_ros.motor_cmd[FL_0].q = msg->points[0].positions[0];
        this->low_cmd_ros.motor_cmd[FL_0].dq = 0.0;
        this->low_cmd_ros.motor_cmd[FL_0].kp = 5.0;
        this->low_cmd_ros.motor_cmd[FL_0].kd = 1.0;

        this->low_cmd_ros.motor_cmd[FL_1].q = msg->points[0].positions[1];
        this->low_cmd_ros.motor_cmd[FL_1].dq = 0.0;
        this->low_cmd_ros.motor_cmd[FL_1].kp = 5.0;
        this->low_cmd_ros.motor_cmd[FL_1].kd = 1.0;

        this->low_cmd_ros.motor_cmd[FL_2].q = msg->points[0].positions[2];
        this->low_cmd_ros.motor_cmd[FL_2].dq = 0.0;
        this->low_cmd_ros.motor_cmd[FL_2].kp = 5.0;
        this->low_cmd_ros.motor_cmd[FL_2].kd = 1.0;

        this->low_cmd_ros.motor_cmd[FR_0].q = msg->points[0].positions[3];
        this->low_cmd_ros.motor_cmd[FR_0].dq = 0.0;
        this->low_cmd_ros.motor_cmd[FR_0].kp = 5.0;
        this->low_cmd_ros.motor_cmd[FR_0].kd = 1.0;

        this->low_cmd_ros.motor_cmd[FR_1].q = msg->points[0].positions[4];
        this->low_cmd_ros.motor_cmd[FR_1].dq = 0.0;
        this->low_cmd_ros.motor_cmd[FR_1].kp = 5.0;
        this->low_cmd_ros.motor_cmd[FR_1].kd = 1.0;

        this->low_cmd_ros.motor_cmd[FR_2].q = msg->points[0].positions[5];
        this->low_cmd_ros.motor_cmd[FR_2].dq = 0.0;
        this->low_cmd_ros.motor_cmd[FR_2].kp = 5.0;
        this->low_cmd_ros.motor_cmd[FR_2].kd = 1.0;

        this->low_cmd_ros.motor_cmd[RL_0].q = msg->points[0].positions[6];
        this->low_cmd_ros.motor_cmd[RL_0].dq = 0.0;
        this->low_cmd_ros.motor_cmd[RL_0].kp = 5.0;
        this->low_cmd_ros.motor_cmd[RL_0].kd = 1.0;

        this->low_cmd_ros.motor_cmd[RL_1].q = msg->points[0].positions[7];
        this->low_cmd_ros.motor_cmd[RL_1].dq = 0.0;
        this->low_cmd_ros.motor_cmd[RL_1].kp = 5.0;
        this->low_cmd_ros.motor_cmd[RL_1].kd = 1.0;

        this->low_cmd_ros.motor_cmd[RL_2].q = msg->points[0].positions[8];
        this->low_cmd_ros.motor_cmd[RL_2].dq = 0.0;
        this->low_cmd_ros.motor_cmd[RL_2].kp = 5.0;
        this->low_cmd_ros.motor_cmd[RL_2].kd = 1.0;

        this->low_cmd_ros.motor_cmd[RR_0].q = msg->points[0].positions[9];
        this->low_cmd_ros.motor_cmd[RR_0].dq = 0.0;
        this->low_cmd_ros.motor_cmd[RR_0].kp = 5.0;
        this->low_cmd_ros.motor_cmd[RR_0].kd = 1.0;

        this->low_cmd_ros.motor_cmd[RR_1].q = msg->points[0].positions[10];
        this->low_cmd_ros.motor_cmd[RR_1].dq = 0.0;
        this->low_cmd_ros.motor_cmd[RR_1].kp = 5.0;
        this->low_cmd_ros.motor_cmd[RR_1].kd = 1.0;

        this->low_cmd_ros.motor_cmd[RR_2].q = msg->points[0].positions[11];
        this->low_cmd_ros.motor_cmd[RR_2].dq = 0.0;
        this->low_cmd_ros.motor_cmd[RR_2].kp = 5.0;
        this->low_cmd_ros.motor_cmd[RR_2].kd = 1.0;

        // std::cout << "----------------------------------------------------------------------------------" << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[FL_0].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[FL_1].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[FL_2].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[FR_0].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[FR_1].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[FR_2].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[RL_0].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[RL_1].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[RL_2].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[RR_0].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[RR_1].q << std::endl;
        // std::cout << "-------------------------------Last point: " << this->low_cmd_ros.motor_cmd[RR_2].q << std::endl;
        // std::cout << "----------------------------------------------------------------------------------" << std::endl;

        joint_trajectory_pub_->publish(low_cmd_ros);        
    }
};

int main(int argc, char **argv)
{
    // ROS2 system initialization
    rclcpp::init(argc, argv);

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // ROS2 node creation with a loop rate of 500 Hz
    auto node = std::make_shared<JointTrajectoryHandler>();

    rclcpp::spin(node);
    // Maintains the loop rates at 500 Hz
    rclcpp::shutdown();

    return 0;
}