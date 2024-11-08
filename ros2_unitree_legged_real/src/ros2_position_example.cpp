#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <cmath>

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv)
{
    // ROS2 system initialization
    rclcpp::init(argc, argv);

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // ROS2 node creation with a loop rate of 500 Hz
    auto node = rclcpp::Node::make_shared("node_ros2_postition_example");
    rclcpp::WallRate loop_rate(500);

    long motiontime = 0;

    ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;

    bool initiated_flag = false; // initiate need time
    int count = 0;

    // Create a publisher on the "/low_cmd" to send custom messages of the type LowCmd
    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1);

    // LowCmd msg fields setting 
    low_cmd_ros.head[0] = 0xFE; // 0xFE is a common header to mark the start of a message (first byte of the header)
    low_cmd_ros.head[1] = 0xEF; // 0xFE is a common header to mark the start of a message (second byte of the header)
    low_cmd_ros.level_flag = LOWLEVEL; // Control mode set to LOWLEVEL (direct control of the joint). Is defined in common.h (unitree_legged_sdk)

    // Actuator command loop initialization 
    for (int i = 0; i < 12; i++)
    {
        low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode (desired working mode. Servo : 0x0A, Damping : 0x00)
        low_cmd_ros.motor_cmd[i].q = PosStopF; // Desired angle (unit: radian). In this case disable position loop (which control the position of the joint). Is defined in common.h (unitree_legged_sdk)
        low_cmd_ros.motor_cmd[i].kp = 0; // Desired position stiffness (unit: N.m/rad )
        low_cmd_ros.motor_cmd[i].dq = VelStopF; // Desired velocity (unit: radian/second). In this case disable velocity loop (which control the speed of the joint). Is defined in common.h (unitree_legged_sdk)
        low_cmd_ros.motor_cmd[i].kd = 0; // Desired velocity stiffness (unit: N.m/(rad/s) 
        low_cmd_ros.motor_cmd[i].tau = 0; // Desired output torque (unit: N.m)
    }

    // Main loop runs till ROS2 is active
    while (rclcpp::ok())
    {

        // When initiated_flag is set to true montiontime is incremented of 2 and low_cmd_ros.motor is setted
        if (initiated_flag == true)
        {
            motiontime += 2;

            low_cmd_ros.motor_cmd[FR_0].tau = -0.65f;
            low_cmd_ros.motor_cmd[FL_0].tau = +0.65f;
            low_cmd_ros.motor_cmd[RR_0].tau = -0.65f;
            low_cmd_ros.motor_cmd[RL_0].tau = +0.65f;

            low_cmd_ros.motor_cmd[FR_2].q = -M_PI / 2 + 0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
            low_cmd_ros.motor_cmd[FR_2].dq = 0.0;
            low_cmd_ros.motor_cmd[FR_2].kp = 5.0;
            low_cmd_ros.motor_cmd[FR_2].kd = 1.0;

            low_cmd_ros.motor_cmd[FR_0].q = 0.0;
            low_cmd_ros.motor_cmd[FR_0].dq = 0.0;
            low_cmd_ros.motor_cmd[FR_0].kp = 5.0;
            low_cmd_ros.motor_cmd[FR_0].kd = 1.0;

            low_cmd_ros.motor_cmd[FR_1].q = 0.0;
            low_cmd_ros.motor_cmd[FR_1].dq = 0.0;
            low_cmd_ros.motor_cmd[FR_1].kp = 5.0;
            low_cmd_ros.motor_cmd[FR_1].kd = 1.0;
        }

        // After then cycles the motion control starts
        count++;
        if (count > 10)
        {
            count = 10;
            initiated_flag = true;
        }

        // Publish the command on low_cmd_ros
        pub->publish(low_cmd_ros);

        // Spin to process events and messages
        rclcpp::spin_some(node);

        // Maintains the loop rates at 500 Hz
        loop_rate.sleep();
    }

    return 0;
}