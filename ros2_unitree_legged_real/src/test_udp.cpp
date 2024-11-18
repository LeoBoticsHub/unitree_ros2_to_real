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

#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "convert.h"


using namespace UNITREE_LEGGED_SDK;

class CustomTest
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    CustomTest()
        : 
        // low_udp(LOWLEVEL),
        low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        high_udp(HIGHLEVEL, 8090, "192.168.123.161", 8082)
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
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

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

CustomTest custom_test;

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;

rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr pub_low;

long high_count = 0;
long low_count = 0;

void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    // printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom_test.high_cmd = rosMsg2Cmd(msg);

    ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom_test.high_state);

    pub_high->publish(high_state_ros);
    // printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
{

    // printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom_test.low_cmd = rosMsg2Cmd(msg);

    custom_test.low_udp.SetSend(custom_test.low_cmd);
    custom_test.low_udp.Send();

    ros2_unitree_legged_msgs::msg::LowState low_state_ros;

    low_state_ros = state2rosMsg(custom_test.low_state);

    pub_low->publish(low_state_ros);


    std::cout << custom_test.low_state.motorState[UNITREE_LEGGED_SDK::FL_0].q << std::endl;
    std::cout << custom_test.low_state.bms.current << std::endl;

    // printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_ros2_udp_test");


    printf("Low level running!\n");

    pub_high = node->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
    pub_low = node->create_publisher<ros2_unitree_legged_msgs::msg::LowState>("low_state", 1);

    sub_low = node->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1, lowCmdCallback);
    // sub_high = node->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);



    LoopFunc loop_udpSendH("high_udp_send", 0.002, 3, boost::bind(&CustomTest::highUdpSend, &custom_test));
    LoopFunc loop_udpRecvH("high_udp_recv", 0.002, 3, boost::bind(&CustomTest::highUdpRecv, &custom_test));

    loop_udpSendH.start();
    loop_udpRecvH.start();

    LoopFunc loop_udpSendL("low_udp_send", 0.002, 3, boost::bind(&CustomTest::lowUdpSend, &custom_test));
    LoopFunc loop_udpRecvL("low_udp_recv", 0.002, 3, boost::bind(&CustomTest::lowUdpRecv, &custom_test));

    loop_udpRecvL.start();
    loop_udpSendL.start();

    std::cout << custom_test.low_state.motorState[UNITREE_LEGGED_SDK::FL_0].q << std::endl;
    std::cout << custom_test.low_state.bms.current << std::endl;

    std::cout<<"yep"<<std::endl;
    std::cout << custom_test.high_state.motorState[UNITREE_LEGGED_SDK::FL_0].q << std::endl;

    // rclcpp::spin(node);
    // rclcpp::shutdown();

    rclcpp::Rate rate(10);  // 10 Hz

    while (rclcpp::ok())
    {
        ros2_unitree_legged_msgs::msg::LowState low_state_ros_test;

        low_state_ros_test = state2rosMsg(custom_test.low_state);

        pub_low->publish(low_state_ros_test);
        std::cout << custom_test.low_state.motorState[UNITREE_LEGGED_SDK::FL_0].q << std::endl;

        ros2_unitree_legged_msgs::msg::HighState high_state_ros_test;

        // Convert the state to a ROS message
        high_state_ros_test = state2rosMsg(custom_test.high_state);

        // Publish the high state
        pub_high->publish(high_state_ros_test);

        std::cout<<"yep"<<std::endl;
        std::cout << custom_test.high_state.motorState[UNITREE_LEGGED_SDK::FL_0].q << std::endl;
        
        // Sleep to maintain the loop rate
        rate.sleep();
    }

    return 0;
}