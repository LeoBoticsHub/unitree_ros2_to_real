/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#include "ros2_unitree_legged_real/unitree_high_ros2_control.hpp"

UnitreeRos2HighController::UnitreeRos2HighController():
    Node("unitree_high_ros2_control")
{   
    // read ros parameters
    this->declare_parameter<std::string>("robot_name", "");
    this->declare_parameter<std::string>("tf_namespace", "");
    this->declare_parameter<double>("udp_loop_time", 0.0);
    this->declare_parameter<double>("state_pub_loop_time", 0.0);
    this->declare_parameter<double>("euler_x_bound", 0.0);
    this->declare_parameter<double>("euler_y_bound", 0.0);
    this->declare_parameter<double>("euler_z_bound", 0.0);
    this->declare_parameter<double>("foot_height_lower_bound", 0.0);
    this->declare_parameter<double>("foot_height_upper_bound", 0.0);
    this->declare_parameter<double>("body_height_lower_bound", 0.0);
    this->declare_parameter<double>("body_height_upper_bound", 0.0);
    this->declare_parameter<double>("min_vx", 0.0);
    this->declare_parameter<double>("max_vx", 0.0);
    this->declare_parameter<double>("vy_bound", 0.0);
    this->declare_parameter<double>("w_bound", 0.0);

    this->get_parameter("robot_name", robot_name_);
    this->get_parameter("tf_namespace", tf_namespace_);
    this->get_parameter("udp_loop_time", udp_loop_time_);
    this->get_parameter("state_pub_loop_time", state_pub_loop_time_);
    this->get_parameter("euler_x_bound", euler_x_bound_);
    this->get_parameter("euler_y_bound", euler_y_bound_);
    this->get_parameter("euler_z_bound", euler_z_bound_);
    this->get_parameter("foot_height_lower_bound", foot_height_lower_bound_);
    this->get_parameter("foot_height_upper_bound", foot_height_upper_bound_);
    this->get_parameter("body_height_lower_bound", body_height_lower_bound_);
    this->get_parameter("body_height_upper_bound", body_height_upper_bound_);
    this->get_parameter("min_vx", min_vx_);
    this->get_parameter("max_vx", max_vx_);
    this->get_parameter("vy_bound", vy_bound_);
    this->get_parameter("w_bound", w_bound_);

    // select the correct robot ip depending on robot name
    std::string robot_ip{};
    std::shared_ptr<Safety> safe;
    if (robot_name_.compare("go1") == 0)
    {
        robot_ip = "192.168.12.1";
        safe = std::make_shared<Safety>(LeggedType::Go1);
    }
    else if (robot_name_.compare("b1") == 0)
    {
        robot_ip = "192.168.123.220";
        safe = std::make_shared<Safety>(LeggedType::B1);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Select a correct robot name between b1 and go1.");
        exit(1);
    }

    custom_ = std::make_shared<Custom>(robot_ip.c_str(), *safe);
}

void UnitreeRos2HighController::init_class()
{
    // * set joint states size
    joint_state_msg_.name.resize(N_MOTORS);
    joint_state_msg_.position.resize(N_MOTORS);
    joint_state_msg_.velocity.resize(N_MOTORS);
    joint_state_msg_.effort.resize(N_MOTORS);

    // * set odom frame ids
    odom_H_trunk_.header.frame_id = tf_namespace_ + ODOM_NAME;
    odom_H_trunk_.child_frame_id = tf_namespace_ + BASE_LINK_NAME;

    // * initialize time
    t_ = t_prev_ = t_timer_ = t_cmd_orient_ = t_cmd_vel_ = this->get_clock()->now();
    timer_on_ = false;

    // * Read ros params
    this->declare_parameter<bool>("publish_odom_tf", false);
    this->get_parameter("publish_odom_tf", publish_odom_tf_);

    // * Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1,
        std::bind(&UnitreeRos2HighController::cmdVelCallback, this, std::placeholders::_1)
    );
    body_orient_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "cmd_orient", 1,
        std::bind(&UnitreeRos2HighController::cmdBodyOrientationCallback, this, std::placeholders::_1)
    );

    // * Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("robot_odom", 1);
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // * Services
    power_off_srv_ = this->create_service<std_srvs::srv::Trigger>("power_off",                 
        std::bind(&UnitreeRos2HighController::powerOffCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    emergency_button_srv_ = this->create_service<std_srvs::srv::Trigger>("emergency_stop",                 
        std::bind(&UnitreeRos2HighController::emergencyStopCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    stand_up_srv_ = this->create_service<std_srvs::srv::Trigger>("stand_up",                 
        std::bind(&UnitreeRos2HighController::standUpCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    stand_down_srv_ = this->create_service<std_srvs::srv::Trigger>("stand_down",                 
        std::bind(&UnitreeRos2HighController::standDownCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    reset_odom_srv_ = this->create_service<std_srvs::srv::Trigger>("reset_odometry",                 
        std::bind(&UnitreeRos2HighController::resetOdometryCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    revert_odom_srv_ = this->create_service<std_srvs::srv::Trigger>("revert_odometry",                 
        std::bind(&UnitreeRos2HighController::revertOdometryCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    get_mode_srv_ = this->create_service<ros2_unitree_legged_msgs::srv::GetInt>("get_robot_mode",                 
        std::bind(&UnitreeRos2HighController::getRobotModeCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    battery_state_srv_ = this->create_service<ros2_unitree_legged_msgs::srv::GetBatteryState>("get_battery_state",                 
        std::bind(&UnitreeRos2HighController::batteryStateCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    set_foot_height_srv_ = this->create_service<ros2_unitree_legged_msgs::srv::SetFloat>("set_foot_height",                 
        std::bind(&UnitreeRos2HighController::setFootHeightCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    get_foot_height_srv_ = this->create_service<ros2_unitree_legged_msgs::srv::GetFloat>("get_foot_height",                 
        std::bind(&UnitreeRos2HighController::getFootHeightCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    set_body_height_srv_ = this->create_service<ros2_unitree_legged_msgs::srv::SetFloat>("set_body_height",                 
        std::bind(&UnitreeRos2HighController::setBodyHeightCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );
    get_body_height_srv_ = this->create_service<ros2_unitree_legged_msgs::srv::GetFloat>("get_body_height",                 
        std::bind(&UnitreeRos2HighController::getBodyHeightCallback,
            this, std::placeholders::_1, std::placeholders::_2
        )
    );

    // High state UDP loop function
    loop_udpSend_ = std::make_shared<LoopFunc>("high_udp_send", udp_loop_time_, 3, boost::bind(&Custom::highUdpSend, &*custom_));
    loop_udpRecv_ = std::make_shared<LoopFunc>("high_udp_recv", udp_loop_time_, 3, boost::bind(&Custom::highUdpRecv, &*custom_));

    // * State publisher loop function
    loop_StatePub_ = std::make_shared<LoopFunc>("high_state_pub", state_pub_loop_time_, 3, boost::bind(&UnitreeRos2HighController::highStatePublisher, this));

    loop_udpSend_->start();
    loop_udpRecv_->start();
    loop_StatePub_->start();

    // * initialize first body height for odom tf
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    initial_body_height_ = custom_->high_state.bodyHeight;
    initial_position_ = custom_->high_state.position;

    printf("HIGHLEVEL is initialized\n");
}

void UnitreeRos2HighController::highStatePublisher()
{
    t_ = this->get_clock()->now();

    // zero cmd vel if no command are received 
    if ((t_ - t_cmd_vel_).seconds() > 1 && custom_->high_state.mode != static_cast<uint8_t>(ROBOT_STATE::DAMPING) && cmd_vel_active_)
    {
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::TARGET_VEL);
        custom_->high_cmd.velocity[0] = 0;
        custom_->high_cmd.velocity[1] = 0;
        custom_->high_cmd.yawSpeed = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::IDLE);
        cmd_vel_active_ = false;
    }

    // zero cmd vel if no command are received 
    if ((t_ - t_cmd_orient_).seconds() > 1 && custom_->high_state.mode != static_cast<uint8_t>(ROBOT_STATE::DAMPING) && cmd_orient_active_)
    {
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::FORCE_STAND);
        custom_->high_cmd.euler[0] = 0;
        custom_->high_cmd.euler[1] = 0;
        custom_->high_cmd.euler[2] = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::TARGET_VEL);
        custom_->high_cmd.velocity[0] = 0;
        custom_->high_cmd.velocity[1] = 0;
        custom_->high_cmd.yawSpeed = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::IDLE);
        cmd_orient_active_ = false;
    }

    if (t_ != t_prev_)
    {
        // joint states
        joint_state_msg_.header.stamp = t_;
        joint_state_msg_.header.frame_id = tf_namespace_ + BASE_LINK_NAME;

        for (unsigned int motor_id = 0; motor_id < N_MOTORS; ++motor_id)
	    {
            joint_state_msg_.name[motor_id]     = b1_motor_names[motor_id];
            joint_state_msg_.position[motor_id] = static_cast<double>(custom_->high_state.motorState[b1_motor_idxs[motor_id]].q);
            joint_state_msg_.velocity[motor_id] = static_cast<double>(custom_->high_state.motorState[b1_motor_idxs[motor_id]].dq); // NOTE: this order is different than google
            joint_state_msg_.effort[motor_id]   = static_cast<double>(custom_->high_state.motorState[b1_motor_idxs[motor_id]].tauEst);
	    }

        // imu
	    imu_msg_.header.stamp = t_;
	    imu_msg_.header.frame_id = tf_namespace_ + IMU_NAME;
	    imu_msg_.orientation.w = static_cast<double>(custom_->high_state.imu.quaternion[0]);
	    imu_msg_.orientation.x = static_cast<double>(custom_->high_state.imu.quaternion[1]);
	    imu_msg_.orientation.y = static_cast<double>(custom_->high_state.imu.quaternion[2]);
	    imu_msg_.orientation.z = static_cast<double>(custom_->high_state.imu.quaternion[3]);
	    imu_msg_.angular_velocity.x = static_cast<double>(custom_->high_state.imu.gyroscope[0]);
	    imu_msg_.angular_velocity.y = static_cast<double>(custom_->high_state.imu.gyroscope[1]);
	    imu_msg_.angular_velocity.z = static_cast<double>(custom_->high_state.imu.gyroscope[2]);
	    imu_msg_.linear_acceleration.x = static_cast<double>(custom_->high_state.imu.accelerometer[0]);
	    imu_msg_.linear_acceleration.y = static_cast<double>(custom_->high_state.imu.accelerometer[1]);
	    imu_msg_.linear_acceleration.z = static_cast<double>(custom_->high_state.imu.accelerometer[2]);

        // odom
	    odom_msg_.header.stamp = t_;
	    odom_msg_.header.frame_id            = tf_namespace_ + ODOM_NAME;
	    odom_msg_.child_frame_id             = tf_namespace_ + BASE_LINK_NAME;
	    odom_msg_.pose.pose.position.x       = static_cast<double>(custom_->high_state.position[0]) - initial_position_[0];
	    odom_msg_.pose.pose.position.y       = static_cast<double>(custom_->high_state.position[1]) - initial_position_[1];
	    odom_msg_.pose.pose.position.z       = static_cast<double>(custom_->high_state.position[2]) - initial_position_[2] + initial_body_height_;
	    odom_msg_.pose.pose.orientation      = imu_msg_.orientation;
	    odom_msg_.twist.twist.linear.x       = static_cast<double>(custom_->high_state.velocity[0]);
	    odom_msg_.twist.twist.linear.y       = static_cast<double>(custom_->high_state.velocity[1]);
	    odom_msg_.twist.twist.linear.z       = static_cast<double>(custom_->high_state.velocity[2]);
	    odom_msg_.twist.twist.angular        = imu_msg_.angular_velocity;

        // odom -> base_link
        if (publish_odom_tf_)
        {
            odom_H_trunk_.header.stamp = t_;
            odom_H_trunk_.transform.translation.x       = static_cast<double>(custom_->high_state.position[0]) - initial_position_[0]; 
            odom_H_trunk_.transform.translation.y       = static_cast<double>(custom_->high_state.position[1]) - initial_position_[1];
            odom_H_trunk_.transform.translation.z       = static_cast<double>(custom_->high_state.position[2]) - initial_position_[2] + initial_body_height_;
            odom_H_trunk_.transform.rotation            = imu_msg_.orientation;
            tf_pub_->sendTransform(odom_H_trunk_);
        }

        // publish state messages
        joint_states_pub_->publish(joint_state_msg_);
        imu_pub_->publish(imu_msg_);
        odom_pub_->publish(odom_msg_);

        // check wirless remote received commands (used to bypass ros cmd_vel if needed)
        memcpy(&custom_->keyData, &custom_->high_state.wirelessRemote[0], 40); 
    }
    
    t_prev_ = t_;
}

void UnitreeRos2HighController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Check if wirless remote command are given
    if ( std::abs(custom_->keyData.rx) < 0.1 && 
         std::abs(custom_->keyData.lx) < 0.1 && 
         std::abs(custom_->keyData.ry) < 0.1 && 
         std::abs(custom_->keyData.ly) < 0.1 )
    {
        // check timer if previous wirless remote commands have been receives
        if(timer_on_ && (t_ - t_timer_).seconds() >= 5 || !timer_on_)
        {
            custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::TARGET_VEL);
            custom_->high_cmd.euler[0] = 0;
            custom_->high_cmd.euler[1] = 0;
            custom_->high_cmd.euler[2] = 0;
            custom_->high_cmd.gaitType = 1;
            custom_->high_cmd.velocity[0] = bound(msg->linear.x, min_vx_, max_vx_);
            custom_->high_cmd.velocity[1] = bound(msg->linear.y, -vy_bound_, vy_bound_);
            custom_->high_cmd.yawSpeed = bound(msg->angular.z, -w_bound_, w_bound_);
  
            timer_on_ = false;
            cmd_vel_active_ = true;
            t_cmd_vel_ = this->get_clock()->now();
        }
    }
    else
    {
        // initialize timer if wirless remote commands are received
        if(!timer_on_)
            timer_on_ = true;
        t_timer_ = this->get_clock()->now();
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::IDLE);
        custom_->high_cmd.velocity[0] = 0;
        custom_->high_cmd.velocity[1] = 0;
        custom_->high_cmd.yawSpeed = 0;
    }
}

void UnitreeRos2HighController::cmdBodyOrientationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    // Check if wirless remote command are given
    if ( std::abs(custom_->keyData.rx) < 0.1 && 
         std::abs(custom_->keyData.lx) < 0.1 && 
         std::abs(custom_->keyData.ry) < 0.1 && 
         std::abs(custom_->keyData.ly) < 0.1 )
    {
        if (!cmd_vel_active_ &&
            (timer_on_ && (t_ - t_timer_).seconds() >= 5 || !timer_on_)
        )
        {
            custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::FORCE_STAND);
            custom_->high_cmd.euler[0] = bound(msg->x, -euler_x_bound_, euler_x_bound_);
            custom_->high_cmd.euler[1] = bound(msg->y, -euler_y_bound_, euler_y_bound_);
            custom_->high_cmd.euler[2] = bound(msg->z, -euler_z_bound_, euler_z_bound_);
            custom_->high_cmd.velocity[0] = 0;
            custom_->high_cmd.velocity[1] = 0;
            custom_->high_cmd.yawSpeed    = 0;

            cmd_orient_active_ = true;
            t_cmd_orient_ = this->get_clock()->now();
            timer_on_ = false;
        }
    }
    else
    {
        // initialize timer if wirless remote commands are received
        if(!timer_on_)
            timer_on_ = true;
        t_timer_ = this->get_clock()->now();
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::IDLE);
        custom_->high_cmd.euler[0] = 0;
        custom_->high_cmd.euler[1] = 0;
        custom_->high_cmd.euler[2] = 0;
    }
}

bool UnitreeRos2HighController::powerOffCallback(
    std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> res
)
{
    // only if in stand down position
    if(custom_->high_state.mode == 7)
    {
        BmsCmd battery_cmd;
        battery_cmd.off = 0xA5;
        sleep(3);
        custom_->high_cmd.bms = battery_cmd;
    }
    else
    {
        res->success = false;
        res->message = "The robot is not in stand down position. Cannot power off in this situation.";
    }

    return true;
}

bool UnitreeRos2HighController::emergencyStopCallback(
    std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> res
)
{
    custom_->high_cmd.euler[0] = 0;
    custom_->high_cmd.euler[1] = 0;
    custom_->high_cmd.euler[2] = 0;
    custom_->high_cmd.velocity[0] = 0;
    custom_->high_cmd.velocity[1] = 0;
    custom_->high_cmd.yawSpeed    = 0;
    custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::DAMPING);
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::IDLE);

    res->success = true;
    res->message = "Emergency Stop! Robot set in dumping mode.";

    return true;
}

bool UnitreeRos2HighController::batteryStateCallback(
    std::shared_ptr<ros2_unitree_legged_msgs::srv::GetBatteryState::Request> req, 
    std::shared_ptr<ros2_unitree_legged_msgs::srv::GetBatteryState::Response> res
)
{
    battery_msg_.header.stamp = t_;
    battery_msg_.header.frame_id = tf_namespace_ + BASE_LINK_NAME;
    battery_msg_.current = static_cast<float>(1000.0 * custom_->high_state.bms.current); // mA -> A
    battery_msg_.voltage = static_cast<float>(1000.0 * avg<uint16_t>(&custom_->high_state.bms.cell_vol[0], 10)); // mV -> V
    battery_msg_.percentage = custom_->high_state.bms.SOC;

    res->battery_state = battery_msg_;
    
    return true;   
}

bool UnitreeRos2HighController::standUpCallback(
    std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> res
)
{
    const std::lock_guard<std::mutex> lock(stand_mtx_);
    if (custom_->high_state.mode == 7) // If in dumping mode
    {
        custom_->high_cmd.euler[0] = 0;
        custom_->high_cmd.euler[1] = 0;
        custom_->high_cmd.euler[2] = 0;
        custom_->high_cmd.velocity[0] = 0;
        custom_->high_cmd.velocity[1] = 0;
        custom_->high_cmd.yawSpeed    = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::STAND_UP);
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::FORCE_STAND);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::TARGET_VEL);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::IDLE);

        res->success = true;
        res->message = "Robot stand up completed.";
    }
    else
    {
        res->success = false;
        res->message = "Error! The robot is not in dumping position. Cannot stand up.";
    }

    return true;
}

bool UnitreeRos2HighController::standDownCallback(
    std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> res
)
{
    const std::lock_guard<std::mutex> lock(stand_mtx_);
    if (custom_->high_state.mode == 1) // If in standing mode
    {
        custom_->high_cmd.euler[0] = 0;
        custom_->high_cmd.euler[1] = 0;
        custom_->high_cmd.euler[2] = 0;
        custom_->high_cmd.velocity[0] = 0;
        custom_->high_cmd.velocity[1] = 0;
        custom_->high_cmd.yawSpeed    = 0;
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::STAND_UP);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::STAND_DOWN);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        if (robot_name_.compare("go1") == 0)
        {
            custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::DAMPING);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }        
        custom_->high_cmd.mode = static_cast<uint8_t>(ROBOT_STATE::IDLE);

        res->success = true;
        res->message = "Robot stand down completed.";
    }
    else
    {
        res->success = false;
        res->message = "Error! The robot is not in fixed standing position. Cannot stand down.";
    }

    return true;
}


bool UnitreeRos2HighController::resetOdometryCallback(
    std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> res
)
{
    initial_body_height_ = custom_->high_state.bodyHeight;
    initial_position_ = custom_->high_state.position;

    return true;
}

bool UnitreeRos2HighController::revertOdometryCallback(
    std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> res
)
{
    initial_body_height_ = 0;
    initial_position_[0] = 0.0;
    initial_position_[1] = 0.0;
    initial_position_[2] = 0.0;

    return true;
}


bool UnitreeRos2HighController::getRobotModeCallback(
    std::shared_ptr<ros2_unitree_legged_msgs::srv::GetInt::Request> /* req */, 
    std::shared_ptr<ros2_unitree_legged_msgs::srv::GetInt::Response> res
)
{
    res->value = custom_->high_state.mode;

    return true;
}

bool UnitreeRos2HighController::setFootHeightCallback(
    std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Request> req, 
    std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Response> res
)
{
    // check if the request is in the allowed delta bounds
    if (req->value >= foot_height_lower_bound_ && req->value <= foot_height_upper_bound_)
    {
        custom_->high_cmd.footRaiseHeight = req->value;
        res->success = true;
        res->message = "Foot hight delta set to: " + std::to_string(req->value);
    }
    else
    {
        res->success = false;
        res->message = "Error! Invalid foot height delta! \
        Please provide a delta in the range [%.2f, %.2f].", foot_height_lower_bound_, foot_height_upper_bound_;
    }

    return true;
}

bool UnitreeRos2HighController::getFootHeightCallback(
    std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Request> /* req */, 
    std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Response> res
)
{
    res->value = custom_->high_state.footRaiseHeight;

    return true;
}

bool UnitreeRos2HighController::setBodyHeightCallback(
    std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Request> req, 
    std::shared_ptr<ros2_unitree_legged_msgs::srv::SetFloat::Response> res
)
{
    // check if the request is in the allowed delta bounds
    if (req->value >= body_height_lower_bound_ && req->value <= body_height_upper_bound_)
    {
        custom_->high_cmd.bodyHeight = req->value;
        res->success = true;
        res->message = "Body height delta set to: " + std::to_string(req->value);
    }
    else
    {
        res->success = false;
        res->message = "Error! Invalid body height delta! \
        Please provide a delta in the range [%.2f, %.2f].", body_height_lower_bound_, body_height_upper_bound_;
    }

    return true;
}

bool UnitreeRos2HighController::getBodyHeightCallback(
    std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Request> /* req */, 
    std::shared_ptr<ros2_unitree_legged_msgs::srv::GetFloat::Response> res
)
{
    res->value = custom_->high_state.bodyHeight;

    return true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Initialize ros 2 interface object
    auto node{std::make_shared<UnitreeRos2HighController>()};
    node->init_class();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}