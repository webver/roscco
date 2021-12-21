#include <roscco/ros_to_oscc.h>

using std::placeholders::_1;

RosToOscc::RosToOscc(rclcpp::Node::SharedPtr public_nh) : public_nh(public_nh) {
    sigset_t mask;
    sigset_t orig_mask;

    sigemptyset(&mask);
    sigemptyset(&orig_mask);
    sigaddset(&mask, SIGIO);

    // Temporary block of OSCC SIGIO while initializing ROS publication to prevent
    // signal conflicts
    if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) {
        RCLCPP_ERROR(public_nh->get_logger(), "Failed to block SIGIO");
    }

    topic_brake_command_ =
            public_nh->create_subscription<roscco_interfaces::msg::BrakeCommand>("brake_command", 10,
                                                                                 std::bind(&RosToOscc::brakeCommandCallback, this, _1));

    topic_steering_command_ =
            public_nh->create_subscription<roscco_interfaces::msg::SteeringCommand>("steering_command", 10,
                                                                                 std::bind(&RosToOscc::steeringCommandCallback, this, _1));

    topic_throttle_command_ =
            public_nh->create_subscription<roscco_interfaces::msg::ThrottleCommand>("throttle_command", 10,
                                                                                 std::bind(&RosToOscc::throttleCommandCallback, this, _1));

    topic_enable_disable_command_ =
            public_nh->create_subscription<roscco_interfaces::msg::EnableDisable>("enable_disable", 10,
                                                                                 std::bind(&RosToOscc::enableDisableCallback, this, _1));

    if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0) {
        RCLCPP_ERROR(public_nh->get_logger(), "Failed to unblock SIGIO");
    }
}

void RosToOscc::brakeCommandCallback(const roscco_interfaces::msg::BrakeCommand::ConstSharedPtr msg) {
    oscc_result_t ret = OSCC_ERROR;

    ret = oscc_publish_brake_position(msg->brake_position);

    if (ret == OSCC_ERROR) {
        RCLCPP_ERROR(public_nh->get_logger(), "OSCC_ERROR occurred while trying send the brake position.");
    } else if (ret == OSCC_WARNING) {
        RCLCPP_WARN(public_nh->get_logger(), "OSCC_WARNING occurred while trying send the brake position.");
    }
}

void RosToOscc::steeringCommandCallback(const roscco_interfaces::msg::SteeringCommand::ConstSharedPtr msg) {
    oscc_result_t ret = OSCC_ERROR;

    ret = oscc_publish_steering_torque(msg->steering_torque);

    if (ret == OSCC_ERROR) {
        RCLCPP_ERROR(public_nh->get_logger(), "OSCC_ERROR occurred while trying send the steering torque.");
    } else if (ret == OSCC_WARNING) {
        RCLCPP_WARN(public_nh->get_logger(), "OSCC_WARNING occurred while trying send the steering torque.");
    }
}

void RosToOscc::throttleCommandCallback(const roscco_interfaces::msg::ThrottleCommand::ConstSharedPtr msg) {
    oscc_result_t ret = OSCC_ERROR;

    ret = oscc_publish_throttle_position(msg->throttle_position);

    if (ret == OSCC_ERROR) {
        RCLCPP_ERROR(public_nh->get_logger(), "OSCC_ERROR occurred while trying send the throttle position.");
    } else if (ret == OSCC_WARNING) {
        RCLCPP_WARN(public_nh->get_logger(), "OSCC_WARNING occurred while trying send the throttle position.");
    }
}

void RosToOscc::enableDisableCallback(const roscco_interfaces::msg::EnableDisable::ConstSharedPtr msg) {
    oscc_result_t ret = OSCC_ERROR;

    ret = msg->enable_control ? oscc_enable() : oscc_disable();

    if (ret == OSCC_ERROR) {
        RCLCPP_ERROR(public_nh->get_logger(), "OSCC_ERROR occurred while trying to enable or disable control.");
    } else if (ret == OSCC_WARNING) {
        RCLCPP_WARN(public_nh->get_logger(), "OSCC_WARNING occurred while trying to enable or disable control.");
    }
}
