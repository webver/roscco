#ifndef ROS_TO_OSCC_H
#define ROS_TO_OSCC_H

#include <signal.h>

extern "C" {
#include <oscc.h>
}

#include "rclcpp/rclcpp.hpp"

#include <roscco_interfaces/msg/brake_command.hpp>
#include <roscco_interfaces/msg/enable_disable.hpp>
#include <roscco_interfaces/msg/steering_command.hpp>
#include <roscco_interfaces/msg/throttle_command.hpp>

class RosToOscc {
public:
    /**
     * @brief RosToOscc class initializer
     *
     * This function constructs ROS subscribers which can publish messages to OSCC API.
     *
     * @param public_nh  The public node handle to use for ROS subscribers.
     * @param private_nh The private node handle for ROS parameters.
     */
    RosToOscc(rclcpp::Node::SharedPtr public_nh);

    /**
     * @brief Callback function to publish ROS BrakeCommand messages to OSCC.
     *
     * This function is a callback that consume a ROS BrakeCommand message and publishes them to the OSCC API.
     *
     * @param msg ROS BrakeCommand message to be consumed.
     */
    void brakeCommandCallback(const roscco_interfaces::msg::BrakeCommand::ConstSharedPtr msg);

    /**
     * @brief Callback function to publish ROS SteeringCommand messages to OSCC.
     *
     * This function is a callback that consumes a ROS SteeringCommand message and publishes them to the OSCC API.
     *
     * @param msg ROS SteeringCommand message to be consumed.
     */
    void steeringCommandCallback(const roscco_interfaces::msg::SteeringCommand::ConstSharedPtr msg);

    /**
     * @brief Callback function to publish ROS ThrottleCommand messages to OSCC.
     *
     * This function is a callback that consumes a ROS ThrottleCommand message and publishes them to the OSCC API.
     *
     * @param msg ROS ThrottleCommand message to be consumed.
     */
    void throttleCommandCallback(const roscco_interfaces::msg::ThrottleCommand::ConstSharedPtr msg);

    /**
     * @brief Callback function to publish ROS EnableDisable messages to OSCC.
     *
     * This function is a callback that consumes a ROS EnableDisable message and publishes them to the OSCC API.
     *
     * @param msg ROS EnableDisable message to be consumed.
     */
    void enableDisableCallback(const roscco_interfaces::msg::EnableDisable::ConstSharedPtr msg);

private:
    rclcpp::Node::SharedPtr public_nh;

    rclcpp::Subscription<roscco_interfaces::msg::BrakeCommand>::SharedPtr topic_brake_command_;

    rclcpp::Subscription<roscco_interfaces::msg::SteeringCommand>::SharedPtr topic_steering_command_;

    rclcpp::Subscription<roscco_interfaces::msg::ThrottleCommand>::SharedPtr topic_throttle_command_;

    rclcpp::Subscription<roscco_interfaces::msg::EnableDisable>::SharedPtr topic_enable_disable_command_;
};

#endif  // ROS_TO_OSCC_H
