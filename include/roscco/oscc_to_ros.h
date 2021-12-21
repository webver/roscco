#ifndef OSCC_TO_ROS_H
#define OSCC_TO_ROS_H

#include <signal.h>

extern "C" {
#include <oscc.h>
}

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <roscco_interfaces/msg/brake_report.hpp>
#include <roscco_interfaces/msg/brake_report_data.hpp>
#include <roscco_interfaces/msg/can_frame.hpp>
#include <roscco_interfaces/msg/fault_report.hpp>
#include <roscco_interfaces/msg/fault_report_data.hpp>
#include <roscco_interfaces/msg/steering_report.hpp>
#include <roscco_interfaces/msg/steering_report_data.hpp>
#include <roscco_interfaces/msg/throttle_report.hpp>
#include <roscco_interfaces/msg/throttle_report_data.hpp>

static rclcpp::Node::SharedPtr public_nh;

static rclcpp::Publisher<roscco_interfaces::msg::BrakeReport>::SharedPtr topic_brake_report_;

static rclcpp::Publisher<roscco_interfaces::msg::SteeringReport>::SharedPtr topic_steering_report_;

static rclcpp::Publisher<roscco_interfaces::msg::ThrottleReport>::SharedPtr topic_throttle_report_;

static rclcpp::Publisher<roscco_interfaces::msg::FaultReport>::SharedPtr topic_fault_report_;

static rclcpp::Publisher<roscco_interfaces::msg::CanFrame>::SharedPtr topic_obd_messages_;

class OsccToRos {
public:
    /**
     * @brief OsccToRos class initializer
     *
     * This function constructs ROS publishers and initializes the OSCC callbacks to begin transfering messages from OSCC
     * to ROS
     *
     * @param public_nh  The public node handle to use for ROS publishers.
     * @param private_nh The private node handle for ROS parameters.
     */
    OsccToRos(rclcpp::Node::SharedPtr public_nh);

    /**
     * @brief Callback function to publish oscc_steering_report messages on ROS.
     *
     * This function is a callback that consumes oscc_steering_report messages by utilizing the OSCC callback and
     * publishes that message with a ROS Publisher as a SteeringReport mesage.
     *
     * @param report The osc_steering_report message to be consumed by ROS.
     */
    static void steering_callback(oscc_steering_report_s *report);

    /**
     * @brief Callback function to publish oscc_brake_report messages on ROS.
     *
     * This function is a callback that consumes oscc_brake_report messages by utilizing the OSCC callback and
     * publishes that message with a ROS Publisher as a BrakeReport message.
     *
     * @param report The oscc_brake_report message to be consumed by ROS.
     */
    static void brake_callback(oscc_brake_report_s *report);

    /**
     * @brief Callback function to publish oscc_throttle_report messages on ROS.
     *
     * This function is a callback that consumes oscc_throttle_report messages by utilizing the OSCC callback and
     * publishes that message with a ROS Publisher as a ThrottleReport message.
     *
     * @param report The oscc_throttle_report message to be consumed by ROS.
     */
    static void throttle_callback(oscc_throttle_report_s *report);

    /**
     * @brief Callback function to publish oscc_fault_report messages on ROS.
     *
     * This function is a callback that consumes oscc_fault_report messages by utilizing the OSCC callback and publishes
     * that message with a ROS Publisher as a FaultReport message.
     *
     * @param report The oscc_fault_report message to be consumed by ROS.
     */
    static void fault_callback(oscc_fault_report_s *report);

    /**
     * @brief Callback function to publish can_frame messages on ROS.
     *
     * This function is a callback that consumes can_frame messages by utilizing the OSCC callback and publishes that
     * message with a ROS Publisher as a CanFrame message.
     *
     * @param report The oscc can_frame message to be consumed by ROS.
     */
    static void obd_callback(can_frame *frame);

    /**
     * @brief Cast OSCC report to ROS message and publish
     *
     * This function casts an OSCC report type to a ros message and publishes the message onto ROS. Where the ROSMSGTYPE
     * is the type that gets published containing the ROS header and wraps the now casted ROSDATATYPE. The OSCCTYPE
     * is cast to the ROSDATATYPE which should be the same memory layout as the OSCCTYPE.
     *
     * Usage:
     *
     *  cast an oscc_steering_report message to ROS and publish with the Publisher topic_steering_report_.
     *
     * cast_callback<oscc_steering_report_s,
     *                roscco::SteeringReport,
     *                roscco::SteeringReportData>(report, &topic_steering_report_);
     *
     * @tparam OSCCTYPE    OSCC report message data type
     * @tparam ROSMSGTYPE  ROS parent message type
     * @tparam ROSDATATYPE ROS data message type
     *
     * @param report       OSCC report message
     * @param pub          ROS publisher to send the OSCC message
     */
    template<class OSCCTYPE, class ROSMSGTYPE, class ROSDATATYPE>
    static void cast_callback(OSCCTYPE *report, typename rclcpp::Publisher<ROSMSGTYPE>::SharedPtr pub);
};

#endif  // OSCC_TO_ROS_H
