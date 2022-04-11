#include <roscco/oscc_to_ros.h>

OsccToRos::OsccToRos(rclcpp::Node::SharedPtr public_nh_){
    //Update static members
    public_nh = public_nh_;

    sigset_t mask;
    sigset_t orig_mask;

    sigemptyset(&mask);
    sigemptyset(&orig_mask);
    sigaddset(&mask, SIGIO);

    // Temporary block of OSCC SIGIO while initializing ROS subscription to
    // prevent signal conflicts
    if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) {
        RCLCPP_ERROR(public_nh->get_logger(), "Failed to block SIGIO");
    }

    topic_brake_report_ = public_nh->create_publisher<roscco_interfaces::msg::BrakeReport>("brake_report", 10);

    topic_steering_report_ = public_nh->create_publisher<roscco_interfaces::msg::SteeringReport>("steering_report", 10);

    topic_throttle_report_ = public_nh->create_publisher<roscco_interfaces::msg::ThrottleReport>("throttle_report", 10);

    topic_selector_report_ = public_nh->create_publisher<roscco_interfaces::msg::SelectorReport>("selector_report", 10);

    topic_fault_report_ = public_nh->create_publisher<roscco_interfaces::msg::FaultReport>("fault_report", 10);

    topic_obd_messages_ = public_nh->create_publisher<roscco_interfaces::msg::CanFrame>("can_frame", 10);

    if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0) {
        RCLCPP_ERROR(public_nh->get_logger(), "Failed to unblock SIGIO");
    }

    oscc_subscribe_to_brake_reports(brake_callback);

    oscc_subscribe_to_steering_reports(steering_callback);

    oscc_subscribe_to_throttle_reports(throttle_callback);

    oscc_subscribe_to_selector_reports(selector_callback);

    oscc_subscribe_to_fault_reports(fault_callback);

    oscc_subscribe_to_obd_messages(obd_callback);
}

void OsccToRos::steering_callback(oscc_steering_report_s *report) {
    cast_callback<oscc_steering_report_s, roscco_interfaces::msg::SteeringReport, roscco_interfaces::msg::SteeringReportData>(
            report,
            topic_steering_report_);
}

void OsccToRos::brake_callback(oscc_brake_report_s *report) {
    cast_callback<oscc_brake_report_s, roscco_interfaces::msg::BrakeReport, roscco_interfaces::msg::BrakeReportData>(
            report, topic_brake_report_);
}

void OsccToRos::throttle_callback(oscc_throttle_report_s *report) {
    cast_callback<oscc_throttle_report_s, roscco_interfaces::msg::ThrottleReport, roscco_interfaces::msg::ThrottleReportData>(
            report,
            topic_throttle_report_);
}

void OsccToRos::selector_callback(oscc_selector_report_s *report) {
    cast_callback<oscc_selector_report_s, roscco_interfaces::msg::SelectorReport, roscco_interfaces::msg::SelectorReportData>(
            report,
            topic_selector_report_);
}

template<class OSCCTYPE, class ROSMSGTYPE, class ROSDATATYPE>
void OsccToRos::cast_callback(OSCCTYPE *report, typename rclcpp::Publisher<ROSMSGTYPE>::SharedPtr pub) {
    ROSMSGTYPE *ros_message(new ROSMSGTYPE);

    ROSDATATYPE *data = (ROSDATATYPE *) report;

    ros_message->data = *data;

    ros_message->header.stamp = public_nh->now();

    pub->publish(*ros_message);

    delete ros_message;
}

void OsccToRos::fault_callback(oscc_fault_report_s *report) {
    roscco_interfaces::msg::FaultReport *ros_message(new roscco_interfaces::msg::FaultReport);

    // ROS does not pack the structs so individual assignment is required over
    // cast
    ros_message->data.magic[0] = report->magic[0];
    ros_message->data.magic[1] = report->magic[1];
    ros_message->data.fault_origin_id = report->fault_origin_id;
    ros_message->data.dtcs = report->dtcs;
    ros_message->data.reserved = report->reserved;

    ros_message->header.stamp = public_nh->now();

    topic_fault_report_->publish(*ros_message);

    delete ros_message;
}

void OsccToRos::obd_callback(can_frame *frame) {
    roscco_interfaces::msg::CanFrame *ros_message(new roscco_interfaces::msg::CanFrame);

    ros_message->frame.can_id = frame->can_id;

    ros_message->frame.can_dlc = frame->can_dlc;

    for (int i = 0; i < ros_message->frame.CAN_FRAME_DATA_MAX_SIZE; i++) {
        ros_message->frame.data[i] = frame->data[i];
    }

    ros_message->header.stamp = public_nh->now();

    topic_obd_messages_->publish(*ros_message);

    delete ros_message;
}
