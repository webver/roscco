#include <string>

extern "C" {
#include <oscc.h>
}

#include "rclcpp/rclcpp.hpp"

#include <roscco/oscc_to_ros.h>
#include <roscco/ros_to_oscc.h>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto public_nh = rclcpp::Node::make_shared("roscco_node");

    int can_channel = 0;
//    public_nh->set_parameter("can_channel", can_channel, 0);

    oscc_result_t ret = OSCC_ERROR;

    ret = oscc_init();

    if (ret != OSCC_OK) {
        RCLCPP_INFO(public_nh->get_logger(), "Could not initialize OSCC");
    }

    RosToOscc subcriber(public_nh);
    OsccToRos publisher(public_nh);

    rclcpp::spin(public_nh);

    ret = oscc_disable();

    if (ret != OSCC_OK) {
        RCLCPP_ERROR(public_nh->get_logger(), "Could not disable OSCC");
    }

    ret = oscc_close(can_channel);

    if (ret != OSCC_OK) {
        RCLCPP_ERROR(public_nh->get_logger(), "Could not close OSCC connection");
    }

    rclcpp::shutdown();
}
