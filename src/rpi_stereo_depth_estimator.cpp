#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "autonomous_navigation/stereo_processing.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("rpi_node");

    // Print a message to the console
    RCLCPP_INFO(node->get_logger(), "Rpi Stereo Depth Estimator");

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}