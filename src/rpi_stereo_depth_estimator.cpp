#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
// #include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <cmath>
#include "autonomous_navigation/stereo_processing.hpp"


// class RPiCameraNode : public rclcpp::Node
// {
// public:
//     RPiCameraNode() : Node("rpi_camera_node")
//     {
//         camera_.set(CV_CAP_PROP_FORMAT, CV_8UC3);
//         if (!camera_.open())
//         {
//             RCLCPP_ERROR(this->get_logger(), "Error opening the camera");
//             rclcpp::shutdown();
//         }

//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(33), // Approx 30 FPS
//             std::bind(&RPiCameraNode::captureAndDisplay, this));
//     }

// private:
//     void captureAndDisplay()
//     {
//         cv::Mat frame;
//         camera_.grab();
//         camera_.retrieve(frame);

//         if (!frame.empty())
//         {
//             cv::imshow("RPi Camera", frame);
//             cv::waitKey(1);
//         }
//     }

//     raspicam::RaspiCam_Cv camera_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<RPiCameraNode>());
//     rclcpp::shutdown();
//     return 0;
// }

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("simple_node");

    // Print a message to the console
    RCLCPP_INFO(node->get_logger(), "Hello, ROS!");

    // Spin the node (not necessary in this case as there are no subscriptions or timers)
    // rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}