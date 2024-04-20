#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <opencv2/opencv.hpp>

class StereoDepthEstimator : public rclcpp::Node
{
public:
    StereoDepthEstimator() : Node("stereo_depth_estimator")
    {
        this->declare_parameter("snapshot", false);

        // Create subscribers for left and right stereo image topics
        left_subscriber_.subscribe(this, "/left_camera/image");
        right_subscriber_.subscribe(this, "/right_camera/image");

        // Create a TimeSynchronizer to synchronize messages from both topics
        time_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(left_subscriber_, right_subscriber_, 10);
        time_sync_->registerCallback(std::bind(&StereoDepthEstimator::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Callback function for synchronized left and right stereo images
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img, const sensor_msgs::msg::Image::ConstSharedPtr &right_img)
    {
        bool snapshot = this->get_parameter("snapshot").as_bool();

        RCLCPP_INFO(this->get_logger(), "Received synchronized stereo images with timestamps: left=%f, right=%f", left_img->header.stamp.sec + left_img->header.stamp.nanosec * 1e-9, right_img->header.stamp.sec + right_img->header.stamp.nanosec * 1e-9);
        RCLCPP_INFO(this->get_logger(), "Snapshot: %s", snapshot ? "true" : "false");

        this->set_parameter(rclcpp::Parameter("snapshot", false));
    }

    // Subscription objects for left and right stereo images
    message_filters::Subscriber<sensor_msgs::msg::Image> left_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_subscriber_;

    // Pointer for TimeSynchronizer object
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> time_sync_;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoDepthEstimator>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2 node
    rclcpp::shutdown();
    return 0;
}
