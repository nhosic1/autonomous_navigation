#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>


class StateMonitor : public rclcpp::Node
{
public:
    StateMonitor() : Node("state_monitor")
    {
        // Create a timer to check FPS
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]()
                                         {
            RCLCPP_INFO(this->get_logger(), "FPS = %d", callback_count_);

            // Reset the callback count
            callback_count_ = 0; });

        // Subscription to the camera image topic
        cam_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/left_camera/image", 10, std::bind(&StateMonitor::camera_image_callback, this, std::placeholders::_1));

        // Subscription to the odometry path image topic
        path_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/autonomous_navigator/path_image", 10, std::bind(&StateMonitor::odometry_path_image_callback, this, std::placeholders::_1));
    }

private:
    void camera_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the ROS image message to an OpenCV image
        cv::Mat camera_image;
        try {
            camera_image = cv_bridge::toCvCopy(msg, "rgb8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        callback_count_++;

        // Display the camera image
        // cv::imshow("Camera Image", camera_image);
        // cv::waitKey(1);  // Update the display window
    }

    void odometry_path_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the ROS image message to an OpenCV image
        cv::Mat odometry_path_image;
        try {
            odometry_path_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Display the odometry path image
        cv::imshow("Odometry Path Image", odometry_path_image);
        cv::waitKey(1);  // Update the display window

        // Log a message
        // RCLCPP_INFO(this->get_logger(), "Received an odometry path image.");
    }
    // Subscribers for the two image topics
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_img_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr path_img_subscription_;

    int callback_count_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create and run the StateMonitor node
    auto node = std::make_shared<StateMonitor>();
    rclcpp::spin(node);

    // Cleanup
    rclcpp::shutdown();
    // cv::destroyAllWindows();
    return 0;
}
