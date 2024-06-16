#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "autonomous_navigation/stereo_processing.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_time_policy;
typedef message_filters::Synchronizer<approximate_time_policy> approximate_time_synchronizer;

class StereoViewer : public rclcpp::Node
{
public:
    StereoViewer() : Node("stereo_viewer")
    {
        // Create a timer to check FPS
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]()
                                         {
            RCLCPP_INFO(this->get_logger(), "fps = %d", callback_count_);

            // Reset the callback count
            callback_count_ = 0;

            snapshot_count_++; });

        // Initialize the callback count
        callback_count_ = 0;

        snapshot_count_ = 0;

        this->declare_parameter("snapshot", false);
        this->declare_parameter("output_folder_L", "");
        this->declare_parameter("output_folder_R", "");
        this->declare_parameter("rectify", false);

        std::string package_name = "autonomous_navigation";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string stereo_camera_params_path = package_share_directory + "/config/stereo_camera_params.yaml";

        rectify_ = this->get_parameter("rectify").as_bool();
        if (rectify_)
        {
            // Load params for stereo camera
            cv::Mat T;
            sp::load_stereo_camera_parameters(stereo_camera_params_path, camera_matrix_L_, dist_coeffs_L_, map_1_L_, map_2_L_, camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_, T); 
        }
        
        // Create subscribers for left and right stereo image topics
        left_subscriber_.subscribe(this, "/left_camera/image");
        right_subscriber_.subscribe(this, "/right_camera/image");

        // Synchronize messages from both topics
        time_sync_ = std::make_shared<approximate_time_synchronizer>(approximate_time_policy(10), left_subscriber_, right_subscriber_);
        time_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(0, 30000000)); // 0.03 sec
        time_sync_->registerCallback(std::bind(&StereoViewer::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Callback function for synchronized left and right stereo images
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img_msg_ptr, const sensor_msgs::msg::Image::ConstSharedPtr &right_img_msg_ptr)
    {
        cv_bridge::CvImagePtr cv_left_img_ptr;
        cv_bridge::CvImagePtr cv_right_img_ptr;

        // Convert ROS2 image messages to cv::Mat objects
        try
        {
            cv_left_img_ptr = cv_bridge::toCvCopy(left_img_msg_ptr, sensor_msgs::image_encodings::RGB8);
            cv_right_img_ptr = cv_bridge::toCvCopy(right_img_msg_ptr, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat left_img = cv_left_img_ptr->image;
        cv::Mat right_img = cv_right_img_ptr->image;

        if (rectify_)
        {
            // Undistort and rectify images
            cv::remap(left_img, left_img, map_1_L_, map_2_L_, cv::INTER_LINEAR);
            cv::remap(right_img, right_img, map_1_R_, map_2_R_, cv::INTER_LINEAR);
        }

        cv::Mat left_right_img;
        cv::hconcat(left_img, right_img, left_right_img);

        cv::imshow("Stereo Images", left_right_img);
        cv::waitKey(1);
        callback_count_++;

        if (snapshot_count_ == 2)
        {
            this->set_parameter(rclcpp::Parameter("snapshot", true));
            snapshot_count_ = 0;
        }

        bool snapshot = this->get_parameter("snapshot").as_bool();
        if (snapshot == true)
        {
            save_snapshots(left_img, right_img, left_img_msg_ptr->header.stamp);
        }
    }

    void save_snapshots(const cv::Mat &left_img, const cv::Mat &right_img, const builtin_interfaces::msg::Time &timestamp)
    {
        std::string output_folder_L = this->get_parameter("output_folder_L").as_string();
        std::string output_folder_R = this->get_parameter("output_folder_R").as_string();
        bool parameter_error = false;

        if (!std::filesystem::path(output_folder_L).is_absolute() || !std::filesystem::is_directory(output_folder_L))
        {
            RCLCPP_WARN(this->get_logger(), "Parameter 'output_folder_L' is invalid or not provided. It must be an absolute path to an existing directory.");
            parameter_error = true;
        }

        if (!std::filesystem::path(output_folder_R).is_absolute() || !std::filesystem::is_directory(output_folder_R))
        {
            RCLCPP_WARN(this->get_logger(), "Parameter 'output_folder_R' is invalid or not provided. It must be an absolute path to an existing directory.");
            parameter_error = true;
        }

        if (parameter_error)
        {
            RCLCPP_WARN(this->get_logger(), "Saving snapshots failed.");
        }
        else
        {
            // Save frames
            std::string timestamp_str = std::to_string(timestamp.sec) + "_" + std::to_string(timestamp.nanosec);
            std::string left_img_path = output_folder_L + "/img_" + timestamp_str + ".jpg";
            std::string right_img_path = output_folder_R + "/img_" + timestamp_str + ".jpg";

            cv::imwrite(left_img_path, left_img);
            cv::imwrite(right_img_path, right_img);

            RCLCPP_INFO(this->get_logger(), "Stereo images saved successfully");
        }

        this->set_parameter(rclcpp::Parameter("snapshot", false));
    }

    // Subscription objects for left and right stereo images
    message_filters::Subscriber<sensor_msgs::msg::Image> left_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_subscriber_;

    // Pointer for the Synchronizer
    std::shared_ptr<approximate_time_synchronizer> time_sync_;

    bool rectify_;
    int callback_count_;
    int snapshot_count_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Stereo camera params
    cv::Mat camera_matrix_L_, dist_coeffs_L_, map_1_L_, map_2_L_;
    cv::Mat camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_;
};

int main(int argc, char **argv)
{
    // Create a named window for visualization
    cv::namedWindow("Stereo Images", cv::WINDOW_AUTOSIZE);

    // Window is black by default
    cv::imshow("Stereo Images", cv::Mat(432, 768, CV_8UC3, cv::Scalar(0, 0, 0)));
    cv::waitKey(1000);

    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoViewer>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2 node
    rclcpp::shutdown();

    // Destroy the window when the node exits
    cv::destroyWindow("Stereo Images");

    return 0;
}
