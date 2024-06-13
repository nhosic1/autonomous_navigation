#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <cmath>
#include "autonomous_navigation/stereo_processing.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_time_policy;
typedef message_filters::Synchronizer<approximate_time_policy> approximate_time_synchronizer;

class StereoDepthEstimator : public rclcpp::Node
{
public:
    StereoDepthEstimator() : Node("stereo_depth_estimator")
    {
        // Create a timer to check FPS
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]()
                                         {
            RCLCPP_INFO(this->get_logger(), "FPS = %d", callback_count_);

            // Reset the callback count
            callback_count_ = 0; });

        // Initialize the callback count
        callback_count_ = 0;

        this->declare_parameter("snapshot", false);
        this->declare_parameter("data_folder", "");

        std::string package_name = "autonomous_navigation";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string stereo_camera_params_path = package_share_directory + "/config/stereo_camera_params.yaml";

        // Load params for stereo camera
        sp::load_stereo_camera_parameters(stereo_camera_params_path, camera_matrix_L_, dist_coeffs_L_, map_1_L_, map_2_L_, camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_, T_);

        // Create subscribers for left and right stereo image topics
        left_subscriber_.subscribe(this, "/left_camera/image");
        right_subscriber_.subscribe(this, "/right_camera/image");

        // Synchronize messages from both topics
        time_sync_ = std::make_shared<approximate_time_synchronizer>(approximate_time_policy(10), left_subscriber_, right_subscriber_);
        time_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(0, 30000000)); // 0.03 sec
        time_sync_->registerCallback(std::bind(&StereoDepthEstimator::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
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

        // Undistort and rectify images
        cv::remap(left_img, left_img, map_1_L_, map_2_L_, cv::INTER_LINEAR);
        cv::remap(right_img, right_img, map_1_R_, map_2_R_, cv::INTER_LINEAR);

        cv::Mat disparity_map = sp::compute_disparity_map_with_consistency_check(left_img, right_img, false);

        // Compute the arithmetic average of camera matrices
        cv::Mat camera_matrix = (camera_matrix_L_ + camera_matrix_R_) / 2.0;
        std::vector<cv::Point3f> points_3D = sp::compute_3D_points(disparity_map, camera_matrix, std::abs(T_.at<double>(0)));
        cv::Point3f closest_point(0.0f, 0.0f, std::numeric_limits<float>::max());
        find_closest_point(points_3D, closest_point);

        std::vector<cv::Point2f> points2D;
        std::vector<cv::Point3f> points3D;
        points3D.push_back(closest_point);
        cv::projectPoints(points3D, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_matrix_L_, dist_coeffs_L_, points2D);

        callback_count_++;
        sp::visualize_live_disparity_map(disparity_map, cv::Point2i(static_cast<int>(points2D[0].x), static_cast<int>(points2D[0].y)), closest_point);

        bool snapshot = this->get_parameter("snapshot").as_bool();
        if (snapshot == true)
        {
            save_snapshots(left_img, right_img, left_img_msg_ptr->header.stamp);
        }
    }

    void save_snapshots(const cv::Mat &left_img, const cv::Mat &right_img, const builtin_interfaces::msg::Time &timestamp)
    {
        std::string data_folder_path = this->get_parameter("data_folder").as_string();
        if (data_folder_path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Saving snapshots failed. Parameter 'data_folder' is not provided.");
        }
        else if (std::filesystem::path(data_folder_path).is_absolute() && std::filesystem::is_directory(data_folder_path))
        {
            std::string timestamp_str = std::to_string(timestamp.sec) + "_" + std::to_string(timestamp.nanosec);
            std::string left_img_path = data_folder_path + "/left_img_" + timestamp_str + ".png";
            std::string right_img_path = data_folder_path + "/right_img_" + timestamp_str + ".png";

            // Save images in PNG format
            cv::imwrite(left_img_path, left_img);
            cv::imwrite(right_img_path, right_img);

            RCLCPP_INFO(this->get_logger(), "Stereo images %s and %s saved in %s", std::filesystem::path(left_img_path).filename().c_str(), std::filesystem::path(right_img_path).filename().c_str(), data_folder_path.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Saving snapshots failed. Path to data folder is invalid.");
        }
        this->set_parameter(rclcpp::Parameter("snapshot", false));
    }

    bool is_path_safe(const std::vector<cv::Point3f> &points, const float &max_depth = 2000)
    {
        // Define parameters (unit: [mm])
        float robot_width = 1300.0;
        float camera_height = 575.0;
        float x_offset_compensation = 90.0; // Compensate for the left camera's x-axis offset from the robot's center, which is half of the stereo baseline
        float side_safety_margin = 250.0;
        float top_safety_margin = 400.0;
        float ground_tolerance = 50;

        for (const auto &point : points)
        {
            // Check if the point is in the robot's way
            if (point.z <= max_depth &&
                std::abs(point.x + x_offset_compensation) <= (robot_width / 2) + side_safety_margin &&
                point.y > -top_safety_margin &&
                point.y + ground_tolerance < camera_height)
            {
                return false;
            }
        }
        return true;
    }

    void find_closest_point(const std::vector<cv::Point3f> &points, cv::Point3f &closest_point)
    {
        // Define parameters (unit: [mm])
        float camera_height = 575.0;
        float ground_tolerance = 50;

        // Initialize variables to track the closest point
        float closest_z = std::numeric_limits<float>::max(); // Initialize with a large value

        for (const auto &point : points)
        {
            if (point.z < closest_z && point.y + ground_tolerance < camera_height)
            {
                closest_z = point.z;
                closest_point = point;
            }
        }
    }

    // Subscription objects for left and right stereo images
    message_filters::Subscriber<sensor_msgs::msg::Image> left_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_subscriber_;

    // Pointer for the Synchronizer
    std::shared_ptr<approximate_time_synchronizer> time_sync_;

    int callback_count_;
    rclcpp::Time last_reset_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Stereo camera params
    cv::Mat camera_matrix_L_, dist_coeffs_L_, map_1_L_, map_2_L_;
    cv::Mat camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_;
    cv::Mat T_;
};

int main(int argc, char **argv)
{
    // Create a named window for visualization
    cv::namedWindow("Disparity Map", cv::WINDOW_AUTOSIZE);

    // Window is black by default
    cv::imshow("Disparity Map", cv::Mat(432, 768, CV_8UC3, cv::Scalar(0, 0, 0)));
    cv::waitKey(1000);

    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoDepthEstimator>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2 node
    rclcpp::shutdown();

    // Destroy the window when the node exits
    cv::destroyWindow("Disparity Map");

    return 0;
}
