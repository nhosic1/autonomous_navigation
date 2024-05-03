#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <cmath>

class StereoDepthEstimator : public rclcpp::Node
{
public:
    StereoDepthEstimator() : Node("stereo_depth_estimator")
    {
        this->declare_parameter("snapshot", false);
        this->declare_parameter("data_folder", "");

        // Create subscribers for left and right stereo image topics
        left_subscriber_.subscribe(this, "/left_camera/image");
        right_subscriber_.subscribe(this, "/right_camera/image");

        // Create a TimeSynchronizer to synchronize messages from both topics
        time_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(left_subscriber_, right_subscriber_, 10);
        time_sync_->registerCallback(std::bind(&StereoDepthEstimator::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Callback function for synchronized left and right stereo images
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img_msg_ptr, const sensor_msgs::msg::Image::ConstSharedPtr &right_img_msg_ptr)
    {
        bool snapshot = this->get_parameter("snapshot").as_bool();
        cv_bridge::CvImagePtr cv_left_img_ptr;
        cv_bridge::CvImagePtr cv_right_img_ptr;

        // Convert ROS2 image messages to cv::Mat objects
        try
        {
            cv_left_img_ptr = cv_bridge::toCvCopy(left_img_msg_ptr, sensor_msgs::image_encodings::BGR8);
            cv_right_img_ptr = cv_bridge::toCvCopy(right_img_msg_ptr, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat left_img = cv_left_img_ptr->image;
        cv::Mat right_img = cv_right_img_ptr->image;

        cv::Mat disparity_map = compute_disparity_map(left_img, right_img);
        visualize_disparity_map(disparity_map);

        cv::Mat depth_map = compute_depth_map(disparity_map);

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

    cv::Mat compute_disparity_map(const cv::Mat &left_img, const cv::Mat &right_img)
    {
        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_img, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_img, right_gray, cv::COLOR_BGR2GRAY);

        cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
        stereo->setNumDisparities(176);
        stereo->setBlockSize(15);
        stereo->setMinDisparity(3);
        stereo->setUniquenessRatio(15);
        stereo->setTextureThreshold(25);
        stereo->setSpeckleRange(3);
        stereo->setSpeckleWindowSize(600);
        cv::Mat disparity_map;
        stereo->compute(left_gray, right_gray, disparity_map);

        return disparity_map; // Data type of disparity_map is CV_16SC1
    }

    // Expects disparity_map data type to be CV_16SC1
    void visualize_disparity_map(const cv::Mat &disparity_map, const int &max_disparity = 176)
    {
        // Scale the disparity map and convert it to CV_8UC1
        cv::Mat scaled_disparity_map;
        cv::convertScaleAbs(disparity_map, scaled_disparity_map, 255.0 / (max_disparity * 16.0));

        // Apply a color map to the scaled disparity map (red for closer objects, blue for farther objects)
        cv::Mat colored_disparity_map;
        cv::applyColorMap(scaled_disparity_map, colored_disparity_map, cv::COLORMAP_JET);

        cv::Mat resized_colored_disparity_map;
        int max_image_width = 600;
        float scale = static_cast<float>(max_image_width) / colored_disparity_map.cols;
        cv::resize(colored_disparity_map, resized_colored_disparity_map, cv::Size(), scale, scale);

        // Display the colored disparity map
        cv::imshow("Disparity Map", resized_colored_disparity_map);
        cv::waitKey(1); // Wait for a key press (1 millisecond)
    }

    cv::Mat compute_depth_map(const cv::Mat &disparity_map)
    {
        // Define camera parameters
        int image_width = 1280;
        double horizontal_fov = 1.1519;
        double focal_length_px = image_width / (2 * tan(horizontal_fov / 2)); // From camera matrix: 985.5322265625
        double baseline = 180.0;

        // Compute depth map from disparity map
        cv::Mat depth_map(disparity_map.size(), CV_32F);
        for (int y = 0; y < disparity_map.rows; y++)
        {
            for (int x = 0; x < disparity_map.cols; x++)
            {
                int16_t disparity_scaled = disparity_map.at<int16_t>(y, x);
                float disparity = disparity_scaled / 16.0f;
                if (disparity > 0)
                {
                    float depth = (focal_length_px * baseline) / (disparity);
                    depth_map.at<float>(y, x) = depth;
                }
                else
                {
                    depth_map.at<float>(y, x) = 0; // Invalid disparity value
                }
            }
        }

        return depth_map;
    }

    // Subscription objects for left and right stereo images
    message_filters::Subscriber<sensor_msgs::msg::Image> left_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_subscriber_;

    // Pointer for TimeSynchronizer object
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> time_sync_;
};

int main(int argc, char **argv)
{
    // Create a named window for visualization
    cv::namedWindow("Disparity Map", cv::WINDOW_AUTOSIZE);

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
