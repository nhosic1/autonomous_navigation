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
#include <chrono>
#include "autonomous_navigation/stereo_processing.hpp"
#include "autonomous_navigation/localization.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_time_policy;
typedef message_filters::Synchronizer<approximate_time_policy> approximate_time_synchronizer;

class AutonomousNavigator : public rclcpp::Node
{
public:
    AutonomousNavigator() : Node("autonomous_navigator")
    {
        // Create a timer to check FPS
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]()
                                         {
            RCLCPP_INFO(this->get_logger(), "FPS = %d", callback_count_);

            // Reset the callback count
            callback_count_ = 0; });

        this->declare_parameter("snapshot", false);
        this->declare_parameter("data_folder", "");

        std::string package_name = "autonomous_navigation";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string stereo_camera_params_path = package_share_directory + "/config/sim_stereo_camera_params.yaml";

        // Load params for stereo camera
        sp::load_stereo_camera_parameters(stereo_camera_params_path, camera_matrix_L_, dist_coeffs_L_, map_1_L_, map_2_L_, P_L_, camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_, P_R_, T_, Q_);

        // Create subscribers for left and right stereo image topics
        left_subscriber_.subscribe(this, "/left_camera/image");
        right_subscriber_.subscribe(this, "/right_camera/image");

        // Create publisher for images of estimated path
        std::string topic_name = "~/path_image";
        AutonomousNavigator::path_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

        // Synchronize messages from both topics
        time_sync_ = std::make_shared<approximate_time_synchronizer>(approximate_time_policy(10), left_subscriber_, right_subscriber_);
        time_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(0, 30000000)); // 0.03 sec
        time_sync_->registerCallback(std::bind(&AutonomousNavigator::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Callback function for synchronized left and right stereo images
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img_msg_ptr, const sensor_msgs::msg::Image::ConstSharedPtr &right_img_msg_ptr)
    {
        auto timestamp = std::chrono::steady_clock::now();

        cv_bridge::CvImagePtr cv_left_img_ptr;
        cv_bridge::CvImagePtr cv_right_img_ptr;

        // Convert ROS2 image messages to cv::Mat objects
        try
        {
            cv_left_img_ptr = cv_bridge::toCvCopy(left_img_msg_ptr);
            cv_right_img_ptr = cv_bridge::toCvCopy(right_img_msg_ptr);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat left_img = cv_left_img_ptr->image;
        cv::Mat right_img = cv_right_img_ptr->image;

        // Undistort and rectify images
        if (!map_1_L_.empty() && !map_2_L_.empty())
        {
            cv::remap(left_img, left_img, map_1_L_, map_2_L_, cv::INTER_LINEAR);
        }
        if (!map_1_R_.empty() && !map_2_R_.empty())
        {
            cv::remap(right_img, right_img, map_1_R_, map_2_R_, cv::INTER_LINEAR);
        }

        std::vector<cv::KeyPoint> keypoints_L, keypoints_R;
        cv::Mat descriptors_L, descriptors_R;

        // Detect keypoints and compute their descriptors
        orb_->detectAndCompute(left_img, cv::Mat(), keypoints_L, descriptors_L);
        orb_->detectAndCompute(right_img, cv::Mat(), keypoints_R, descriptors_R);

        // Compute 3D points
        std::vector<cv::Point3d> points_3D_stereo;
        std::vector<cv::Point2d> points_2D_stereo;
        double average_depth;

        if (descriptors_L.empty() || descriptors_R.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Odometry chain is broken (not enough detected features). Reinitializing...");
            start_vo_ = false;
            path_points_.clear();

            save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
            save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");
        }
        else
        {
            bool success = sp::compute_3D_points_from_features(matcher_, P_L_, keypoints_L, descriptors_L, P_R_, keypoints_R, descriptors_R, points_3D_stereo, points_2D_stereo, average_depth);

            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Odometry chain is broken (failed to compute 3D points). Reinitializing...");
                start_vo_ = false;
                path_points_.clear();

                std::cout << "descriptors L:" << descriptors_L.size() << std::endl;
                std::cout << "descriptors R:" << descriptors_R.size() << std::endl;

                std::cout << "Guess:" << std::endl;
                std::cout << "rvec = " << rvec_ << std::endl;
                std::cout << "tvec = " << tvec_ << std::endl;

                save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");
            }
        }

        if (start_vo_)
        {            
            bool success = false;
            cv::Mat rvec_guess = rvec_.clone();
            cv::Mat tvec_guess = tvec_.clone();
            success = loc::compute_local_pose(camera_matrix_L_, dist_coeffs_L_, matcher_, keypoints_L_prev_, descriptors_L_prev_, keypoints_L, descriptors_L, points_2D_stereo_prev_, points_3D_stereo_prev_, rvec_guess, tvec_guess);

            if (success)
            {
                double velocity = 0.5; // unit: [m/s]

                // Calculate time duration from previous frame
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - timestamp_prev_);
                double time_diff = duration.count(); // unit: [ms]

                // Calculate max expected distance
                double max_expected_distance = velocity * time_diff; // unit: [mm]
                double tolerance = 200; // unit: [mm]

                if (std::abs(cv::norm(tvec_) - cv::norm(tvec_guess)) > max_expected_distance + tolerance)
                {
                    RCLCPP_WARN(this->get_logger(), "Computed local pose is invalid. Translation magnitude exceeds the expected value.");

                    save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                    save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");

                    std::cout << "Guess:" << std::endl;
                    std::cout << "rvec = " << rvec_ << std::endl;
                    std::cout << "tvec = " << tvec_ << std::endl;

                    std::cout << "Result:" << std::endl;
                    std::cout << "rvec = " << rvec_guess << std::endl;
                    std::cout << "tvec = " << tvec_guess << std::endl;
                }
                
                rvec_ = rvec_guess;
                tvec_ = tvec_guess;
                timestamp_prev_ = timestamp;
                double keyframe_distance = cv::norm(tvec_); // unit: [mm]
                double total_rotation = cv::norm(rvec_); // unit: [rad]

                // Keyframe slection
                if ((((keyframe_distance / average_depth) > 0.07 || total_rotation > 5 * CV_PI / 180)) && keyframe_distance > 50)
                {
                    // Convert rvec to a rotation matrix
                    cv::Mat R;
                    cv::Rodrigues(rvec_, R);

                    // Invert rotation and translation to represent camera's pose relative to 3D points
                    cv::Mat R_inv = R.t();
                    cv::Mat tvec_inv = -R_inv * tvec_;

                    // Create current transformation matrix
                    cv::Mat local_pose = cv::Mat::eye(4, 4, CV_64F);
                    
                    R_inv.copyTo(local_pose(cv::Rect(0, 0, 3, 3)));
                    tvec_inv.copyTo(local_pose(cv::Rect(3, 0, 1, 3)));

                    // Update the global pose
                    global_pose_ = global_pose_ * local_pose;

                    double x = global_pose_.at<double>(0, 3); // X coordinate (camera coordinate system)
                    double z = global_pose_.at<double>(2, 3); // Z coordinate (camera coordinate system)

                    cv::Point2d path_point(x, z);
                    path_points_.push_back(path_point);

                    loc::draw_path(path_points_, path_image_);

                    // Update class members for next iteration
                    keypoints_L_prev_ = keypoints_L;
                    descriptors_L_prev_ = descriptors_L;
                    points_2D_stereo_prev_ = points_2D_stereo;
                    points_3D_stereo_prev_ = points_3D_stereo;
                    keyframe_L_prev_ = left_img;
                    keyframe_R_prev_ = right_img;
                    rvec_ = cv::Mat::zeros(3, 1, CV_64F); // No rotation
                    tvec_ = cv::Mat::zeros(3, 1, CV_64F); // No translation
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Odometry chain is broken (failed to compute local pose). Reinitializing global pose.");
                start_vo_ = false;
                path_points_.clear();

                save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");
            }
        }
        else
        {
            start_vo_ = true;
            global_pose_ = cv::Mat::eye(4, 4, CV_64F);
        
            // Add initial position
            cv::Point2d path_point(0.0, 0.0);
            path_points_.push_back(path_point);

            loc::draw_path(path_points_, path_image_);

            // Update class members for next iteration
            keypoints_L_prev_ = keypoints_L;
            descriptors_L_prev_ = descriptors_L;
            points_2D_stereo_prev_ = points_2D_stereo;
            points_3D_stereo_prev_ = points_3D_stereo;
            keyframe_L_prev_ = left_img;
            keyframe_R_prev_ = right_img;
            timestamp_prev_ = timestamp;
            rvec_ = cv::Mat::zeros(3, 1, CV_64F); // No rotation
            tvec_ = cv::Mat::zeros(3, 1, CV_64F); // No translation
        }

        // // Display the updated trajectory
        // cv::imshow("Estimated Path", path_image_);
        // cv::waitKey(1);

        // Publish image with estimated path
        sensor_msgs::msg::Image::SharedPtr msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", path_image_).toImageMsg();
        path_img_publisher_->publish(*msg_img);

        callback_count_++;

        bool snapshot = this->get_parameter("snapshot").as_bool();
        if (snapshot == true)
        {
            save_snapshots(left_img, right_img, left_img_msg_ptr->header.stamp, "stereo");
        }
    }

    void save_snapshots(const cv::Mat &img_1, const cv::Mat &img_2, const builtin_interfaces::msg::Time &timestamp, std::string prefix = "")
    {
        std::string data_folder_path = this->get_parameter("data_folder").as_string();
        if (data_folder_path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Saving snapshots failed. Parameter 'data_folder' is not provided.");
        }
        else if (std::filesystem::path(data_folder_path).is_absolute() && std::filesystem::is_directory(data_folder_path))
        {
            std::string timestamp_str = std::to_string(timestamp.sec) + "_" + std::to_string(timestamp.nanosec);
            std::string img_1_path = data_folder_path + "/" + prefix + "img_1_" + timestamp_str + ".png";
            std::string img_2_path = data_folder_path + "/" + prefix + "img_2_" + timestamp_str + ".png";

            // Save images in PNG format
            cv::imwrite(img_1_path, img_1);
            cv::imwrite(img_2_path, img_2);

            RCLCPP_INFO(this->get_logger(), "Images %s and %s saved in %s", std::filesystem::path(img_1_path).filename().c_str(), std::filesystem::path(img_2_path).filename().c_str(), data_folder_path.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Saving snapshots failed. Path to data folder is invalid.");
        }
        this->set_parameter(rclcpp::Parameter("snapshot", false));
    }

    bool is_path_safe(const std::vector<cv::Point3d> &points, const double &max_depth = 2000)
    {
        // Define parameters (unit: [mm])
        double robot_width = 1300.0;
        double camera_height = 575.0;
        double x_offset_compensation = 90.0; // Compensate for the left camera's x-axis offset from the robot's center, which is half of the stereo baseline
        double side_safety_margin = 250.0;
        double top_safety_margin = 400.0;
        double ground_tolerance = 50.0;

        for (const auto &point : points)
        {
            // Check if the point is in robot's way
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

    void find_closest_point(const std::vector<cv::Point3d> &points, cv::Point3d &closest_point)
    {
        // Define parameters (unit: [mm])
        double camera_height = 575.0;
        double ground_tolerance = 50.0;

        // Initialize variables to track the closest point
        double closest_z = std::numeric_limits<double>::max(); // Initialize with a large value

        for (const auto &point : points)
        {
            if (point.z < closest_z && point.y + ground_tolerance < camera_height)
            {
                closest_z = point.z;
                closest_point = point;
            }
        }
    }

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr path_img_publisher_;

    // Subscription objects for left and right stereo images
    message_filters::Subscriber<sensor_msgs::msg::Image> left_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_subscriber_;

    // Pointer for the Synchronizer
    std::shared_ptr<approximate_time_synchronizer> time_sync_;

    int callback_count_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;

    // Stereo camera params
    cv::Mat camera_matrix_L_, dist_coeffs_L_, map_1_L_, map_2_L_, P_L_;
    cv::Mat camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_, P_R_;
    cv::Mat T_;
    cv::Mat Q_;

    // ORB detector
    cv::Ptr<cv::ORB> orb_ = cv::ORB::create(
        3000,                  // nfeatures
        1.2f,                  // scaleFactor
        8,                     // nlevels
        25,                    // edgeThreshold
        0,                     // firstLevel
        2,                     // WTA_K
        cv::ORB::HARRIS_SCORE, // scoreType
        31,                    // patchSize
        10                     // fastThreshold
    );

    // Descriptor matcher
    cv::Ptr<cv::BFMatcher> matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);

    // Data from previous iteration
    std::vector<cv::KeyPoint> keypoints_L_prev_;
    cv::Mat descriptors_L_prev_;
    std::vector<cv::Point2d> points_2D_stereo_prev_;
    std::vector<cv::Point3d> points_3D_stereo_prev_;
    cv::Mat keyframe_L_prev_, keyframe_R_prev_;

    // Global pose
    cv::Mat global_pose_ = cv::Mat::eye(4, 4, CV_64F);

    // Initial guess for rotation (rvec) and translation (tvec)
    cv::Mat rvec_ = cv::Mat::zeros(3, 1, CV_64F); // No rotation
    cv::Mat tvec_ = cv::Mat::zeros(3, 1, CV_64F); // No translation

    // Path image
    cv::Mat path_image_;
    std::vector<cv::Point2d> path_points_;

    // Keyframe timestamp
    std::chrono::time_point<std::chrono::steady_clock> timestamp_prev_;

    bool start_vo_ = false;
};

int main(int argc, char **argv)
{
    // // Create a named window for visualization
    // cv::namedWindow("Estimated Path", cv::WINDOW_AUTOSIZE);

    // // Window is white by default
    // cv::imshow("Estimated Path", cv::Mat(600, 600, CV_8UC3, cv::Scalar(255, 255, 255)));
    // cv::waitKey(10);

    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousNavigator>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2 node
    rclcpp::shutdown();

    // // Destroy the window when the node exits
    // cv::destroyWindow("Estimated Path");

    return 0;
}
