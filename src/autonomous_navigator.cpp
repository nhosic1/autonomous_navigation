#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
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
        std::string stereo_camera_params_path = package_share_directory + "/config/stereo_camera_params.yaml";

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Load params for stereo camera
        sp::load_stereo_camera_parameters(stereo_camera_params_path, camera_matrix_L_, dist_coeffs_L_, map_1_L_, map_2_L_, P_L_, camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_, P_R_, T_, Q_);

        camera_matrix_L_rect_ = P_L_(cv::Rect(0, 0, 3, 3)).clone();

        // Create odometry publisher
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/autonomous_vehicle/odometry", 10);

        // Create subscribers for left and right stereo image topics
        left_subscriber_.subscribe(this, "/autonomous_vehicle/left_camera/image", "raw");
        right_subscriber_.subscribe(this, "/autonomous_vehicle/right_camera/image", "raw");

        // Synchronize messages from both topics
        time_sync_ = std::make_shared<approximate_time_synchronizer>(approximate_time_policy(10), left_subscriber_, right_subscriber_);
        time_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(0, 35000000)); // 0.035 sec
        time_sync_->registerCallback(std::bind(&AutonomousNavigator::image_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Callback function for synchronized left and right stereo images
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img_msg_ptr, const sensor_msgs::msg::Image::ConstSharedPtr &right_img_msg_ptr)
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
            RCLCPP_ERROR(this->get_logger(), "Odometry chain is broken (not enough detected features).");
            start_vo_ = false;

            save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
            save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");
            
            rclcpp::shutdown();
            return;
        }
        else
        {
            bool success = sp::compute_3D_points_from_features(matcher_, P_L_, keypoints_L, descriptors_L, P_R_, keypoints_R, descriptors_R, points_3D_stereo, points_2D_stereo, average_depth);

            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Odometry chain is broken (failed to compute 3D points).");
                start_vo_ = false;

                std::cout << "descriptors L:" << descriptors_L.size() << std::endl;
                std::cout << "descriptors R:" << descriptors_R.size() << std::endl;

                std::cout << "Guess:" << std::endl;
                std::cout << "rvec = " << rvec_ << std::endl;
                std::cout << "tvec = " << tvec_ << std::endl;

                save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");

                rclcpp::shutdown();
                return;
            }
        }

        if (start_vo_)
        {            
            bool success = false;
            cv::Mat rvec_guess = rvec_.clone();
            cv::Mat tvec_guess = tvec_.clone();
            success = loc::compute_local_pose(camera_matrix_L_rect_, cv::Mat(), matcher_, keypoints_L_prev_, descriptors_L_prev_, keypoints_L, descriptors_L, points_2D_stereo_prev_, points_3D_stereo_prev_, rvec_guess, tvec_guess);

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
                    RCLCPP_ERROR(this->get_logger(), "Computed local pose is invalid. Translation magnitude exceeds the expected value.");

                    save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                    save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");

                    std::cout << "Guess:" << std::endl;
                    std::cout << "rvec = " << rvec_ << std::endl;
                    std::cout << "tvec = " << tvec_ << std::endl;

                    std::cout << "Result:" << std::endl;
                    std::cout << "rvec = " << rvec_guess << std::endl;
                    std::cout << "tvec = " << tvec_guess << std::endl;

                    rclcpp::shutdown();
                    return;
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

                    // Invert rotation and translation to represent camera's motion in world frame (3D points are static)
                    cv::Mat R_inv = R.t();
                    cv::Mat tvec_inv = -R_inv * tvec_;

                    // Create current transformation matrix
                    cv::Mat local_pose = cv::Mat::eye(4, 4, CV_64F);
                    
                    R_inv.copyTo(local_pose(cv::Rect(0, 0, 3, 3)));
                    tvec_inv.copyTo(local_pose(cv::Rect(3, 0, 1, 3)));

                    // Update the global pose
                    global_pose_ = global_pose_ * local_pose;

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
                RCLCPP_ERROR(this->get_logger(), "Odometry chain is broken (failed to compute local pose).");

                save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");

                std::cout << "Guess:" << std::endl;
                std::cout << "rvec = " << rvec_ << std::endl;
                std::cout << "tvec = " << tvec_ << std::endl;

                rclcpp::shutdown();
                return;
            }

            // Create odometry msg from global pose
            double x = global_pose_.at<double>(2, 3) / 1000.0; // Z coordinate (camera coordinate system), unit: [m]
            double y = - global_pose_.at<double>(0, 3) / 1000.0; // X coordinate (camera coordinate system), unit: [m]
            double z = global_pose_.at<double>(1, 3) / 1000.0; // Y coordinate (camera coordinate system), unit: [m]

            tf2::Matrix3x3 R_cam_cs(
                global_pose_.at<double>(0, 0), global_pose_.at<double>(0, 1), global_pose_.at<double>(0, 2),
                global_pose_.at<double>(1, 0), global_pose_.at<double>(1, 1), global_pose_.at<double>(1, 2),
                global_pose_.at<double>(2, 0), global_pose_.at<double>(2, 1), global_pose_.at<double>(2, 2)
            );

            double roll, pitch, yaw;
            R_cam_cs.getRPY(roll, pitch, yaw);

            tf2::Quaternion Q_world_cs;
            Q_world_cs.setRPY(yaw, -roll, -pitch);

            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();
            header.frame_id = "autonomous_vehicle/odom";

            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header = header;
            odom_msg.child_frame_id = "autonomous_vehicle/base_link";
            odom_msg.pose.pose.position.x = x;
            odom_msg.pose.pose.position.y = y;
            odom_msg.pose.pose.position.z = z;
            odom_msg.pose.pose.orientation = tf2::toMsg(Q_world_cs);

            // Publish odometry msg
            odom_publisher_->publish(odom_msg);

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "autonomous_vehicle/odom";
            t.child_frame_id = "base_link";

            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = z;

            t.transform.rotation.x = Q_world_cs.x();
            t.transform.rotation.y = Q_world_cs.y();
            t.transform.rotation.z = Q_world_cs.z();
            t.transform.rotation.w = Q_world_cs.w();

            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
        else
        {
            start_vo_ = true;
            global_pose_ = cv::Mat::eye(4, 4, CV_64F);

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

    // Odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    // Subscribers for left and right stereo images)
    image_transport::SubscriberFilter left_subscriber_;
    image_transport::SubscriberFilter right_subscriber_;

    // Pointer for the Synchronizer
    std::shared_ptr<approximate_time_synchronizer> time_sync_;

    int callback_count_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;

    // Stereo camera params
    cv::Mat camera_matrix_L_, camera_matrix_L_rect_, dist_coeffs_L_, map_1_L_, map_2_L_, P_L_;
    cv::Mat camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_, P_R_;
    cv::Mat T_;
    cv::Mat Q_;

    // ORB detector
    cv::Ptr<cv::ORB> orb_ = cv::ORB::create(
        1400,                  // nfeatures
        1.2f,                  // scaleFactor
        8,                     // nlevels
        25,                    // edgeThreshold
        0,                     // firstLevel
        2,                     // WTA_K
        cv::ORB::HARRIS_SCORE, // scoreType
        31,                    // patchSize
        12                     // fastThreshold
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

    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Keyframe timestamp
    std::chrono::time_point<std::chrono::steady_clock> timestamp_prev_;

    bool start_vo_ = false;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousNavigator>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2 node
    rclcpp::shutdown();

    return 0;
}
