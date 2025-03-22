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
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include "autonomous_navigation/stereo_processing.hpp"
#include "autonomous_navigation/localization.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_time_policy;
typedef message_filters::Synchronizer<approximate_time_policy> approximate_time_synchronizer;

class VisualOdometryEstimator : public rclcpp::Node
{
public:
    VisualOdometryEstimator() : Node("vo_estimator")
    {
        // Create a timer to check FPS
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]()
                                         {
            RCLCPP_INFO(this->get_logger(), "FPS = %d", callback_count_);

            // Reset the callback count
            callback_count_ = 0; });

        this->declare_parameter("snapshot", false);
        this->declare_parameter("data_folder", "");
        this->declare_parameter("sim", false);

        std::string package_name = "autonomous_navigation";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);

        bool sim = this->get_parameter("sim").as_bool();
        std::string stereo_camera_params_path = package_share_directory + "/config/" + (sim ? "sim_stereo_camera_params.yaml" : "stereo_camera_params.yaml");

        // Load params for stereo camera
        sp::load_stereo_camera_parameters(stereo_camera_params_path, camera_matrix_L_, dist_coeffs_L_, map_1_L_, map_2_L_, P_L_, camera_matrix_R_, dist_coeffs_R_, map_1_R_, map_2_R_, P_R_, T_, Q_);

        camera_matrix_L_rect_ = P_L_(cv::Rect(0, 0, 3, 3)).clone();

        // Create odometry publisher
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/autonomous_vehicle/odometry/visual", 10);

        ekf_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
            {
                latest_ekf_odom_ = msg;
            });

        // Create subscribers for left and right stereo image topics
        left_subscriber_.subscribe(this, "/autonomous_vehicle/left_camera/image", "raw");
        right_subscriber_.subscribe(this, "/autonomous_vehicle/right_camera/image", "raw");

        // Synchronize messages from both topics
        time_sync_ = std::make_shared<approximate_time_synchronizer>(approximate_time_policy(10), left_subscriber_, right_subscriber_);
        time_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(0, 35000000)); // 0.035 sec
        time_sync_->registerCallback(std::bind(&VisualOdometryEstimator::image_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize transform broadcaster and listener
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize transformation matrices
        global_pose_.setIdentity();
        T_cam_to_base_ = get_transform("left_camera", "base_link");
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
            RCLCPP_WARN(this->get_logger(), "Visual odometry chain is broken (not enough detected features). Resorting to estimating odometry with IMU sensor data.");

            save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
            save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");

            start_vo_ = false;
        }
        else
        {
            bool success = sp::compute_3D_points_from_features(matcher_, P_L_, keypoints_L, descriptors_L, P_R_, keypoints_R, descriptors_R, points_3D_stereo, points_2D_stereo, average_depth);

            if (!success)
            {
                RCLCPP_WARN(this->get_logger(), "Visual odometry chain is broken (failed to compute 3D points). Resorting to estimating odometry with IMU sensor data.");

                save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");

                start_vo_ = false;
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
                double tolerance = 200;                              // unit: [mm]

                if (std::abs(cv::norm(tvec_) - cv::norm(tvec_guess)) > max_expected_distance + tolerance)
                {
                    RCLCPP_WARN(this->get_logger(), "Visual odometry chain is broken (translation magnitude exceeds the expected value). Resorting to estimating odometry with IMU sensor data.");

                    save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                    save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");

                    start_vo_ = false;
                }

                rvec_ = rvec_guess;
                tvec_ = tvec_guess;
                timestamp_prev_ = timestamp;
                double keyframe_distance = cv::norm(tvec_); // unit: [mm]
                double total_rotation = cv::norm(rvec_);    // unit: [rad]

                // Keyframe slection
                if ((((keyframe_distance / average_depth) > 0.07 || total_rotation > 5 * CV_PI / 180)) && keyframe_distance > 50)
                {
                    // Convert rvec to a rotation matrix
                    cv::Mat R_cv;
                    cv::Rodrigues(rvec_, R_cv);

                    // Create transformation matrix for local pose in camera coordinate system (camera assumed to be static)
                    tf2::Vector3 t_cam_cs(tvec_.at<double>(0, 0), tvec_.at<double>(1, 0), tvec_.at<double>(2, 0));
                    tf2::Matrix3x3 R_cam_cs(R_cv.at<double>(0, 0), R_cv.at<double>(0, 1), R_cv.at<double>(0, 2),
                                            R_cv.at<double>(1, 0), R_cv.at<double>(1, 1), R_cv.at<double>(1, 2),
                                            R_cv.at<double>(2, 0), R_cv.at<double>(2, 1), R_cv.at<double>(2, 2));
                    tf2::Quaternion q_cam_cs;
                    R_cam_cs.getRotation(q_cam_cs);

                    tf2::Transform local_pose_cam_cs;
                    local_pose_cam_cs.setOrigin(t_cam_cs);
                    local_pose_cam_cs.setRotation(q_cam_cs);

                    // Invert rotation and translation to represent camera's motion in odometry frame (3D points are static)
                    local_pose_cam_cs = local_pose_cam_cs.inverse();

                    // Update the global pose
                    tf2::Transform local_pose = convert_cam_to_rh_coordinate_sys(local_pose_cam_cs);
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
                RCLCPP_WARN(this->get_logger(), "Visual odometry chain is broken (failed to compute local pose). Resorting to estimating odometry with IMU sensor data.");

                save_snapshots(keyframe_L_prev_, left_img, left_img_msg_ptr->header.stamp);
                save_snapshots(keyframe_L_prev_, keyframe_R_prev_, left_img_msg_ptr->header.stamp, "stereo_prev_");

                start_vo_ = false;
            }

            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();
            header.frame_id = "odom";

            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header = header;
            odom_msg.child_frame_id = "left_camera";

            tf2::Vector3 t_global = global_pose_.getOrigin();
            odom_msg.pose.pose.position.x = t_global.x();
            odom_msg.pose.pose.position.y = t_global.y();
            odom_msg.pose.pose.position.z = t_global.z();

            tf2::Quaternion q_global = global_pose_.getRotation();
            odom_msg.pose.pose.orientation = tf2::toMsg(q_global);

            odom_msg.pose.covariance = {
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0, // X
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0, // Y
                0.0, 0.0, 0.2, 0.0, 0.0, 0.0,  // Z
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0, // Roll
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0, // Pitch
                0.0, 0.0, 0.0, 0.0, 0.0, 0.05  // Yaw
            };

            odom_msg.twist.covariance = {
                0.3, 0.0, 0.0, 0.0, 0.0, 0.0, // Vx
                0.0, 0.3, 0.0, 0.0, 0.0, 0.0, // Vy
                0.0, 0.0, 0.4, 0.0, 0.0, 0.0, // Vz
                0.0, 0.0, 0.0, 0.3, 0.0, 0.0, // Angular Vx
                0.0, 0.0, 0.0, 0.0, 0.3, 0.0, // Angular Vy
                0.0, 0.0, 0.0, 0.0, 0.0, 0.3  // Angular Vz
            };

            // Publish odometry msg
            odom_publisher_->publish(odom_msg);

            geometry_msgs::msg::TransformStamped T_msg;
            T_msg.header.stamp = this->get_clock()->now();
            T_msg.header.frame_id = "odom";
            T_msg.child_frame_id = "base_link";

            tf2::Transform global_base_link_pose = global_pose_ * T_cam_to_base_;
            T_msg.transform = tf2::toMsg(global_base_link_pose);

            // Send odom-base_link transformation
            // tf_broadcaster_->sendTransform(T_msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Starting visual odometry...");

            start_vo_ = true;

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

            if (latest_ekf_odom_)
            {
                tf2::fromMsg(latest_ekf_odom_->pose.pose, global_pose_);
            }
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

    tf2::Transform get_transform(const std::string &target_frame, const std::string &source_frame)
    {
        geometry_msgs::msg::TransformStamped T_msg;
        tf2::Transform T;
        T.setIdentity();

        try
        {
            T_msg = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), source_frame.c_str(), ex.what());
            return T;
        }

        // Convert translation
        tf2::Vector3 t(
            T_msg.transform.translation.x,
            T_msg.transform.translation.y,
            T_msg.transform.translation.z);

        // Convert rotation (quaternion)
        tf2::Quaternion q(
            T_msg.transform.rotation.x,
            T_msg.transform.rotation.y,
            T_msg.transform.rotation.z,
            T_msg.transform.rotation.w);

        // Set translation and rotation to the tf2::Transform object
        T.setOrigin(t);
        T.setRotation(q);

        return T;
    }

    tf2::Transform convert_cam_to_rh_coordinate_sys(const tf2::Transform &T_cam_cs)
    {
        tf2::Vector3 t_cam_cs = T_cam_cs.getOrigin();

        double x = t_cam_cs.z() / 1000.0;  // Z coordinate (camera coordinate system), unit: [m]
        double y = -t_cam_cs.x() / 1000.0; // X coordinate (camera coordinate system), unit: [m]
        double z = -t_cam_cs.y() / 1000.0; // Y coordinate (camera coordinate system), unit: [m]

        tf2::Vector3 t_rh_cs(x, y, z);

        tf2::Quaternion q_cam_cs = T_cam_cs.getRotation();
        tf2::Matrix3x3 R_cam_cs(q_cam_cs);

        double roll, pitch, yaw;
        R_cam_cs.getRPY(roll, pitch, yaw);

        tf2::Quaternion q_rh_cs;
        q_rh_cs.setRPY(yaw, -roll, -pitch);

        tf2::Transform T_rh_cs;
        T_rh_cs.setOrigin(t_rh_cs);
        T_rh_cs.setRotation(q_rh_cs);

        return T_rh_cs;
    }

    // Odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    // Subscriber for odometry messages from Extended Kalman Filter
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_subscription_;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_ekf_odom_;

    // Subscribers for left and right stereo images
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

    // Global camera pose relative to camera's odometry frame
    tf2::Transform global_pose_;

    // Initial guess for rotation (rvec) and translation (tvec)
    cv::Mat rvec_ = cv::Mat::zeros(3, 1, CV_64F); // No rotation
    cv::Mat tvec_ = cv::Mat::zeros(3, 1, CV_64F); // No translation

    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Transform buffer and listener
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Transformation matrices
    tf2::Transform T_cam_to_base_;

    // Keyframe timestamp
    std::chrono::time_point<std::chrono::steady_clock> timestamp_prev_;

    bool start_vo_ = false;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualOdometryEstimator>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2 node
    rclcpp::shutdown();

    return 0;
}
