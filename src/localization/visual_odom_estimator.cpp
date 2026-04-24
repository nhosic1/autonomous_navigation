#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include "autonomous_navigation/localization/localization.hpp"
#include "autonomous_navigation/perception/stereo_processing.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_time_policy;
typedef message_filters::Synchronizer<approximate_time_policy> approximate_time_synchronizer;

class VisualOdometryEstimator : public rclcpp::Node
{
public:
    VisualOdometryEstimator() : Node("visual_odom_estimator")
    {
        // Report average accepted visual odometry updates over a longer window.
        timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]()
                                         {
            RCLCPP_INFO(this->get_logger(), "VO update rate = %.1f", callback_count_ / 5.0);

            // Reset the accepted update count for the next reporting window.
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

        // Create publishers
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/autonomous_vehicle/odometry/visual", 10);
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/autonomous_vehicle/stereo/pointcloud", rclcpp::SensorDataQoS());

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        try
        {
            const tf2::Duration static_transform_wait = tf2::durationFromSec(3.0);

            const geometry_msgs::msg::TransformStamped T_base_link_left_camera_msg =
                tf_buffer_->lookupTransform("base_link", "left_camera", tf2::TimePointZero, static_transform_wait);
            tf2::fromMsg(T_base_link_left_camera_msg.transform, T_base_link_left_camera_);
            T_left_camera_base_link_ = T_base_link_left_camera_.inverse();

            const geometry_msgs::msg::TransformStamped T_base_footprint_base_link_msg =
                tf_buffer_->lookupTransform("base_footprint", "base_link", tf2::TimePointZero, static_transform_wait);
            tf2::fromMsg(T_base_footprint_base_link_msg.transform, T_base_footprint_base_link_);
        }
        catch (const tf2::TransformException &ex)
        {
            throw std::runtime_error(std::string("visual_odom_estimator startup failed: missing required static transform: ") + ex.what());
        }

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
    }

private:
    void prepare_stereo_cv_images(const sensor_msgs::msg::Image::ConstSharedPtr &left_img_msg_ptr,
                                  const sensor_msgs::msg::Image::ConstSharedPtr &right_img_msg_ptr,
                                  cv::Mat &left_img,
                                  cv::Mat &right_img)
    {
        cv_bridge::CvImagePtr cv_left_img_ptr;
        cv_bridge::CvImagePtr cv_right_img_ptr;

        cv_left_img_ptr = cv_bridge::toCvCopy(left_img_msg_ptr);
        cv_right_img_ptr = cv_bridge::toCvCopy(right_img_msg_ptr);

        left_img = cv_left_img_ptr->image;
        right_img = cv_right_img_ptr->image;

        if (!map_1_L_.empty() && !map_2_L_.empty())
        {
            cv::remap(left_img, left_img, map_1_L_, map_2_L_, cv::INTER_LINEAR);
        }
        if (!map_1_R_.empty() && !map_2_R_.empty())
        {
            cv::remap(right_img, right_img, map_1_R_, map_2_R_, cv::INTER_LINEAR);
        }
    }

    void save_failure_snapshots(const cv::Mat &left_img, const builtin_interfaces::msg::Time &timestamp)
    {
        save_snapshots(keyframe_L_prev_, left_img, timestamp);
        save_snapshots(keyframe_L_prev_, keyframe_R_prev_, timestamp, "stereo_prev_");
    }

    void reset_motion_estimate_guess()
    {
        rvec_guess_ = cv::Mat::zeros(3, 1, CV_64F);
        tvec_guess_ = cv::Mat::zeros(3, 1, CV_64F);
    }

    void handle_visual_odometry_failure(const cv::Mat &left_img, const builtin_interfaces::msg::Time &timestamp)
    {
        save_failure_snapshots(left_img, timestamp);
        reset_motion_estimate_guess();

        bootstrap_complete_ = false;
    }

    void update_reference_state(const std::vector<cv::KeyPoint> &keypoints_L,
                                const cv::Mat &descriptors_L,
                                const std::vector<cv::Point2d> &points_2D_stereo_filtered,
                                const std::vector<cv::Point3d> &points_3D_stereo_filtered,
                                const cv::Mat &left_img,
                                const cv::Mat &right_img,
                                const std::chrono::time_point<std::chrono::steady_clock> &timestamp,
                                const nav_msgs::msg::Odometry &odom_msg,
                                const tf2::Transform &T_odom_left_camera)
    {
        keypoints_L_prev_ = keypoints_L;
        descriptors_L_prev_ = descriptors_L;
        points_2D_stereo_prev_ = points_2D_stereo_filtered;
        points_3D_stereo_prev_ = points_3D_stereo_filtered;
        keyframe_L_prev_ = left_img;
        keyframe_R_prev_ = right_img;
        timestamp_prev_ = timestamp;
        odom_msg_prev_ = odom_msg;
        T_odom_left_camera_prev_ = T_odom_left_camera;
        rvec_guess_ = cv::Mat::zeros(3, 1, CV_64F); // No rotation
        tvec_guess_ = cv::Mat::zeros(3, 1, CV_64F); // No translation
    }

    bool initialize_visual_odometry(const tf2::Transform &T_odom_base_link, nav_msgs::msg::Odometry &odom_msg)
    {
        T_odom_left_camera_prev_ = T_odom_base_link * T_base_link_left_camera_;

        // Publish initial odometry message
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "odom";

        odom_msg.header = header;
        odom_msg.child_frame_id = "base_link";

        const tf2::Transform T_odom_base_link_current = T_odom_left_camera_prev_ * T_left_camera_base_link_;
        tf2::Vector3 t_global = T_odom_base_link_current.getOrigin();
        odom_msg.pose.pose.position.x = t_global.x();
        odom_msg.pose.pose.position.y = t_global.y();
        odom_msg.pose.pose.position.z = t_global.z();

        tf2::Quaternion q_global = T_odom_base_link_current.getRotation();
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
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0, // v_x
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0, // v_y
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,  // v_z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0, // w_x
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0, // w_y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05  // w_z
        };

        odom_publisher_->publish(odom_msg);
        return true;
    }

    void apply_visual_odometry_update(const cv::Mat &rvec,
                                      const cv::Mat &tvec,
                                      double dt,
                                      nav_msgs::msg::Odometry &odom_msg,
                                      tf2::Transform &T_odom_left_camera)
    {
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);

        // Create transformation matrix for local pose in camera coordinate system (camera assumed to be static)
        tf2::Vector3 t_cam_cs(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
        tf2::Matrix3x3 R_cam_cs(R_cv.at<double>(0, 0), R_cv.at<double>(0, 1), R_cv.at<double>(0, 2),
                                R_cv.at<double>(1, 0), R_cv.at<double>(1, 1), R_cv.at<double>(1, 2),
                                R_cv.at<double>(2, 0), R_cv.at<double>(2, 1), R_cv.at<double>(2, 2));
        tf2::Quaternion q_cam_cs;
        R_cam_cs.getRotation(q_cam_cs);

        tf2::Transform T_left_camera_prev_left_camera_cam_cs;
        T_left_camera_prev_left_camera_cam_cs.setOrigin(t_cam_cs);
        T_left_camera_prev_left_camera_cam_cs.setRotation(q_cam_cs);

        // Invert rotation and translation to represent camera's motion in odometry frame (3D points are static)
        T_left_camera_prev_left_camera_cam_cs = T_left_camera_prev_left_camera_cam_cs.inverse(); // unit: [m]

        // Update the global pose
        const tf2::Transform T_left_camera_prev_left_camera = convert_cam_to_rh_coordinate_sys(T_left_camera_prev_left_camera_cam_cs); // unit: [m]
        T_odom_left_camera = T_odom_left_camera_prev_ * T_left_camera_prev_left_camera;

        // Fill the odometry message
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "odom";

        odom_msg.header = header;
        odom_msg.child_frame_id = "base_link";

        const tf2::Transform T_odom_base_link = T_odom_left_camera * T_left_camera_base_link_;
        tf2::Vector3 t_global = T_odom_base_link.getOrigin();
        odom_msg.pose.pose.position.x = t_global.x();
        odom_msg.pose.pose.position.y = t_global.y();
        odom_msg.pose.pose.position.z = t_global.z();

        tf2::Quaternion q_global = T_odom_base_link.getRotation();
        odom_msg.pose.pose.orientation = tf2::toMsg(q_global);

        odom_msg.pose.covariance = {
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0, // X
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0, // Y
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,  // Z
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  // Roll
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  // Pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1   // Yaw
        };

        tf2::Vector3 t_local = T_left_camera_prev_left_camera.getOrigin();
        double v_x = t_local.x() / dt;
        double v_y = t_local.y() / dt;
        double v_z = t_local.z() / dt;

        tf2::Quaternion q_local = T_left_camera_prev_left_camera.getRotation();
        double angle = q_local.getAngle();
        tf2::Vector3 axis = q_local.getAxis();

        const double epsilon = 1e-6;
        tf2::Vector3 w = (angle < epsilon) ? tf2::Vector3(0, 0, 0) : axis * (angle / dt);

        const tf2::Matrix3x3 R_base_link_left_camera = T_base_link_left_camera_.getBasis();
        const tf2::Vector3 p_base_link_left_camera = T_base_link_left_camera_.getOrigin();
        const tf2::Vector3 v_left_camera_base_link = R_base_link_left_camera * tf2::Vector3(v_x, v_y, v_z);
        const tf2::Vector3 w_base_link_base_link = R_base_link_left_camera * w;
        
        // Rigid-body velocity relation between two points:
        // v_cam = v_base + w x p_base->cam  =>  v_base = v_cam - w x p_base->cam
        const tf2::Vector3 v_base_link_base_link = v_left_camera_base_link - w_base_link_base_link.cross(p_base_link_left_camera);

        odom_msg.twist.twist.linear.x = v_base_link_base_link.x();
        odom_msg.twist.twist.linear.y = v_base_link_base_link.y();
        odom_msg.twist.twist.linear.z = v_base_link_base_link.z();
        odom_msg.twist.twist.angular.x = w_base_link_base_link.x();
        odom_msg.twist.twist.angular.y = w_base_link_base_link.y();
        odom_msg.twist.twist.angular.z = w_base_link_base_link.z();

        odom_msg.twist.covariance = {
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0, // v_x
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0, // v_y
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,  // v_z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0, // w_x
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0, // w_y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05  // w_z
        };

        odom_publisher_->publish(odom_msg);
    }

    bool build_stereo_observations(const std::vector<cv::KeyPoint> &keypoints_L,
                                   const cv::Mat &descriptors_L,
                                   const std::vector<cv::KeyPoint> &keypoints_R,
                                   const cv::Mat &descriptors_R,
                                   std::vector<cv::Point2d> &points_2D_stereo_filtered,
                                   std::vector<cv::Point3d> &points_3D_stereo_filtered,
                                   double &average_depth)
    {
        std::vector<cv::Point3d> points_3D_stereo;
        std::vector<cv::Point2d> points_2D_stereo;

        bool success = sp::compute_3D_points_from_features(
            matcher_, P_L_, keypoints_L, descriptors_L, P_R_, keypoints_R, descriptors_R,
            points_3D_stereo, points_2D_stereo, average_depth);

        if (!success)
        {
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        cloud->reserve(points_3D_stereo.size());
        for (const auto &point : points_3D_stereo)
        {
            cloud->emplace_back(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z));
        }
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = true;

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.2);
        outrem.setMinNeighborsInRadius(3);
        outrem.filter(*cloud_filtered);

        pcl::IndicesConstPtr removed_indices = outrem.getRemovedIndices();

        std::vector<bool> outlier_mask(cloud->size(), false);
        for (size_t i : *removed_indices)
        {
            if (i < outlier_mask.size())
            {
                outlier_mask[i] = true;
            }
        }

        for (size_t i = 0; i < points_2D_stereo.size(); ++i)
        {
            if (!outlier_mask[i])
            {
                points_2D_stereo_filtered.push_back(points_2D_stereo[i]);
                points_3D_stereo_filtered.push_back(points_3D_stereo[i]);
            }
        }

        sensor_msgs::msg::PointCloud2 point_cloud_msg = convert_to_PointCloud2_msg(cloud_filtered, "left_camera");
        point_cloud_publisher_->publish(point_cloud_msg);

        return true;
    }

    // Callback function for synchronized left and right stereo images
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img_msg_ptr, const sensor_msgs::msg::Image::ConstSharedPtr &right_img_msg_ptr)
    {
        auto timestamp = std::chrono::steady_clock::now();

        cv::Mat left_img;
        cv::Mat right_img;
        try
        {
            prepare_stereo_cv_images(left_img_msg_ptr, right_img_msg_ptr, left_img, right_img);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert stereo image messages to OpenCV images: %s", e.what());
            return;
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to undistort and rectify stereo OpenCV images: %s", e.what());
            return;
        }

        std::vector<cv::KeyPoint> keypoints_L, keypoints_R;
        cv::Mat descriptors_L, descriptors_R;

        // Detect keypoints and compute their descriptors
        orb_->detectAndCompute(left_img, cv::Mat(), keypoints_L, descriptors_L);
        orb_->detectAndCompute(right_img, cv::Mat(), keypoints_R, descriptors_R);

        // Build stereo observations
        std::vector<cv::Point3d> points_3D_stereo_filtered;
        std::vector<cv::Point2d> points_2D_stereo_filtered;
        double average_depth = 0.0;

        if (descriptors_L.empty() || descriptors_R.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Visual odometry chain is broken: not enough detected features. Restarting visual odometry.");

            handle_visual_odometry_failure(left_img, left_img_msg_ptr->header.stamp);
            return;
        }
        else
        {
            try
            {
                if (!build_stereo_observations(keypoints_L, descriptors_L, keypoints_R, descriptors_R, points_2D_stereo_filtered, points_3D_stereo_filtered, average_depth))
                {
                    RCLCPP_ERROR(this->get_logger(), "Visual odometry chain is broken: failed to compute 3D points. Restarting visual odometry.");

                    handle_visual_odometry_failure(left_img, left_img_msg_ptr->header.stamp);
                    return;
                }
            }
            catch (const cv::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Visual odometry chain is broken: OpenCV exception during stereo reconstruction. Restarting visual odometry. OpenCV error: %s", e.what());

                handle_visual_odometry_failure(left_img, left_img_msg_ptr->header.stamp);
                return;
            }
        }

        if (bootstrap_complete_)
        {
            bool success = false;
            cv::Mat rvec = rvec_guess_.clone();
            cv::Mat tvec = tvec_guess_.clone();

            // Placeholders for possible future reuse of motion-consistent PnP inliers
            std::vector<cv::Point2d> points_2D_stereo_prev_filtered;
            std::vector<cv::Point3d> points_3D_stereo_prev_filtered;

            try
            {
                success = loc::compute_local_pose(camera_matrix_L_rect_, cv::Mat(), matcher_, keypoints_L_prev_, descriptors_L_prev_, keypoints_L, descriptors_L, points_2D_stereo_prev_, points_3D_stereo_prev_, points_2D_stereo_prev_filtered, points_3D_stereo_prev_filtered, rvec, tvec);
            }
            catch (const cv::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Visual odometry chain is broken: OpenCV exception during local pose estimation. Restarting visual odometry. OpenCV error: %s", e.what());

                handle_visual_odometry_failure(left_img, left_img_msg_ptr->header.stamp);
                return;
            }

            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Visual odometry chain is broken: failed to compute local pose. Restarting visual odometry.");

                handle_visual_odometry_failure(left_img, left_img_msg_ptr->header.stamp);
                return;
            }

                double dt = std::chrono::duration_cast<std::chrono::duration<double>>(timestamp - timestamp_prev_).count(); // unit: [s]

                double t_norm = cv::norm(tvec); // unit: [m]
                double r_norm = cv::norm(rvec); // unit: [rad]

                double v_estimate = t_norm / dt; // unit: [m/s]
                if (std::abs(v_estimate) > v_max_)
                {
                    if (dt >= velocity_rejection_timeout_)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Visual odometry chain is broken: velocity rejection timeout exceeded after rejected updates. Restarting visual odometry.");

                        handle_visual_odometry_failure(left_img, left_img_msg_ptr->header.stamp);
                        return;
                }

                RCLCPP_WARN(this->get_logger(), "Visual odometry update rejected: estimated linear velocity exceeds the maximum expected value. Skipping publication for this frame.");
                return;
            }

            rvec_guess_ = rvec;
            tvec_guess_ = tvec;

            nav_msgs::msg::Odometry odom_msg;
            tf2::Transform T_odom_left_camera;
            apply_visual_odometry_update(rvec, tvec, dt, odom_msg, T_odom_left_camera);

            // Keyframe slection
            if ((((t_norm / average_depth) > 0.07 || r_norm > 5 * CV_PI / 180)) && t_norm > 0.05)
            {
                update_reference_state(keypoints_L, descriptors_L, points_2D_stereo_filtered, points_3D_stereo_filtered, left_img, right_img, timestamp, odom_msg, T_odom_left_camera);
            }
        }
        else
        {
            nav_msgs::msg::Odometry odom_msg;
            tf2::Transform T_odom_base_link = tf2::Transform::getIdentity();
            if (!latest_ekf_odom_)
            {
                T_odom_base_link = T_base_footprint_base_link_;
                RCLCPP_WARN(this->get_logger(), "Latest EKF odometry is unavailable during VO bootstrap. Using fallback odom == base_footprint.");
            }
            else
            {
                tf2::fromMsg(latest_ekf_odom_->pose.pose, T_odom_base_link);
                RCLCPP_INFO(this->get_logger(), "Using latest EKF odometry for VO bootstrap.");
            }

            initialize_visual_odometry(T_odom_base_link, odom_msg);
            update_reference_state(keypoints_L, descriptors_L, points_2D_stereo_filtered, points_3D_stereo_filtered, left_img, right_img, timestamp, odom_msg, T_odom_left_camera_prev_);
            bootstrap_complete_ = true;
        }

        callback_count_++;

        bool snapshot = this->get_parameter("snapshot").as_bool();
        if (snapshot == true)
        {
            save_snapshots(left_img, right_img, left_img_msg_ptr->header.stamp, "stereo_");
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

    tf2::Transform convert_cam_to_rh_coordinate_sys(const tf2::Transform &T_cam_cs)
    {
        tf2::Vector3 t_cam_cs = T_cam_cs.getOrigin();

        double x = t_cam_cs.z();  // Z coordinate (camera coordinate system), unit: [m]
        double y = -t_cam_cs.x(); // X coordinate (camera coordinate system), unit: [m]
        double z = -t_cam_cs.y(); // Y coordinate (camera coordinate system), unit: [m]

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

    sensor_msgs::msg::PointCloud2 convert_to_PointCloud2_msg(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &frame_id)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);

        cloud_transformed->reserve(cloud->size());
        cloud_transformed->height = 1;
        cloud_transformed->width = cloud->size();
        cloud_transformed->is_dense = true;

        for (auto &point : cloud->points)
        {
            tf2::Transform pt_tf;
            pt_tf.setOrigin(tf2::Vector3(point.x, point.y, point.z)); // unit: [m]
            pt_tf.setRotation(tf2::Quaternion::getIdentity());        // No rotation
            pt_tf = convert_cam_to_rh_coordinate_sys(pt_tf);

            cloud_transformed->emplace_back(
                static_cast<float>(pt_tf.getOrigin().x()),
                static_cast<float>(pt_tf.getOrigin().y()),
                static_cast<float>(pt_tf.getOrigin().z()));
        }

        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*cloud_transformed, point_cloud_msg);
        point_cloud_msg.header.stamp = this->get_clock()->now();
        point_cloud_msg.header.frame_id = frame_id;

        return point_cloud_msg;
    }

    sensor_msgs::msg::PointCloud2 convert_to_PointCloud2_msg(const std::vector<cv::Point3d> &points, const std::string &frame_id)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = this->get_clock()->now();
        cloud.header.frame_id = frame_id;
        cloud.height = 1;
        cloud.width = points.size();
        cloud.is_dense = true;
        cloud.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto &pt : points)
        {
            tf2::Transform pt_tf;
            pt_tf.setOrigin(tf2::Vector3(pt.x, pt.y, pt.z));   // unit: [m]
            pt_tf.setRotation(tf2::Quaternion::getIdentity()); // No rotation
            pt_tf = convert_cam_to_rh_coordinate_sys(pt_tf);

            *iter_x = static_cast<float>(pt_tf.getOrigin().x());
            *iter_y = static_cast<float>(pt_tf.getOrigin().y());
            *iter_z = static_cast<float>(pt_tf.getOrigin().z());
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        return cloud;
    }

    // Odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    // Subscriber for odometry messages from Extended Kalman Filter
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_subscription_;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_ekf_odom_;

    // Subscribers for left and right stereo images
    image_transport::SubscriberFilter left_subscriber_;
    image_transport::SubscriberFilter right_subscriber_;

    // Pointcloud publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

    // Global left camera pose in odom frame
    tf2::Transform T_odom_left_camera_prev_;

    // Static extrinsics between base_link and left_camera
    tf2::Transform T_base_link_left_camera_ = tf2::Transform::getIdentity();
    tf2::Transform T_left_camera_base_link_ = tf2::Transform::getIdentity();
    tf2::Transform T_base_footprint_base_link_ = tf2::Transform::getIdentity();

    // Previous odometry meesage
    nav_msgs::msg::Odometry odom_msg_prev_;

    // Initial guess for rotation (rvec) and translation (tvec)
    cv::Mat rvec_guess_ = cv::Mat::zeros(3, 1, CV_64F); // No rotation
    cv::Mat tvec_guess_ = cv::Mat::zeros(3, 1, CV_64F); // No translation

    // Keyframe timestamp
    std::chrono::time_point<std::chrono::steady_clock> timestamp_prev_;

    bool bootstrap_complete_ = false;

    double v_max_ = 1.0; // unit: [m/s]
    double velocity_rejection_timeout_ = 0.2; // unit: [s]
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
