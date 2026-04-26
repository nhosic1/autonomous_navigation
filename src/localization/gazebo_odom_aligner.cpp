#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GazeboOdomAligner : public rclcpp::Node
{
public:
    GazeboOdomAligner() : Node("gazebo_odom_aligner")
    {
        aligned_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/autonomous_vehicle/odometry/gazebo_aligned", 10);

        gazebo_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/autonomous_vehicle/odometry/gazebo", 10,
            std::bind(&GazeboOdomAligner::gazebo_odom_callback, this, std::placeholders::_1));

        reset_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/reset_pose", 10,
            std::bind(&GazeboOdomAligner::reset_pose_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    static tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose &pose_msg)
    {
        tf2::Transform transform;
        tf2::fromMsg(pose_msg, transform);
        return transform;
    }

    static geometry_msgs::msg::Pose transform_to_pose(const tf2::Transform &transform)
    {
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = transform.getOrigin().x();
        pose_msg.position.y = transform.getOrigin().y();
        pose_msg.position.z = transform.getOrigin().z();
        pose_msg.orientation = tf2::toMsg(transform.getRotation());
        return pose_msg;
    }

    void set_alignment(const tf2::Transform &T_odom_base_link,
                       const tf2::Transform &T_sim_odom_base_link)
    {
        T_odom_sim_odom_ = T_odom_base_link * T_sim_odom_base_link.inverse();
        alignment_initialized_ = true;
    }

    void initialize_alignment(const nav_msgs::msg::Odometry::ConstSharedPtr &gazebo_odom)
    {
        const tf2::Transform T_sim_odom_base_link = pose_to_transform(gazebo_odom->pose.pose);
        const tf2::Vector3 initial_position = T_sim_odom_base_link.getOrigin();

        tf2::Quaternion aligned_rotation;
        aligned_rotation.setRPY(0.0, 0.0, 0.0);

        tf2::Transform T_odom_base_link_initial;
        T_odom_base_link_initial.setOrigin(tf2::Vector3(0.0, 0.0, initial_position.z()));
        T_odom_base_link_initial.setRotation(aligned_rotation);

        set_alignment(T_odom_base_link_initial, T_sim_odom_base_link);
        RCLCPP_INFO(this->get_logger(), "Gazebo odometry alignment initialized.");
    }

    bool apply_pending_reset_alignment()
    {
        if (!reset_pose_pending_ || !latest_gazebo_odom_)
        {
            return false;
        }

        geometry_msgs::msg::TransformStamped T_base_footprint_base_link_msg;
        try
        {
            T_base_footprint_base_link_msg = tf_buffer_->lookupTransform(
                "base_footprint", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Gazebo odometry reset is waiting for base_footprint -> base_link: %s",
                ex.what());
            return false;
        }

        tf2::Transform T_base_footprint_base_link;
        tf2::fromMsg(T_base_footprint_base_link_msg.transform, T_base_footprint_base_link);

        const tf2::Transform T_sim_odom_base_link =
            pose_to_transform(latest_gazebo_odom_->pose.pose);
        set_alignment(
            T_odom_base_footprint_reset_ * T_base_footprint_base_link,
            T_sim_odom_base_link);

        reset_pose_pending_ = false;
        RCLCPP_INFO(this->get_logger(), "Gazebo odometry alignment hard-reset to localization reset pose.");
        return true;
    }

    void reset_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg)
    {
        tf2::fromMsg(msg->pose.pose, T_odom_base_footprint_reset_);
        reset_pose_pending_ = true;
        apply_pending_reset_alignment();
    }

    void gazebo_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
    {
        latest_gazebo_odom_ = msg;

        if (reset_pose_pending_)
        {
            apply_pending_reset_alignment();
        }
        else if (!alignment_initialized_)
        {
            initialize_alignment(msg);
        }

        if (!alignment_initialized_)
        {
            return;
        }

        nav_msgs::msg::Odometry aligned_odom = *msg;
        aligned_odom.header.frame_id = "odom";

        const tf2::Transform T_sim_odom_base_link = pose_to_transform(msg->pose.pose);
        const tf2::Transform T_odom_base_link = T_odom_sim_odom_ * T_sim_odom_base_link;
        aligned_odom.pose.pose = transform_to_pose(T_odom_base_link);

        const tf2::Quaternion rotation = T_odom_sim_odom_.getRotation();
        tf2::Vector3 linear_twist(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);
        tf2::Vector3 angular_twist(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z);

        linear_twist = tf2::quatRotate(rotation, linear_twist);
        angular_twist = tf2::quatRotate(rotation, angular_twist);

        aligned_odom.twist.twist.linear.x = linear_twist.x();
        aligned_odom.twist.twist.linear.y = linear_twist.y();
        aligned_odom.twist.twist.linear.z = linear_twist.z();
        aligned_odom.twist.twist.angular.x = angular_twist.x();
        aligned_odom.twist.twist.angular.y = angular_twist.y();
        aligned_odom.twist.twist.angular.z = angular_twist.z();

        aligned_odom_publisher_->publish(aligned_odom);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aligned_odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gazebo_odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reset_pose_subscription_;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_gazebo_odom_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    tf2::Transform T_odom_sim_odom_;
    tf2::Transform T_odom_base_footprint_reset_ = tf2::Transform::getIdentity();
    bool alignment_initialized_ = false;
    bool reset_pose_pending_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboOdomAligner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
