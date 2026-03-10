#include <algorithm>
#include <array>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class SensorMessageNormalizer : public rclcpp::Node
{
public:
    SensorMessageNormalizer()
        : Node("sensor_message_normalizer")
    {
        const auto sensor_input_qos = rclcpp::SensorDataQoS();

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sim/autonomous_vehicle/imu_sensor/imu",
            sensor_input_qos,
            std::bind(&SensorMessageNormalizer::imu_callback, this, std::placeholders::_1));

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/autonomous_vehicle/imu_sensor/imu",
            sensor_input_qos);

        sonar_front_bumper_center_publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/autonomous_vehicle/sonar_front_bumper_center/scan",
                sensor_input_qos);
        sonar_front_bumper_center_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sim/autonomous_vehicle/sonar_front_bumper_center/scan",
                sensor_input_qos,
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                {
                    auto normalized = *msg;
                    normalized.header.frame_id = "sonar_front_bumper_center";
                    sonar_front_bumper_center_publisher_->publish(normalized);
                });

        sonar_front_bumper_left_publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/autonomous_vehicle/sonar_front_bumper_left/scan",
                sensor_input_qos);
        sonar_front_bumper_left_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sim/autonomous_vehicle/sonar_front_bumper_left/scan",
                sensor_input_qos,
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                {
                    auto normalized = *msg;
                    normalized.header.frame_id = "sonar_front_bumper_left";
                    sonar_front_bumper_left_publisher_->publish(normalized);
                });

        sonar_front_bumper_right_publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/autonomous_vehicle/sonar_front_bumper_right/scan",
                sensor_input_qos);
        sonar_front_bumper_right_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sim/autonomous_vehicle/sonar_front_bumper_right/scan",
                sensor_input_qos,
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                {
                    auto normalized = *msg;
                    normalized.header.frame_id = "sonar_front_bumper_right";
                    sonar_front_bumper_right_publisher_->publish(normalized);
                });

        sonar_left_side_bumper_publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/autonomous_vehicle/sonar_left_side_bumper/scan",
                sensor_input_qos);
        sonar_left_side_bumper_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sim/autonomous_vehicle/sonar_left_side_bumper/scan",
                sensor_input_qos,
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                {
                    auto normalized = *msg;
                    normalized.header.frame_id = "sonar_left_side_bumper";
                    sonar_left_side_bumper_publisher_->publish(normalized);
                });

        sonar_right_side_bumper_publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/autonomous_vehicle/sonar_right_side_bumper/scan",
                sensor_input_qos);
        sonar_right_side_bumper_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sim/autonomous_vehicle/sonar_right_side_bumper/scan",
                sensor_input_qos,
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                {
                    auto normalized = *msg;
                    normalized.header.frame_id = "sonar_right_side_bumper";
                    sonar_right_side_bumper_publisher_->publish(normalized);
                });

        sonar_rear_bumper_center_publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/autonomous_vehicle/sonar_rear_bumper_center/scan",
                sensor_input_qos);
        sonar_rear_bumper_center_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sim/autonomous_vehicle/sonar_rear_bumper_center/scan",
                sensor_input_qos,
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                {
                    auto normalized = *msg;
                    normalized.header.frame_id = "sonar_rear_bumper_center";
                    sonar_rear_bumper_center_publisher_->publish(normalized);
                });

        sonar_rear_bumper_left_publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/autonomous_vehicle/sonar_rear_bumper_left/scan",
                sensor_input_qos);
        sonar_rear_bumper_left_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sim/autonomous_vehicle/sonar_rear_bumper_left/scan",
                sensor_input_qos,
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                {
                    auto normalized = *msg;
                    normalized.header.frame_id = "sonar_rear_bumper_left";
                    sonar_rear_bumper_left_publisher_->publish(normalized);
                });

        sonar_rear_bumper_right_publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/autonomous_vehicle/sonar_rear_bumper_right/scan",
                sensor_input_qos);
        sonar_rear_bumper_right_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sim/autonomous_vehicle/sonar_rear_bumper_right/scan",
                sensor_input_qos,
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                {
                    auto normalized = *msg;
                    normalized.header.frame_id = "sonar_rear_bumper_right";
                    sonar_rear_bumper_right_publisher_->publish(normalized);
                });

        RCLCPP_INFO(
            this->get_logger(),
            "Sensor message normalizer started. Republishing normalized IMU and bumper scans...");
    }

private:
    static bool covariance_is_all_zero(const std::array<double, 9> &covariance)
    {
        return std::all_of(covariance.begin(), covariance.end(), 
                          [](double v) { return v == 0.0; });
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        auto normalized = *msg;
        normalized.header.frame_id = "imu_sensor";

        if (covariance_is_all_zero(normalized.orientation_covariance))
        {
            normalized.orientation_covariance = {
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01};
        }

        if (covariance_is_all_zero(normalized.angular_velocity_covariance))
        {
            normalized.angular_velocity_covariance = {
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001};
        }

        if (covariance_is_all_zero(normalized.linear_acceleration_covariance))
        {
            normalized.linear_acceleration_covariance = {
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1};
        }

        imu_publisher_->publish(normalized);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_front_bumper_center_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_front_bumper_center_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_front_bumper_left_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_front_bumper_left_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_front_bumper_right_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_front_bumper_right_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_left_side_bumper_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_left_side_bumper_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_right_side_bumper_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_right_side_bumper_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_rear_bumper_center_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_rear_bumper_center_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_rear_bumper_left_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_rear_bumper_left_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_rear_bumper_right_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        sonar_rear_bumper_right_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMessageNormalizer>());
    rclcpp::shutdown();
    return 0;
}
