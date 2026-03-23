#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

        initialize_sonar_publishers(sensor_input_qos);
        initialize_sonar_subscriptions(sensor_input_qos);

        RCLCPP_INFO(
            this->get_logger(),
            "Sensor message normalizer started. Republishing normalized IMU and per-sonar ultrasonic point clouds...");
    }

private:
    struct SonarConfig
    {
        const char *scan_topic;
        const char *point_cloud_topic;
        const char *frame_id;
    };

    static constexpr std::size_t kSonarCount = 8;
    static constexpr std::array<double, 5> kArcSampleFactors = {-1.0, -0.5, 0.0, 0.5, 1.0};
    static constexpr double kDefaultArcHalfAngle = 0.15;
    static constexpr std::array<double, 9> kOrientationCovariance = {
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.01};
    static constexpr std::array<double, 9> kAngularVelocityCovariance = {
        0.001, 0.0, 0.0,
        0.0, 0.001, 0.0,
        0.0, 0.0, 0.001};
    static constexpr std::array<double, 9> kLinearAccelerationCovariance = {
        0.1, 0.0, 0.0,
        0.0, 0.1, 0.0,
        0.0, 0.0, 0.1};

    static constexpr std::array<SonarConfig, kSonarCount> kSonarConfigs = {{
        {"/sim/autonomous_vehicle/sonar_front_bumper_center/scan",
         "/autonomous_vehicle/sonar_front_bumper_center/pointcloud",
         "sonar_front_bumper_center"},
        {"/sim/autonomous_vehicle/sonar_front_bumper_left/scan",
         "/autonomous_vehicle/sonar_front_bumper_left/pointcloud",
         "sonar_front_bumper_left"},
        {"/sim/autonomous_vehicle/sonar_front_bumper_right/scan",
         "/autonomous_vehicle/sonar_front_bumper_right/pointcloud",
         "sonar_front_bumper_right"},
        {"/sim/autonomous_vehicle/sonar_left_side_bumper/scan",
         "/autonomous_vehicle/sonar_left_side_bumper/pointcloud",
         "sonar_left_side_bumper"},
        {"/sim/autonomous_vehicle/sonar_right_side_bumper/scan",
         "/autonomous_vehicle/sonar_right_side_bumper/pointcloud",
         "sonar_right_side_bumper"},
        {"/sim/autonomous_vehicle/sonar_rear_bumper_center/scan",
         "/autonomous_vehicle/sonar_rear_bumper_center/pointcloud",
         "sonar_rear_bumper_center"},
        {"/sim/autonomous_vehicle/sonar_rear_bumper_left/scan",
         "/autonomous_vehicle/sonar_rear_bumper_left/pointcloud",
         "sonar_rear_bumper_left"},
        {"/sim/autonomous_vehicle/sonar_rear_bumper_right/scan",
         "/autonomous_vehicle/sonar_rear_bumper_right/pointcloud",
         "sonar_rear_bumper_right"},
    }};

    static bool covariance_is_all_zero(const std::array<double, 9> &covariance)
    {
        return std::all_of(
            covariance.begin(),
            covariance.end(),
            [](double value) { return value == 0.0; });
    }

    void initialize_sonar_publishers(const rclcpp::QoS &qos)
    {
        for (std::size_t index = 0; index < kSonarConfigs.size(); ++index)
        {
            sonar_point_cloud_publishers_[index] =
                this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    kSonarConfigs[index].point_cloud_topic,
                    qos);
        }
    }

    void initialize_sonar_subscriptions(const rclcpp::QoS &qos)
    {
        for (std::size_t index = 0; index < kSonarConfigs.size(); ++index)
        {
            sonar_subscriptions_[index] =
                this->create_subscription<sensor_msgs::msg::LaserScan>(
                    kSonarConfigs[index].scan_topic,
                    qos,
                    [this, index](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
                    {
                        handle_sonar_scan(*msg, index);
                    });
        }
    }

    void handle_sonar_scan(const sensor_msgs::msg::LaserScan &scan, const std::size_t sonar_index)
    {
        sonar_point_cloud_publishers_[sonar_index]->publish(
            create_point_cloud_message(scan, sonar_index));
    }

    std::optional<float> get_closest_valid_range(const sensor_msgs::msg::LaserScan &scan) const
    {
        float closest_range = scan.range_max;
        bool found_valid_range = false;

        for (const float range : scan.ranges)
        {
            if (!std::isfinite(range))
            {
                continue;
            }

            if (range < scan.range_min || range > scan.range_max)
            {
                continue;
            }

            closest_range = std::min(closest_range, range);
            found_valid_range = true;
        }

        if (!found_valid_range)
        {
            return std::nullopt;
        }

        return closest_range;
    }

    sensor_msgs::msg::PointCloud2 create_point_cloud_message(
        const sensor_msgs::msg::LaserScan &scan,
        const std::size_t sonar_index) const
    {
        auto cloud = sensor_msgs::msg::PointCloud2();
        cloud.header.stamp = scan.header.stamp;
        cloud.header.frame_id = kSonarConfigs[sonar_index].frame_id;
        cloud.height = 1;
        cloud.is_bigendian = false;
        cloud.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        const auto range = get_closest_valid_range(scan);
        if (!range.has_value())
        {
            modifier.resize(0);
            return cloud;
        }

        const auto points = create_local_arc_points(*range);
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto &point : points)
        {
            *iter_x = point[0];
            *iter_y = point[1];
            *iter_z = point[2];
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        return cloud;
    }

    static std::array<std::array<float, 3>, 5> create_local_arc_points(const float range)
    {
        auto points = std::array<std::array<float, 3>, 5>{};

        for (std::size_t index = 0; index < kArcSampleFactors.size(); ++index)
        {
            const double beam_angle = kArcSampleFactors[index] * kDefaultArcHalfAngle;
            points[index] = {
                static_cast<float>(static_cast<double>(range) * std::cos(beam_angle)),
                static_cast<float>(static_cast<double>(range) * std::sin(beam_angle)),
                0.0F};
        }

        return points;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        imu_publisher_->publish(normalize_imu_message(*msg));
    }

    sensor_msgs::msg::Imu normalize_imu_message(const sensor_msgs::msg::Imu &imu_message) const
    {
        auto normalized = imu_message;
        normalized.header.frame_id = "imu_sensor";

        if (covariance_is_all_zero(normalized.orientation_covariance))
        {
            normalized.orientation_covariance = kOrientationCovariance;
        }

        if (covariance_is_all_zero(normalized.angular_velocity_covariance))
        {
            normalized.angular_velocity_covariance = kAngularVelocityCovariance;
        }

        if (covariance_is_all_zero(normalized.linear_acceleration_covariance))
        {
            normalized.linear_acceleration_covariance = kLinearAccelerationCovariance;
        }

        return normalized;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, kSonarCount>
        sonar_point_cloud_publishers_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr, kSonarCount>
        sonar_subscriptions_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMessageNormalizer>());
    rclcpp::shutdown();
    return 0;
}
