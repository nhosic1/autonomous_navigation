#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
        std::string package_name = "autonomous_driving";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string camera_params_path = package_share_directory + "/config/sim_camera_params.yaml";
        cv::Mat camera_matrix, dist_coeffs;
        loadCameraParameters(camera_params_path, camera_matrix, dist_coeffs);

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

        std::vector<cv::Point3f> points_3D = compute_3D_points(disparity_map, camera_matrix);
        cv::Point3f closest_point(0.0f, 0.0f, std::numeric_limits<float>::max());
        find_closest_point(points_3D, closest_point);

        std::vector<cv::Point2f> points2D;
        std::vector<cv::Point3f> points3D;
        points3D.push_back(closest_point);
        cv::projectPoints(points3D, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_matrix, dist_coeffs, points2D);

        visualize_disparity_map(disparity_map, cv::Point2i(static_cast<int>(points2D[0].x), static_cast<int>(points2D[0].y)), closest_point);

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
    void visualize_disparity_map(const cv::Mat &disparity_map, const cv::Point2i &closest_point_2D, const cv::Point3f &closest_point_3D, const int &max_disparity = 176)
    {
        // Scale the disparity map and convert it to CV_8UC1
        cv::Mat scaled_disparity_map;
        cv::convertScaleAbs(disparity_map, scaled_disparity_map, 255.0 / (max_disparity * 16.0));

        // Apply a color map to the scaled disparity map (red for closer objects, blue for farther objects)
        cv::Mat colored_disparity_map;
        cv::applyColorMap(scaled_disparity_map, colored_disparity_map, cv::COLORMAP_JET);

        if (closest_point_3D.z < 10000)
        {
            cv::circle(colored_disparity_map, closest_point_2D, 10, cv::Scalar(255, 0, 255), -1);

            // Convert points to from [mm] to [m] and set precision to 2 decimal places for text output
            std::stringstream ss_x, ss_y, ss_z;
            ss_x << std::fixed << std::setprecision(2) << closest_point_3D.x / 1000.0;
            ss_y << std::fixed << std::setprecision(2) << closest_point_3D.y / 1000.0;
            ss_z << std::fixed << std::setprecision(2) << closest_point_3D.z / 1000.0;

            std::string text = "(" + ss_x.str() + ", " + ss_y.str() + ", " + ss_z.str() + ")";

            // Define the font parameters for the point text
            int fontFace = cv::FONT_HERSHEY_SIMPLEX; // Font type
            double fontScale = 1.1;                  // Font scale factor
            cv::Scalar color(255, 0, 255);           // Text color (BGR format)
            int thickness = 3;                       // Thickness of the text

            // Calculate the size of the text bounding box
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
            cv::Point org(closest_point_2D.x - textSize.width / 2, closest_point_2D.y + textSize.height + 25); // Position of the text (below the reprojected point)

            cv::putText(colored_disparity_map, text, org, fontFace, fontScale, color, thickness);
        }

        cv::Mat resized_colored_disparity_map;
        int max_image_width = 800;
        float scale = static_cast<float>(max_image_width) / colored_disparity_map.cols;
        cv::resize(colored_disparity_map, resized_colored_disparity_map, cv::Size(), scale, scale);

        // Display the colored disparity map
        cv::imshow("Disparity Map", resized_colored_disparity_map);
        cv::waitKey(1); // Wait for a key press (1 millisecond)
    }

    std::vector<cv::Point3f> compute_3D_points(const cv::Mat &disparity_map, const cv::Mat &camera_matrix)
    {
        float fx = camera_matrix.at<double>(0, 0); // unit: [mm]
        float fy = camera_matrix.at<double>(1, 1); // unit: [mm]
        float cx = camera_matrix.at<double>(0, 2); // unit: [px]
        float cy = camera_matrix.at<double>(1, 2); // unit: [px]
        float baseline = 180.0;                    // unit: [mm]

        // Compute 3D points from disparity map
        std::vector<cv::Point3f> points_3D;
        for (int y = 0; y < disparity_map.rows; y++)
        {
            for (int x = 0; x < disparity_map.cols; x++)
            {
                int16_t disparity_scaled = disparity_map.at<int16_t>(y, x);
                float disparity = disparity_scaled / 16.0f;
                if (disparity > 0)
                {
                    // Calculate 3D point (X, Y, Z) using the disparity and camera parameters
                    // 3D points are  represented in the left camera coordinate system.
                    float Z = (fx * baseline) / disparity;
                    float X = (x - cx) * Z / fx;
                    float Y = (y - cy) * Z / fy;

                    // Set the 3D point in the points_3D matrix
                    points_3D.push_back(cv::Point3f(X, Y, Z));
                }
            }
        }

        return points_3D; // unit: [mm]
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

    void loadCameraParameters(const std::string &file_path, cv::Mat &camera_matrix, cv::Mat &dist_coeffs)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cerr << "Failed to open YAML file: " << file_path << std::endl;
            return;
        }

        // Load camera matrix
        fs["camera_matrix"] >> camera_matrix;
        camera_matrix.convertTo(camera_matrix, CV_64F);

        // Load distortion coefficients
        fs["dist_coeffs"] >> dist_coeffs;
        dist_coeffs.convertTo(dist_coeffs, CV_64F);

        fs.release();
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
