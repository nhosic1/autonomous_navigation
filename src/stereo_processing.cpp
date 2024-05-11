#include <opencv2/opencv.hpp>
#include <filesystem>
#include <cmath>
#include "autonomous_driving/stereo_processing.hpp"

namespace sp
{

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
void visualize_disparity_map(const cv::Mat &disparity_map, const cv::Point2i &closest_point_2D, const cv::Point3f &closest_point_3D, const int &max_disparity)
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

} // namespace sp