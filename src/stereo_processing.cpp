#include <opencv2/opencv.hpp>
#include <filesystem>
#include <cmath>
#include <opencv2/ximgproc.hpp>
#include "autonomous_driving/stereo_processing.hpp"

namespace sp
{

void filter_speckles(cv::Mat &disparity_map, const int &speckle_threshold, const int &max_speckle_size, const int &replacement_disparity)
{
    // Apply thresholding to find speckles
    cv::Mat thresholded_map;
    cv::threshold(disparity_map, thresholded_map, speckle_threshold * 16, 255, cv::THRESH_BINARY);

    thresholded_map.convertTo(thresholded_map, CV_8UC1);

    // Find connected components
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(thresholded_map, labels, stats, centroids);

    // Filter out small components (speckles)
    for (int i = 1; i < num_labels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < max_speckle_size) {
            cv::Mat mask = (labels == i);
            disparity_map.setTo(replacement_disparity * 16, mask);
        }
    }
}

void filter_invalid_disparities(cv::Mat &disparity_map, const int &min_disparity)
{
    // Create a mask to filter out negative disparities
    cv::Mat mask = (disparity_map < min_disparity * 16);
    int replacement_disparity = 0;
    if (min_disparity > 0)
    {
        replacement_disparity = min_disparity - 1;
    }

    // Apply the mask to the disparity map
    disparity_map.setTo(replacement_disparity * 16, mask);
}

cv::Ptr<cv::StereoBM> create_default_stereo_matcher()
{
    cv::Ptr<cv::StereoBM> matcher = cv::StereoBM::create();
    matcher->setNumDisparities(176);
    matcher->setBlockSize(11);
    matcher->setMinDisparity(3);
    matcher->setUniquenessRatio(30);
    matcher->setTextureThreshold(50);
    matcher->setSpeckleRange(3);
    matcher->setSpeckleWindowSize(600);
    matcher->setPreFilterSize(9);
    matcher->setPreFilterCap(31);
    matcher->setSmallerBlockSize(5);

    return matcher;
}

cv::Ptr<cv::StereoSGBM> create_default_stereo_SG_matcher()
{
    int block_size = 10;
    cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create();
    matcher->setNumDisparities(176);
    matcher->setBlockSize(block_size);
    matcher->setMinDisparity(1);
    matcher->setUniquenessRatio(70);
    matcher->setSpeckleRange(3);
    matcher->setSpeckleWindowSize(400);
    matcher->setDisp12MaxDiff(2);
    matcher->setP1(8*block_size*block_size);
    matcher->setP2(32*block_size*block_size);
    matcher->setPreFilterCap(31);

    return matcher;
}

cv::Mat compute_disparity_map(const cv::Mat &left_img, const cv::Mat &right_img)
{
    cv::Mat left_gray, right_gray;
    cv::cvtColor(left_img, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_img, right_gray, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::StereoBM> matcher = sp::create_default_stereo_matcher();

    cv::Mat disparity_map;
    matcher->compute(left_gray, right_gray, disparity_map);

    // Apply filters
    sp::filter_speckles(disparity_map, 100, 6200, 3);
    sp::filter_invalid_disparities(disparity_map, 3);
    cv::medianBlur(disparity_map, disparity_map, 5);

    return disparity_map; // Data type of disparity_map is CV_16SC1
}

cv::Mat compute_disparity_map_wls(const cv::Mat &left_img, const cv::Mat &right_img)
{
    cv::Mat left_gray, right_gray;
    cv::cvtColor(left_img, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_img, right_gray, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::StereoBM> left_matcher = sp::create_default_stereo_matcher();
    cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

    cv::Mat left_disparity_map, right_disparity_map, filtered_disparity_map;
    left_matcher->compute(left_gray, right_gray, left_disparity_map);
    right_matcher->compute(right_gray, left_gray, right_disparity_map);

    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
    wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
    wls_filter->setLambda(8000.0);
    wls_filter->setSigmaColor(1.9);
    wls_filter->setDepthDiscontinuityRadius(5);
    wls_filter->filter(left_disparity_map, left_img, filtered_disparity_map, right_disparity_map);

    sp::filter_invalid_disparities(filtered_disparity_map, 3);

    return filtered_disparity_map; // Data type of disparity_map is CV_16SC1
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