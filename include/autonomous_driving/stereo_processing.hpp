#ifndef STEREO_PROCESSING_HPP
#define STEREO_PROCESSING_HPP

#include <opencv2/opencv.hpp>

namespace sp
{
    void filter_speckles(cv::Mat &disparity_map, const int &speckle_threshold, const int &min_speckle_size, const int &replacement_disparity = 0);
    void filter_invalid_disparities(cv::Mat &disparity_map, const int &min_disparity);
    cv::Mat compute_disparity_map(const cv::Mat &left_img, const cv::Mat &right_img);
    cv::Mat compute_disparity_map_wls(const cv::Mat &left_img, const cv::Mat &right_img);
    void visualize_disparity_map(const cv::Mat &disparity_map, const cv::Point2i &closest_point_2D, const cv::Point3f &closest_point_3D, const int &max_disparity = 176);
    std::vector<cv::Point3f> compute_3D_points(const cv::Mat &disparity_map, const cv::Mat &camera_matrix);
    void loadCameraParameters(const std::string &file_path, cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
} // namespace sp

#endif // STEREO_PROCESSING_HPP