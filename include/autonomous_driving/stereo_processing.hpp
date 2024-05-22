#ifndef STEREO_PROCESSING_HPP
#define STEREO_PROCESSING_HPP

#include <opencv2/opencv.hpp>

namespace sp
{
    const int MIN_DISPARITY = 0;
    const int NUM_DISPARITIES = 176;

    void filter_invalid_disparities(const cv::Mat &disparity_map, cv::Mat &output_disparity_map);
    void apply_wls_filter(const cv::Mat &disparity_map, cv::Mat &filtered_disparity_map, const cv::Mat &left_img, const cv::Mat &right_img, const cv::Ptr<cv::StereoMatcher> &left_matcher);
    cv::Mat compute_disparity_map_with_consistency_checks(const cv::Mat &left_img, const cv::Mat &right_img, bool wls_filter = false);
    void display_disparity_map(const cv::Mat &map, std::string name, bool inverse = false);
    void visualize_live_disparity_map(const cv::Mat &disparity_map, const cv::Point2i &closest_point_2D, const cv::Point3f &closest_point_3D);
    std::vector<cv::Point3f> compute_3D_points(const cv::Mat &disparity_map, const cv::Mat &camera_matrix);
    void load_camera_parameters(const std::string &file_path, cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
    cv::Ptr<cv::StereoBM> create_default_stereo_matcher();
    cv::Ptr<cv::StereoSGBM> create_default_stereo_SG_matcher();
} // namespace sp

#endif // STEREO_PROCESSING_HPP