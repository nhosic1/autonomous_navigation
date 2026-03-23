#ifndef STEREO_PROCESSING_HPP
#define STEREO_PROCESSING_HPP

#include <opencv2/opencv.hpp>

namespace sp
{
    const int MIN_DISPARITY = 10;
    const int NUM_DISPARITIES = 112;

    void print_mat_type(const cv::Mat &mat, const std::string mat_name);
    void filter_invalid_disparities(const cv::Mat &disparity_map, cv::Mat &output_disparity_map);
    void apply_wls_filter(const cv::Mat &disparity_map, cv::Mat &filtered_disparity_map, const cv::Mat &left_img, const cv::Mat &right_img, const cv::Ptr<cv::StereoMatcher> &left_matcher);
    cv::Mat compute_disparity_map_with_consistency_check(const cv::Mat &left_img, const cv::Mat &right_img, bool wls_filter = false);
    void display_disparity_map(const cv::Mat &map, std::string name, bool inverse = false);
    void visualize_live_disparity_map(const cv::Mat &disparity_map, int image_width, const cv::Point2i &closest_point_2D, const cv::Point3d &closest_point_3D);
    void compute_3D_points_from_disparity(const cv::Mat &disparity_map, const cv::Mat &Q, std::vector<cv::Point3d> &points_3D, std::vector<cv::Point2d> &points_2D, double &average_depth);
    bool compute_3D_points_from_features(const cv::Ptr<cv::DescriptorMatcher> &matcher, const cv::Mat &P_L, const std::vector<cv::KeyPoint> &keypoints_L, const cv::Mat &descriptors_L, const cv::Mat &P_R, const std::vector<cv::KeyPoint> &keypoints_R, const cv::Mat &descriptors_R, std::vector<cv::Point3d> &points_3D, std::vector<cv::Point2d> &points_2D, double &average_depth);
    void load_stereo_camera_parameters(const std::string &file_path, cv::Mat &camera_matrix_L, cv::Mat &dist_coeffs_L, cv::Mat &map_1_L, cv::Mat &map_2_L, cv::Mat &P_L, cv::Mat &camera_matrix_R, cv::Mat &dist_coeffs_R, cv::Mat &map_1_R, cv::Mat &map_2_R, cv::Mat &P_R, cv::Mat &T, cv::Mat &Q);
    cv::Ptr<cv::StereoBM> create_default_stereo_matcher();
    cv::Ptr<cv::StereoSGBM> create_default_stereo_SG_matcher();
} // namespace sp

#endif // STEREO_PROCESSING_HPP