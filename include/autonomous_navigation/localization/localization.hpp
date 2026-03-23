#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <opencv2/opencv.hpp>

namespace loc
{
    void filter_points_with_RANSAC(const std::vector<cv::Point2d> &points_1, const std::vector<cv::Point2d> &points_2, const cv::Mat& camera_matrix, std::vector<cv::Point2d> &points_1_filtered, std::vector<cv::Point2d> &points_2_filtered, double confidence = 0.99, double reproj_threshold = 1.0);
    bool compute_local_pose(cv::Mat camera_matrix, cv::Mat dist_coeffs, const cv::Ptr<cv::DescriptorMatcher> &matcher, const std::vector<cv::KeyPoint> &keypoints_prev, const cv::Mat &descriptors_prev, const std::vector<cv::KeyPoint> &keypoints, const cv::Mat &descriptors, const std::vector<cv::Point2d> &points_2D, const std::vector<cv::Point3d> &points_3D, std::vector<cv::Point2d> &points_2D_filtered, std::vector<cv::Point3d> &points_3D_filtered, cv::Mat &rvec, cv::Mat &tvec);
    void draw_path(const std::vector<cv::Point2d> &path_points, cv::Mat &image);
} // namespace sp

#endif // LOCALIZATION_HPP