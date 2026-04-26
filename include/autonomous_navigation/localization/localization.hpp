#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <opencv2/opencv.hpp>
#include <cstddef>
#include <limits>

namespace loc
{
    struct PoseEstimateQuality
    {
        size_t matched_count = 0;
        size_t pnp_candidate_count = 0;
        size_t inlier_count = 0;
        double inlier_ratio = 0.0;
        double reprojection_rmse = std::numeric_limits<double>::infinity();
        double reprojection_max_error = std::numeric_limits<double>::infinity();
    };

    struct PoseEstimate
    {
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
        PoseEstimateQuality quality;
    };

    void filter_points_with_RANSAC(const std::vector<cv::Point2d> &points_1, const std::vector<cv::Point2d> &points_2, const cv::Mat& camera_matrix, std::vector<cv::Point2d> &points_1_filtered, std::vector<cv::Point2d> &points_2_filtered, double confidence = 0.99, double reproj_threshold = 1.0);
    bool compute_local_pose(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, const cv::Ptr<cv::DescriptorMatcher> &matcher, const std::vector<cv::KeyPoint> &keypoints_prev, const cv::Mat &descriptors_prev, const std::vector<cv::KeyPoint> &keypoints, const cv::Mat &descriptors, const std::vector<cv::Point2d> &points_2D, const std::vector<cv::Point3d> &points_3D, std::vector<cv::Point2d> &points_2D_filtered, std::vector<cv::Point3d> &points_3D_filtered, PoseEstimate &pose_estimate);
    void draw_path(const std::vector<cv::Point2d> &path_points, cv::Mat &image);
} // namespace loc

#endif // LOCALIZATION_HPP
