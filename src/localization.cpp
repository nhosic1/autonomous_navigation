#include <opencv2/opencv.hpp>

namespace loc
{
    // Accepts FLANN based matcher
    // Accepts ORB keypoints and descriptors
    // Accepts 2D and 3D points relative to the previous camera position
    // Outputs rvec and tvec have CV_64F data type
    bool compute_local_pose(cv::Mat camera_matrix, cv::Mat dist_coeffs, const cv::Ptr<cv::DescriptorMatcher> &matcher, const std::vector<cv::KeyPoint> &keypoints_prev, const cv::Mat &descriptors_prev, const std::vector<cv::KeyPoint> &keypoints, const cv::Mat &descriptors, const std::vector<cv::Point2d> &points_2D, const std::vector<cv::Point3d> &points_3D, cv::Mat &rvec, cv::Mat &tvec)
    {
        std::vector<std::vector<cv::DMatch>> matches;

        // Find matches
        if (!descriptors_prev.empty() && !descriptors.empty())
        {
            matcher->knnMatch(descriptors_prev, descriptors, matches, 2);
        }
        else 
        {
            return false;
        }

        // Filter matches using Lowe's ratio test
        const float ratio_threshold = 0.65f;
        std::vector<cv::DMatch> good_matches;
        std::set<int> unique_train_ids;

        for (size_t i = 0; i < matches.size(); i++)
        {
            if ((matches[i].size() == 1) || (matches[i].size() > 1 && matches[i][0].distance < ratio_threshold * matches[i][1].distance))
            {
                // Ensure 1-to-1 matching
                if (unique_train_ids.insert(matches[i][0].trainIdx).second) 
                {
                    good_matches.push_back(matches[i][0]);
                }
            }
        }
    
        // Find 3D-2D pairs for PnP algorithm
        std::vector<cv::Point2d> points_2D_pnp;
        std::vector<cv::Point3d> points_3D_pnp;
        for (size_t i = 0; i < points_2D.size(); i++)
        {
            const cv::Point2d &point_2D = points_2D[i];
            // const cv::Point2d point_2D_rounded(std::round(point_2D.x), std::round(point_2D.y));
            for (size_t j = 0; j < good_matches.size(); j++)
            {
                const cv::Point2d &point_2D_matched_prev = keypoints_prev[good_matches[j].queryIdx].pt;
                // const cv::Point2d point_2D_matched_prev_rounded(std::round(point_2D_matched_prev.x), std::round(point_2D_matched_prev.y));
                const cv::Point2d &point_2D_matched = keypoints[good_matches[j].trainIdx].pt;
                
                if (point_2D == point_2D_matched_prev)
                {
                    points_2D_pnp.push_back(point_2D_matched);
                    points_3D_pnp.push_back(points_3D[i]);
                    break;
                }
            }
        }

        // Run PnP algorithm to find local pose (rotation and translation vector)
        bool success = false;

        if (points_2D_pnp.size() >= 4)
        {
            // Vectors with Point3d and Point2d objects are expected by cv::solvePnPRansac()
            success = cv::solvePnPRansac(
                points_3D_pnp,         // 3D points from previous iteration
                points_2D_pnp,         // Corresponding 2D points from current frame
                camera_matrix,         // Camera matrix
                dist_coeffs,           // Distortion coefficients
                rvec,                  // Output rotation vector
                tvec,                  // Output translation vector
                true,                  // initial guess
                100,                   // Number of RANSAC iterations
                8.0,                   // Reprojection error threshold
                0.99,                  // Confidence level
                cv::noArray(),         // Output inlier mask (optional)
                cv::SOLVEPNP_ITERATIVE // Flag (algorithm to use).
            );
        }

        return success;
    }
}