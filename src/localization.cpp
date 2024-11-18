#include <opencv2/opencv.hpp>

namespace loc
{
    void filter_points_with_RANSAC(const std::vector<cv::Point2d> &points_1, const std::vector<cv::Point2d> &points_2, const cv::Mat& camera_matrix, std::vector<cv::Point2d> &points_1_filtered, std::vector<cv::Point2d> &points_2_filtered, double confidence = 0.99, double reproj_threshold = 1.0)
    {
        // Use RANSAC to compute the essential matrix and filter outliers
        std::vector<uchar> inliers_mask;
        cv::Mat E = cv::findEssentialMat(points_1, points_2, camera_matrix, cv::RANSAC, confidence, reproj_threshold, inliers_mask);

        // Clear the output vectors before filling them
        points_1_filtered.clear();
        points_2_filtered.clear();

        // Filter points based on the inliers mask
        for (size_t i = 0; i < points_1.size(); i++) {
            if (inliers_mask[i]) {
                points_1_filtered.push_back(points_1[i]);
                points_2_filtered.push_back(points_2[i]);
            }
        }
    }

    // Accepts descriptor matcher compatible with ORB descriptors
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
        const float ratio_threshold = 0.8f;
        std::set<int> unique_train_ids;
        std::vector<cv::Point2d> points_1, points_2;

        for (size_t i = 0; i < matches.size(); i++)
        {
            if ((matches[i].size() == 1) || (matches[i].size() > 1 && matches[i][0].distance < ratio_threshold * matches[i][1].distance))
            {
                // Ensure 1-to-1 matching
                if (unique_train_ids.insert(matches[i][0].trainIdx).second) 
                {
                    points_1.push_back(keypoints_prev[matches[i][0].queryIdx].pt);
                    points_2.push_back(keypoints[matches[i][0].trainIdx].pt);
                }
            }
        }

        // Apply RANSAC with essential matrix as fitting model
        std::vector<cv::Point2d> points_2D_matched_prev, points_2D_matched;
        filter_points_with_RANSAC(points_1, points_2, camera_matrix, points_2D_matched_prev, points_2D_matched);

        // Convert 3D points from Point3d to Point3f (required by cv::projectPoints())
        std::vector<cv::Point3f> points_3D_f;
        points_3D_f.reserve(points_3D.size());
        for (const auto& point : points_3D) {
            points_3D_f.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z));
        }

        std::vector<cv::Point2f> points_2D_proj;
        cv::projectPoints(points_3D_f, rvec, tvec, camera_matrix, dist_coeffs, points_2D_proj);

        // Find 3D-2D pairs for PnP algorithm
        std::vector<cv::Point2d> points_2D_pnp;
        std::vector<cv::Point3d> points_3D_pnp;
        for (size_t i = 0; i < points_2D.size(); i++)
        {
            const cv::Point2d &point_2D = points_2D[i];
            for (size_t j = 0; j < points_2D_matched.size(); j++)
            {
                const cv::Point2d &point_2D_matched_prev = points_2D_matched_prev[j];
                const cv::Point2d &point_2D_matched = points_2D_matched[j];
                
                if (point_2D == point_2D_matched_prev)
                {
                    // Ignore matched points inconsistent with estimated position
                    const cv::Point2d point_2D_proj(static_cast<double>(points_2D_proj[i].x), static_cast<double>(points_2D_proj[i].y));
                    if (cv::norm(point_2D_matched - point_2D_proj) < 40.0)
                    {
                        points_2D_pnp.push_back(point_2D_matched);
                        points_3D_pnp.push_back(points_3D[i]);
                        break;
                    }
                }
            }
        }

        // Run PnP algorithm to find local pose (rotation and translation vector)
        bool success = false;

        if (points_2D_pnp.size() >= 4)
        {
            // Vectors with Point3d and Point2d objects are expected by cv::solvePnP()
            success = cv::solvePnP(
            points_3D_pnp,         // 3D points from previous iteration
            points_2D_pnp,         // Corresponding 2D points from current frame
            camera_matrix,         // Camera matrix
            dist_coeffs,           // Distortion coefficients
            rvec,                  // Output rotation vector
            tvec,                  // Output translation vector
            true,                  // Use initial guess
            cv::SOLVEPNP_ITERATIVE // Flag (algorithm to use).
            );
        }

        return success;
    }

    void draw_point_coordinates(const cv::Point2d &path_point, double scale, cv::Mat &path_image)
    {
        cv::Point origin = cv::Point(path_image.cols / 2, path_image.rows / 2);
        cv::Point image_point(origin.x + static_cast<int>(path_point.x * scale), origin.y - static_cast<int>(path_point.y * scale));

        cv::circle(path_image, image_point, 8, cv::Scalar(255, 0, 255), -1);

        // Convert points from [mm] to [m] and set precision to 2 decimal places for text output
        std::stringstream ss_x, ss_y, ss_z;
        ss_x << std::fixed << std::setprecision(2) << path_point.x / 1000.0;
        ss_y << std::fixed << std::setprecision(2) << path_point.y / 1000.0;

        std::string text = "(" + ss_x.str() + ", " + ss_y.str() + ")";

        // Define the font parameters for the point text
        int font_face = cv::FONT_HERSHEY_SIMPLEX; // Font type
        float font_scale = 0.8;                   // Font scale factor
        cv::Scalar color(255, 0, 255);            // Text color (BGR format)
        int thickness = 2;                        // Thickness of the text

        // Calculate the size of the text bounding box
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
        cv::Point org(image_point.x - textSize.width / 2, image_point.y + textSize.height + 25); // Position of the text (below the reprojected point)

        cv::putText(path_image, text, org, font_face, font_scale, color, thickness);
    }

    void draw_path(const std::vector<cv::Point2d> &path_points, cv::Mat &image)
    {
        image = cv::Mat(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::Point origin = cv::Point(image.cols / 2, image.rows / 2);

        // Draw the axes
        cv::line(image, cv::Point(origin.x, 0), cv::Point(origin.x, image.rows), cv::Scalar(0, 0, 0), 1);
        cv::line(image, cv::Point(0, origin.y), cv::Point(image.cols, origin.y), cv::Scalar(0, 0, 0), 1);

        double max_x_abs = 0.0;
        double max_y_abs = 0.0;

        for (const auto &point : path_points) {
            if (std::abs(point.x) > max_x_abs) max_x_abs = std::abs(point.x);
            if (std::abs(point.y) > max_y_abs) max_y_abs = std::abs(point.y);
        }

        double max_abs = std::max(max_x_abs, max_y_abs);

        int padding = 50;
        double scale = 1;

        if (max_abs > std::max(origin.x, origin.y) - padding)
        {
            scale = static_cast<double>(std::max(origin.x, origin.y) - padding) / max_abs;
        }

        for (size_t i = 1; i < path_points.size(); ++i)
        {
            int x1 = origin.x + static_cast<int>(path_points[i - 1].x * scale);
            int y1 = origin.y - static_cast<int>(path_points[i - 1].y * scale);
            int x2 = origin.x + static_cast<int>(path_points[i].x * scale);
            int y2 = origin.y - static_cast<int>(path_points[i].y * scale);

            // Draw the line connecting the points (red)
            cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), CV_RGB(255, 0, 0), 2);
        }

        // Draw coordinates for the latest path point
        draw_point_coordinates(path_points.back(), scale, image);
    }
}