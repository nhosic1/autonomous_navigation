#include <opencv2/opencv.hpp>
#include <filesystem>
#include <cmath>
#include <opencv2/ximgproc.hpp>
#include "autonomous_navigation/stereo_processing.hpp"

namespace sp
{
    void onMouse(int event, int x, int y, int flags, void *userdata)
    {
        (void)flags;
        (void)userdata;

        if (event == cv::EVENT_MOUSEMOVE)
        {
            std::cout << "Mouse Position: (" << x << ", " << y << ")" << std::endl;
        }
    }

    void print_mat_type(const cv::Mat &mat, const std::string mat_name)
    {
        int type = mat.type();
        int depth = CV_MAT_DEPTH(type);
        int channels = CV_MAT_CN(type);

        std::string mat_type;

        switch (depth)
        {
        case CV_8U:
            mat_type = "CV_8U";
            break;
        case CV_8S:
            mat_type = "CV_8S";
            break;
        case CV_16U:
            mat_type = "CV_16U";
            break;
        case CV_16S:
            mat_type = "CV_16S";
            break;
        case CV_32S:
            mat_type = "CV_32S";
            break;
        case CV_32F:
            mat_type = "CV_32F";
            break;
        case CV_64F:
            mat_type = "CV_64F";
            break;
        default:
            mat_type = "Unknown";
            break;
        }

        if (mat_type != "Unknown")
        {
            mat_type += "C";
            mat_type += (channels + '0');
        }

        std::cout << "Mat '" << mat_name << "' type: " << mat_type << std::endl;
    }

    void write_cv_mat_to_yaml(const cv::Mat &mat, const std::string &path)
    {
        // Open the YAML file for writing
        cv::FileStorage fs(path, cv::FileStorage::WRITE);

        if (fs.isOpened())
        {
            // Write the cv::Mat object to the YAML file
            fs << "mat" << mat;

            // Close the file
            fs.release();
        }
    }

    int count_valid_disparities(const cv::Mat &disparity_map)
    {
        cv::Mat mask = (disparity_map > MIN_DISPARITY * 16);
        int count = cv::countNonZero(mask);
        return count;
    }

    void format_disp_map_for_visualization(const cv::Mat &disparity_map, cv::Mat &output_disparity_map, const float &scale, bool inverse_values = false)
    {
        cv::Mat unfiltered_disparity_map = disparity_map.clone();
        if (inverse_values)
        {
            // Inverse disparity values for right-to-left disparity map
            for (int i = 0; i < unfiltered_disparity_map.rows; ++i)
            {
                for (int j = 0; j < unfiltered_disparity_map.cols; ++j)
                {
                    unfiltered_disparity_map.at<int16_t>(i, j) *= -1;
                    if (unfiltered_disparity_map.at<int16_t>(i, j) > NUM_DISPARITIES * 16)
                    {
                        unfiltered_disparity_map.at<int16_t>(i, j) = 0;
                    }
                }
            }
        }

        // Filter invalid disparities
        cv::Mat filtered_disparity_map;
        filter_invalid_disparities(unfiltered_disparity_map, filtered_disparity_map);

        // Scale the disparity map and convert it to CV_8UC1
        cv::Mat scaled_disparity_map;
        cv::convertScaleAbs(filtered_disparity_map, scaled_disparity_map, 255.0 / (NUM_DISPARITIES * 16.0));

        // Apply a color map to the scaled disparity map (red for closer objects, blue for farther objects)
        cv::Mat colored_disparity_map;
        cv::applyColorMap(scaled_disparity_map, colored_disparity_map, cv::COLORMAP_JET);

        // Resize disparity map
        cv::Mat resized_colored_disparity_map;
        cv::resize(colored_disparity_map, resized_colored_disparity_map, cv::Size(), scale, scale);

        output_disparity_map = resized_colored_disparity_map;
    }

    void display_disparity_map(const cv::Mat &map, std::string name, bool inverse)
    {
        int max_image_width = 1280;
        float scale = static_cast<float>(max_image_width) / map.cols;

        cv::Mat formatted_map;
        format_disp_map_for_visualization(map, formatted_map, scale, inverse);

        cv::namedWindow(name);

        // Set the mouse callback for showing pixel coordinates
        cv::setMouseCallback(name, onMouse);

        cv::imshow(name, formatted_map);
        cv::waitKey(0);
    }

    void draw_stereo_match(const cv::Mat &left_img, const cv::Mat &right_img, const cv::Mat &disparity_map, const cv::Point2i &point, int block_size)
    {
        // Calculate the half-length of one side of the rectangle
        int half_side_length = block_size / 2;

        // Draw the point in the left image
        cv::circle(left_img, point, 8, cv::Scalar(0, 255, 0), 2); // Green circle

        // Draw the searching block
        cv::Point top_left1(point.x - half_side_length, point.y - half_side_length);
        cv::Point bottom_right1(point.x + half_side_length, point.y + half_side_length);
        cv::rectangle(left_img, top_left1, bottom_right1, cv::Scalar(0, 0, 255), 2); // Red rectangle

        // Calculate corresponding point in the right image based on disparity map
        int16_t disparity_scaled = disparity_map.at<int16_t>(point.y, point.x);
        float disparity = disparity_scaled / 16.0f;
        cv::Point2i corresponding_point(point.x - int(disparity), point.y);

        // Draw the corresponding point in the right image
        cv::circle(right_img, corresponding_point, 8, cv::Scalar(0, 255, 0), 2); // Green circle

        // Draw the searching block
        cv::Point top_left2(corresponding_point.x - half_side_length, corresponding_point.y - half_side_length);
        cv::Point bottom_right2(corresponding_point.x + half_side_length, corresponding_point.y + half_side_length);
        cv::rectangle(right_img, top_left2, bottom_right2, cv::Scalar(0, 0, 255), 2); // Red rectangle

        // Display both images
        cv::imshow("Left Image with Point of Interest", left_img);
        cv::imshow("Right Image with Corresponding Point", right_img);
        cv::waitKey(0);
    }

    void filter_invalid_disparities(const cv::Mat &disparity_map, cv::Mat &output_disparity_map)
    {
        output_disparity_map = disparity_map.clone();
        cv::Mat mask;

        mask = (disparity_map < MIN_DISPARITY * 16);

        // Apply the mask to the disparity map
        int replacement_disparity = 0;
        output_disparity_map.setTo(replacement_disparity * 16, mask);
    }

    cv::Ptr<cv::StereoBM> create_default_stereo_matcher()
    {
        cv::Ptr<cv::StereoBM> matcher = cv::StereoBM::create();
        matcher->setNumDisparities(NUM_DISPARITIES);
        matcher->setBlockSize(11);
        matcher->setMinDisparity(MIN_DISPARITY);
        matcher->setUniquenessRatio(10);
        matcher->setTextureThreshold(30);
        matcher->setSpeckleRange(10);
        matcher->setSpeckleWindowSize(200);
        // matcher->setPreFilterType(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE);
        matcher->setPreFilterSize(5);
        matcher->setPreFilterCap(32);
        matcher->setSmallerBlockSize(3);

        return matcher;
    }

    cv::Ptr<cv::StereoSGBM> create_default_stereo_SG_matcher()
    {
        int block_size = 10;
        cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create();
        matcher->setNumDisparities(NUM_DISPARITIES);
        matcher->setBlockSize(block_size);
        matcher->setMinDisparity(MIN_DISPARITY);
        matcher->setUniquenessRatio(30);
        matcher->setSpeckleRange(15);
        matcher->setSpeckleWindowSize(400);
        matcher->setDisp12MaxDiff(1);
        matcher->setP1(8 * block_size * block_size);
        matcher->setP2(32 * block_size * block_size);
        matcher->setPreFilterCap(31);

        return matcher;
    }

    // Accepts CV_8UC3 color images
    void apply_wls_filter(const cv::Mat &disparity_map, cv::Mat &filtered_disparity_map, const cv::Mat &left_img, const cv::Mat &right_img, const cv::Ptr<cv::StereoMatcher> &left_matcher)
    {
        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_img, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_img, right_gray, cv::COLOR_BGR2GRAY);

        cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

        cv::Mat right_disparity_map;
        right_matcher->compute(right_gray, left_gray, right_disparity_map);

        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        wls_filter->setLambda(8000.0);
        wls_filter->setSigmaColor(1.5);
        wls_filter->setDepthDiscontinuityRadius(5);
        wls_filter->filter(disparity_map, left_img, filtered_disparity_map, right_disparity_map); // Data type of filtered_disparity_map is CV_16SC1
    }

    // Accepts CV_8UC3 color images
    cv::Mat compute_disparity_map_with_consistency_check(const cv::Mat &left_img, const cv::Mat &right_img, bool wls_filter)
    {
        cv::Mat left_gray_img, right_gray_img;
        cv::cvtColor(left_img, left_gray_img, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_img, right_gray_img, cv::COLOR_BGR2GRAY);

        const int small_block_size = 7;
        const int large_block_size = 51;

        cv::Ptr<cv::StereoBM> matcher = sp::create_default_stereo_matcher();
        cv::Mat disparity_map_small, disparity_map_large;

        // Compute disparity map with small block size
        matcher->setBlockSize(small_block_size);
        matcher->compute(left_gray_img, right_gray_img, disparity_map_small);

        // Compute disparity map with large block size
        matcher->setBlockSize(large_block_size);
        matcher->compute(left_gray_img, right_gray_img, disparity_map_large);

        // Convert disparity maps to CV_32F
        disparity_map_small.convertTo(disparity_map_small, CV_32F, 1.0 / 16.0);
        disparity_map_large.convertTo(disparity_map_large, CV_32F, 1.0 / 16.0);

        // Initialize the final disparity map
        const float invalid_disp = -1.0f;
        cv::Mat disparity_map = cv::Mat(left_gray_img.size(), CV_32F, cv::Scalar(invalid_disp));

        // Consistency checks
        const float consistency_threshold = 30.0f;
        for (int y = 0; y < disparity_map.rows; ++y)
        {
            for (int x = 0; x < disparity_map.cols; ++x)
            {
                float disp_small = disparity_map_small.at<float>(y, x);
                float disp_large = disparity_map_large.at<float>(y, x);

                // Check disparity validation and block size consistency
                if (disp_small > 0 && disp_large > 0 && std::abs(disp_small - disp_large) <= consistency_threshold)
                {
                    disparity_map.at<float>(y, x) = disp_small;
                }
            }
        }

        // Convert the final disparity map back to CV_16S
        disparity_map.convertTo(disparity_map, CV_16S, 16.0);

        // Filter remaining speckles
        cv::filterSpeckles(disparity_map, -16, 200, 16);

        // Apply median filter to smooth disparity map
        cv::medianBlur(disparity_map, disparity_map, 3);

        if (wls_filter)
        {
            // Use the same matcher parameters used for computing disparity map with small block size
            matcher->setBlockSize(small_block_size);

            apply_wls_filter(disparity_map, disparity_map, left_img, right_img, matcher);
        }

        return disparity_map;
    }

    // Expects disparity_map data type to be CV_16SC1
    void visualize_live_disparity_map(const cv::Mat &disparity_map, int image_width, const cv::Point2i &closest_point_2D, const cv::Point3d &closest_point_3D)
    {
        cv::Mat formatted_disparity_map;
        format_disp_map_for_visualization(disparity_map, formatted_disparity_map, 1.0);

        if (closest_point_3D.z < 10000)
        {
            cv::circle(formatted_disparity_map, closest_point_2D, 8, cv::Scalar(255, 0, 255), -1);

            // Convert points from [mm] to [m] and set precision to 2 decimal places for text output
            std::stringstream ss_x, ss_y, ss_z;
            ss_x << std::fixed << std::setprecision(2) << closest_point_3D.x / 1000.0;
            ss_y << std::fixed << std::setprecision(2) << closest_point_3D.y / 1000.0;
            ss_z << std::fixed << std::setprecision(2) << closest_point_3D.z / 1000.0;

            std::string text = "(" + ss_x.str() + ", " + ss_y.str() + ", " + ss_z.str() + ")";

            // Define the font parameters for the point text
            int font_face = cv::FONT_HERSHEY_SIMPLEX; // Font type
            float font_scale = 0.9;                   // Font scale factor
            cv::Scalar color(255, 0, 255);            // Text color (BGR format)
            int thickness = 2;                        // Thickness of the text

            // Calculate the size of the text bounding box
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
            cv::Point org(closest_point_2D.x - textSize.width / 2, closest_point_2D.y + textSize.height + 25); // Position of the text (below the reprojected point)

            cv::putText(formatted_disparity_map, text, org, font_face, font_scale, color, thickness);
        }

        cv::Mat resized_disparity_map;
        float scale = static_cast<float>(image_width) / formatted_disparity_map.cols;
        cv::resize(formatted_disparity_map, resized_disparity_map, cv::Size(), scale, scale);

        // Display the colored disparity map
        cv::imshow("Disparity Map", resized_disparity_map);
        cv::waitKey(1); // Wait for a key press (1 millisecond)
    }

    // Output unit: [mm]
    // Output 3D points are represented in left camera's coordinate system
    // Output 2D points are pixels in left image that correspond to 3D points
    void compute_3D_points_from_disparity(const cv::Mat &disparity_map, const cv::Mat &Q, std::vector<cv::Point3d> &points_3D, std::vector<cv::Point2d> &points_2D, double &average_depth)
    {
        double f = Q.at<double>(2, 3);        // unit: [mm]
        double cx = -Q.at<double>(0, 3);      // unit: [px]
        double cy = -Q.at<double>(1, 3);      // unit: [px]
        double Tx = 1.0 / Q.at<double>(3, 2); // unit: [mm] (value must be positive)
        double total_depth = 0.0;

        // Compute 3D points from disparity map
        for (int y = 0; y < disparity_map.rows; y++)
        {
            for (int x = 0; x < disparity_map.cols; x++)
            {
                int16_t disparity_scaled = disparity_map.at<int16_t>(y, x);
                double disparity = disparity_scaled / 16.0;
                if (disparity > MIN_DISPARITY)
                {
                    // Calculate 3D point (X, Y, Z) using the disparity and camera parameters
                    double Z = (f * Tx) / disparity;
                    double X = (x - cx) * Z / f;
                    double Y = (y - cy) * Z / f;

                    points_3D.push_back(cv::Point3d(X, Y, Z));
                    points_2D.push_back(cv::Point2d(x, y));
                    total_depth += Z;
                }
            }
        }
        average_depth = total_depth / points_3D.size();
    }

    // Output unit: [mm]
    // Output 3D points are represented in left camera's coordinate system
    // Output 2D points are pixels in left image that correspond to 3D points
    bool compute_3D_points_from_features(const cv::Ptr<cv::DescriptorMatcher> &matcher, const cv::Mat &P_L, const std::vector<cv::KeyPoint> &keypoints_L, const cv::Mat &descriptors_L, const cv::Mat &P_R, const std::vector<cv::KeyPoint> &keypoints_R, const cv::Mat &descriptors_R, std::vector<cv::Point3d> &points_3D, std::vector<cv::Point2d> &points_2D, double &average_depth)
    {
        std::vector<std::vector<cv::DMatch>> matches;

        // Find matches
        if (!descriptors_L.empty() && !descriptors_R.empty())
        {
            matcher->knnMatch(descriptors_L, descriptors_R, matches, 2);
        }
        else
        {
            return false;
        }

        // Filter matches using Lowe's ratio test
        const float ratio_threshold = 0.75f;
        std::set<int> unique_train_ids;
        std::vector<cv::Point2f> points_L, points_R;

        for (size_t i = 0; i < matches.size(); i++)
        {
            if ((matches[i].size() == 1) || (matches[i].size() > 1 && matches[i][0].distance < ratio_threshold * matches[i][1].distance))
            {
                cv::Point2f pt_L = keypoints_L[matches[i][0].queryIdx].pt;
                cv::Point2f pt_R = keypoints_R[matches[i][0].trainIdx].pt;

                // Check if y-coordinates are approximately equal, minimum stereo disparity is 7 and match is 1-to-1
                if (std::abs(pt_L.y - pt_R.y) < 2.0 && (pt_L.x - pt_R.x >= 7.0) && unique_train_ids.insert(matches[i][0].trainIdx).second)
                {
                    points_L.push_back(pt_L);
                    points_R.push_back(pt_R);
                    points_2D.push_back(cv::Point2d(pt_L.x, pt_L.y));
                }
            }
        }

        // Triangulate points
        cv::Mat points_4D;
        if (!points_L.empty() && !points_R.empty())
        {
            // All input data should be of float type in order for cv::triangulatePoints() to work.
            cv::Mat P_L_f, P_R_f;
            P_L.convertTo(P_L_f, CV_32F);
            P_R.convertTo(P_R_f, CV_32F);
            cv::triangulatePoints(P_L_f, P_R_f, points_L, points_R, points_4D);
        }
        else
        {
            return false;
        }

        double total_depth = 0.0;

        // Normalize homogeneous coordinates
        for (int i = 0; i < points_4D.cols; i++)
        {
            cv::Mat x = points_4D.col(i);
            x /= x.at<float>(3);
            points_3D.push_back(cv::Point3d(x.at<float>(0), x.at<float>(1), x.at<float>(2)));
            total_depth += static_cast<double>(x.at<float>(2));
        }

        average_depth = total_depth / points_3D.size();

        return true;
    }

    void load_stereo_camera_parameters(const std::string &file_path, cv::Mat &camera_matrix_L, cv::Mat &dist_coeffs_L, cv::Mat &map_1_L, cv::Mat &map_2_L, cv::Mat &P_L, cv::Mat &camera_matrix_R, cv::Mat &dist_coeffs_R, cv::Mat &map_1_R, cv::Mat &map_2_R, cv::Mat &P_R, cv::Mat &T, cv::Mat &Q)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cerr << "Failed to open YAML file: " << file_path << std::endl;
            return;
        }

        // Load camera matrices
        fs["camera_matrix_L"] >> camera_matrix_L;
        fs["camera_matrix_R"] >> camera_matrix_R;
        camera_matrix_L.convertTo(camera_matrix_L, CV_64F);
        camera_matrix_R.convertTo(camera_matrix_R, CV_64F);

        // Load distortion coefficients
        fs["dist_coeffs_L"] >> dist_coeffs_L;
        fs["dist_coeffs_R"] >> dist_coeffs_R;
        dist_coeffs_L.convertTo(dist_coeffs_L, CV_64F);
        dist_coeffs_R.convertTo(dist_coeffs_R, CV_64F);

        // Load undistortion and rectification maps
        fs["map_1_L"] >> map_1_L;
        fs["map_1_R"] >> map_1_R;
        map_1_L.convertTo(map_1_L, CV_16SC2);
        map_1_R.convertTo(map_1_R, CV_16SC2);

        fs["map_2_L"] >> map_2_L;
        fs["map_2_R"] >> map_2_R;
        map_2_L.convertTo(map_2_L, CV_16UC1);
        map_2_R.convertTo(map_2_R, CV_16UC1);

        // Load projection matrices
        fs["P_L"] >> P_L;
        fs["P_R"] >> P_R;
        P_L.convertTo(P_L, CV_64F);
        P_R.convertTo(P_R, CV_64F);

        // Load translation vector
        fs["T"] >> T;
        T.convertTo(T, CV_64F);

        // Load disparity-to-depth mapping matrix
        fs["Q"] >> Q;
        Q.convertTo(T, CV_64F);

        fs.release();
    }

} // namespace sp