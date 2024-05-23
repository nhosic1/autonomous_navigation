#include <opencv2/opencv.hpp>
#include <filesystem>
#include <cmath>
#include <opencv2/ximgproc.hpp>
#include "autonomous_driving/stereo_processing.hpp"

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
        int replacement_disparity = MIN_DISPARITY;
        output_disparity_map.setTo(replacement_disparity * 16, mask);
    }

    cv::Ptr<cv::StereoBM> create_default_stereo_matcher()
    {
        cv::Ptr<cv::StereoBM> matcher = cv::StereoBM::create();
        matcher->setNumDisparities(NUM_DISPARITIES);
        matcher->setBlockSize(9);
        matcher->setMinDisparity(MIN_DISPARITY);
        matcher->setUniquenessRatio(12);
        matcher->setTextureThreshold(50);
        matcher->setSpeckleRange(10);
        matcher->setSpeckleWindowSize(400);
        matcher->setPreFilterType(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE);
        matcher->setPreFilterSize(5);
        matcher->setPreFilterCap(32);
        matcher->setSmallerBlockSize(5);

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

        const int small_block_size = 9;
        const int large_block_size = 71;

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
        const float consistency_threshold = 10.0f;
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
        cv::filterSpeckles(disparity_map, -16, 300, 16);

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
    void visualize_live_disparity_map(const cv::Mat &disparity_map, const cv::Point2i &closest_point_2D, const cv::Point3f &closest_point_3D)
    {
        cv::Mat formatted_disparity_map;
        format_disp_map_for_visualization(disparity_map, formatted_disparity_map, 1.0);

        if (closest_point_3D.z < 10000)
        {
            cv::circle(formatted_disparity_map, closest_point_2D, 10, cv::Scalar(255, 0, 255), -1);

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

            cv::putText(formatted_disparity_map, text, org, fontFace, fontScale, color, thickness);
        }

        cv::Mat resized_disparity_map;
        int max_image_width = 800;
        float scale = static_cast<float>(max_image_width) / formatted_disparity_map.cols;
        cv::resize(formatted_disparity_map, resized_disparity_map, cv::Size(), scale, scale);

        // Display the colored disparity map
        cv::imshow("Disparity Map", resized_disparity_map);
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
                if (disparity > MIN_DISPARITY)
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

    void load_camera_parameters(const std::string &file_path, cv::Mat &camera_matrix, cv::Mat &dist_coeffs)
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