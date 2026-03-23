#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("stereo_camera_calibrator");
    bool parameter_error = false;

    node->declare_parameter<std::string>("chessboard_images_L", "");
    std::string images_dir_L = node->get_parameter("chessboard_images_L").as_string();

    node->declare_parameter<std::string>("chessboard_images_R", "");
    std::string images_dir_R = node->get_parameter("chessboard_images_R").as_string();

    node->declare_parameter<std::vector<int64_t>>("inner_corners", std::vector<int64_t>{});
    std::vector<int64_t> inner_corners = node->get_parameter("inner_corners").as_integer_array();

    node->declare_parameter<std::string>("output_folder", "");
    std::string output_dir = node->get_parameter("output_folder").as_string();

    if (!std::filesystem::path(images_dir_L).is_absolute() || !std::filesystem::is_directory(images_dir_L))
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'chessboard_images_L' is invalid or not provided. It must be an absolute path to an existing directory.");
        parameter_error = true;
    }

    if (!std::filesystem::path(images_dir_R).is_absolute() || !std::filesystem::is_directory(images_dir_R))
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'chessboard_images_R' is invalid or not provided. It must be an absolute path to an existing directory.");
        parameter_error = true;
    }

    if (inner_corners.size() != 2 || inner_corners[0] <= 0 || inner_corners[1] <= 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'inner_corners' is invalid or not provided. It must be a pair of values > 0.");
        parameter_error = true;
    }

    if (!std::filesystem::path(output_dir).is_absolute() || !std::filesystem::is_directory(output_dir))
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'output_folder' is invalid or not provided. It must be an absolute path to an existing directory.");
        parameter_error = true;
    }

    if (parameter_error)
    {
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Finding chessboard corners...");

    // Set chessboard size
    int inner_corners_v = inner_corners[0]; // vertical direction
    int inner_corners_h = inner_corners[1]; // horizontal direction

    // 3D coordinates of chessboard corners for each image
    std::vector<std::vector<cv::Point3f>> all_corners_3D;

    // Pixel coordinates of chessboard corners for each image
    std::vector<std::vector<cv::Point2f>> all_corners_2D_L, all_corners_2D_R;

    // 3D coordinates of chessboard corners (single image)
    std::vector<cv::Point3f> corners_3D;
    float d = 0.021; // distance between corners in [m]
    for (int i = 0; i < inner_corners_h; i++)
    {
        for (int j = 0; j < inner_corners_v; j++)
            corners_3D.push_back(cv::Point3f(j * d, i * d, 0.0));
    }

    std::vector<cv::String> image_paths_L, image_paths_R;
    std::string pattern_L = images_dir_L + "/*.jpg";
    std::string pattern_R = images_dir_R + "/*.jpg";

    // Get paths to images
    cv::glob(pattern_L, image_paths_L);
    cv::glob(pattern_R, image_paths_R);

    // Pixel coordinates of chessboard corners (single image)
    std::vector<cv::Point2f> corners_2D_L, corners_2D_R;

    cv::Mat image_L, image_R, image_gray_L, image_gray_R;
    bool success_L, success_R;

    // Find chessboard corners
    for (size_t i = 0; i < image_paths_L.size(); i++)
    {
        image_L = cv::imread(image_paths_L[i]);
        cv::cvtColor(image_L, image_gray_L, cv::COLOR_BGR2GRAY);

        image_R = cv::imread(image_paths_R[i]);
        cv::cvtColor(image_R, image_gray_R, cv::COLOR_BGR2GRAY);

        success_L = cv::findChessboardCorners(image_gray_L, cv::Size(inner_corners_v, inner_corners_h), corners_2D_L);
        success_R = cv::findChessboardCorners(image_gray_R, cv::Size(inner_corners_v, inner_corners_h), corners_2D_R);

        if (success_L && success_R)
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // Refine pixel coordinates
            cv::cornerSubPix(image_gray_L, corners_2D_L, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cv::cornerSubPix(image_gray_R, corners_2D_R, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            cv::drawChessboardCorners(image_L, cv::Size(inner_corners_v, inner_corners_h), corners_2D_L, success_L);
            cv::drawChessboardCorners(image_R, cv::Size(inner_corners_v, inner_corners_h), corners_2D_R, success_R);

            all_corners_3D.push_back(corners_3D);
            all_corners_2D_L.push_back(corners_2D_L);
            all_corners_2D_R.push_back(corners_2D_R);
        }

        // Display the detected corners
        cv::Mat image_L_R;
        cv::hconcat(image_L, image_R, image_L_R);
        cv::imshow("Image", image_L_R);
        std::cout << std::filesystem::path(image_paths_L[i]).filename().string() << std::endl;
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    char input;

    std::cout << "Start calibration? (y/n): ";

    while (rclcpp::ok()) {
        std::cin >> input;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        
        input = std::tolower(input);

        if (input == 'y' || input == 'n') {
            break;
        } else {
            std::cout << "Invalid input. Enter 'y' or 'n': ";
        }
    }

    if (input == 'n') {
        rclcpp::shutdown();
        return 0;
    }

    cv::Mat camera_matrix_L, dist_coeffs_L;
    cv::Mat camera_matrix_R, dist_coeffs_R;
    cv::Mat opt_camera_matrix_L, opt_camera_matrix_R;
    std::vector<cv::Mat> R_L, T_L, R_R, T_R;

    RCLCPP_INFO(node->get_logger(), "Calibrating...");

    // Calibrate cameras
    cv::calibrateCamera(all_corners_3D, all_corners_2D_L, cv::Size(image_gray_L.rows, image_gray_L.cols), camera_matrix_L, dist_coeffs_L, R_L, T_L);
    opt_camera_matrix_L = cv::getOptimalNewCameraMatrix(camera_matrix_L, dist_coeffs_L, image_gray_L.size(), 1, image_gray_L.size(), 0);

    cv::calibrateCamera(all_corners_3D, all_corners_2D_R, cv::Size(image_gray_R.rows, image_gray_R.cols), camera_matrix_R, dist_coeffs_R, R_R, T_R);
    opt_camera_matrix_R = cv::getOptimalNewCameraMatrix(camera_matrix_R, dist_coeffs_R, image_gray_R.size(), 1, image_gray_R.size(), 0);

    // Compute re-projection error
    double mean_error_L = 0.0;
    double mean_error_R = 0.0;

    for (size_t i = 0; i < all_corners_3D.size(); i++)
    {
        std::vector<cv::Point2f> proj_corners_L, proj_corners_R;
        cv::projectPoints(all_corners_3D[i], R_L[i], T_L[i], camera_matrix_L, dist_coeffs_L, proj_corners_L);
        cv::projectPoints(all_corners_3D[i], R_R[i], T_R[i], camera_matrix_R, dist_coeffs_R, proj_corners_R);

        double error_L = norm(all_corners_2D_L[i], proj_corners_L, cv::NORM_L2) / proj_corners_L.size();
        double error_R = norm(all_corners_2D_R[i], proj_corners_R, cv::NORM_L2) / proj_corners_R.size();
        mean_error_L += error_L;
        mean_error_R += error_R;
    }

    mean_error_L /= all_corners_3D.size();
    mean_error_R /= all_corners_3D.size();
    RCLCPP_INFO(node->get_logger(), "Mean re-projection error (left camera): %f", mean_error_L);
    RCLCPP_INFO(node->get_logger(), "Mean re-projection error (right camera): %f", mean_error_R);

    cv::Mat R, T, E, F;

    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    // Compute transformation between the two cameras
    cv::stereoCalibrate(all_corners_3D,
                        all_corners_2D_L,
                        all_corners_2D_R,
                        opt_camera_matrix_L,
                        dist_coeffs_L,
                        opt_camera_matrix_R,
                        dist_coeffs_R,
                        image_gray_R.size(),
                        R,
                        T,
                        E,
                        F,
                        flag,
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

    cv::Mat rect_L, rect_R, proj_mat_L, proj_mat_R, Q;

    // Perform stereo rectification
    cv::stereoRectify(opt_camera_matrix_L,
                      dist_coeffs_L,
                      opt_camera_matrix_R,
                      dist_coeffs_R,
                      image_gray_R.size(),
                      R,
                      T,
                      rect_L,
                      rect_R,
                      proj_mat_L,
                      proj_mat_R,
                      Q,
                      cv::CALIB_ZERO_DISPARITY,
                      1);

    // Compute the undistortion and rectification maps for left and right camera frames
    cv::Mat map_1_L, map_2_L;
    cv::Mat map_1_R, map_2_R;

    cv::initUndistortRectifyMap(opt_camera_matrix_L,
                                dist_coeffs_L,
                                rect_L,
                                proj_mat_L,
                                image_gray_L.size(),
                                CV_16SC2,
                                map_1_L,
                                map_2_L);

    cv::initUndistortRectifyMap(opt_camera_matrix_R,
                                dist_coeffs_R,
                                rect_R,
                                proj_mat_R,
                                image_gray_R.size(),
                                CV_16SC2,
                                map_1_R,
                                map_2_R);

    // Write camera params to yaml files
    cv::FileStorage fs(output_dir + "/stereo_camera_params.yaml", cv::FileStorage::WRITE);

    if (fs.isOpened())
    {
        fs << "camera_matrix_L" << opt_camera_matrix_L;
        fs << "dist_coeffs_L" << dist_coeffs_L;
        fs << "map_1_L" << map_1_L;
        fs << "map_2_L" << map_2_L;
        fs << "P_L" << proj_mat_L;


        fs << "camera_matrix_R" << opt_camera_matrix_R;
        fs << "dist_coeffs_R" << dist_coeffs_R;
        fs << "map_1_R" << map_1_R;
        fs << "map_2_R" << map_2_R;
        fs << "P_R" << proj_mat_R;

        fs << "T" << T;
        fs << "Q" << Q;

        fs.release();
    }

    RCLCPP_INFO(node->get_logger(), "Cameras calibrated successfully");

    rclcpp::shutdown();
    return 0;
}