#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/xfeatures2d.hpp>

// using namespace std;
using namespace cv;

class FeatureMatcher {
private:
    Ptr<Feature2D> m_detector; 
    FlannBasedMatcher m_descriptorMatcher;

public:
    // Constructor
    FeatureMatcher(Ptr<Feature2D> detector, FlannBasedMatcher descriptorMatcher) : m_detector(detector), m_descriptorMatcher(descriptorMatcher) {}

    void showMatches(Mat& image1, std::vector<KeyPoint>& keypoints1, Mat& image2, std::vector<KeyPoint>& keypoints2, std::vector<DMatch>& matches, int maxImageWidth=800){
        // Calculate the scaling factor to maintain aspect ratio
        float scale = static_cast<float>(maxImageWidth) / std::max(image1.cols, image2.cols);

        // Resize img1 and img2 while maintaining aspect ratio
        Mat resizedImg1, resizedImg2;
        resize(image1, resizedImg1, Size(), scale, scale);
        resize(image2, resizedImg2, Size(), scale, scale);

        // Scale keypoints coordinates in img1 and img2
        std::vector<KeyPoint> scaledKeypoints1, scaledKeypoints2;
        for (const auto& kp : keypoints1) {
            scaledKeypoints1.push_back(KeyPoint(kp.pt * scale, kp.size, kp.angle, kp.response, kp.octave, kp.class_id));
        }
        for (const auto& kp : keypoints2) {
            scaledKeypoints2.push_back(KeyPoint(kp.pt * scale, kp.size, kp.angle, kp.response, kp.octave, kp.class_id));
        }
        Mat img_matches;
        drawMatches( resizedImg1, scaledKeypoints1, resizedImg2, scaledKeypoints2, matches, img_matches, Scalar::all(-1),
        Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        // Show detected matches
        imshow("Good Matches", img_matches );
        
        waitKey();
    }

    // Implementation of findMatchingFeatures using detector and FLANN matcher
    std::tuple<std::vector<Point2f>, std::vector<cv::Point2f>> findMatchingFeatures(Mat& img1, Mat& img2, bool showMatches=false) {
        std::vector<KeyPoint> keypoints1, keypoints2; 
        Mat descriptors1, descriptors2;
        std::vector<std::vector<DMatch>> matches;
        m_detector->detectAndCompute(img1, Mat(), keypoints1, descriptors1);
        m_detector->detectAndCompute(img2, Mat(), keypoints2, descriptors2);
        
        m_descriptorMatcher.knnMatch(descriptors1, descriptors2, matches, 2);

        std::vector<Point2f> points1, points2;  // Initialize empty vectors

        if (matches.size() == 0) {
            return std::make_tuple(points1, points2);  // Return empty vectors
        }
        
        // Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f;
        std::vector<DMatch> good_matches;
        for (size_t i = 0; i < matches.size(); i++)
        {
            if (matches[i][0].distance < ratio_thresh * matches[i][1].distance)
            {
                good_matches.push_back(matches[i][0]);
                points1.push_back(keypoints1[matches[i][0].queryIdx].pt);
                points2.push_back(keypoints2[matches[i][0].trainIdx].pt);
            }
        }

        if (showMatches==true) {
            this->showMatches(img1, keypoints1, img2, keypoints2, good_matches);
        }

        return std::make_tuple(points1, points2);  // Return vectors with matching features
    }
};

class FeatureTracker {
public:
    void showOptFlow(Mat& currFrame, std::vector<Point2f>& points1, std::vector<Point2f>& points2, int maxImageWidth=1200){
        // Calculate the scaling factor to maintain aspect ratio
        float scale = static_cast<float>(maxImageWidth) / currFrame.cols;

        // Resize current frame while maintaining aspect ratio
        Mat resizedCurrFrame;
        resize(currFrame, resizedCurrFrame, Size(), scale, scale);

        // Scale feature coordinates
        std::vector<Point2f> scaledPoints1, scaledPoints2;
        for (const auto& p : points1) {
            scaledPoints1.push_back(Point2f(std::round(p.x*scale), std::round(p.y*scale)));
        }
        for (const auto& p : points2) {
            scaledPoints2.push_back(Point2f(p.x*scale, p.y*scale));
        }

        // Create some random colors
        std::vector<Scalar> colors;
        RNG rng;
        for(int i = 0; i < 100; i++){
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            colors.push_back(Scalar(r,g,b));
        }
        Mat mask = Mat::zeros(resizedCurrFrame.size(), resizedCurrFrame.type());
        for(int i = 0; i < scaledPoints1.size(); i++){
            // draw the tracks
            line(mask, scaledPoints2[i], scaledPoints1[i], colors[i], 2);
            circle(resizedCurrFrame, scaledPoints2[i], 5, colors[i], -1);
        }
        Mat img;
        add(resizedCurrFrame, mask, img);
        imshow("Optical Flow", resizedCurrFrame);
        waitKey();
    }

    std::tuple<std::vector<Point2f>, std::vector<cv::Point2f>> detectAndTrackFeatures(Mat& prevFrame, Mat& currFrame, InputArray mask=Mat(), bool showOptFlow=false) {
        Mat prevFrameGray, currFrameGray;
        cvtColor(prevFrame, prevFrameGray, COLOR_BGR2GRAY);
        cvtColor(currFrame, currFrameGray, COLOR_BGR2GRAY);
        Canny(prevFrameGray, prevFrameGray, 10, 30);
        Canny(currFrameGray, currFrameGray, 10, 30);
        std::vector<Point2f> points1, points2;  // Initialize empty vectors
        goodFeaturesToTrack(
            prevFrameGray,   // image
            points1,     // corners
            300,        // maxCorners
            0.6,         // qualityLevel
            10,          // minDistanced
            Mat(),       // mask
            3,           // blockSize
            false,       // useHarrisDetector
            0.04         // k
        );

        if (points1.size() == 0) {
            return std::make_tuple(points1, points2);  // Return empty vectors
        }

        std::vector<uchar> status;
        std::vector<float> err;

        calcOpticalFlowPyrLK(
            prevFrameGray,   // prevImg   
            currFrameGray,   // nextImg
            points1,     // prevPts
            points2,     // nextPts
            status,       // status
            err,       // err
            Size(25, 25),   // winSize
            4,           // maxLevel
            TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 12, (0.03)),   // criteria
            0,           // flags
            1.0E-4       // minEigThreshold
        );

        if (showOptFlow==true) {
            this->showOptFlow(currFrame, points1, points2);
        }

        return std::make_tuple(points1, points2);
    }
};

int main() {
    // Read the image files
    Mat image1 = imread("../data/left01.png");
    Mat image2 = imread("../data/right01.png");

    // Check if the images were successfully loaded
    if (image1.empty() || image2.empty()) {
        std::cerr << "Error: Could not open or find the stereo image pair." << std::endl;
        return -1;
    }

    // Initialize descriptor detectors
    Ptr<ORB> orb = ORB::create(
        3000,    // nfeatures
        1.2f,   // scaleFactor
        8,      // nlevels
        25,     // edgeThreshold
        0,      // firstLevel
        2,      // WTA_K
        cv::ORB::HARRIS_SCORE,  // scoreType
        25,      // patchSize
        10      // fastThreshold
    );
    Ptr<xfeatures2d::SURF> surf = xfeatures2d::SURF::create(20);
    Ptr<SIFT> sift = SIFT::create();

    // Initialize Flann matchers
    FlannBasedMatcher lshDescriptorMatcher(makePtr<flann::LshIndexParams>(6, 12, 1));
    FlannBasedMatcher kdTreeDescriptorMatcher(makePtr<flann::KDTreeIndexParams>(5));

    // Initialize feature matchers
    FeatureMatcher orbMatcher(orb, lshDescriptorMatcher);
    FeatureMatcher surfMatcher(surf, kdTreeDescriptorMatcher);
    FeatureMatcher siftMatcher(sift, kdTreeDescriptorMatcher);

    // Call findMatchingFeatures function and unpack the returned tuple
    // auto [points1, points2] = orbMatcher.findMatchingFeatures(image1, image2, true);
    // auto [points1, points2] = surfMatcher.findMatchingFeatures(image1, image2, true);
    // auto [points1, points2] = siftMatcher.findMatchingFeatures(image1, image2, true);

    // FeatureTracker tracker;
    // auto [points1, points2] = tracker.detectAndTrackFeatures(image1, image2, Mat(), true);

    Mat leftGray, rightGray;
    cvtColor(image1, leftGray, COLOR_BGR2GRAY);
    cvtColor(image2, rightGray, COLOR_BGR2GRAY);
    Ptr<StereoBM> stereo = StereoBM::create(16, 11); // Block size and disparity range
    Mat disparityMap;
    stereo->compute(leftGray, rightGray, disparityMap);
    imshow("gray", disparityMap);
    waitKey();

    return 0;
}
