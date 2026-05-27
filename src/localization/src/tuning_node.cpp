#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

using namespace cv;
using std::cout;
using std::endl;

// ---------------- TUNING PARAMETERS (Edit these) ----------------
// Camera intrinsics - these are what you're trying to tune/verify
static Mat cameraMatrix = (Mat_<double>(3,3) <<
    1.25649815e+03, 0., 7.12996774e+02,
    0., 1.25820533e+03, 4.69551858e+02,
    0., 0., 1.);

static Mat distCoeffs = (Mat_<double>(1,5) <<
    -3.72271817e-03, 5.33786890e-01, -4.99625728e-04, -1.65101232e-03, -1.78505927e+00);

// ArUco marker physical size in meters (measure your printed marker!)
static const float MARKER_SIZE_METERS = 0.15f;

// Checkerboard calibration parameters
static const Size CHECKERBOARD_SIZE(9, 6);  // Inner corners (width, height)
static const float SQUARE_SIZE_METERS = 0.025f;  // Size of each square in meters


class TuningNode : public rclcpp::Node {
public:
    TuningNode() : Node("tuning_node") {
        RCLCPP_INFO(this->get_logger(), "Camera Tuning Node Started");
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "Available modes:");
        RCLCPP_INFO(this->get_logger(), "  1. Test ArUco pose estimation with current calibration");
        RCLCPP_INFO(this->get_logger(), "  2. Capture checkerboard images for calibration");
        RCLCPP_INFO(this->get_logger(), "  3. Live checkerboard corner detection test");
        RCLCPP_INFO(this->get_logger(), "==========================================");
    }
    
    // Test ArUco marker detection and pose estimation
    void testArucoPose() {
        VideoCapture cap(0);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera!");
            return;
        }
        
        // Create ArUco dictionary (compatible with OpenCV 4.x)
        auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        auto detectorParams = cv::aruco::DetectorParameters::create();
        
        RCLCPP_INFO(this->get_logger(), "Testing ArUco pose estimation...");
        RCLCPP_INFO(this->get_logger(), "Press 'q' to quit, 's' to save frame");
        
        Mat frame;
        while (rclcpp::ok()) {
            cap >> frame;
            if (frame.empty()) break;
            
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            
            // Detect markers (older API compatible with OpenCV 4.x)
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams);
            
            if (!markerIds.empty()) {
                // Draw detected markers
                cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
                
                // Estimate pose for each marker
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(markerCorners, MARKER_SIZE_METERS,
                                                     cameraMatrix, distCoeffs, rvecs, tvecs);
                
                // Draw axis for each marker
                for (size_t i = 0; i < markerIds.size(); i++) {
                    cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, 
                                     rvecs[i], tvecs[i], MARKER_SIZE_METERS * 0.5f);
                    
                    // Calculate and display distance
                    double distance = cv::norm(tvecs[i]);
                    
                    // Get camera position relative to tag
                    Mat R;
                    cv::Rodrigues(rvecs[i], R);
                    Mat R_inv = R.t();
                    Mat tvec_mat = (Mat_<double>(3,1) << tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                    Mat cam_pos = -R_inv * tvec_mat;
                    
                    // Display info
                    std::string info = cv::format("ID:%d Dist:%.2fm", markerIds[i], distance);
                    cv::Point2f center(0, 0);
                    for (const auto& pt : markerCorners[i]) center += pt;
                    center *= 0.25f;
                    
                    cv::putText(frame, info, cv::Point(center.x, center.y - 20),
                               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                    
                    // Print detailed pose info
                    RCLCPP_INFO(this->get_logger(),
                               "Marker %d: Distance=%.3fm, Position(x,y,z)=(%.3f,%.3f,%.3f)m, "
                               "Camera relative to tag=(%.3f,%.3f,%.3f)m",
                               markerIds[i], distance,
                               tvecs[i][0], tvecs[i][1], tvecs[i][2],
                               cam_pos.at<double>(0), cam_pos.at<double>(1), cam_pos.at<double>(2));
                }
            }
            
            cv::imshow("ArUco Pose Test (q=quit, s=save)", frame);
            char key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') break;
            if (key == 's' || key == 'S') {
                std::string filename = "aruco_test_" + 
                    std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".jpg";
                cv::imwrite(filename, frame);
                RCLCPP_INFO(this->get_logger(), "Saved: %s", filename.c_str());
            }
        }
        
        cap.release();
        cv::destroyAllWindows();
    }
    
    // Capture checkerboard images for calibration
    void captureCalibrationImages() {
        VideoCapture cap(0);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera!");
            return;
        }
        
        // Create output directory
        system("mkdir -p calibration_images");
        
        RCLCPP_INFO(this->get_logger(), "Checkerboard Calibration Image Capture");
        RCLCPP_INFO(this->get_logger(), "Expected checkerboard: %dx%d inner corners", 
                   CHECKERBOARD_SIZE.width, CHECKERBOARD_SIZE.height);
        RCLCPP_INFO(this->get_logger(), "Instructions:");
        RCLCPP_INFO(this->get_logger(), "  - Press SPACE to capture image when corners are detected");
        RCLCPP_INFO(this->get_logger(), "  - Move checkerboard to different positions/angles");
        RCLCPP_INFO(this->get_logger(), "  - Capture 15-30 images from various viewpoints");
        RCLCPP_INFO(this->get_logger(), "  - Press 'q' to quit");
        
        Mat frame, gray;
        int captureCount = 0;
        
        while (rclcpp::ok()) {
            cap >> frame;
            if (frame.empty()) break;
            
            Mat displayFrame = frame.clone();
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            
            // Find checkerboard corners
            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, CHECKERBOARD_SIZE, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
            
            if (found) {
                // Refine corner positions
                cv::cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
                
                // Draw corners
                cv::drawChessboardCorners(displayFrame, CHECKERBOARD_SIZE, corners, found);
                
                // Show "READY TO CAPTURE" message
                cv::putText(displayFrame, "CORNERS DETECTED - Press SPACE to capture",
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                           cv::Scalar(0, 255, 0), 2);
            } else {
                cv::putText(displayFrame, "Move checkerboard into view",
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                           cv::Scalar(0, 0, 255), 2);
            }
            
            // Show capture count
            std::string countText = cv::format("Captured: %d images", captureCount);
            cv::putText(displayFrame, countText, cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            
            cv::imshow("Calibration Capture (SPACE=capture, q=quit)", displayFrame);
            
            char key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') {
                break;
            } else if (key == ' ' && found) {
                // Save image
                std::string filename = cv::format("calibration_images/calib_%03d.jpg", captureCount);
                cv::imwrite(filename, frame);
                captureCount++;
                RCLCPP_INFO(this->get_logger(), "Captured image %d: %s", captureCount, filename.c_str());
                
                // Brief pause and visual feedback
                Mat flashFrame = displayFrame.clone();
                cv::rectangle(flashFrame, cv::Point(0, 0), 
                             cv::Point(flashFrame.cols, flashFrame.rows),
                             cv::Scalar(255, 255, 255), 20);
                cv::imshow("Calibration Capture (SPACE=capture, q=quit)", flashFrame);
                cv::waitKey(200);
            }
        }
        
        cap.release();
        cv::destroyAllWindows();
        
        RCLCPP_INFO(this->get_logger(), "Captured %d images total", captureCount);
        RCLCPP_INFO(this->get_logger(), "Now run: python3 scripts/calibrate_camera.py");
    }
    
    // Live test of checkerboard corner detection
    void liveCheckerboardTest() {
        VideoCapture cap(0);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Live Checkerboard Corner Detection Test");
        RCLCPP_INFO(this->get_logger(), "Expected: %dx%d inner corners", 
                   CHECKERBOARD_SIZE.width, CHECKERBOARD_SIZE.height);
        RCLCPP_INFO(this->get_logger(), "Press 'q' to quit");
        
        Mat frame, gray;
        
        while (rclcpp::ok()) {
            cap >> frame;
            if (frame.empty()) break;
            
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            
            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, CHECKERBOARD_SIZE, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            
            if (found) {
                cv::cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
                cv::drawChessboardCorners(frame, CHECKERBOARD_SIZE, corners, found);
                
                cv::putText(frame, "DETECTED", cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            } else {
                cv::putText(frame, "NOT DETECTED", cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            }
            
            cv::imshow("Checkerboard Detection Test (q=quit)", frame);
            if (cv::waitKey(1) == 'q') break;
        }
        
        cap.release();
        cv::destroyAllWindows();
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TuningNode>();
    
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <mode>\n";
        std::cerr << "Modes:\n";
        std::cerr << "  1 - Test ArUco pose estimation\n";
        std::cerr << "  2 - Capture checkerboard calibration images\n";
        std::cerr << "  3 - Live checkerboard detection test\n";
        return 1;
    }
    
    int mode = atoi(argv[1]);
    
    switch(mode) {
        case 1:
            node->testArucoPose();
            break;
        case 2:
            node->captureCalibrationImages();
            break;
        case 3:
            node->liveCheckerboardTest();
            break;
        default:
            std::cerr << "Invalid mode. Use 1, 2, or 3\n";
            return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
