#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <numeric>
#include <fstream>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include "TagDeclaration.hpp"
#include "../include/frame_analyzer.hpp"
#include <bitset>

// #include <opencv2/objdetect/aruco_detector.hpp>

using namespace cv;
using std::cout;
using std::endl;

// ---------------- Camera intrinsics (match your Python) ----------------
static const float fid_size_m = 0.135f; // meters

// Original calibration done on 640x480 landscape image
static const int CALIB_WIDTH = 640;
static const int CALIB_HEIGHT = 480;

static const Mat cameraMatrixCalib = (Mat_<float>(3,3) <<
    5.17082019e+02f, 0.f, 3.16535550e+02f,
    0.f, 5.19695062e+02f, 2.58225643e+02f,
    0.f, 0.f, 1.f);

static const Mat distCoeffs = (Mat_<float>(1,5) <<
    -5.79368652e-02f, -3.04432177e-01f, 7.16883286e-03f, -2.24841665e-03f, 1.61370593e+00f);

    /*
    ----------------------------------------------------------------------
Individual Parameters:
----------------------------------------------------------------------
Focal Length X (fx): 517.08 pixels
Focal Length Y (fy): 519.70 pixels
Principal Point X (cx): 316.54 pixels
Principal Point Y (cy): 258.23 pixels

Distortion Coefficients:
  k1 (radial): -0.057937
  k2 (radial): -0.304432
  p1 (tangential): 0.007169
  p2 (tangential): -0.002248
  k3 (radial): 1.613706
======================================================================
*/

// Preprocessing orientation behavior.
// 0 = no rotation, 1 = 90 CW, 2 = 90 CCW, 3 = 180
static const int ORIENTATION_MISMATCH_ROTATION_MODE = 0;

std::string matToString(const cv::Mat& m) {
    std::ostringstream oss;
    oss << m;
    return oss.str();
}

// Function to adjust camera matrix for runtime image size
static Mat getScaledCameraMatrix(int runtimeWidth, int runtimeHeight) {
    float scale_x = static_cast<float>(runtimeWidth) / CALIB_WIDTH;
    float scale_y = static_cast<float>(runtimeHeight) / CALIB_HEIGHT;
    
    float fx = cameraMatrixCalib.at<float>(0, 0) * scale_x;
    float fy = cameraMatrixCalib.at<float>(1, 1) * scale_y;
    float cx = cameraMatrixCalib.at<float>(0, 2) * scale_x;
    float cy = cameraMatrixCalib.at<float>(1, 2) * scale_y;
    
    std::cout << "\n=== CAMERA MATRIX SCALING ===" << std::endl;
    std::cout << "Calibration: " << CALIB_WIDTH << "x" << CALIB_HEIGHT << std::endl;
    std::cout << "Runtime: " << runtimeWidth << "x" << runtimeHeight << std::endl;
    std::cout << "Scale factors: x=" << scale_x << ", y=" << scale_y << std::endl;
    std::cout << "Original focal length: fx=" << cameraMatrixCalib.at<float>(0, 0) 
              << ", fy=" << cameraMatrixCalib.at<float>(1, 1) << std::endl;
    std::cout << "Scaled focal length: fx=" << fx << ", fy=" << fy << std::endl;
    std::cout << "Tag size: " << fid_size_m << " meters" << std::endl;
    std::cout << "=========================" << std::endl;
    
    Mat scaled = (Mat_<float>(3,3) <<
        fx, 0.f, cx,
        0.f, fy, cy,
        0.f, 0.f, 1.f);
    
    return scaled;
}

// Function to prepare image for processing (rotate if needed to match calibration orientation)
static Mat prepareImageForProcessing(const Mat& inputFrame, Mat& adjustedCameraMatrix) {
    int width = inputFrame.cols;
    int height = inputFrame.rows;
    
    bool calibIsLandscape = (CALIB_WIDTH > CALIB_HEIGHT);
    bool runtimeIsLandscape = (width > height);
    
    Mat processedFrame;
    
    if (calibIsLandscape != runtimeIsLandscape) {
        if (ORIENTATION_MISMATCH_ROTATION_MODE == 1) {
            cv::rotate(inputFrame, processedFrame, cv::ROTATE_90_CLOCKWISE);
            std::cout << "Rotated image CLOCKWISE from " << width << "x" << height
                      << " to " << processedFrame.cols << "x" << processedFrame.rows << std::endl;
        } else if (ORIENTATION_MISMATCH_ROTATION_MODE == 2) {
            cv::rotate(inputFrame, processedFrame, cv::ROTATE_90_COUNTERCLOCKWISE);
            std::cout << "Rotated image COUNTER-CLOCKWISE from " << width << "x" << height
                      << " to " << processedFrame.cols << "x" << processedFrame.rows << std::endl;
        } else if (ORIENTATION_MISMATCH_ROTATION_MODE == 3) {
            cv::rotate(inputFrame, processedFrame, cv::ROTATE_180);
            std::cout << "Rotated image 180 degrees from " << width << "x" << height
                      << " to " << processedFrame.cols << "x" << processedFrame.rows << std::endl;
        } else {
            processedFrame = inputFrame.clone();
            std::cout << "Orientation mismatch detected but rotation disabled; using raw orientation "
                      << width << "x" << height << std::endl;
        }
        
        adjustedCameraMatrix = getScaledCameraMatrix(processedFrame.cols, processedFrame.rows);
    } else {
        processedFrame = inputFrame.clone();
        adjustedCameraMatrix = getScaledCameraMatrix(width, height);
    }
    
    std::cout << "Calibration: " << CALIB_WIDTH << "x" << CALIB_HEIGHT << std::endl;
    std::cout << "Runtime (after prep): " << processedFrame.cols << "x" << processedFrame.rows << std::endl;
    std::cout << "Adjusted camera matrix:\n" << adjustedCameraMatrix << std::endl;
    
    return processedFrame;
}

// Test case structure for validation (DEPRECATED - commented out testing mode)
// struct TestCase {
//     std::string filename;
//     double expected_z;  // meters
//     double expected_x;  // meters
// };

class LocalNode : public rclcpp::Node {
public:
    LocalNode() : Node("LocalNode"), dim_(175) {
        
        RCLCPP_INFO(this->get_logger(), "Localization node started");
        RCLCPP_INFO(this->get_logger(), "Input video: %s", input_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output video: %s", output_video_path_.c_str());
        
        t_prev_ = std::chrono::steady_clock::now();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LocalNode::callback, this));
    }
    
    ~LocalNode() {
        if (video_output_.isOpened()) {
            video_output_.release();
        }
        if (video_input_.isOpened()) {
            video_input_.release();
        }
    }

    private:

    VideoCapture cap_;
    VideoCapture video_input_;  // For input video file
    VideoWriter video_output_;   // For output video with overlay
    const int dim_;
    std::chrono::steady_clock::time_point t_prev_;
    std::string input_video_path_ = "test_videos/rosbag2_2026_03_29-16_32_11.mp4";
    std::string output_video_path_ = "result_videos/output_with_overlay.mp4";
    
    // Frame analyzer (reused across frames)
    std::unique_ptr<FrameAnalyzer> analyzer_;

    void callback() {
        // Initialize video input and analyzer on first run
        if (!analyzer_) {
            auto dict = createCustomDictionary();
            analyzer_ = std::make_unique<FrameAnalyzer>(
                dict,
                cameraMatrixCalib,
                distCoeffs,
                fid_size_m
            );
        }

        // Initialize video capture on first run
        if (!video_input_.isOpened()) {
            // Use member variable for input video path
            video_input_.open(input_video_path_);
            
            if (!video_input_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s", input_video_path_.c_str());
                timer_->cancel();
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Opened video: %s", input_video_path_.c_str());
            
            // Get video properties
            int frame_width = static_cast<int>(video_input_.get(cv::CAP_PROP_FRAME_WIDTH));
            int frame_height = static_cast<int>(video_input_.get(cv::CAP_PROP_FRAME_HEIGHT));
            double fps = video_input_.get(cv::CAP_PROP_FPS);
            int total_frames = static_cast<int>(video_input_.get(cv::CAP_PROP_FRAME_COUNT));
            
            RCLCPP_INFO(this->get_logger(), 
                       "Video properties: %dx%d @ %.1f fps, %d total frames",
                       frame_width, frame_height, fps, total_frames);
            
            // Initialize video writer if output is needed
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            video_output_.open(output_video_path_, fourcc, fps, 
                              cv::Size(frame_width, frame_height), true);
            
            if (!video_output_.isOpened()) {
                RCLCPP_WARN(this->get_logger(), "Could not open video writer for output");
            } else {
                RCLCPP_INFO(this->get_logger(), "Opened output video: %s", output_video_path_.c_str());
            }
        }
        
        // Read one frame from video
        Mat frame;
        if (!video_input_.read(frame)) {
            // End of video reached
            RCLCPP_INFO(this->get_logger(), "End of video reached");
            if (video_output_.isOpened()) {
                video_output_.release();
                RCLCPP_INFO(this->get_logger(), "Output video saved");
            }
            timer_->cancel();
            return;
        }
        
        // Prepare image and get adjusted camera matrix
        Mat adjustedCameraMatrix;
        Mat processedFrame = prepareImageForProcessing(frame, adjustedCameraMatrix);
        
        // Update analyzer with new camera matrix if dimensions changed
        if (adjustedCameraMatrix.data) {
            auto dict = createCustomDictionary();
            analyzer_ = std::make_unique<FrameAnalyzer>(
                dict,
                adjustedCameraMatrix,
                distCoeffs,
                fid_size_m
            );
        }
        
        // ========== ANALYZE FRAME ==========
        bool detection_valid = analyzer_->analyzeFrame(processedFrame);
        
        if (detection_valid && analyzer_->hasValidDetection()) {
            const auto& relPos = analyzer_->getCameraPosition();
            const auto& absPos = analyzer_->getAbsolutePosition();
            
            RCLCPP_INFO(this->get_logger(),
                       "Detected %zu marker(s) - Relative(avg): x=%.3f m, y=%.3f m, z=%.3f m | Absolute2D(avg): x=%.3f m, y=%.3f m",
                       analyzer_->getMarkerCount(),
                       relPos[0], relPos[1], relPos[2],
                       absPos[0], absPos[1]);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No markers detected in this frame");
        }
        
        // Visualize and optionally write to output video
        cv::Mat vizFrame = analyzer_->generatePositionOverlay();
        cv::Mat markerViz = analyzer_->visualizeMarkers(vizFrame);
        
        // FPS overlay (small text in top-right)
        auto t_now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(t_now - t_prev_).count();
        t_prev_ = t_now;
        int fps = (dt > 0.0) ? (int)std::round(1.0 / dt) : 0;
        cv::putText(markerViz, std::to_string(fps) + " FPS", {7, 25}, 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

        // Write frame to output video
        if (video_output_.isOpened()) {
            video_output_.write(markerViz);
        }
        
        // Also save latest frame for debugging
        cv::imwrite("latest_frame.png", markerViz);
    }
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalNode>());
    rclcpp::shutdown();
    return 0;
}
