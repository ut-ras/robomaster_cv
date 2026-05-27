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
#include <algorithm>
#include <deque>
#include "TagDeclaration.hpp"
#include "../include/frame_analyzer.hpp"
#include <bitset>
#include <std_msgs/msg/string.hpp>

// #include <opencv2/objdetect/aruco_detector.hpp>

using namespace cv;
using std::cout;
using std::endl;

// ---------------- Camera intrinsics (match your Python) ----------------
static const float fid_size_m = 0.135f; // meters

// Original calibration done on 640x480 landscape image
static const int CALIB_WIDTH = 640;
static const int CALIB_HEIGHT = 480;
static bool hasAdjustedCam = false;
static bool hasScaledCam = false;

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
    Mat scaled = (Mat_<float>(3,3) <<
            fx, 0.f, cx,
            0.f, fy, cy,
            0.f, 0.f, 1.f);
    if (!hasScaledCam)
    {
        std::cout << "\n=== CAMERA MATRIX SCALING ===" << std::endl;
        std::cout << "Calibration: " << CALIB_WIDTH << "x" << CALIB_HEIGHT << std::endl;
        std::cout << "Runtime: " << runtimeWidth << "x" << runtimeHeight << std::endl;
        std::cout << "Scale factors: x=" << scale_x << ", y=" << scale_y << std::endl;
        std::cout << "Original focal length: fx=" << cameraMatrixCalib.at<float>(0, 0) 
                << ", fy=" << cameraMatrixCalib.at<float>(1, 1) << std::endl;
        std::cout << "Scaled focal length: fx=" << fx << ", fy=" << fy << std::endl;
        std::cout << "Tag size: " << fid_size_m << " meters" << std::endl;
        std::cout << "=========================" << std::endl;
         
        hasScaledCam = true;
    }
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

    if (!hasAdjustedCam) {
        std::cout << "Calibration: " << CALIB_WIDTH << "x" << CALIB_HEIGHT << std::endl;
        std::cout << "Runtime (after prep): " << processedFrame.cols << "x" << processedFrame.rows << std::endl;
        std::cout << "Adjusted camera matrix:\n" << adjustedCameraMatrix << std::endl;
        hasAdjustedCam = true;
    }
    
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
    LocalNode() : Node("localization_node"), dim_(175) {
        processing_mode_ = this->declare_parameter<std::string>("processing_mode", processing_mode_);
        input_video_path_ = this->declare_parameter<std::string>("input_video_path", input_video_path_);
        input_image_path_ = this->declare_parameter<std::string>("input_image_path", input_image_path_);
        output_video_path_ = this->declare_parameter<std::string>("output_video_path", output_video_path_);
        camera_index_ = this->declare_parameter<int>("camera_index", camera_index_);
        timer_period_ms_ = this->declare_parameter<int>("timer_period_ms", timer_period_ms_);
        numFramesForMedian_ = this->declare_parameter<int>("numFramesForMedian", 5);
        originMetersX_ = this->declare_parameter<double>("originMetersX", originMetersX_);
        originMetersY_ = this->declare_parameter<double>("originMetersY", originMetersY_);
        superRotation_ = this->declare_parameter<std::string>("superRotation", superRotation_);

        result_publisher_ = this->create_publisher<std_msgs::msg::String>("localization/result", 10);

        RCLCPP_INFO(this->get_logger(), "Localization node started in mode: %s", processing_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "Video input: %s", input_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Image input: %s", input_image_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output video: %s", output_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "numFramesForMedian=%zu originMetersX=%.3f originMetersY=%.3f superRotation=%s",
            numFramesForMedian_, originMetersX_, originMetersY_, superRotation_.c_str());

        t_prev_ = std::chrono::steady_clock::now();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms_), std::bind(&LocalNode::callback, this));
    }
    
    ~LocalNode() {
        if (video_output_.isOpened()) {
            video_output_.release();
        }
        if (video_input_.isOpened()) {
            video_input_.release();
        }
        if (camera_input_.isOpened()) {
            camera_input_.release();
        }
    }

private:

    enum class ProcessingMode {
        Video,
        Image,
        Camera,
        CameraMedian,
        VideoMedian
    };

    VideoCapture cap_;
    VideoCapture video_input_;  // For input video file
    VideoCapture camera_input_;  // For live camera input
    VideoWriter video_output_;   // For output video with overlay
    const int dim_;
    std::chrono::steady_clock::time_point t_prev_;
    std::string input_video_path_ = "test_videos/rosbag2_2026_03_29-16_32_11.mp4";
    std::string input_image_path_ = "test_images/61d_-28t_1.jpeg";
    std::string output_video_path_ = "result_videos/output_with_overlay.mp4";
    std::string processing_mode_ = "camera_median";
    size_t numFramesForMedian_;
    double originMetersX_ = 0.0;
    double originMetersY_ = 0.0;
    std::string superRotation_ = "positiveXIsRight_positiveYIsUp";
    int camera_index_ = 0;
    int timer_period_ms_ = 33;

    struct PositionSample {
        cv::Vec3d relative;
        cv::Vec2d absolute;
    };

    struct FrameDeltaDiagnostics {
        bool has_previous = false;
        PositionSample previous_sample{};
        size_t delta_count = 0;
        double mean_jump_m = 0.0;
        double m2_jump_m = 0.0;
        double max_jump_m = 0.0;

        void reset() {
            has_previous = false;
            previous_sample = PositionSample{};
            delta_count = 0;
            mean_jump_m = 0.0;
            m2_jump_m = 0.0;
            max_jump_m = 0.0;
        }

        void addSample(const PositionSample& sample) {
            if (!has_previous) {
                previous_sample = sample;
                has_previous = true;
                return;
            }

            const double dx = sample.relative[0] - previous_sample.relative[0];
            const double dz = sample.relative[2] - previous_sample.relative[2];
            const double jump_m = std::sqrt(dx * dx + dz * dz);

            ++delta_count;
            const double delta = jump_m - mean_jump_m;
            mean_jump_m += delta / static_cast<double>(delta_count);
            m2_jump_m += delta * (jump_m - mean_jump_m);
            if (jump_m > max_jump_m) {
                max_jump_m = jump_m;
            }
            previous_sample = sample;
        }

        std::string summaryString() const {
            std::ostringstream oss;
            if (delta_count == 0) {
                oss << "frame_delta=no_consecutive_samples";
                return oss.str();
            }

            const double stddev_m = (delta_count > 1)
                ? std::sqrt(m2_jump_m / static_cast<double>(delta_count - 1))
                : 0.0;

            oss << "frame_delta_avg_m=" << mean_jump_m
                << "; frame_delta_stddev_m=" << stddev_m
                << "; frame_delta_max_m=" << max_jump_m
                << "; frame_delta_pairs=" << delta_count;
            return oss.str();
        }
    };

    std::deque<PositionSample> position_history_;
    
    // Frame analyzer (reused across frames)
    std::unique_ptr<FrameAnalyzer> analyzer_;
    cv::Size analyzer_frame_size_;
    bool analyzer_ready_ = false;
    bool image_processed_ = false;
    bool video_initialized_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;

    // Callback timing / counting
    bool callbacks_started_ = false;
    std::chrono::steady_clock::time_point callbacks_start_time_;
    std::chrono::steady_clock::time_point callbacks_end_time_;
    size_t callback_count_ = 0;
    FrameDeltaDiagnostics frame_delta_diagnostics_;

    void endCallbacks_(const std::string& reason = "") {
        if (callbacks_started_) {
            callbacks_end_time_ = std::chrono::steady_clock::now();
            auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(callbacks_end_time_ - callbacks_start_time_).count();
            double avg_per_callback = (callback_count_ > 0) ? static_cast<double>(total_ms) / static_cast<double>(callback_count_) : 0.0;
            RCLCPP_INFO(this->get_logger(), "endCallbacks: reason=%s count=%zu total_ms=%lld avg_ms_per_callback=%.2f",
                        reason.c_str(), callback_count_, (long long)total_ms, avg_per_callback);
            callbacks_started_ = false;
            frame_delta_diagnostics_.reset();
        } else {
            RCLCPP_INFO(this->get_logger(), "endCallbacks called but callbacks had not started. reason=%s", reason.c_str());
        }
        if (timer_) {
            timer_->cancel();
        }
    }
    void callback() {
        if (!callbacks_started_) {
            callbacks_started_ = true;
            callbacks_start_time_ = std::chrono::steady_clock::now();
            callback_count_ = 0;
            frame_delta_diagnostics_.reset();
        }
        ++callback_count_;

        const ProcessingMode mode = parseProcessingMode_();

        switch (mode) {
        case ProcessingMode::Video:
            processVideoFrame_(false);
            break;
        case ProcessingMode::VideoMedian:
            processVideoFrame_(true);
            break;
        case ProcessingMode::Image:
            processSingleImage_();
            break;
        case ProcessingMode::Camera:
            processCameraFrame_(false);
            break;
        case ProcessingMode::CameraMedian:
            processCameraFrame_(true);
            break;
        }
    }

    ProcessingMode parseProcessingMode_() const {
        if (processing_mode_ == "image") {
            return ProcessingMode::Image;
        }
        if (processing_mode_ == "camera") {
            return ProcessingMode::Camera;
        }
        if (processing_mode_ == "camera_median") {
            return ProcessingMode::CameraMedian;
        }
        if (processing_mode_ == "video_median") {
            return ProcessingMode::VideoMedian;
        }
        return ProcessingMode::Video;
    }

    void ensureAnalyzer_(const cv::Mat& frame) {
        if (analyzer_ && analyzer_ready_ && analyzer_frame_size_ == frame.size()) {
            return;
        }

        auto dict = createCustomDictionary();
        cv::Mat adjustedCameraMatrix = getScaledCameraMatrix(frame.cols, frame.rows);
        analyzer_ = std::make_unique<FrameAnalyzer>(
            dict,
            adjustedCameraMatrix,
            distCoeffs,
            fid_size_m,
            cv::Vec2d(originMetersX_, originMetersY_),
            superRotation_
        );
        analyzer_frame_size_ = frame.size();
        analyzer_ready_ = true;
    }

    std::string buildResultMessage_(const std::string& modeLabel, const PositionSample* sample) const {
        std::ostringstream oss;
        oss << "mode=" << modeLabel << "; ";

        if (sample == nullptr) {
            oss << "detection=false";
            return oss.str();
        }

        oss << "detection=true";
        if (analyzer_) {
            oss << "; markers=" << analyzer_->getMarkerCount();
        }
        oss << "; rel_xyz_m=[" << sample->relative[0] << "," << sample->relative[1] << "," << sample->relative[2] << "]"
            << "; abs_xy_m=[" << sample->absolute[0] << "," << sample->absolute[1] << "]";
        return oss.str();
    }

    std::string buildOverlayText_(const std::string& label, const PositionSample& sample) const {
        std::ostringstream oss;
        oss << label
    << "\nrel=[" << std::round(sample.relative[0] * 100)
    << ", " << std::round(sample.relative[1] * 100)
    << ", " << std::round(sample.relative[2] * 100) << "]"
    << "\nabs=[" << std::round(sample.absolute[0] * 100)
    << ", " << std::round(sample.absolute[1] * 100) << "]";

return oss.str();
    }

    void publishResult_(const std::string& modeLabel, const PositionSample* sample) {
        if (sample != nullptr) {
            //doDiagnostics_(*sample);
        }

        if (!result_publisher_) {
            return;
        }

        std_msgs::msg::String msg;
        msg.data = buildResultMessage_(modeLabel, sample);
        result_publisher_->publish(msg);
    }

    void doDiagnostics_(const PositionSample& sample) {
        frame_delta_diagnostics_.addSample(sample);
    }

    bool analyzeFrame_(const cv::Mat& frame, const std::string& modeLabel, cv::Mat& processedFrame, PositionSample& sampleOut) {
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty frame for mode %s", modeLabel.c_str());
            return false;
        }

        Mat adjustedCameraMatrix;
        processedFrame = prepareImageForProcessing(frame, adjustedCameraMatrix);
        ensureAnalyzer_(processedFrame);

        const bool detection_valid = analyzer_->analyzeFrame(processedFrame);
        if (detection_valid && analyzer_->hasValidDetection()) {
            sampleOut.relative = analyzer_->getCameraPosition();
            sampleOut.absolute = analyzer_->getAbsolutePosition();

            RCLCPP_INFO(this->get_logger(),
                       "[%s] Detected %zu marker(s) - Relative(avg): x=%.3f m, y=%.3f m, z=%.3f m | Absolute2D(avg): x=%.3f m, y=%.3f m",
                       modeLabel.c_str(),
                       analyzer_->getMarkerCount(),
                       sampleOut.relative[0], sampleOut.relative[1], sampleOut.relative[2],
                       sampleOut.absolute[0], sampleOut.absolute[1]);
            return true;
        }

        RCLCPP_DEBUG(this->get_logger(), "[%s] No markers detected", modeLabel.c_str());
        return false;
    }

    cv::Mat renderAnnotatedFrame_(const cv::Mat& processedFrame, const std::string& overlayText) {
        cv::Mat vizFrame = analyzer_->generatePositionOverlay(processedFrame, overlayText);
        cv::Mat markerViz = analyzer_->visualizeMarkers(vizFrame);

        auto t_now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(t_now - t_prev_).count();
        t_prev_ = t_now;
        int fps = (dt > 0.0) ? (int)std::round(1.0 / dt) : 0;
        cv::putText(markerViz, std::to_string(fps) + " FPS", {7, 25},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

        return markerViz;
    }

    PositionSample medianPositionSample_() const {
        PositionSample medianSample{};
        if (position_history_.empty()) {
            return medianSample;
        }

        std::vector<double> relX;
        std::vector<double> relY;
        std::vector<double> relZ;
        std::vector<double> absX;
        std::vector<double> absY;
        relX.reserve(position_history_.size());
        relY.reserve(position_history_.size());
        relZ.reserve(position_history_.size());
        absX.reserve(position_history_.size());
        absY.reserve(position_history_.size());

        for (const auto& sample : position_history_) {
            relX.push_back(sample.relative[0]);
            relY.push_back(sample.relative[1]);
            relZ.push_back(sample.relative[2]);
            absX.push_back(sample.absolute[0]);
            absY.push_back(sample.absolute[1]);
        }

        auto medianOf = [](std::vector<double> values) {
            std::sort(values.begin(), values.end());
            return values[values.size() / 2];
        };

        medianSample.relative = cv::Vec3d(
            medianOf(relX),
            medianOf(relY),
            medianOf(relZ));
        medianSample.absolute = cv::Vec2d(
            medianOf(absX),
            medianOf(absY));
        return medianSample;
    }

    void appendPositionSample_(const PositionSample& sample) {
        position_history_.push_back(sample);
        if (position_history_.size() > numFramesForMedian_) {
            position_history_.pop_front();
        }
    }

    PositionSample popPositionSample_() {
        PositionSample sample{};
        if (!position_history_.empty()) {
            sample = position_history_.front();
            position_history_.pop_front();
        }
        return sample;
    }

    bool openVideoIfNeeded_() {
        if (video_initialized_) {
            return true;
        }

        video_input_.open(input_video_path_);
        if (!video_input_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s", input_video_path_.c_str());
            endCallbacks_("open_video_failed");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Opened video: %s", input_video_path_.c_str());

        const int frame_width = static_cast<int>(video_input_.get(cv::CAP_PROP_FRAME_WIDTH));
        const int frame_height = static_cast<int>(video_input_.get(cv::CAP_PROP_FRAME_HEIGHT));
        const double fps = video_input_.get(cv::CAP_PROP_FPS);
        const int total_frames = static_cast<int>(video_input_.get(cv::CAP_PROP_FRAME_COUNT));

        RCLCPP_INFO(this->get_logger(),
                   "Video properties: %dx%d @ %.1f fps, %d total frames",
                   frame_width, frame_height, fps, total_frames);

        const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        video_output_.open(output_video_path_, fourcc, fps,
                          cv::Size(frame_width, frame_height), true);
        if (!video_output_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Could not open video writer for output");
        } else {
            RCLCPP_INFO(this->get_logger(), "Opened output video: %s", output_video_path_.c_str());
        }

        video_initialized_ = true;
        return true;
    }

    void processVideoFrame_(bool useMedian) {
        if (!openVideoIfNeeded_()) {
            return;
        }

        Mat frame;
        if (!video_input_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "End of video reached");
            if (video_output_.isOpened()) {
                video_output_.release();
                RCLCPP_INFO(this->get_logger(), "Output video saved");
            }
            endCallbacks_("video_end");
            return;
        }

        cv::Mat processedFrame;
        PositionSample currentSample{};
        const bool valid = analyzeFrame_(frame, useMedian ? "video_median" : "video", processedFrame, currentSample);
        cv::Mat markerViz;

        if (useMedian) {
            if (!valid)
            {
                popPositionSample_();
                publishResult_("NO_POSITION", nullptr);
                markerViz = renderAnnotatedFrame_(processedFrame, "NO DETECTIONS");
            }
            else {
                appendPositionSample_(currentSample);
                PositionSample medianSample = medianPositionSample_();
                publishResult_("video_median", &medianSample);
                markerViz = renderAnnotatedFrame_(processedFrame, buildOverlayText_("video_M5", currentSample));
            }
        }
        else {
            markerViz = renderAnnotatedFrame_(processedFrame, valid ? buildOverlayText_("video", currentSample) : std::string());
            publishResult_(valid ? "video" : "NO_POSITION", valid ? &currentSample : nullptr);
        }

        if (video_output_.isOpened()) {
            video_output_.write(markerViz);
        }
    }

    void processSingleImage_() {
        if (image_processed_) {
            endCallbacks_("image_already_processed");
            return;
        }

        Mat frame = cv::imread(input_image_path_);
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open image: %s", input_image_path_.c_str());
            endCallbacks_("image_open_failed");
            return;
        }

        cv::Mat processedFrame;
        PositionSample sample{};
        const bool valid = analyzeFrame_(frame, "image", processedFrame, sample);
        publishResult_(valid ? "image" : "NO_POSITION", valid ? &sample : nullptr);
        image_processed_ = true;
        endCallbacks_("image_done");
    }

    void processCameraFrame_(bool useMedian) {
        if (!camera_input_.isOpened()) {
            camera_input_.open(camera_index_);
            if (!camera_input_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera index %d", camera_index_);
                endCallbacks_("camera_open_failed");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Opened camera index %d", camera_index_);
        }

        Mat frame;
        if (!camera_input_.read(frame) || frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Camera frame read failed");
            return;
        }

        cv::Mat processedFrame;
        PositionSample sample{};
        const bool valid = analyzeFrame_(frame, useMedian ? "camera_median" : "camera", processedFrame, sample);

        if (useMedian) {
            if (!valid)
            {
                popPositionSample_();
                publishResult_("NO_POSITION", nullptr);
                return;
            }

            appendPositionSample_(sample);
            PositionSample medianSample = medianPositionSample_();
            publishResult_("camera_median", &medianSample);
            return;
        }
        publishResult_(valid ? "camera" : "NO_POSITION", valid ? &sample : nullptr);
    }

    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalNode>());
    rclcpp::shutdown();
    return 0;
}
