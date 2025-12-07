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
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include "TagDeclaration.hpp"
#include <bitset>

// #include <opencv2/objdetect/aruco_detector.hpp>

using namespace cv;
using std::cout;
using std::endl;

// ---------------- Camera intrinsics (match your Python) ----------------
static const float fid_size_m = 0.15f; // meters
static const Mat cameraMatrix = (Mat_<float>(3,3) <<
    1.25649815e+03f, 0.f, 7.12996774e+02f,
    0.f, 1.25820533e+03f, 4.69551858e+02f,
    0.f, 0.f, 1.f);
static const Mat distCoeffs = (Mat_<float>(1,5) <<
    -3.72271817e-03f, 5.33786890e-01f, -4.99625728e-04f, -1.65101232e-03f, -1.78505927e+00f);

// ---------------- Helpers ----------------

static std::vector<Point2f> order(const std::vector<Point2f>& pts) {
    CV_Assert(pts.size() == 4);
    std::vector<Point2f> rect(4);
    std::vector<float> s(4), d(4);
    for (int i=0;i<4;++i){ s[i]=pts[i].x+pts[i].y; d[i]=pts[i].y-pts[i].x; }
    int tl = int(std::distance(s.begin(), std::min_element(s.begin(), s.end())));
    int br = int(std::distance(s.begin(), std::max_element(s.begin(), s.end())));
    int tr = int(std::distance(d.begin(), std::min_element(d.begin(), d.end())));
    int bl = int(std::distance(d.begin(), std::max_element(d.begin(), d.end())));
    rect[0]=pts[tl]; rect[2]=pts[br]; rect[1]=pts[tr]; rect[3]=pts[bl];
    return rect;
}

// Determine letter from warped grayscale tag
static std::string determineLetter(const Mat& markerGray) {
    Mat img_bw; threshold(markerGray, img_bw, 200, 255, THRESH_BINARY);
    const uchar white = 255;

    if (img_bw.at<uchar>(5,5) == white) {
        return "None"; // False
    }
    Mat cropped = img_bw(Rect(25,25,125,125));
    imwrite("Cropped.jpeg", cropped);
    
    auto px = [&](int y, int x)->uchar { return cropped.at<uchar>(y,x); };

    // replicate your pixel tests (note: second comment said 'B' but code returned 'C')
    if (px(12,37) != white)                              return "A";
    else if (px(12,12) != white)                         return "C";
    else if (px(12,112) == white)                        return "E";
    else if (px(62,112) == white)                        return "D";
    else                                                 return "B";
}

static cv::Vec3d rotToEul(const Mat& R) {
    double sy = std::sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
    bool singular = sy < 1e-6;
    double yaw, pitch, roll;
    if (!singular) {
        yaw   = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
        pitch = std::atan2(-R.at<double>(2,0), sy);
        roll  = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        yaw   = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
        pitch = std::atan2(-R.at<double>(2,0), sy);
        roll  = 0.0;
    }
    return { yaw*180.0/M_PI, pitch*180.0/M_PI, roll*180.0/M_PI };
}

static bool findTranslationAndRotation(const std::vector<Point2f>& imgPts,
                                       Vec3d& tvec, Vec3d& rpy_deg)
{
    std::vector<Point3f> objPts = {
        { -fid_size_m/2.f,  fid_size_m/2.f, 0.f },
        {  fid_size_m/2.f,  fid_size_m/2.f, 0.f },
        {  fid_size_m/2.f, -fid_size_m/2.f, 0.f },
        { -fid_size_m/2.f, -fid_size_m/2.f, 0.f }
    };
    Mat rvec, tvecMat;
    bool ok = solvePnP(objPts, imgPts, cameraMatrix, distCoeffs, rvec, tvecMat, false, SOLVEPNP_IPPE_SQUARE);
    if (!ok) return false;

    Mat R; Rodrigues(rvec, R);
    rpy_deg = rotToEul(R);
    tvec = Vec3d(tvecMat.at<double>(0,0), tvecMat.at<double>(1,0), tvecMat.at<double>(2,0));
    return true;
}

std::string sampleGridColors(const cv::Mat& img, int rowsSearched, int colsSearched) {
    // Basic validation
    if (img.empty() || rowsSearched <= 0 || colsSearched <= 0) {
        return R"({"samples":[]})";
    }
    if (img.type() != CV_8UC3) {
        throw std::runtime_error("sampleGridColors expects CV_8UC3 image");
    }

    const int height = img.rows;
    const int width  = img.cols;

    std::ostringstream json;
    json << R"({"samples":[)";

    bool first = true;

    for (int r = 0; r < rowsSearched; ++r) {
        // Row position as fraction: (r+1)/(rowsSearched+1)
        float rowFrac = static_cast<float>(r + 1) / static_cast<float>(rowsSearched + 1);
        int y = static_cast<int>(std::round(rowFrac * height));

        // Clamp to valid range just in case
        if (y < 0) y = 0;
        if (y >= height) y = height - 1;

        for (int c = 0; c < colsSearched; ++c) {
            // Column position as fraction: (c+1)/(colsSearched+1)
            float colFrac = static_cast<float>(c + 1) / static_cast<float>(colsSearched + 1);
            int x = static_cast<int>(std::round(colFrac * width));

            if (x < 0) x = 0;
            if (x >= width) x = width - 1;

            // Get BGR pixel
            const cv::Vec3b &bgr = img.at<cv::Vec3b>(y, x);
            int b = static_cast<int>(bgr[0]);
            int g = static_cast<int>(bgr[1]);
            int rChannel = static_cast<int>(bgr[2]);

            // Convert to RGB order for output
            int rVal = rChannel;
            int gVal = g;
            int bVal = b;

            // Human color: white if all channels > 128, else black
            std::string humanColor =
                (rVal > 128 && gVal > 128 && bVal > 128) ? "white" : "black";

            // Append comma if not the first element
            if (!first) {
                json << ",";
            }
            first = false;

            // JSON object for this sample
            json << "{"
                 << R"("row":)" << (r + 1) << ","
                 << R"("col":)" << (c + 1) << ","
                 << R"("rgb":[)" << rVal << "," << gVal << "," << bVal << "],"
                 << R"("color":")" << humanColor << R"(")"
                 << "}";
        }
    }

    json << "]}";
    return json.str();
}

std::vector<int> sampleMarkerBitsFromCorners(const cv::Mat& frame,
                                             const std::vector<cv::Point2f>& corners,
                                             bool blackIsOne = true)
{
    CV_Assert(!frame.empty());
    CV_Assert(corners.size() == 4);

    // Warp to a canonical square
    const int side = 120; // any reasonable size
    std::vector<cv::Point2f> dstPts = {
        cv::Point2f(0, 0),
        cv::Point2f(side - 1, 0),
        cv::Point2f(side - 1, side - 1),
        cv::Point2f(0, side - 1)
    };

    cv::Mat H = cv::getPerspectiveTransform(corners, dstPts);
    cv::Mat warped;
    cv::warpPerspective(frame, warped, H, cv::Size(side, side));

    // Convert to gray and threshold
    cv::Mat gray, bin;
    if (warped.channels() == 3 || warped.channels() == 4) {
        cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = warped;
    }

    cv::adaptiveThreshold(gray, bin, 255,
                          cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY_INV, 11, 5);

    // Ignore outer border: crop ~15% from each side
    int margin = static_cast<int>(side * 0.15);
    cv::Rect innerR(margin, margin,
                    side - 2 * margin,
                    side - 2 * margin);
    innerR &= cv::Rect(0, 0, side, side);
    cv::Mat inner = bin(innerR);

    const int N = 6;
    int cellH = inner.rows / N;
    int cellW = inner.cols / N;

    std::vector<int> bits;
    bits.reserve(N * N);

    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            int y0 = r * cellH;
            int x0 = c * cellW;
            int y1 = (r == N - 1) ? inner.rows : (r + 1) * cellH;
            int x1 = (c == N - 1) ? inner.cols : (c + 1) * cellW;

            cv::Rect cellR(x0, y0, x1 - x0, y1 - y0);
            cv::Mat cell = inner(cellR);

            double meanVal = cv::mean(cell)[0];
            bool isBlack = (meanVal > 128.0);
            int bit = blackIsOne ? (isBlack ? 1 : 0)
                                 : (isBlack ? 0 : 1);
            bits.push_back(bit);
        }
    }

    // Log it nicely
    std::ostringstream oss;
    oss << "Sampled bits:\n";
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            oss << bits[r * N + c] << " ";
        }
        oss << "\n";
    }
    RCLCPP_INFO(rclcpp::get_logger("debug"), "%s", oss.str().c_str());

    return bits;
}



class LocalNode : public rclcpp::Node {
public:
    LocalNode() : Node("LocalNode"), dim_(175) {
        
        // // Declare camera index parameter with default value
        // this->declare_parameter("camera_index", 0);
        // int camera_index = this->get_parameter("camera_index").as_int();
        
        // // Try opening the camera with the specified index
        // cap_.open(camera_index);
        // if (!cap_.isOpened()) {
        //     // If failed with index, try with /dev/video0
        //     cap_.open("/dev/video0");
        //     if (!cap_.isOpened()) {
        //         RCLCPP_ERROR(this->get_logger(), "Error: Could not open webcam at index %d or /dev/video0", camera_index);
        //         // Print available cameras
        //         std::string cmd = "ls -l /dev/video*";
        //         int ret = system(cmd.c_str());
        //         if (ret == 0) {
        //             RCLCPP_ERROR(this->get_logger(), "Available video devices are listed above");
        //         }
        //         throw std::runtime_error("Failed to open webcam");
        //     }
        // }
        
        RCLCPP_INFO(this->get_logger(), "Successfully opened camera");
        
        
        t_prev_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Initialized previous time point");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&LocalNode::callback, this));
        RCLCPP_INFO(this->get_logger(), "Timer started for 30 FPS processing");
    }

    private:

    VideoCapture cap_;
    const int dim_;
    std::chrono::steady_clock::time_point t_prev_;
    Mat frame;

    void callback() {
        Mat frame;
        frame = imread("src/localization/src/tag1.png");
        RCLCPP_INFO(this->get_logger(), "Past the reading frame + frame = %dx%d", frame.cols, frame.rows);

        // if (!cap_.read(frame) || frame.empty()) {
        //     RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
        //     return;
        // }

        
        
        if (frame.empty()){
            RCLCPP_INFO(this->get_logger(), "frame is empty");
            return; 
        }
        
        imwrite("frame.jpeg", frame);


        // --- ArUco detection (6x6) ---
        // TODO CHECK IF CORRECT DICTIONARY
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejected;

        //std::string resultJson = sampleGridColors(frame, 30, 30);

        // For debugging:
        //std::cout << resultJson << std::endl;

// Try 4 rotation steps: 0°, 90°, 180°, 270° CW
    // If your createArcMarkersDictionary currently takes no args,
    // make an overload: createArcMarkersDictionary(int rotationSteps)

    ids.clear();
    corners.clear();
    rejected.clear();

    auto dict = createCustomDictionary();
    cv::FileStorage fs("mydict.yml", cv::FileStorage::WRITE);

fs << "nmarkers" << (int)dict->bytesList.rows;
fs << "markersize" << dict->markerSize;
fs << "maxCorrectionBits" << dict->maxCorrectionBits;

fs << "bytesList" << dict->bytesList;

fs.release();
for (int r = 0; r < dict->bytesList.rows; ++r) {
    std::ostringstream oss;
    oss << "Row " << r << ": ";
    for (int c = 0; c < dict->bytesList.cols; ++c) {
        oss << std::bitset<8>(dict->bytesList.at<uchar>(r, c)) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}
    dict->maxCorrectionBits = 0;
   // params->minMarkerPerimeterRate = 0.1f;  // try 0.08–0.15 range
//params->maxMarkerPerimeterRate = 4.0f;
    RCLCPP_INFO(this->get_logger(), "Custom dict rows=%d cols=%d",
            dict->bytesList.rows, dict->bytesList.cols);

    cv::aruco::detectMarkers(frame, dict, corners, ids, params, rejected);



    if (!ids.empty()) {
        RCLCPP_INFO(this->get_logger(), "Detected IDs:");
        auto sampled = sampleMarkerBitsFromCorners(frame, corners[0], /*blackIsOne=*/true);
        for (size_t i = 0; i < ids.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  ID %d", ids[i]);
        }

    } else {
        RCLCPP_INFO(this->get_logger(), "detected NO markers.");
    }

        // For each detected marker, compute pose using your existing solver
        for (size_t i = 0; i < ids.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Processing marker ID: %d", ids[i]);
            // ArUco returns corners in order: tl, tr, br, bl
            const auto &c = corners[i];

            // Pose using your existing function (expects tl,tr,br,bl as Point2f)
            cv::Vec3d tvec, rpy_deg;
            if (findTranslationAndRotation(c, tvec, rpy_deg)) {
                // Build a tiny JSON string (no new deps) for your existing messaging
                // (Adjust field names as your framework expects.)
                std::ostringstream oss;
                oss << "{"
                    << "\"type\":\"aruco\","
                    << "\"id\":" << ids[i] << ","
                    << "\"corners\":["
                    << "[" << c[0].x << "," << c[0].y << "],"
                    << "[" << c[1].x << "," << c[1].y << "],"
                    << "[" << c[2].x << "," << c[2].y << "],"
                    << "[" << c[3].x << "," << c[3].y << "]"
                    << "],"
                    << "\"camera_t\":["
                    << tvec[0] << "," << tvec[1] << "," << tvec[2] << "],"
                    << "\"camera_rpy_deg\":["
                    << rpy_deg[0] << "," << rpy_deg[1] << "," << rpy_deg[2] << "]"
                    << "}";

                // For now, log it. If your outer framework already has a publisher,
                // just publish this JSON string there without changing the schema around it.
                RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
            }
        }

        // FPS overlay
        auto t_now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(t_now - t_prev_).count();
        t_prev_ = t_now;
        int fps = (dt > 0.0) ? (int)std::round(1.0 / dt) : 0;
        putText(frame, std::to_string(fps), {7,70}, FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0,255,0), 3, LINE_AA);

        imwrite("Outline.jpeg", frame);
        
        // Process window events - using waitKey(1) for OpenCV window updates
        int key = waitKey(1) & 0xFF;
        if (key == 'q') {
            rclcpp::shutdown();
        }
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalNode>());
    rclcpp::shutdown();
    return 0;
}
