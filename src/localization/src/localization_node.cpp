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
#include <bitset>

// #include <opencv2/objdetect/aruco_detector.hpp>

using namespace cv;
using std::cout;
using std::endl;

// ---------------- Camera intrinsics (match your Python) ----------------
static const float fid_size_m = 0.115f; // meters

// Original calibration done on 4024x3024 landscape image
static const int CALIB_WIDTH = 4024;
static const int CALIB_HEIGHT = 3024;

static const Mat cameraMatrixCalib = (Mat_<float>(3,3) <<
    3.02676977e+03f, 0.f, 2.00614248e+03f,
    0.f, 3.02806416e+03f, 1.49949976e+03f,
    0.f, 0.f, 1.f);

static const Mat distCoeffs = (Mat_<float>(1,5) <<
    2.38428408e-01f, -1.10663831e+00f, -1.71794742e-03f, 8.04988144e-04f, 1.78553317e+00f);

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
    
    return scaled;
}

// Function to prepare image for processing (rotate if needed to match calibration orientation)
static Mat prepareImageForProcessing(const Mat& inputFrame, Mat& adjustedCameraMatrix) {
    int width = inputFrame.cols;
    int height = inputFrame.rows;
    
    bool calibIsLandscape = (CALIB_WIDTH > CALIB_HEIGHT);
    bool runtimeIsLandscape = (width > height);
    
    Mat processedFrame;
    
    // If orientations don't match, rotate the runtime image
    if (calibIsLandscape != runtimeIsLandscape) {
        // Rotate 90 degrees COUNTER-clockwise to match calibration orientation
        // This preserves the coordinate system correctly
        cv::rotate(inputFrame, processedFrame, cv::ROTATE_90_COUNTERCLOCKWISE);
        std::cout << "Rotated image COUNTER-CLOCKWISE from " << width << "x" << height 
                  << " to " << processedFrame.cols << "x" << processedFrame.rows << std::endl;
        
        // Get scaled camera matrix for rotated dimensions
        adjustedCameraMatrix = getScaledCameraMatrix(processedFrame.cols, processedFrame.rows);
    } else {
        // No rotation needed, just scale the camera matrix
        processedFrame = inputFrame.clone();
        adjustedCameraMatrix = getScaledCameraMatrix(width, height);
    }
    
    std::cout << "Calibration: " << CALIB_WIDTH << "x" << CALIB_HEIGHT << std::endl;
    std::cout << "Runtime (after prep): " << processedFrame.cols << "x" << processedFrame.rows << std::endl;
    std::cout << "Adjusted camera matrix:\n" << adjustedCameraMatrix << std::endl;
    
    return processedFrame;
}

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
    cv::imwrite("Cropped.jpeg", cropped);
    
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
                                       Vec3d& tvec, Vec3d& rpy_deg,
                                       const Mat& cameraMatrix)
{
    std::vector<Point3f> objPts = {
        { -fid_size_m/2.f, -fid_size_m/2.f, 0.f },  // TL: top-left
        {  fid_size_m/2.f, -fid_size_m/2.f, 0.f },  // TR: top-right
        {  fid_size_m/2.f,  fid_size_m/2.f, 0.f },  // BR: bottom-right
        { -fid_size_m/2.f,  fid_size_m/2.f, 0.f }   // BL: bottom-left
    };
    Mat rvec, tvecMat;
    bool ok = solvePnP(objPts, imgPts, cameraMatrix, distCoeffs, rvec, tvecMat, false, SOLVEPNP_IPPE_SQUARE);
    if (!ok) return false;

    Mat R; Rodrigues(rvec, R);
    rpy_deg = rotToEul(R);
    tvec = Vec3d(tvecMat.at<double>(0,0), tvecMat.at<double>(1,0), tvecMat.at<double>(2,0));
    return true;
}

// Compute camera position relative to tag (inverted transformation)
// Returns camera's XYZ coordinates in the tag's reference frame
static bool getCameraPositionRelativeToTag(const std::vector<Point2f>& imgPts,
                                           Vec3d& camPosRelativeToTag,
                                           const Mat& cameraMatrix)
{
    std::vector<Point3f> objPts = {
        { -fid_size_m/2.f, -fid_size_m/2.f, 0.f },  // TL: top-left
        {  fid_size_m/2.f, -fid_size_m/2.f, 0.f },  // TR: top-right
        {  fid_size_m/2.f,  fid_size_m/2.f, 0.f },  // BR: bottom-right
        { -fid_size_m/2.f,  fid_size_m/2.f, 0.f }   // BL: bottom-left
    };
    Mat rvec, tvecMat;
    bool ok = solvePnP(objPts, imgPts, cameraMatrix, distCoeffs, rvec, tvecMat, false, SOLVEPNP_IPPE_SQUARE);
    if (!ok) return false;

    Vec3d rvecVec = Vec3d(tvecMat.at<double>(0,0), 
                                 tvecMat.at<double>(1,0), 
                                 tvecMat.at<double>(2,0));
    // Convert rotation vector to rotation matrix
    Mat R;
    Rodrigues(rvec, R);
    
    // Invert the transformation to get camera position in tag frame
    // R_inv = R^T (transpose), t_inv = -R^T * t
    Mat R_inv = R.t();
    Mat tvec_inv = -R_inv * tvecMat;
    
    camPosRelativeToTag = Vec3d(tvec_inv.at<double>(0,0), 
                                 tvec_inv.at<double>(1,0), 
                                 tvec_inv.at<double>(2,0));
    return true;
}

// Visualize reprojected corners to verify pose estimation
static void visualizeReprojection(Mat& frame,
                                  const std::vector<Point2f>& detectedCorners,
                                  const Mat& rvec,
                                  const Mat& tvec,
                                  const Mat& cameraMatrix,
                                  const std::string& filename = "reprojection_debug.jpg", )
{
    // Define 3D object points (must match solvePnP)
    std::vector<Point3f> objPts = {
        { -fid_size_m/2.f, -fid_size_m/2.f, 0.f },  // TL
        {  fid_size_m/2.f, -fid_size_m/2.f, 0.f },  // TR
        {  fid_size_m/2.f,  fid_size_m/2.f, 0.f },  // BR
        { -fid_size_m/2.f,  fid_size_m/2.f, 0.f }   // BL
    };
    
    // Project 3D points back to 2D using the estimated pose
    std::vector<Point2f> reprojectedCorners;
    projectPoints(objPts, rvec, tvec, cameraMatrix, distCoeffs, reprojectedCorners);
    
    Mat debugFrame = frame.clone();
    
    // Draw detected corners in GREEN (circles)
    for (size_t i = 0; i < detectedCorners.size(); i++) {
        circle(debugFrame, detectedCorners[i], 8, Scalar(0, 255, 0), -1);
        putText(debugFrame, "D" + std::to_string(i), detectedCorners[i] + Point2f(10, 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
    }
    
    // Draw reprojected corners in RED (crosses)
    for (size_t i = 0; i < reprojectedCorners.size(); i++) {
        // Draw cross
        Point2f pt = reprojectedCorners[i];
        line(debugFrame, pt + Point2f(-8, 0), pt + Point2f(8, 0), Scalar(0, 0, 255), 2);
        line(debugFrame, pt + Point2f(0, -8), pt + Point2f(0, 8), Scalar(0, 0, 255), 2);
        putText(debugFrame, "R" + std::to_string(i), pt + Point2f(10, -10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
    }
    
    // Draw axes for visualization
    std::vector<Point3f> axisPoints = {
        {0, 0, 0},                    // Origin
        {fid_size_m, 0, 0},          // X-axis (red)
        {0, fid_size_m, 0},          // Y-axis (green)
        {0, 0, -fid_size_m}          // Z-axis (blue, pointing toward camera)
    };
    std::vector<Point2f> imageAxisPoints;
    projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imageAxisPoints);
    
    // Draw coordinate axes
    line(debugFrame, imageAxisPoints[0], imageAxisPoints[1], Scalar(0, 0, 255), 3);  // X: red
    line(debugFrame, imageAxisPoints[0], imageAxisPoints[2], Scalar(0, 255, 0), 3);  // Y: green
    line(debugFrame, imageAxisPoints[0], imageAxisPoints[3], Scalar(255, 0, 0), 3);  // Z: blue
    
    // Add labels
    putText(debugFrame, "X", imageAxisPoints[1], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    putText(debugFrame, "Y", imageAxisPoints[2], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
    putText(debugFrame, "Z", imageAxisPoints[3], FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);
    
    // Calculate and display reprojection error
    double totalError = 0.0;
    for (size_t i = 0; i < detectedCorners.size(); i++) {
        double error = norm(detectedCorners[i] - reprojectedCorners[i]);
        totalError += error;
    }
    double avgError = totalError / detectedCorners.size();
    
    putText(debugFrame, "Avg Reproj Error: " + std::to_string(avgError) + "px",
            Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 0), 2);
    putText(debugFrame, "GREEN=Detected, RED=Reprojected",
            Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
    
    imwrite(filename, debugFrame);
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


// Helper to order corners: TL, TR, BR, BL
std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point2f>& pts) {
    CV_Assert(pts.size() == 4);
    std::vector<cv::Point2f> ordered(4);
    
    // Sum: top-left has smallest, bottom-right has largest
    std::vector<float> sums(4);
    for (int i = 0; i < 4; i++) {
        sums[i] = pts[i].x + pts[i].y;
    }
    
    int tlIdx = std::distance(sums.begin(), std::min_element(sums.begin(), sums.end()));
    int brIdx = std::distance(sums.begin(), std::max_element(sums.begin(), sums.end()));
    
    // Diff: top-right has smallest, bottom-left has largest
    std::vector<float> diffs(4);
    for (int i = 0; i < 4; i++) {
        diffs[i] = pts[i].y - pts[i].x;
    }
    
    int trIdx = std::distance(diffs.begin(), std::min_element(diffs.begin(), diffs.end()));
    int blIdx = std::distance(diffs.begin(), std::max_element(diffs.begin(), diffs.end()));
    
    ordered[0] = pts[tlIdx]; // TL
    ordered[1] = pts[trIdx]; // TR
    ordered[2] = pts[brIdx]; // BR
    ordered[3] = pts[blIdx]; // BL
    
    return ordered;
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

    // Use BINARY (not INV) so black pixels = 0, white = 255
    cv::adaptiveThreshold(gray, bin, 255,
                          cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY, 11, 5);

    // Skip border: ArUco has 1 white + 1 black border = 2 cells on each side
    // With 6x6 data, total is 8x8, so skip 2/8 = 25% on each side
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

    int zeroCount = 0;
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            int y0 = r * cellH;
            int x0 = c * cellW;
            int y1 = (r == N - 1) ? inner.rows : (r + 1) * cellH;
            int x1 = (c == N - 1) ? inner.cols : (c + 1) * cellW;

            cv::Rect cellR(x0, y0, x1 - x0, y1 - y0);
            cv::Mat cell = inner(cellR);

            double meanVal = cv::mean(cell)[0];
            // After BINARY threshold: black=0, white=255
            bool isBlack = (meanVal < 128.0);
            int bit = blackIsOne ? (isBlack ? 1 : 0)
                                 : (isBlack ? 0 : 1);
            if (bit == 0) ++zeroCount;
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
    if(zeroCount < N*N)
        RCLCPP_INFO(rclcpp::get_logger("debug"), "%s", oss.str().c_str());

    return bits;
}

// ============================================================================
// CUSTOM MARKER DETECTION FUNCTION
// ============================================================================
// This function replaces OpenCV's ArUco detection with a custom implementation
// that gives full control over the matching process.
//
// HIGH-LEVEL ALGORITHM:
// 1. Find all quadrilaterals (4-sided shapes) in the image
// 2. For each quadrilateral:
//    a. Warp it to a flat square
//    b. Extract the 6x6 bit pattern from inside
//    c. Try matching it against all markers in the dictionary (with 4 rotations)
//    d. If match found within error tolerance, add to results
// ============================================================================
void customMarkerDetection(const cv::Mat& frame,
                          const cv::Ptr<cv::aruco::Dictionary>& dict,
                          std::vector<std::vector<cv::Point2f>>& outCorners,
                          std::vector<int>& outIds,
                          int maxCorrectionBits = 5)
{
    outCorners.clear();
    outIds.clear();
    
    // STEP 1: Convert to grayscale for processing
    cv::Mat gray;
    if (frame.channels() == 3 || frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }
    
    // STEP 2: Threshold to black and white
    // Adaptive threshold works better with varying lighting conditions
    // Using BINARY_INV to find BLACK regions instead of white
    cv::Mat binary;
    cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, 
                         cv::THRESH_BINARY_INV, 19, 9);
    
    // Add white border so edge-touching regions are fully enclosed
    cv::Mat binaryWithBorder;
    cv::copyMakeBorder(binary, binaryWithBorder, 5, 5, 5, 5, 
                      cv::BORDER_CONSTANT, cv::Scalar(0)); // 0 = black border (background)
    
    // STEP 3: Find all contours (outlines) in the binary image
    // THIS IS WHERE WE FIND EACH INDIVIDUAL BOX/QUADRILATERAL
    // findContours traces the boundary of every white region, so with BINARY_INV
    // it will find the black regions from the original image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryWithBorder, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    // Optional: Visualize filled contours for debugging
    cv::Mat contoursVis = frame.clone();
    //cv::drawContours(contoursVis, contours, -1, cv::Scalar(0, 0, 0), -1); // Fill with black
    //cv::imwrite("filled_contours_debug.jpg", contoursVis);
    
    // STEP 4: Process each contour to see if it's a marker candidate
    // Loop through every outline found in the image
    for (const auto& contour : contours) {
        // STEP 4a: Simplify the contour to a polygon
        // approxPolyDP reduces the number of points while preserving the shape
        std::vector<cv::Point> approx;
        double epsilon = 0.05 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);
        
        // STEP 4b: Check if it's a valid quadrilateral (4-sided shape)
        // Markers must be 4-cornered and convex (no indentations)
        if (approx.size() != 4 || !cv::isContourConvex(approx)) {
            continue; // Skip this contour, not a valid box
        }
        
        // STEP 4c: Filter out tiny shapes that are too small to be markers
        double area = cv::contourArea(approx);
        if (area < 100) continue; // Skip tiny candidates
        
        // STEP 4d: Convert corner points to floating point for precision
        // Adjust coordinates back by subtracting border offset
        std::vector<cv::Point2f> corners(4);
        for (int i = 0; i < 4; i++) {
            corners[i] = cv::Point2f(approx[i].x - 5, approx[i].y - 5); // Subtract border offset
        }
        
        // STEP 4e: Order corners consistently: TL, TR, BR, BL
        // This ensures we always read the marker in the same orientation
        corners = orderCorners(corners);
        
        // STEP 5: Warp the quadrilateral to a flat square for analysis
        // This removes perspective distortion so we can read the bits accurately
        const int warpSize = 80; // 8x8 cells @ 10 pixels each (higher res = better accuracy)
        std::vector<cv::Point2f> dstPts = {
            cv::Point2f(0, 0),                          // Top-left
            cv::Point2f(warpSize - 1, 0),               // Top-right
            cv::Point2f(warpSize - 1, warpSize - 1),    // Bottom-right
            cv::Point2f(0, warpSize - 1)                // Bottom-left
        };
        
        // Calculate perspective transform matrix and warp the marker
        cv::Mat H = cv::getPerspectiveTransform(corners, dstPts);
        cv::Mat warped;
        cv::warpPerspective(gray, warped, H, cv::Size(warpSize, warpSize));
        
        // STEP 6: Threshold the warped image to pure black/white
        // Use larger window size for better local contrast adaptation
        cv::Mat warpedBin;
        cv::adaptiveThreshold(warped, warpedBin, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                            cv::THRESH_BINARY, 11, 3);
        
        // STEP 7: Save debug image to see what the detector sees
        static int candidateCounter = 0;
        std::string debugFilename = "candidate_" + std::to_string(candidateCounter++) + "_warped.jpg";
        //cv::imwrite(debugFilename, warpedBin);
        
        // STEP 8: Extract the inner 6x6 data grid (skip the borders)
        // Marker structure: 1 white border + 1 black border + 6x6 data = 8x8 total
        int cellSize = warpSize / 8;
        
        // Skip the outer 2 cells (white + black border) to get to the data
        int margin = cellSize * 1; // Skip 1 cell on each side (white + black borders)
        cv::Rect innerRect(margin, margin, warpSize - 2*margin, warpSize - 2*margin);
        cv::Mat innerData = warpedBin(innerRect);
        
        // STEP 9: Read the 6x6 bit pattern from the inner data area
        // Divide the inner area into a 6x6 grid and sample each cell
        std::vector<int> extractedBits(36);
        int dataSize = innerData.rows;
        int dataCellSize = dataSize / 6;
        
        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 6; c++) {
                // Calculate cell boundaries
                int y0 = r * dataCellSize;
                int x0 = c * dataCellSize;
                int y1 = (r == 5) ? innerData.rows : (r + 1) * dataCellSize;
                int x1 = (c == 5) ? innerData.cols : (c + 1) * dataCellSize;
                
                // Sample from the center 50% of the cell to avoid edge effects
                int cellW = x1 - x0;
                int cellH = y1 - y0;
                int centerMargin = std::max(cellW / 4, 1); // 25% margin on each side
                cv::Rect cellRect(x0 + centerMargin, y0 + centerMargin, 
                                 cellW - 2*centerMargin, cellH - 2*centerMargin);
                cv::Mat cell = innerData(cellRect);
                
                // Determine if cell is black (1) or white (0)
                double meanVal = cv::mean(cell)[0];
                // After BINARY threshold: black pixels have low values (< 128)
                extractedBits[r * 6 + c] = (meanVal < 128.0) ? 1 : 0;
            }
        }
        
        
        // STEP 11: Try matching this pattern against the dictionary
        // We'll try all 4 rotations (0°, 90°, 180°, 270°) since we don't know orientation
        for (int rot = 0; rot < 4; rot++) {
            // STEP 11a: Compare against each marker in the dictionary
            for (int markerIdx = 0; markerIdx < dict->bytesList.rows; markerIdx++) {
                // STEP 11b: Decode the dictionary pattern for this marker ID
                // Dictionary stores bits packed into bytes, so we unpack them
                std::vector<int> dictBits(36);
                for (int bit = 0; bit < 36; bit++) {
                    int byteIdx = bit / 8;        // Which byte contains this bit
                    int bitIdx = 7 - (bit % 8);   // Position within that byte (MSB first)
                    uchar byteVal = dict->bytesList.at<uchar>(markerIdx, byteIdx);
                    dictBits[bit] = (byteVal >> bitIdx) & 1;
                }
                
                // STEP 11c: Count how many bits differ between extracted and dictionary
                int differences = 0;
                for (int i = 0; i < 36; i++) {
                    if (extractedBits[i] != dictBits[i]) {
                        differences++;
                    }
                }
                
                // STEP 11d: If close enough match, we found a marker!
                if (differences <= maxCorrectionBits) {
                    outCorners.push_back(corners);
                    outIds.push_back(markerIdx);
                    goto next_candidate; // Found match, skip checking other rotations/markers
                }
            }
            
            // STEP 11e: Rotate the extracted pattern 90° clockwise and try again
            std::vector<int> rotated(36);
            for (int r = 0; r < 6; r++) {
                for (int c = 0; c < 6; c++) {
                    rotated[c * 6 + (5 - r)] = extractedBits[r * 6 + c];
                }
            }
            extractedBits = rotated;
        }
        
        next_candidate:;
    }
}

// Test case structure for validation
struct TestCase {
    std::string filename;
    double expected_z;  // meters
    double expected_x;  // meters
};

class LocalNode : public rclcpp::Node {
public:
    LocalNode() : Node("LocalNode"), dim_(175), current_test_index_(0), enable_testing_(true) {
        
        // ========== CONFIGURE YOUR TEST CASES HERE ==========
        // Add test images with their expected x and z offsets (in meters)
        // x = horizontal offset, z = distance from tag
        test_cases_ = {
            // Example format: {"path/to/image.png", expected_z, expected_x}
             {"test_images/61d_-28t_1.jpeg", .61, -.28},
             {"test_images/61d_0t_1.jpeg", .61, 0},
             {"test_images/61d_0t_2.jpeg", .61, 0},
             {"test_images/61d_28t_1.jpeg", .61, .28},
             {"test_images/81d_-31.5t_1.jpeg", .81, -.315},
             {"test_images/81d_0t_1.jpeg", .81, 0},
             {"test_images/81d_0t_2.jpeg", .81, 0},
             {"test_images/81d_0t_3.jpeg", .81, 0},
             {"test_images/81d_31.5t_1.jpeg", .81, .315},
             {"test_images/81d_31.5t_2.jpeg", .81, .315},
             {"test_images/140d_0t_1.jpeg", 1.40, 0},
             {"test_images/140d_0t_2.jpeg", 1.40, 0},
             {"test_images/140d_-25.4t_1.jpeg", 1.40, -.254},
             {"test_images/140d_25.4t_1.jpeg", 1.40, .254},
        };
        // ====================================================
        
        if (enable_testing_ && !test_cases_.empty()) {
            // Open CSV file for writing results
            csv_file_.open("localization_test_results.csv");
            csv_file_ << "Test#,Filename,Expected_X,Expected_Z,Calculated_X,Calculated_Z,"
                      << "Error_X,Error_Z,Distance_Error,Status\n";
            
            RCLCPP_INFO(this->get_logger(), "Testing mode enabled with %zu test cases", test_cases_.size());
            RCLCPP_INFO(this->get_logger(), "Results will be saved to: localization_test_results.csv");
        } else {
            RCLCPP_INFO(this->get_logger(), "Live camera mode (testing disabled)");
        }
        
        t_prev_ = std::chrono::steady_clock::now();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LocalNode::callback, this));
        RCLCPP_INFO(this->get_logger(), "Localization node started");
    }
    
    ~LocalNode() {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "Test results saved to localization_test_results.csv");
        }
    }

    private:

    VideoCapture cap_;
    const int dim_;
    std::chrono::steady_clock::time_point t_prev_;
    Mat frame;
    
    // Testing variables
    std::vector<TestCase> test_cases_;
    size_t current_test_index_;
    bool enable_testing_;
    std::ofstream csv_file_;

    void callback() {
        // Check if testing is complete
        if (enable_testing_ && current_test_index_ >= test_cases_.size()) {
            RCLCPP_INFO(this->get_logger(), "\n========== ALL TESTS COMPLETE ==========");
            RCLCPP_INFO(this->get_logger(), "Results saved to: localization_test_results.csv");
            timer_->cancel();  // Stop the timer
            return;
        }
        
        Mat frame;
        std::string current_filename;
        double expected_x = 0.0, expected_z = 0.0;
        
        if (enable_testing_ && !test_cases_.empty()) {
            // Testing mode: load test image
            const TestCase& test = test_cases_[current_test_index_];
            current_filename = test.filename;
            expected_x = test.expected_x;
            expected_z = test.expected_z;
            
            frame = imread(test.filename);
            RCLCPP_INFO(this->get_logger(), "\n========== TEST %zu/%zu ==========", 
                       current_test_index_ + 1, test_cases_.size());
            RCLCPP_INFO(this->get_logger(), "File: %s", test.filename.c_str());
            RCLCPP_INFO(this->get_logger(), "Expected: x=%.3f m, z=%.3f m", expected_x, expected_z);
        } else {
            // Live camera mode (future use)
            // if (!cap_.read(frame) || frame.empty()) {
            //     RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
            //     return;
            // }
            current_filename = "live_camera";
        }
        
        if (frame.empty()){
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", current_filename.c_str());
            if (enable_testing_) {
                csv_file_ << current_test_index_ + 1 << "," << current_filename << ","
                          << expected_x << "," << expected_z << ","
                          << "N/A,N/A,N/A,N/A,N/A,FAILED_TO_LOAD\n";
                current_test_index_++;
            }
            return; 
        }
        
        RCLCPP_INFO(this->get_logger(), "Original image size: %dx%d", frame.cols, frame.rows);
        
        // Prepare image and get adjusted camera matrix
        Mat adjustedCameraMatrix;
        Mat processedFrame = prepareImageForProcessing(frame, adjustedCameraMatrix);
        
        cv::imwrite("frame.jpeg", processedFrame);

        // --- CUSTOM MARKER DETECTION ---
        auto dict = createCustomDictionary();
        
        std::vector<std::vector<cv::Point2f>> detectedCorners;
        std::vector<int> detectedIds;
        
        customMarkerDetection(processedFrame, dict, detectedCorners, detectedIds);
        
        RCLCPP_INFO(this->get_logger(), "Custom detection: %zu markers found", detectedIds.size());

        if (!detectedIds.empty()) {
            RCLCPP_INFO(this->get_logger(), "\n=== DETECTED MARKERS ===");
            
            //for (size_t i = 0; i < detectedIds.size(); ++i) {
                size_t i = detectedIds.size() - 1;
                int markerID = detectedIds[i];
                
                RCLCPP_INFO(this->get_logger(), "Marker ID %d at corners: [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f]",
                           markerID,
                           detectedCorners[i][0].x, detectedCorners[i][0].y,
                           detectedCorners[i][1].x, detectedCorners[i][1].y,
                           detectedCorners[i][2].x, detectedCorners[i][2].y,
                           detectedCorners[i][3].x, detectedCorners[i][3].y);
            //}
            
            // Draw detected markers
            Mat frameWithMarkers = processedFrame.clone();
            for (size_t i = 0; i < detectedIds.size(); ++i) {
                // Draw blue polylines
                std::vector<cv::Point> intPoints;
                for (const auto& pt : detectedCorners[i]) {
                    intPoints.push_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
                }
                cv::polylines(frameWithMarkers, intPoints, true, cv::Scalar(255, 0, 0), 4);
                
                // Add ID label
                cv::Point2f center(0, 0);
                for (const auto& pt : detectedCorners[i]) {
                    center += pt;
                }
                center.x /= 4;
                center.y /= 4;
                
                cv::putText(frameWithMarkers, "ID:" + std::to_string(detectedIds[i]), 
                           cv::Point(center.x, center.y),
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            }
            
            cv::imwrite("detected_markers.jpg", frameWithMarkers);
            RCLCPP_INFO(this->get_logger(), "Saved detected markers to detected_markers.jpg");
        } else {
            RCLCPP_WARN(this->get_logger(), "No markers detected");
        }

        // For each detected marker, compute pose using your existing solver
        bool marker_processed = false;
        if (!detectedIds.empty()) {
            //for (size_t i = 0; i < detectedIds.size(); ++i) {
                size_t i = detectedIds.size() - 1;
                RCLCPP_INFO(this->get_logger(), "Processing marker ID: %d", detectedIds[i]);
                const auto &c = detectedCorners[i];

                // Pose using your existing function (expects tl,tr,br,bl as Point2f)
                cv::Vec3d tvec, rpy_deg;
            if (findTranslationAndRotation(c, tvec, rpy_deg, adjustedCameraMatrix)) {
                // Visualize the reprojection to verify pose accuracy
                Mat rvec, tvecMat;
                std::vector<Point3f> objPts = {
                    { -fid_size_m/2.f, -fid_size_m/2.f, 0.f },
                    {  fid_size_m/2.f, -fid_size_m/2.f, 0.f },
                    {  fid_size_m/2.f,  fid_size_m/2.f, 0.f },
                    { -fid_size_m/2.f,  fid_size_m/2.f, 0.f }
                };
                solvePnP(objPts, c, adjustedCameraMatrix, distCoeffs, rvec, tvecMat, false, SOLVEPNP_IPPE_SQUARE);
                visualizeReprojection(processedFrame, c, rvec, tvecMat, adjustedCameraMatrix, "reprojection_" + std::to_string(detectedIds[i]) + ".jpg");
                
                // Camera position relative to tag (x, y, z in tag's reference frame)
                cv::Vec3d cameraPositionRelativeToTag;
                if (getCameraPositionRelativeToTag(c, cameraPositionRelativeToTag, adjustedCameraMatrix)) {
                    double calc_x = cameraPositionRelativeToTag[0];
                    double calc_y = cameraPositionRelativeToTag[1];
                    double calc_z = cameraPositionRelativeToTag[2];
                    
                    RCLCPP_INFO(this->get_logger(), 
                               "Calculated position: x=%.3f m, y=%.3f m, z=%.3f m",
                               calc_x, calc_y, calc_z);
                    
                    // Calculate errors if in testing mode
                    if (enable_testing_) {
                        double error_x = calc_x - expected_x;
                        double error_z = calc_z - expected_z;
                        double distance_error = std::sqrt(error_x*error_x + error_z*error_z);
                        
                        RCLCPP_INFO(this->get_logger(), "Expected position: x=%.3f m, z=%.3f m", 
                                   expected_x, expected_z);
                        RCLCPP_INFO(this->get_logger(), "Error: x=%.3f m, z=%.3f m, distance=%.3f m",
                                   error_x, error_z, distance_error);
                        
                        // Write to CSV
                        csv_file_ << current_test_index_ + 1 << ","
                                  << current_filename << ","
                                  << expected_x << "," << expected_z << ","
                                  << calc_x << "," << calc_z << ","
                                  << error_x << "," << error_z << ","
                                  << distance_error << ",SUCCESS\n";
                        csv_file_.flush();  // Ensure data is written immediately
                        
                        marker_processed = true;
                    }
                }
                // Build a tiny JSON string (no new deps) for your existing messaging
                std::ostringstream oss;
                oss << "{"
                    << "\"type\":\"aruco\","
                    << "\"id\":" << detectedIds[i] << ","
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
        //}
        } // end if (!detectedIds.empty())
        
        // Handle case where no markers detected in testing mode
        if (enable_testing_ && !marker_processed && detectedIds.empty()) {
            RCLCPP_WARN(this->get_logger(), "No markers detected in test image");
            csv_file_ << current_test_index_ + 1 << ","
                      << current_filename << ","
                      << expected_x << "," << expected_z << ","
                      << "N/A,N/A,N/A,N/A,N/A,NO_MARKER_DETECTED\n";
            csv_file_.flush();
        }
        
        // Move to next test in testing mode
        if (enable_testing_) {
            current_test_index_++;
        }

        // FPS overlay
        auto t_now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(t_now - t_prev_).count();
        t_prev_ = t_now;
        int fps = (dt > 0.0) ? (int)std::round(1.0 / dt) : 0;
        cv::putText(processedFrame, std::to_string(fps), {7,70}, cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0,255,0), 3, cv::LINE_AA);

        cv::imwrite("Outline.jpeg", processedFrame);
        
        // Process window events - using waitKey(1) for OpenCV window updates
        int key = cv::waitKey(1) & 0xFF;
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
