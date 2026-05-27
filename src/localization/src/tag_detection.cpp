#include "tag_detection.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>

using namespace cv;
using namespace std;

TagDetection::TagDetection(const cv::Ptr<cv::aruco::Dictionary>& dict,
                           const cv::Mat& cameraMatrix,
                           const cv::Mat& distCoeffs)
    : dict_(dict), cameraMatrix_(cameraMatrix), distCoeffs_(distCoeffs) {
}

bool TagDetection::detectMarkers(const cv::Mat& frame, int maxCorrectionBits) {
    detectedCorners_.clear();
    detectedIds_.clear();

    if (frame.empty()) {
        std::cerr << "TagDetection::detectMarkers - Empty frame provided" << std::endl;
        return false;
    }

    customMarkerDetection(frame, maxCorrectionBits);
    return !detectedIds_.empty();
}

const std::vector<std::vector<cv::Point2f>>& TagDetection::getDetectedCorners() const {
    return detectedCorners_;
}

const std::vector<int>& TagDetection::getDetectedIds() const {
    return detectedIds_;
}

size_t TagDetection::getMarkerCount() const {
    return detectedIds_.size();
}

void TagDetection::printDetectedMarkers() const {
    if (detectedIds_.empty()) {
        std::cout << "No markers detected" << std::endl;
        return;
    }

    std::cout << "\n=== DETECTED MARKERS ===" << std::endl;
    for (size_t i = 0; i < detectedIds_.size(); ++i) {
        int markerID = detectedIds_[i];
        std::cout << "Marker ID " << markerID << " at corners: ["
                  << detectedCorners_[i][0].x << "," << detectedCorners_[i][0].y << "] ["
                  << detectedCorners_[i][1].x << "," << detectedCorners_[i][1].y << "] ["
                  << detectedCorners_[i][2].x << "," << detectedCorners_[i][2].y << "] ["
                  << detectedCorners_[i][3].x << "," << detectedCorners_[i][3].y << "]"
                  << std::endl;
    }
}

cv::Mat TagDetection::visualizeDetectedMarkers(const cv::Mat& frame,
                                               const std::string& filename) const {
    cv::Mat frameWithMarkers = frame.clone();

    if (detectedIds_.empty()) {
        return frameWithMarkers;
    }

    for (size_t i = 0; i < detectedIds_.size(); ++i) {
        // Draw blue polylines
        std::vector<cv::Point> intPoints;
        for (const auto& pt : detectedCorners_[i]) {
            intPoints.push_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
        }
        cv::polylines(frameWithMarkers, intPoints, true, cv::Scalar(255, 0, 0), 4);

        // Add ID label
        cv::Point2f center(0, 0);
        for (const auto& pt : detectedCorners_[i]) {
            center += pt;
        }
        center.x /= 4;
        center.y /= 4;

        cv::putText(frameWithMarkers, "ID:" + std::to_string(detectedIds_[i]),
                    cv::Point(center.x, center.y),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    }

    if (!filename.empty()) {
        cv::imwrite(filename, frameWithMarkers);
    }

    return frameWithMarkers;
}

// ============================================================================
// PRIVATE HELPER METHODS
// ============================================================================

std::vector<cv::Point2f> TagDetection::orderCorners(const std::vector<cv::Point2f>& pts) {
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

std::vector<int> TagDetection::sampleMarkerBits(const cv::Mat& frame,
                                                const std::vector<cv::Point2f>& corners,
                                                bool blackIsOne) {
    CV_Assert(!frame.empty());
    CV_Assert(corners.size() == 4);

    // Warp to a canonical square
    const int side = 120;
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
                          cv::THRESH_BINARY, 11, 5);

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
            bool isBlack = (meanVal < 128.0);
            int bit = blackIsOne ? (isBlack ? 1 : 0)
                                 : (isBlack ? 0 : 1);
            bits.push_back(bit);
        }
    }

    return bits;
}

void TagDetection::customMarkerDetection(const cv::Mat& frame, int maxCorrectionBits) {
    detectedCorners_.clear();
    detectedIds_.clear();

    // STEP 1: Convert to grayscale for processing
    cv::Mat gray;
    if (frame.channels() == 3 || frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }

    // STEP 2: Threshold to black and white
    cv::Mat binary;
    cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                         cv::THRESH_BINARY_INV, 19, 9);

    // Add white border so edge-touching regions are fully enclosed
    cv::Mat binaryWithBorder;
    cv::copyMakeBorder(binary, binaryWithBorder, 5, 5, 5, 5,
                      cv::BORDER_CONSTANT, cv::Scalar(0));

    // STEP 3: Find all contours in the binary image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryWithBorder, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // STEP 4: Process each contour to see if it's a marker candidate
    for (const auto& contour : contours) {
        // STEP 4a: Simplify the contour to a polygon
        std::vector<cv::Point> approx;
        double epsilon = 0.05 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);

        // STEP 4b: Check if it's a valid quadrilateral
        if (approx.size() != 4 || !cv::isContourConvex(approx)) {
            continue;
        }

        // STEP 4c: Filter out tiny shapes
        double area = cv::contourArea(approx);
        if (area < 100) continue;

        // STEP 4d: Convert to floating point and adjust for border offset
        std::vector<cv::Point2f> corners(4);
        for (int i = 0; i < 4; i++) {
            corners[i] = cv::Point2f(approx[i].x - 5, approx[i].y - 5);
        }

        // STEP 4e: Order corners consistently
        corners = orderCorners(corners);

        // STEP 5: Warp the quadrilateral to a flat square
        const int warpSize = 80;
        std::vector<cv::Point2f> dstPts = {
            cv::Point2f(0, 0),
            cv::Point2f(warpSize - 1, 0),
            cv::Point2f(warpSize - 1, warpSize - 1),
            cv::Point2f(0, warpSize - 1)
        };

        cv::Mat H = cv::getPerspectiveTransform(corners, dstPts);
        cv::Mat warped;
        cv::warpPerspective(gray, warped, H, cv::Size(warpSize, warpSize));

        // STEP 6: Threshold the warped image
        cv::Mat warpedBin;
        cv::adaptiveThreshold(warped, warpedBin, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                            cv::THRESH_BINARY, 11, 3);

        // STEP 8: Extract the inner 6x6 data grid
        int cellSize = warpSize / 8;
        int margin = cellSize * 1;
        cv::Rect innerRect(margin, margin, warpSize - 2*margin, warpSize - 2*margin);
        cv::Mat innerData = warpedBin(innerRect);

        // STEP 9: Read the 6x6 bit pattern
        std::vector<int> extractedBits(36);
        int dataSize = innerData.rows;
        int dataCellSize = dataSize / 6;

        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 6; c++) {
                int y0 = r * dataCellSize;
                int x0 = c * dataCellSize;
                int y1 = (r == 5) ? innerData.rows : (r + 1) * dataCellSize;
                int x1 = (c == 5) ? innerData.cols : (c + 1) * dataCellSize;

                int cellW = x1 - x0;
                int cellH = y1 - y0;
                int centerMargin = std::max(cellW / 4, 1);
                cv::Rect cellRect(x0 + centerMargin, y0 + centerMargin,
                                 cellW - 2*centerMargin, cellH - 2*centerMargin);
                cv::Mat cell = innerData(cellRect);

                double meanVal = cv::mean(cell)[0];
                extractedBits[r * 6 + c] = (meanVal < 128.0) ? 1 : 0;
            }
        }

        // STEP 11: Try matching against the dictionary with 4 rotations
        for (int rot = 0; rot < 4; rot++) {
            for (int markerIdx = 0; markerIdx < dict_->bytesList.rows; markerIdx++) {
                std::vector<int> dictBits(36);
                for (int bit = 0; bit < 36; bit++) {
                    int byteIdx = bit / 8;
                    int bitIdx = 7 - (bit % 8);
                    uchar byteVal = dict_->bytesList.at<uchar>(markerIdx, byteIdx);
                    dictBits[bit] = (byteVal >> bitIdx) & 1;
                }

                int differences = 0;
                for (int i = 0; i < 36; i++) {
                    if (extractedBits[i] != dictBits[i]) {
                        differences++;
                    }
                }

                if (differences <= maxCorrectionBits) {
                    detectedCorners_.push_back(corners);
                    detectedIds_.push_back(markerIdx);
                    goto next_candidate;
                }
            }

            // STEP 11e: Rotate for next iteration
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
