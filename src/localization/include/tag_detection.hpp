#pragma once

#include <cstddef>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>

/**
 * @class TagDetection
 * @brief Handles detection and analysis of ArUco markers in images
 * 
 * This class encapsulates all marker detection logic including:
 * - Custom marker detection with perspective correction
 * - Bit pattern extraction and matching
 * - Corner ordering and validation
 */
class TagDetection {
public:
    /**
     * @brief Initialize with camera calibration data and dictionary
     */
    TagDetection(const cv::Ptr<cv::aruco::Dictionary>& dict, 
                 const cv::Mat& cameraMatrix,
                 const cv::Mat& distCoeffs);

    /**
     * @brief Detect markers in the given frame
     * @param frame Input image
     * @param maxCorrectionBits Maximum bit errors allowed in pattern matching
     * @return true if markers were detected
     */
    bool detectMarkers(const cv::Mat& frame, int maxCorrectionBits = 5);

    /**
     * @brief Get the corners of all detected markers
     * @return Vector of corner point vectors (each marker has 4 corners: TL, TR, BR, BL)
     */
    const std::vector<std::vector<cv::Point2f>>& getDetectedCorners() const;

    /**
     * @brief Get the IDs of all detected markers
     * @return Vector of marker IDs
     */
    const std::vector<int>& getDetectedIds() const;

    /**
     * @brief Get the number of detected markers
     */
    std::size_t getMarkerCount() const;

    /**
     * @brief Print detected marker information to console
     */
    void printDetectedMarkers() const;

    /**
     * @brief Draw detected markers on a frame for visualization
     * @param frame Frame to draw on
     * @param filename Optional filename to save the annotated frame
     * @return Annotated frame with markers drawn
     */
    cv::Mat visualizeDetectedMarkers(const cv::Mat& frame, 
                                     const std::string& filename = "") const;

private:
    cv::Ptr<cv::aruco::Dictionary> dict_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    std::vector<std::vector<cv::Point2f>> detectedCorners_;
    std::vector<int> detectedIds_;

    // Helper methods
    std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point2f>& pts);
    std::vector<int> sampleMarkerBits(const cv::Mat& frame,
                                     const std::vector<cv::Point2f>& corners,
                                     bool blackIsOne = true);
    void customMarkerDetection(const cv::Mat& frame, int maxCorrectionBits);
};
