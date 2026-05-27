#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "tag_detection.hpp"
#include "position_calculator.hpp"

/**
 * @class FrameAnalyzer
 * @brief High-level frame processing pipeline for marker detection and position calculation
 * 
 * This class encapsulates the complete workflow:
 * - Detecting markers in a frame
 * - Calculating camera position relative to markers
 * - Generating annotated/overlay frames on demand
 * 
 * Design: Reuse the same instance across multiple frames for efficiency
 */
class FrameAnalyzer {
public:
    struct DetectionData {
        int markerId;
        cv::Vec3d positionRelativeToTag;
    };

    /**
     * @brief Initialize the analyzer with camera calibration
     * @param dict ArUco dictionary for marker detection
     * @param cameraMatrix Camera intrinsic matrix
     * @param distCoeffs Distortion coefficients
     * @param markerSizeMeters Physical marker size in meters
     */
    FrameAnalyzer(const cv::Ptr<cv::aruco::Dictionary>& dict,
                  const cv::Mat& cameraMatrix,
                  const cv::Mat& distCoeffs,
                  float markerSizeMeters,
                  const cv::Vec2d& originMeters = cv::Vec2d(0.0, 0.0),
                  const std::string& superRotationName = "positiveXIsRight_positiveYIsUp");

    /**
     * @brief Analyze a new frame (main entry point)
     * @param frame Input image to process
     * @return true if marker(s) detected and position calculated
     */
    bool analyzeFrame(const cv::Mat& frame);

    /**
     * @brief Check if current frame has valid detection
     */
    bool hasValidDetection() const { return hasValidDetection_; }

    /**
     * @brief Get averaged camera position in tag frames (meters)
     * @return Vec3d with [x, y, z] averaged over the last marker per ID
     */
    const cv::Vec3d& getCameraPosition() const { return averagedRelativePosition_; }

    /**
     * @brief Get averaged camera absolute 2D position on the field (meters)
     */
    const cv::Vec2d& getAbsolutePosition() const { return averagedAbsolutePosition2D_; }

    /**
     * @brief Get relative position results for last marker of each detected ID
     */
    const std::vector<DetectionData>& getRelativeDetections() const { return relativeDetections_; }

    /**
     * @brief Get absolute 2D position results for each relative detection
     */
    const std::vector<cv::Vec2d>& getAbsoluteDetections() const { return absoluteDetections2D_; }

    /**
     * @brief Get detected marker IDs in current frame
     */
    const std::vector<int>& getDetectedIds() const { return detectedIds_; }

    /**
     * @brief Get detected marker corners in current frame
     */
    const std::vector<std::vector<cv::Point2f>>& getDetectedCorners() const 
    { 
        return detectedCorners_; 
    }

    /**
     * @brief Get the current processed frame
     */
    const cv::Mat& getCurrentFrame() const { return currentFrame_; }

    /**
     * @brief Get number of detected markers in current frame
     */
    size_t getMarkerCount() const { return detectedIds_.size(); }

    /**
     * @brief Draw markers with ID labels on the frame
     * @param sourceFrame Frame to draw on (optional, uses currentFrame_ if empty)
     * @return Annotated frame with marker outlines and IDs
     */
    cv::Mat visualizeMarkers(const cv::Mat& sourceFrame = cv::Mat()) const;

    /**
     * @brief Draw marker reprojection for debugging
     * @param sourceFrame Frame to draw on (optional, uses currentFrame_ if empty)
     * @return Frame with reprojected marker visualized
     */
    cv::Mat visualizeReprojection(const cv::Mat& sourceFrame = cv::Mat()) const;

    /**
     * @brief Generate frame with position overlay in top-left corner
     * @param sourceFrame Frame to draw on (optional, uses currentFrame_ if empty)
     * @param customText Optional custom text to display instead of auto-generated
     * @return Frame with overlay (or original if no detection)
     */
    cv::Mat generatePositionOverlay(const cv::Mat& sourceFrame = cv::Mat(),
                                    const std::string& customText = "") const;

    /**
     * @brief Get auto-generated position text (e.g., "X: 10.5 cm, Z: 50.2 cm")
     * @param unitCm If true, displays in centimeters; otherwise meters
     * @return Formatted position string
     */
    std::string getPositionText(bool unitCm = true) const;

private:
    // Analysis objects (reused across frames)
    std::unique_ptr<TagDetection> detector_;
    std::unique_ptr<PositionCalculator> posCalc_;

    // Current frame state
    cv::Mat currentFrame_;
    std::vector<int> detectedIds_;
    std::vector<std::vector<cv::Point2f>> detectedCorners_;
    std::vector<DetectionData> relativeDetections_;
    std::vector<cv::Vec2d> absoluteDetections2D_;
    cv::Vec3d averagedRelativePosition_;
    cv::Vec2d averagedAbsolutePosition2D_;
    bool hasValidDetection_;

    // Helper methods
    bool processDetectedMarkers_();
};
