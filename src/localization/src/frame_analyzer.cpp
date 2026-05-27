#include "../include/frame_analyzer.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <unordered_map>

FrameAnalyzer::FrameAnalyzer(const cv::Ptr<cv::aruco::Dictionary>& dict,
                             const cv::Mat& cameraMatrix,
                             const cv::Mat& distCoeffs,
                     float markerSizeMeters,
                     const cv::Vec2d& originMeters,
                     const std::string& superRotationName)
    : detector_(std::make_unique<TagDetection>(dict, cameraMatrix, distCoeffs)),
    posCalc_(std::make_unique<PositionCalculator>(cameraMatrix, distCoeffs, markerSizeMeters, originMeters, superRotationName)),
    averagedRelativePosition_(0, 0, 0),
    averagedAbsolutePosition2D_(0, 0),
      hasValidDetection_(false)
{
}

bool FrameAnalyzer::analyzeFrame(const cv::Mat& frame)
{
    if (frame.empty()) {
        hasValidDetection_ = false;
        relativeDetections_.clear();
        absoluteDetections2D_.clear();
        return false;
    }

    // Store frame (clone to avoid external modifications)
    // TODO: REMOVE THIS... THIS COULD TAKE UNNECESSARY TIME AS WE DONT NEED THE OG "frame" LATER
    currentFrame_ = frame.clone();

    // Step 1: Detect markers
    if (!detector_->detectMarkers(currentFrame_)) {
        hasValidDetection_ = false;
        detectedIds_.clear();
        detectedCorners_.clear();
        relativeDetections_.clear();
        absoluteDetections2D_.clear();
        return false;
    }

    // Step 2: Extract detection results
    detectedIds_ = detector_->getDetectedIds();
    detectedCorners_ = detector_->getDetectedCorners();

    // Step 3: Process detected markers (calculate positions)
    return processDetectedMarkers_();
}

bool FrameAnalyzer::processDetectedMarkers_()
{
    if (detectedIds_.empty()) {
        hasValidDetection_ = false;
        relativeDetections_.clear();
        absoluteDetections2D_.clear();
        return false;
    }

    relativeDetections_.clear();
    absoluteDetections2D_.clear();

    std::unordered_map<int, std::size_t> lastIdxById;
    for (std::size_t i = 0; i < detectedIds_.size(); ++i) {
        lastIdxById[detectedIds_[i]] = i;
    }

    std::vector<cv::Vec3d> relativePositions;
    std::vector<cv::Vec2d> absolutePositions2D;

    for (const auto& idAndIdx : lastIdxById) {
        const int markerId = idAndIdx.first;
        const std::size_t markerIdx = idAndIdx.second;

        cv::Vec3d relativePosition;
        if (!posCalc_->getCameraPositionRelativeToTag(detectedCorners_[markerIdx], relativePosition)) {
            continue;
        }

        relativeDetections_.push_back({markerId, relativePosition});
        relativePositions.push_back(relativePosition);

        cv::Vec2d absolutePosition2D;
        if (posCalc_->getAbsolutePosition2D(markerId, relativePosition, absolutePosition2D)) {
            absoluteDetections2D_.push_back(absolutePosition2D);
            absolutePositions2D.push_back(absolutePosition2D);
        }
    }

    const bool hasRelativeAverage = posCalc_->averagePositions(relativePositions, averagedRelativePosition_);
    const bool hasAbsoluteAverage = posCalc_->averagePositions2D(absolutePositions2D, averagedAbsolutePosition2D_);

    if (hasRelativeAverage) {
        hasValidDetection_ = true;
        if (!hasAbsoluteAverage) {
            averagedAbsolutePosition2D_ = cv::Vec2d(averagedRelativePosition_[0], -averagedRelativePosition_[2]);
        }
        return true;
    }

    hasValidDetection_ = false;
    return false;
}

cv::Mat FrameAnalyzer::visualizeMarkers(const cv::Mat& sourceFrame) const
{
    const cv::Mat& frame = sourceFrame.empty() ? currentFrame_ : sourceFrame;
    if (frame.empty()) {
        return frame;
    }

    cv::Mat viz = frame.clone();

    // Draw each detected marker
    for (size_t i = 0; i < detectedIds_.size(); ++i) {
        const auto& corners = detectedCorners_[i];
        if (corners.size() < 4) continue;

        // Draw contour
        std::vector<cv::Point> intCorners;
        for (const auto& pt : corners) {
            intCorners.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
        }
        intCorners.push_back(intCorners[0]); // Close the loop

        cv::polylines(viz, intCorners, true, cv::Scalar(0, 255, 0), 2);

        // Draw ID at center
        cv::Point2f center(0, 0);
        for (const auto& pt : corners) {
            center += pt;
        }
        center *= 0.25f;

        std::string idText = "ID:" + std::to_string(detectedIds_[i]);
        cv::putText(viz, idText, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    }

    return viz;
}

cv::Mat FrameAnalyzer::visualizeReprojection(const cv::Mat& sourceFrame) const
{
    const cv::Mat& frame = sourceFrame.empty() ? currentFrame_ : sourceFrame;
    if (frame.empty() || !hasValidDetection_ || detectedCorners_.empty()) {
        return frame;
    }

    cv::Mat viz = frame.clone();

    // Use the position_calculator's visualization method
    // It would need to be public or we call it through the posCalc_ pointer
    // For now, just return the original frame
    // (The actual reprojection visualization would be handled by PositionCalculator)

    return viz;
}

std::string FrameAnalyzer::getPositionText(bool unitCm) const
{
    (void)unitCm;

    if (!hasValidDetection_) {
        return "No detection";
    }

    std::ostringstream oss;
    
    // Convert meters to inches (1 meter = 39.3701 inches)
    int x_inches = static_cast<int>(std::round(averagedAbsolutePosition2D_[0] * 100));
    int z_inches = static_cast<int>(std::round(averagedAbsolutePosition2D_[1] * 100));
    
    oss << "X: " << x_inches << " cm, Z: " << z_inches << " cm";

    return oss.str();
}

cv::Mat FrameAnalyzer::generatePositionOverlay(const cv::Mat& sourceFrame,
                                               const std::string& customText) const
{
    const cv::Mat& frame = sourceFrame.empty() ? currentFrame_ : sourceFrame;
    if (frame.empty()) {
        return frame;
    }

    cv::Mat viz = frame.clone();

    if (!customText.empty()) {
        // Use custom text
        cv::putText(viz, customText, cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar(0, 255, 0), 2);
    } else if (hasValidDetection_) {
        // Use auto-generated position text
        std::string posText = getPositionText(true); // Display in inches
        cv::putText(viz, posText, cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 2);
    } else {
        cv::putText(viz, "No detection", cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2);
    }

    return viz;
}
