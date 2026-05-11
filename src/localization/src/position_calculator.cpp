#include "position_calculator.hpp"
#include <iostream>
#include <cmath>

using namespace cv;

PositionCalculator::PositionCalculator(const cv::Mat& cameraMatrix,
                                       const cv::Mat& distCoeffs,
                                       float markerSizeMeters)
    : cameraMatrix_(cameraMatrix),
      distCoeffs_(distCoeffs),
      markerSizeMeters_(markerSizeMeters) {
        initializeDefaultMarkerFieldPoses_();
}

bool PositionCalculator::solvePose(const std::vector<cv::Point2f>& imageCorners,
                                   cv::Vec3d& tvec,
                                   cv::Vec3d& eulerAngles) {
    if (imageCorners.size() != 4) {
        std::cerr << "PositionCalculator::solvePose - Expected 4 corners, got " 
                  << imageCorners.size() << std::endl;
        return false;
    }

    // Define 3D object points (marker in world frame, z=0, centered)
    std::vector<Point3f> objPts = {
        { -markerSizeMeters_/2.f, -markerSizeMeters_/2.f, 0.f },  // TL
        {  markerSizeMeters_/2.f, -markerSizeMeters_/2.f, 0.f },  // TR
        {  markerSizeMeters_/2.f,  markerSizeMeters_/2.f, 0.f },  // BR
        { -markerSizeMeters_/2.f,  markerSizeMeters_/2.f, 0.f }   // BL
    };

    Mat rvec, tvecMat;
    bool ok = solvePnP(objPts, imageCorners, cameraMatrix_, distCoeffs_,
                       rvec, tvecMat, false, SOLVEPNP_IPPE_SQUARE);

    if (!ok) {
        std::cerr << "PositionCalculator::solvePose - solvePnP failed" << std::endl;
        return false;
    }

    // Convert rotation vector to rotation matrix, then to Euler angles
    Mat R;
    Rodrigues(rvec, R);
    eulerAngles = rotationMatrixToEulerAngles(R);

    // Extract translation vector
    tvec = Vec3d(tvecMat.at<double>(0, 0),
                 tvecMat.at<double>(1, 0),
                 tvecMat.at<double>(2, 0));

    double dist_m = cv::norm(tvec);
std::cout << "tvec (m): [" << tvec[0] << ", " << tvec[1] << ", " << tvec[2] << "]"
          << "  norm (m): " << dist_m
          << "  norm (mm): " << dist_m*1000.0 << std::endl;

          

    return true;
}

bool PositionCalculator::getCameraPositionRelativeToTag(const std::vector<cv::Point2f>& imageCorners,
                                                        cv::Vec3d& cameraPosition) {
    if (imageCorners.size() != 4) {
        std::cerr << "PositionCalculator::getCameraPositionRelativeToTag - Expected 4 corners, got "
                  << imageCorners.size() << std::endl;
        return false;
    }

    // Define 3D object points (marker in world frame)
    std::vector<Point3f> objPts = {
        { -markerSizeMeters_/2.f, markerSizeMeters_/2.f, 0.f },  // TL
        {  markerSizeMeters_/2.f, markerSizeMeters_/2.f, 0.f },  // TR
        {  markerSizeMeters_/2.f,  -markerSizeMeters_/2.f, 0.f },  // BR
        { -markerSizeMeters_/2.f,  -markerSizeMeters_/2.f, 0.f }   // BL
    };

    Mat rvec, tvecMat;
    cv::Mat cameraMatrixDouble;
    cameraMatrix_.convertTo(cameraMatrixDouble, CV_64F);
    cv::Mat distCoefDouble;
    distCoeffs_.convertTo(distCoefDouble, CV_64F);
    bool ok = solvePnP(objPts, imageCorners, cameraMatrixDouble, distCoefDouble,
                       rvec, tvecMat, false, SOLVEPNP_IPPE_SQUARE);

    if (!ok) {
        std::cerr << "PositionCalculator::getCameraPositionRelativeToTag - solvePnP failed" << std::endl;
        return false;
    }

    // Convert rotation vector to rotation matrix
    Mat R;
    Rodrigues(rvec, R);

    // Invert the transformation to get camera position in tag frame
    // R_inv = R^T (transpose), t_inv = -R^T * t
    Mat R_inv = R.t();
    Mat tvec_inv = -R_inv * tvecMat;

    cameraPosition = Vec3d(tvec_inv.at<double>(0, 0),
                           tvec_inv.at<double>(1, 0),
                           tvec_inv.at<double>(2, 0));

    return true;
}

bool PositionCalculator::getAbsolutePosition2D(int markerId,
                                               const cv::Vec3d& cameraPositionRelativeToTag,
                                               cv::Vec2d& cameraAbsolutePosition2D) const {
    const auto markerIt = markerFieldPoses_.find(markerId);
    if (markerIt == markerFieldPoses_.end()) {
        return false;
    }

    const MarkerFieldPose& markerPose = markerIt->second;

    // Field is treated as 2D: use tag-relative x and NEGATED z.
    const cv::Vec2d relative2D(cameraPositionRelativeToTag[0], -cameraPositionRelativeToTag[2]);
    cameraAbsolutePosition2D = markerPose.rotationTagToField2D * relative2D + markerPose.fieldTranslation2D;
    return true;
}

bool PositionCalculator::averagePositions(const std::vector<cv::Vec3d>& positions,
                                          cv::Vec3d& averagedPosition) const {
    if (positions.empty()) {
        return false;
    }

    cv::Vec3d accumulator(0.0, 0.0, 0.0);
    for (const auto& pos : positions) {
        accumulator += pos;
    }

    averagedPosition = accumulator * (1.0 / static_cast<double>(positions.size()));
    return true;
}

bool PositionCalculator::averagePositions2D(const std::vector<cv::Vec2d>& positions,
                                            cv::Vec2d& averagedPosition) const {
    if (positions.empty()) {
        return false;
    }

    cv::Vec2d accumulator(0.0, 0.0);
    for (const auto& pos : positions) {
        accumulator += pos;
    }

    averagedPosition = accumulator * (1.0 / static_cast<double>(positions.size()));
    return true;
}

bool PositionCalculator::visualizeReprojection(const cv::Mat& frame,
                                               const std::vector<cv::Point2f>& imageCorners,
                                               const std::string& filename) {
    if (imageCorners.size() != 4) {
        std::cerr << "PositionCalculator::visualizeReprojection - Expected 4 corners" << std::endl;
        return false;
    }

    // Solve for rotation and translation
    std::vector<Point3f> objPts = {
        { -markerSizeMeters_/2.f, -markerSizeMeters_/2.f, 0.f },
        {  markerSizeMeters_/2.f, -markerSizeMeters_/2.f, 0.f },
        {  markerSizeMeters_/2.f,  markerSizeMeters_/2.f, 0.f },
        { -markerSizeMeters_/2.f,  markerSizeMeters_/2.f, 0.f }
    };

    Mat rvec, tvecMat;
    cv::Mat cameraMatrixDouble;
    cameraMatrix_.convertTo(cameraMatrixDouble, CV_64F);
    bool ok = solvePnP(objPts, imageCorners, cameraMatrixDouble, distCoeffs_,
                       rvec, tvecMat, false, SOLVEPNP_IPPE_SQUARE);

    if (!ok) {
        std::cerr << "PositionCalculator::visualizeReprojection - solvePnP failed" << std::endl;
        return false;
    }

    // Project 3D points back to 2D
    std::vector<Point2f> reprojectedCorners;
    projectPoints(objPts, rvec, tvecMat, cameraMatrix_, distCoeffs_, reprojectedCorners);

    Mat debugFrame = frame.clone();

    // Draw detected corners in GREEN (circles)
    for (size_t i = 0; i < imageCorners.size(); i++) {
        circle(debugFrame, imageCorners[i], 8, Scalar(0, 255, 0), -1);
        putText(debugFrame, "D" + std::to_string(i), imageCorners[i] + Point2f(10, 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
    }

    // Draw reprojected corners in RED (crosses)
    for (size_t i = 0; i < reprojectedCorners.size(); i++) {
        Point2f pt = reprojectedCorners[i];
        line(debugFrame, pt + Point2f(-8, 0), pt + Point2f(8, 0), Scalar(0, 0, 255), 2);
        line(debugFrame, pt + Point2f(0, -8), pt + Point2f(0, 8), Scalar(0, 0, 255), 2);
        putText(debugFrame, "R" + std::to_string(i), pt + Point2f(10, -10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
    }

    // Draw coordinate axes
    std::vector<Point3f> axisPoints = {
        {0, 0, 0},                        // Origin
        {markerSizeMeters_, 0, 0},        // X-axis
        {0, markerSizeMeters_, 0},        // Y-axis
        {0, 0, -markerSizeMeters_}        // Z-axis
    };
    std::vector<Point2f> imageAxisPoints;
    projectPoints(axisPoints, rvec, tvecMat, cameraMatrix_, distCoeffs_, imageAxisPoints);

    line(debugFrame, imageAxisPoints[0], imageAxisPoints[1], Scalar(0, 0, 255), 3);  // X: red
    line(debugFrame, imageAxisPoints[0], imageAxisPoints[2], Scalar(0, 255, 0), 3);  // Y: green
    line(debugFrame, imageAxisPoints[0], imageAxisPoints[3], Scalar(255, 0, 0), 3);  // Z: blue

    putText(debugFrame, "X", imageAxisPoints[1], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    putText(debugFrame, "Y", imageAxisPoints[2], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
    putText(debugFrame, "Z", imageAxisPoints[3], FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);

    // Calculate reprojection error
    double totalError = 0.0;
    for (size_t i = 0; i < imageCorners.size(); i++) {
        double error = norm(imageCorners[i] - reprojectedCorners[i]);
        totalError += error;
    }
    double avgError = totalError / imageCorners.size();

    putText(debugFrame, "Avg Reproj Error: " + std::to_string(avgError) + "px",
            Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 0), 2);
    putText(debugFrame, "GREEN=Detected, RED=Reprojected",
            Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);

    imwrite(filename, debugFrame);
    return true;
}

float PositionCalculator::getMarkerSize() const {
    return markerSizeMeters_;
}

// ============================================================================
// PRIVATE HELPER METHODS
// ============================================================================

cv::Vec3d PositionCalculator::rotationMatrixToEulerAngles(const cv::Mat& R) {
    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                          R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6;

    double yaw, pitch, roll;
    if (!singular) {
        yaw = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        roll = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        yaw = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        roll = 0.0;
    }

    return cv::Vec3d(yaw * 180.0 / M_PI, pitch * 180.0 / M_PI, roll * 180.0 / M_PI);
}

void PositionCalculator::initializeDefaultMarkerFieldPoses_() {
    markerFieldPoses_.clear();

    // Temporary field map: replace with measured values for competition field.
    markerFieldPoses_[3] = {
        cv::Vec2d(1.0, 0.0),
        cv::Matx22d::eye()
    };

    markerFieldPoses_[4] = {
        cv::Vec2d(0.0, 1.0),
        cv::Matx22d(
            0.0, -1.0,
            1.0, 0.0)
    };

    markerFieldPoses_[5] = {
        cv::Vec2d(-1.0, 0.0),
        cv::Matx22d(
            -1.0, 0.0,
            0.0, -1.0)
    };

    markerFieldPoses_[6] = {
        cv::Vec2d(0.0, -1.0),
        cv::Matx22d(
            0.0, 1.0,
            -1.0, 0.0)
    };
}
