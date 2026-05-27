#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <unordered_map>

/**
 * @class PositionCalculator
 * @brief Calculates 3D positions and poses from marker corners and camera calibration
 * 
 * This class encapsulates all position/pose calculation logic:
 * - PnP solving to compute camera pose relative to marker
 * - Camera position relative to tag (inverted transformation)
 * - Rotation matrix to Euler angle conversion
 * - Reprojection visualization
 */
class PositionCalculator {
public:
    struct MarkerFieldPose {
        cv::Vec2d fieldTranslation2D;
        cv::Matx22d rotationTagToField2D;
    };

    struct markerRotMatrices{
        cv::Matx22d facingUp;
        cv::Matx22d facingDown;
        cv::Matx22d facingLeft;
        cv::Matx22d facingRight;
    };

    /**
     * @brief Initialize with camera calibration and marker size
     * @param cameraMatrix OpenCV camera intrinsic matrix
     * @param distCoeffs OpenCV distortion coefficients
     * @param markerSizeMeters Physical size of the marker in meters
     */
    PositionCalculator(const cv::Mat& cameraMatrix,
                       const cv::Mat& distCoeffs,
                       float markerSizeMeters,
                       const cv::Vec2d& originMeters = cv::Vec2d(0.0, 0.0),
                       const std::string& superRotationName = "positiveXIsRight_positiveYIsUp");

    /**
     * @brief Solve pose (translation + rotation) from marker corners
     * @param imageCorners 4 corner points in image space (TL, TR, BR, BL order)
     * @param tvec Output: translation vector in meters
     * @param eulerAngles Output: rotation as Euler angles (yaw, pitch, roll) in degrees
     * @return true if successful
     */
    bool solvePose(const std::vector<cv::Point2f>& imageCorners,
                   cv::Vec3d& tvec,
                   cv::Vec3d& eulerAngles);

    /**
     * @brief Get camera position in tag's reference frame
     * @param imageCorners 4 corner points in image space (TL, TR, BR, BL order)
     * @param cameraPosition Output: camera's X, Y, Z in tag frame (meters)
     * @return true if successful
     */
    bool getCameraPositionRelativeToTag(const std::vector<cv::Point2f>& imageCorners,
                                        cv::Vec3d& cameraPosition);

     /**
      * @brief Transform camera position from tag frame to 2D field frame
      * @param markerId Marker ID used to lookup field transform
      * @param cameraPositionRelativeToTag Camera position in marker tag frame
      * @param cameraAbsolutePosition2D Output layout: [field_x, field_y]
      * Uses relative [x, -z] as the 2D input before rotation/translation.
      * @return true if marker transform exists and output was computed
      */
    bool getAbsolutePosition2D(int markerId,
                               const cv::Vec3d& cameraPositionRelativeToTag,
                               cv::Vec2d& cameraAbsolutePosition2D) const;

    /**
     * @brief Average a list of positions
     * @return true if at least one position was averaged
     */
    bool averagePositions(const std::vector<cv::Vec3d>& positions,
                          cv::Vec3d& averagedPosition) const;

    /**
     * @brief Average a list of 2D positions
     * @return true if at least one position was averaged
     */
    bool averagePositions2D(const std::vector<cv::Vec2d>& positions,
                            cv::Vec2d& averagedPosition) const;

    /**
     * @brief Visualize reprojection of the marker
     * @param frame Frame to draw on
     * @param imageCorners Detected corner points
     * @param filename Output filename for visualization
     * @return true if visualization was created
     */
    bool visualizeReprojection(const cv::Mat& frame,
                               const std::vector<cv::Point2f>& imageCorners,
                               const std::string& filename = "reprojection_debug.jpg");

    /**
     * @brief Get the marker size in meters
     */
    float getMarkerSize() const;

private:
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    float markerSizeMeters_;
    cv::Vec2d originMeters_;
    std::string superRotationName_;
    std::unordered_map<int, MarkerFieldPose> markerFieldPoses_;

    // Helper methods
    cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat& R);
    cv::Matx22d superRotationMatrixFromName_(const std::string& name) const;
    void initializeDefaultMarkerFieldPoses_();
};
