#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <unordered_map>

class PositionCalculator {
public:
	struct MarkerFieldPose {
		cv::Vec2d fieldTranslation2D;
		cv::Matx22d rotationTagToField2D;
	};

	PositionCalculator(const cv::Mat& cameraMatrix,
					   const cv::Mat& distCoeffs,
					   float markerSizeMeters,
					   const cv::Vec2d& originMeters = cv::Vec2d(0.0, 0.0),
					   const std::string& superRotationName = "positiveXIsRight_positiveYIsUp");

	bool solvePose(const std::vector<cv::Point2f>& imageCorners,
				   cv::Vec3d& tvec,
				   cv::Vec3d& eulerAngles);

	bool getCameraPositionRelativeToTag(const std::vector<cv::Point2f>& imageCorners,
										cv::Vec3d& cameraPosition);

	bool getAbsolutePosition2D(int markerId,
						   const cv::Vec3d& cameraPositionRelativeToTag,
						   cv::Vec2d& cameraAbsolutePosition2D) const;

	bool averagePositions(const std::vector<cv::Vec3d>& positions,
					 cv::Vec3d& averagedPosition) const;

	bool averagePositions2D(const std::vector<cv::Vec2d>& positions,
					   cv::Vec2d& averagedPosition) const;

	bool visualizeReprojection(const cv::Mat& frame,
							   const std::vector<cv::Point2f>& imageCorners,
							   const std::string& filename = "reprojection_debug.jpg");

	float getMarkerSize() const;

private:
	cv::Mat cameraMatrix_;
	cv::Mat distCoeffs_;
	float markerSizeMeters_;
	cv::Vec2d originMeters_;
	std::string superRotationName_;
	std::unordered_map<int, MarkerFieldPose> markerFieldPoses_;

	cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat& R);
	cv::Matx22d superRotationMatrixFromName_(const std::string& name) const;
	void initializeDefaultMarkerFieldPoses_();
};
