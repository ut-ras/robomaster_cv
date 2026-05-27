#pragma once

#include <cstddef>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class TagDetection {
public:
	TagDetection(const cv::Ptr<cv::aruco::Dictionary>& dict,
				 const cv::Mat& cameraMatrix,
				 const cv::Mat& distCoeffs);

	bool detectMarkers(const cv::Mat& frame, int maxCorrectionBits = 5);

	const std::vector<std::vector<cv::Point2f>>& getDetectedCorners() const;
	const std::vector<int>& getDetectedIds() const;
	std::size_t getMarkerCount() const;

	void printDetectedMarkers() const;

	cv::Mat visualizeDetectedMarkers(const cv::Mat& frame,
									 const std::string& filename = "") const;

private:
	cv::Ptr<cv::aruco::Dictionary> dict_;
	cv::Mat cameraMatrix_;
	cv::Mat distCoeffs_;

	std::vector<std::vector<cv::Point2f>> detectedCorners_;
	std::vector<int> detectedIds_;

	std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point2f>& pts);
	std::vector<int> sampleMarkerBits(const cv::Mat& frame,
									  const std::vector<cv::Point2f>& corners,
									  bool blackIsOne = true);
	void customMarkerDetection(const cv::Mat& frame, int maxCorrectionBits);
};
