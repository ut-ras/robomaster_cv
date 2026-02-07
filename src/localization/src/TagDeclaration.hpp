#pragma once
#include <opencv2/aruco.hpp>
#include <string>

cv::Ptr<cv::aruco::Dictionary> createCustomDictionary();

void saveMarkerImages(const cv::Ptr<cv::aruco::Dictionary>& dict, 
                      const std::string& outputDir = ".", 
                      int markerSizePixels = 200);
