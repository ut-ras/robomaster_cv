#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

using namespace cv;
using namespace cv::aruco;

// Your 10 patterns, 6x6 = 36 bits each, row-major, 1 = black, 0 = white
static std::vector<std::vector<int>> getBasePatterns() {
    return {
        // Marker ID 0 (flipped)
        { 0,0,1,0,1,0,
          1,0,0,0,1,0,
          1,0,0,1,1,1,
          0,1,0,1,1,1,
          1,0,1,0,0,1,
          1,1,1,0,1,1 },

        // Marker ID 1
        { 0,0,1,0,0,1,
          1,0,1,0,0,0,
          0,0,0,0,1,1,
          1,0,0,1,1,1,
          0,1,0,0,1,0,
          1,1,0,1,1,0 },

        // Marker ID 2
        { 0,0,1,0,0,0,
          1,0,1,1,0,1,
          0,1,1,1,1,1,
          1,1,0,1,1,0,
          1,1,1,0,1,1,
          1,1,0,0,0,1 },

        // Marker ID 3
        { 0,0,0,1,1,0,
          1,1,1,0,0,0,
          0,1,1,0,0,0,
          0,1,0,1,1,0,
          0,0,1,1,0,1,
          1,0,0,1,1,1 },

        // Marker ID 4
        { 0,0,0,1,0,1,
          0,0,0,0,1,1,
          0,1,0,0,0,0,
          1,1,0,1,0,1,
          0,1,1,1,1,1,
          0,1,1,1,0,1 },

        // Marker ID 5
        { 0,0,0,0,1,1,
          0,0,1,1,1,0,
          0,0,1,0,0,1,
          0,1,0,1,0,0,
          1,1,0,0,0,1,
          0,1,0,0,1,1 },

        // Marker ID 6
        { 1,1,1,1,1,0,
          1,0,1,0,0,1,
          0,1,0,1,1,0,
          1,0,0,0,1,0,
          1,1,1,1,0,1,
          1,1,1,0,1,0 },

        // Marker ID 7
        { 1,1,1,0,1,1,
          1,1,1,0,0,1,
          1,0,1,0,1,1,
          0,1,0,0,0,1,
          1,1,1,0,0,0,
          1,0,1,0,1,1 },

        // Marker ID 8
        { 1,1,0,1,1,1,
          0,1,0,1,0,0,
          1,1,1,0,0,0,
          1,0,0,0,0,0,
          0,0,0,1,0,1,
          0,1,0,0,1,0 },

        // Marker ID 9
        { 1,1,0,1,1,0,
          0,1,1,0,1,0,
          0,1,0,1,0,0,
          1,0,1,1,1,1,
          1,0,1,1,1,0,
          0,0,1,1,0,1 }
    };
}

// Build a cv::Mat bytesList from basePatterns (N x 5, packed bits)
static cv::Mat buildMarkerBitsMat(const std::vector<std::vector<int>>& patterns,
                                  int markerSize) {
    const int nMarkers      = static_cast<int>(patterns.size());
    const int bitsPerMarker = markerSize * markerSize;  // 36
    const int bytesPerMarker = (bitsPerMarker + 7) / 8; // 5

    cv::Mat markerBits(nMarkers, bytesPerMarker, CV_8UC1, cv::Scalar(0));

    for (int m = 0; m < nMarkers; ++m) {
        const auto& p = patterns[m];
        CV_Assert(static_cast<int>(p.size()) == bitsPerMarker);

        for (int bit = 0; bit < bitsPerMarker; ++bit) {
            int v = p[bit];            // 0 or 1
            if (v == 0) continue;      // leave bit as 0

            int byteIdx = bit / 8;
            int bitIdx  = bit % 8;

            uchar& b = markerBits.at<uchar>(m, byteIdx);
            b |= static_cast<uchar>(1 << (7 - bitIdx)); // MSB first
        }
    }

    return markerBits;
}

// This is what your node should call
cv::Ptr<cv::aruco::Dictionary> createArcMarkersDictionary() {
    const int markerSize = 6;
    auto patterns = getBasePatterns();
    cv::Mat markerBits = buildMarkerBitsMat(patterns, markerSize);

    int nMarkers = markerBits.rows;
    cv::Ptr<cv::aruco::Dictionary> dict =
        cv::aruco::Dictionary::create(nMarkers, markerSize);

    dict->bytesList = markerBits.clone();
    dict->maxCorrectionBits = 0; // start with strict matching

    return dict;
}
