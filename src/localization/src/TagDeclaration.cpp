#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

using namespace cv;
using namespace cv::aruco;

// Helper: pack a vector of 0/1 bits into a row of bytes
cv::Mat packBitsToBytes(const std::vector<int>& bits, int markerSize) {
    int totalBits = markerSize * markerSize;
    CV_Assert((int)bits.size() == totalBits);

    int nbytes = (totalBits + 7) / 8; // ceil(totalBits / 8)
    cv::Mat row(1, nbytes, CV_8UC1, cv::Scalar(0));

    for (int i = 0; i < totalBits; ++i) {
        int byteIdx = i / 8;
        int bitIdx  = 7 - (i % 8); // MSB first (OpenCV format)
        if (bits[i]) {
            row.at<uchar>(0, byteIdx) |= (1 << bitIdx);
        }
    }

    return row;
}

cv::Ptr<cv::aruco::Dictionary> createCustomDictionary() {
    const int markerSize = 6;    // inner grid size (4x4 as an example)
    const int numMarkers = 10;   // you have 10 custom markers
    const int totalBits = markerSize * markerSize;
    const int nbytes = (totalBits + 7) / 8;

    // Initialize empty dictionary
    auto dict = cv::aruco::generateCustomDictionary(0, markerSize);
    dict->bytesList = cv::Mat(numMarkers, nbytes, CV_8UC1, cv::Scalar(0));
    dict->markerSize = markerSize;
    dict->maxCorrectionBits = 0; // start with 0; adjust later if desired

    // TODO: fill these with your real bit patterns
    // Order is row-major over inner grid (top-left to bottom-right).
    // Example: marker0bits[0] is (row 0, col 0), marker0bits[1] is (row 0, col 1), etc.
    std::vector<std::vector<int>> markersBits(numMarkers);

    // Example dummy pattern: checkerboard-like
        markersBits[0] = { 0,0,1,0,1,0,
          1,0,0,0,1,0,
          1,0,0,1,1,1,
          0,1,0,1,1,1,
          1,0,1,0,0,1,
          1,1,1,0,1,1 };

        // Marker ID 1
        markersBits[1] = { 0,0,1,0,0,1,
          1,0,1,0,0,0,
          0,0,0,0,1,1,
          1,0,0,1,1,1,
          0,1,0,0,1,0,
          1,1,0,1,1,0 };

        // Marker ID 2
        markersBits[2] = { 0,0,1,0,0,0,
          1,0,1,1,0,1,
          0,1,1,1,1,1,
          1,1,0,1,1,0,
          1,1,1,0,1,1,
          1,1,0,0,0,1 };

        // Marker ID 3
        markersBits[3] = { 0,0,0,1,1,0,
          1,1,1,0,0,0,
          0,1,1,0,0,0,
          0,1,0,1,1,0,
          0,0,1,1,0,1,
          1,0,0,1,1,1 };

        // Marker ID 4
        markersBits[4] = { 0,0,0,1,0,1,
          0,0,0,0,1,1,
          0,1,0,0,0,0,
          1,1,0,1,0,1,
          0,1,1,1,1,1,
          0,1,1,1,0,1 };

        // Marker ID 5
        markersBits[5] = { 0,0,0,0,1,1,
          0,0,1,1,1,0,
          0,0,1,0,0,1,
          0,1,0,1,0,0,
          1,1,0,0,0,1,
          0,1,0,0,1,1 };

        // Marker ID 6
        markersBits[6] = { 1,1,1,1,1,0,
          1,0,1,0,0,1,
          0,1,0,1,1,0,
          1,0,0,0,1,0,
          1,1,1,1,0,1,
          1,1,1,0,1,0 };

        // Marker ID 7
        markersBits[7] = { 1,1,1,0,1,1,
          1,1,1,0,0,1,
          1,0,1,0,1,1,
          0,1,0,0,0,1,
          1,1,1,0,0,0,
          1,0,1,0,1,1 };

        // Marker ID 8
        markersBits[8] = { 1,1,0,1,1,1,
          0,1,0,1,0,0,
          1,1,1,0,0,0,
          1,0,0,0,0,0,
          0,0,0,1,0,1,
          0,1,0,0,1,0 };

        // Marker ID 9
        markersBits[9] = { 1,1,0,1,1,0,
          0,1,1,0,1,0,
          0,1,0,1,0,0,
          1,0,1,1,1,1,
          1,0,1,1,1,0,
          0,0,1,1,0,1 }; 
    // Fill markersBits[1] ... markersBits[9] with your actual 0/1 patterns

    for (int i = 0; i < numMarkers; ++i) {
        cv::Mat row = packBitsToBytes(markersBits[i], markerSize);
        row.copyTo(dict->bytesList.row(i));
    }


    return dict;
}



