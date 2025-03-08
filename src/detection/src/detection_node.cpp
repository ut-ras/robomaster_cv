#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

using std::placeholders::_1;
using namespace cv;
using namespace std;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::Detection2D;
using vision_msgs::msg::ObjectHypothesisWithPose;

double sim(double a, double b);
double distSq(Point2f a, Point2f b);
double angle(Point2f a, Point2f b);
vector<vector<Point>> getContours(Mat& frame, const string& color);
vector<Point2f> calculateCenters(Mat& frame, const string& color);

Mat applyCanny(Mat& frame) {
    Mat edges;
    GaussianBlur(frame, frame, Size(5, 5), 1.5);
    // Canny with thresholds 100 and 200
    Canny(frame, edges, 100, 200);
    Mat edgesColor;
    cvtColor(edges, edgesColor, COLOR_GRAY2BGR);
    return edgesColor;
}

class CVNode : public rclcpp::Node {
public:
    CVNode() : Node("CVNode"), writer_initialized(false), last_frame_time(this->now()) {
        printf("Initializing subscriber...\n");
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/robot/rs2/color/image_raw/compressed", 10, std::bind(&CVNode::topic_callback, this, _1));

        detections_publisher_ = this->create_publisher<Detection2DArray>("detections", 10);
    }

private:
    rclcpp::Publisher<Detection2DArray>::SharedPtr detections_publisher_;
    bool writer_initialized;
    std::string output_video = "output.mp4";
    rclcpp::Time last_frame_time;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    void topic_callback(const sensor_msgs::msg::CompressedImage &msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            Mat frame = cv_ptr->image;

            Mat edges = applyCanny(frame);

            vector<Point2f> centers = calculateCenters(frame, "blue");

            Detection2DArray detections_msg;
            detections_msg.header = msg.header;

            for (const auto& center : centers) {
                Detection2D detection;
                detection.bbox.center.position.x = center.x;
                detection.bbox.center.position.y = center.y;
                detections_msg.detections.push_back(detection);
            }

            detections_publisher_->publish(detections_msg);

            RCLCPP_INFO(this->get_logger(), "Published %zu detections", detections_msg.detections.size());

            // Update the last received frame timestamp
            last_frame_time = this->now();

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

double sim(double a, double b) {
    return min(a, b) / (a + b);
}

double distSq(Point2f a, Point2f b) {
    return pow(a.x - b.x, 2) + pow(a.y - b.y, 2);
}

double angle(Point2f a, Point2f b) {
    if (a.x == b.x) return 90;
    Point2f right = (a.x > b.x) ? a : b;
    Point2f left = (a.x > b.x) ? b : a;
    return atan((right.y - left.y) / (right.x - left.x)) * 180 / CV_PI;
}

vector<vector<Point>> getContours(Mat& frame, const string& color) {
    Mat frameHSV, mask1, mask2, frameThreshold;

    cvtColor(frame, frameHSV, COLOR_BGR2HSV);

    if (color == "red") {
        inRange(frameHSV, Scalar(0, 70, 50), Scalar(20, 255, 255), mask1);
        inRange(frameHSV, Scalar(170, 70, 50), Scalar(230, 255, 255), mask2);
    } else {
        inRange(frameHSV, Scalar(90, 70, 50), Scalar(120, 255, 255), mask1);
        inRange(frameHSV, Scalar(170, 70, 50), Scalar(200, 255, 255), mask2);
    }
    frameThreshold = mask1 | mask2;

    erode(frameThreshold, frameThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(frameThreshold, frameThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(frameThreshold, frameThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(frameThreshold, frameThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    vector<vector<Point>> contours;
    findContours(frameThreshold, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    return contours;
}

vector<Point2f> calculateCenters(Mat& frame, const string& color) {
    auto contours = getContours(frame, color);
    vector<RotatedRect> bboxes;
    vector<Point2f> centers;

    for (auto& contour : contours) {
        RotatedRect bbox = minAreaRect(contour);
        bboxes.push_back(bbox);

        Point2f bboxPoints[4];
        bbox.points(bboxPoints);
        for (int j = 0; j < 4; j++) {
            line(frame, bboxPoints[j], bboxPoints[(j + 1) % 4], Scalar(0, 255, 0), 2);
        }
    }

    int thresh = 20;
    double widthSimThresh = 0.1, lengthSimThresh = 0.3, yThresh = 15, angleThresh = 15;

    for (size_t i = 0; i < bboxes.size(); i++) {
        auto& bbox1 = bboxes[i];
        Point2f center1 = bbox1.center;
        double width1 = bbox1.size.width;
        double length1 = bbox1.size.height;
        double angle1 = bbox1.angle;

        if (max(length1, width1) < thresh) continue;

        for (size_t j = i + 1; j < bboxes.size(); j++) {
            auto& bbox2 = bboxes[j];
            Point2f center2 = bbox2.center;
            double width2 = bbox2.size.width;
            double length2 = bbox2.size.height;
            double angle2 = bbox2.angle;

            if (max(length2, width2) < thresh) continue;

            double angleDiff = abs(angle1 - angle2);
            double yDiff = abs(center1.y - center2.y);

            if (sim(width1, width2) > widthSimThresh &&
                sim(length1, length2) > lengthSimThresh &&
                yDiff < yThresh &&
                (angleDiff < angleThresh || angleDiff > 180 - angleThresh)) {
                Point centerMid((center1.x + center2.x) / 2, (center1.y + center2.y) / 2);
                circle(frame, centerMid, 10, Scalar(255, 0, 255), -1);
                centers.push_back(centerMid);
            }
        }
    }

    return centers;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CVNode>());
    rclcpp::shutdown();
    return 0;
}