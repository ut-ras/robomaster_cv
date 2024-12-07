#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using std::placeholders::_1;

using namespace cv;
using namespace std;

constexpr char INPUT_TOPIC_NAME[] = "/camera/camera/rgbd";
constexpr char OUTPUT_TOPIC_NAME[] = "/plate_detection/rgbd";

class PlateDetectionNode : public rclcpp::Node {
    rclcpp::Publisher<stampede_msgs::msg::VectorPoint2D>::SharedPtr centers_publisher_;
    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subscription_;

    public:
        PlateDetectionNode() : Node("plate_detection_node") {
            this->declare_parameter("target_color", "red");
            centers_publisher_ = this->create_publisher<stampede_msgs::msg::VectorPoint2D>(OUTPUT_TOPIC_NAME, 1);
            subscription_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(INPUT_TOPIC_NAME, 1, std::bind(&PlateDetectionNode::topic_callback, this, _1));
        }

    private:
        // Measures how similar two numbers are
        double sim(double a, double b) {
            return min(a, b) / (a + b);
        }

        // Calculates the squared Euclidean distance between two points
        double distSq(Point2f a, Point2f b) {
            return pow(a.x - b.x, 2) + pow(a.y - b.y, 2);
        }

        // Calculates the angle between two points in degrees
        double angle(Point2f a, Point2f b) {
            if (a.x == b.x) return 90;
            Point2f right = (a.x > b.x) ? a : b;
            Point2f left = (a.x > b.x) ? b : a;
            return atan((right.y - left.y) / (right.x - left.x)) * 180 / CV_PI;
        }

        // Applies a mask to detect color-specific contours in the frame
        vector<vector<Point>> getContours(Mat& frame) {
            Mat frameHSV, mask1, mask2, frameThreshold;

            cvtColor(frame, frameHSV, COLOR_BGR2HSV);

            if (this->get_parameter("target_color").as_string() == "red") {
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

        // Draws the bounding box centers and processes relationships between them
        vector<Point2f> getCenters(Mat frame) {
            vector<Point2f> centers;

            auto contours = getContours(frame);
            vector<RotatedRect> bboxes;

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
            double widthSimThresh = 0.1, lengthSimThresh = 0.3, yThresh = 0.15, angleThresh = 15;

            for (size_t i = 0; i < bboxes.size(); i++) {
                auto& bbox1 = bboxes[i];
                Point2f center1 = bbox1.center;

                Point2f vert1[4];
                bbox1.points(vert1);

                int longest[] = {0, 1};
                Point2f bl = vert1[0];
                Point2f tl = vert1[1];
                Point2f br = vert1[3];
                double width1 = sqrt(distSq(vert1[0], vert1[3]));
                if (distSq(bl, br) > distSq(bl, tl)) {
                    longest[1] = 3;
                    width1 = sqrt(distSq(vert1[0], vert1[1]));
                }
                double length1 = sqrt(distSq(vert1[longest[0]], vert1[longest[1]]));
                double angle1 = angle(vert1[longest[0]], vert1[longest[1]]);

                if (max(length1, width1) < thresh) continue;

                for (size_t j = i + 1; j < bboxes.size(); j++) {
                    auto& bbox2 = bboxes[j];
                    Point2f center2 = bbox2.center;

                    Point2f vert2[4];
                    bbox2.points(vert2);

                    int longest[] = {0, 1};
                    Point2f bl = vert2[0];
                    Point2f tl = vert2[1];
                    Point2f br = vert2[3];
                    double width2 = sqrt(distSq(vert2[0], vert2[3]));
                    if (distSq(bl, br) > distSq(bl, tl)) {
                        longest[1] = 3;
                        width2 = sqrt(distSq(vert2[0], vert2[1]));
                    }
                    double length2 = sqrt(distSq(vert2[longest[0]], vert2[longest[1]]));
                    double angle2 = angle(vert2[longest[0]], vert2[longest[1]]);

                    if (max(length2, width2) < thresh) continue;

                    double angleDiff = abs(angle1 - angle2);
                    double yDiff = abs(center1.y - center2.y);

                    if (sim(width1, width2) > widthSimThresh &&
                        sim(length1, length2) > lengthSimThresh &&
                        yDiff < yThresh * (length1 + length2) / 2.0 &&
                        (angleDiff < angleThresh || angleDiff > 180 - angleThresh)) {
                        Point centerMid((center1.x + center2.x) / 2, (center1.y + center2.y) / 2);
                        centers.push_back(centerMid);
                        // circle(frame, centerMid, 10, Scalar(255, 0, 255), -1);
                    }
                }
            }

            return centers;
        }

        void topic_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
              cv_ptr = cv_bridge::toCvCopy(msg.rgb, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e) {
              RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
              return;
            }

            vector<Point2f> centers = getCenters(cv_ptr->image);
            stampede_msgs::msg::VectorPoint2D centers_msg;

            for (int i = 0; i < centers.size(); i++) {
                vision_msgs::Point2D center;
                center.x = centers[i].x;
                center.y = centers[i].y;
                stampede_msgs::msg::centers_msg.push_back(center);
            }

            centers_publisher_->publish(*centers_msg);
        }
};

int main(int argc, char ** argv) {
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlateDetectionNode>());
    rclcpp::shutdown();

    return 0;
}

