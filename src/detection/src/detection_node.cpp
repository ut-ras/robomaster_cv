#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/parameter.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <librealsense2/rs.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace cv;
using namespace std;
using vision_msgs::msg::Detection3DArray;
using vision_msgs::msg::Detection3D;
using vision_msgs::msg::ObjectHypothesisWithPose;

double sim(double a, double b);
double distSq(Point2f a, Point2f b);
double angle(Point2f a, Point2f b);

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
    CVNode() : Node("CVNode"), intrinsics_initialized_(false), writer_initialized_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing subscriber...");

        this->declare_parameter<bool>("debug", false);
        this->declare_parameter<bool>("write_color_video", false);
        this->declare_parameter<bool>("write_depth_video", false);
        this->declare_parameter<string>("color_output_path", "color_output.mp4");
        this->declare_parameter<string>("depth_output_path", "depth_output.mp4");
        this->get_parameter("debug", flag_debug_);
        this->get_parameter("write_color_video", flag_write_color_video_);
        this->get_parameter("write_depth_video", flag_write_depth_video_);
        this->get_parameter("color_output_path", output_color_video_path_);
        this->get_parameter("depth_output_path", output_depth_video_path_);
        
        rgbd_sub_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
            "/robot/rs2/rgbd", 10, std::bind(&CVNode::rgbd_callback, this, _1));
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/robot/rs2/aligned_depth_to_color/camera_info", 10, std::bind(&CVNode::camera_info_callback, this, _1));
        detections_publisher_ = this->create_publisher<Detection3DArray>("detections", 10);
    }

private:
    rclcpp::Publisher<Detection3DArray>::SharedPtr detections_publisher_;
    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    bool intrinsics_initialized_;
    rs2_intrinsics intrinsics_;

    bool flag_debug_;
    bool flag_write_color_video_;
    bool flag_write_depth_video_;
    string output_color_video_path_;
    string output_depth_video_path_;

    bool writer_initialized_;
    cv::VideoWriter color_writer_;
    cv::VideoWriter depth_writer_;

    // New rgbd callback
    void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg) {
        if (!intrinsics_initialized_) {
            RCLCPP_WARN(this->get_logger(), "Camera intrinsics not initialized yet, skipping frame processing.");
            return;
        }
        if (msg->rgb.data.empty() || msg->depth.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty RGB or depth image, skipping frame processing.");
            return;
        }

        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->rgb, sensor_msgs::image_encodings::BGR8);
            Mat rgb_frame = cv_ptr->image;

            // Robustly decode depth image (handle 16UC1 and 32FC1)
            cv_bridge::CvImagePtr cv_depth_ptr;
            Mat depth_frame;
            try {
                cv_depth_ptr = cv_bridge::toCvCopy(msg->depth, sensor_msgs::image_encodings::TYPE_16UC1);
                depth_frame = cv_depth_ptr->image;
                depth_frame.convertTo(depth_frame, CV_32FC1, 0.001); // mm to meters
            } catch (...) {
                cv_depth_ptr = cv_bridge::toCvCopy(msg->depth, sensor_msgs::image_encodings::TYPE_32FC1);
                depth_frame = cv_depth_ptr->image;
            }

            Mat edges = applyCanny(rgb_frame);
            vector<Point2f> centers = calculateCenters(rgb_frame, "blue");

            Detection3DArray detections_msg;
            detections_msg.header = msg->rgb.header;
            // 3D detection array
            for (const auto& center : centers) {
                Detection3D detection;
                detection.bbox.center.position.x = center.x;
                detection.bbox.center.position.y = center.y;

                // Get depth at (x, y)
                int px = static_cast<int>(center.x);
                int py = static_cast<int>(center.y);
                float z = 0.0f;
                if (px >= 0 && py >= 0 && px < depth_frame.cols && py < depth_frame.rows) {
                    z = depth_frame.at<float>(py, px); // meters
                }
                
                float pixel[2] = {center.x, center.y};
                float point[3];
                rs2_deproject_pixel_to_point(point, &intrinsics_, pixel, z);

                detection.bbox.center.position.x = point[0];
                detection.bbox.center.position.y = point[1];
                detection.bbox.center.position.z = point[2];
                
                detections_msg.detections.push_back(detection);
            }
            detections_publisher_->publish(detections_msg);

            RCLCPP_INFO(this->get_logger(), "Published %zu 3D detections", detections_msg.detections.size());

            // Write color video
            if (flag_write_color_video_ || flag_write_depth_video_) {
                if (!writer_initialized_) {
                    int fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G');
                    if (flag_write_color_video_) {
                        color_writer_.open(output_color_video_path_, fourcc, 30, rgb_frame.size(), true);
                        if (!color_writer_.isOpened()) {
                            RCLCPP_ERROR(this->get_logger(), "Could not open color video writer");
                        }
                    }
                    if (flag_write_depth_video_) {
                        // Visualize depth for video (normalize to 0-255)
                        Mat depth_vis;
                        normalize(depth_frame, depth_vis, 0, 255, NORM_MINMAX, CV_8UC1);
                        cvtColor(depth_vis, depth_vis, COLOR_GRAY2BGR);
                        depth_writer_.open(output_depth_video_path_, fourcc, 30, depth_vis.size(), true);
                        if (!depth_writer_.isOpened()) {
                            RCLCPP_ERROR(this->get_logger(), "Could not open depth video writer");
                        }
                    }
                    writer_initialized_ = true;
                }
                if (flag_write_color_video_) {
                    color_writer_.write(rgb_frame);
                }
                if (flag_write_depth_video_) {
                    Mat depth_vis;
                    normalize(depth_frame, depth_vis, 0, 255, NORM_MINMAX, CV_8UC1);
                    cvtColor(depth_vis, depth_vis, COLOR_GRAY2BGR);
                    depth_writer_.write(depth_vis);
                }
            }
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!intrinsics_initialized_) {
            intrinsics_.width = msg->width;
            intrinsics_.height = msg->height;
            intrinsics_.ppx = msg->k[2];
            intrinsics_.ppy = msg->k[5];
            intrinsics_.fx = msg->k[0];
            intrinsics_.fy = msg->k[4];
            intrinsics_.model = RS2_DISTORTION_NONE; // Assuming no distortion for simplicity
            for (int i = 0; i < msg->d.size(); i++) {
                intrinsics_.coeffs[i] = msg->d[i];
            }
            intrinsics_initialized_ = true;

            RCLCPP_INFO(this->get_logger(), "Camera intrinsics initialized: %dx%d", intrinsics_.width, intrinsics_.height);
        }
    }

    vector<vector<Point>> getContours(Mat& frame, const string& color) {
        Mat frameHSV, mask1, mask2, frameThreshold;

        cvtColor(frame, frameHSV, COLOR_BGR2HSV);

        if (color == "red") {
            inRange(frameHSV, Scalar(0, 70, 50), Scalar(20, 255, 255), mask1);
            inRange(frameHSV, Scalar(170, 70, 50), Scalar(230, 255, 255), mask2);
        } else {
            inRange(frameHSV, Scalar(90, 100, 100), Scalar(115, 255, 255), mask1);
            inRange(frameHSV, Scalar(115, 100, 100), Scalar(135, 255, 255), mask2);
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
            if (flag_write_color_video_) {
                for (int j = 0; j < 4; j++) {
                    line(frame, bboxPoints[j], bboxPoints[(j + 1) % 4], Scalar(0, 255, 0), 2);
                }
            }
        }

        int thresh = 20;
        double widthSimThresh = 0.1, lengthSimThresh = 0.3, yThresh = 0.15, angleThresh = 15;

        for (size_t i = 0; i < bboxes.size(); i++) {
            auto& bbox1 = bboxes[i];
            Point2f center1 = bbox1.center;
            double width1 = bbox1.size.width;
            double length1 = bbox1.size.height;
            double angle1 = bbox1.angle;

            if (length1 < width1) {
                swap(length1, width1); // Ensure length1 is always the longer dimension
                angle1 += 90; // Adjust angle accordingly
                if (angle1 >= 180) {
                    angle1 -= 180; // Normalize angle to be within [0, 180)
                }
            }

            if (max(length1, width1) < thresh) continue;

            for (size_t j = i + 1; j < bboxes.size(); j++) {
                auto& bbox2 = bboxes[j];
                Point2f center2 = bbox2.center;
                double width2 = bbox2.size.width;
                double length2 = bbox2.size.height;
                double angle2 = bbox2.angle;

                if (length2 < width2) {
                    swap(length2, width2); // Ensure length2 is always the longer dimension
                    angle2 += 90; // Adjust angle accordingly
                    if (angle2 >= 180) {
                        angle2 -= 180; // Normalize angle to be within [0, 180)
                    }
                }

                if (max(length2, width2) < thresh) continue;

                double angleDiff = abs(angle1 - angle2);
                double yDiff = abs(center1.y - center2.y);

                bool isWidthValid = sim(width1, width2) > widthSimThresh;
                bool isLengthValid = sim(length1, length2) > lengthSimThresh;
                bool isYDiffValid = yDiff < yThresh * (length1 + length2) / 2.0;
                bool isAngleValid = (angleDiff < angleThresh || (angleDiff > 180 - angleThresh));

                int validConditions = 0;
                if (isWidthValid) validConditions++;
                if (isLengthValid) validConditions++;
                if (isYDiffValid) validConditions++;
                if (isAngleValid) validConditions++;

                Point centerMid((center1.x + center2.x) / 2, (center1.y + center2.y) / 2);

                if (validConditions == 4) {
                    centers.push_back(centerMid);
                    if (flag_write_color_video_) {
                        circle(frame, centerMid, 10, Scalar(255, 0, 255), -1);
                    }
                }

                if (!flag_write_color_video_ || !flag_debug_ || validConditions < 2) continue; // Skip if less than 3 conditions are satisfied
                line(frame, center1, center2, Scalar(255, 255, 255), 1);
                if (validConditions <= 3) {
                    if (validConditions == 3) {
                        // Highlight the center if 3 conditions are satisfied
                        circle(frame, centerMid, 7, Scalar(0, 255, 255), -1);
                    } else {
                        // If only 2 conditions are satisfied, use a smaller circle
                        circle(frame, centerMid, 5, Scalar(0, 0, 255), -1);
                    }
                    // Display which condition(s) failed
                    putText(frame, 
                            "Width: " + to_string(isWidthValid) + 
                            ", Length: " + to_string(isLengthValid) + 
                            ", YDiff: " + to_string(isYDiffValid) + 
                            ", Angle: " + to_string(isAngleValid), 
                            Point(centerMid.x + 10, centerMid.y - 10), 
                            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
                }
            }
        }

        return centers;
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
    return atan((right.y - left.y) / (right.x - left.x)) * 180.0 / CV_PI;
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CVNode>());
    rclcpp::shutdown();
    return 0;
}