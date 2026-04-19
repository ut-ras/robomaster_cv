#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/parameter.hpp>

using std::placeholders::_1;
using namespace cv;
using namespace std;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::Detection2D;
using vision_msgs::msg::ObjectHypothesisWithPose;

// ──────────────────────────────────────────────────────────────────
// TUNABLE DETECTION PARAMETERS
// ──────────────────────────────────────────────────────────────────

static const Scalar RED_LOW1(0, 80, 90);
static const Scalar RED_HIGH1(8, 255, 255);
static const Scalar RED_LOW2(172, 80, 90);
static const Scalar RED_HIGH2(180, 255, 255);

static const double MIN_BAR_AREA = 10.0;
static const double MAX_BAR_AREA = 80000.0;
static const double MIN_ASPECT_RATIO = 0.15;
static const double MAX_AREA_RATIO = 3.0;
static const double MAX_VERTICAL_GAP = 55.0;
static const double MIN_HORIZONTAL_SEP = 5.0;

// Distance estimation: distance = K / pixel_separation
static const double K_DISTANCE = 116.5;

// ──────────────────────────────────────────────────────────────────
// BAR CANDIDATE STRUCT
// ──────────────────────────────────────────────────────────────────

struct BarCandidate {
    int cx, cy, x, y, w, h;
    double area, sat, val, aspect;
};

// ──────────────────────────────────────────────────────────────────
// DETECTION FUNCTIONS
// ──────────────────────────────────────────────────────────────────

static double estimate_distance(double pixel_separation) {
    if (pixel_separation <= 0.0) return -1.0;
    return K_DISTANCE / pixel_separation;
}

static vector<BarCandidate> detect_color_bars(const Mat& hsv) {
    Mat mask_r1, mask_r2, mask;

    inRange(hsv, RED_LOW1, RED_HIGH1, mask_r1);
    inRange(hsv, RED_LOW2, RED_HIGH2, mask_r2);
    mask = mask_r1 | mask_r2;

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel, Point(-1, -1), 2);
    morphologyEx(mask, mask, MORPH_OPEN, kernel, Point(-1, -1), 1);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Split HSV channels once for mean calculations
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);
    const Mat& s_channel = hsv_channels[1];
    const Mat& v_channel = hsv_channels[2];

    vector<BarCandidate> bars;
    for (auto& cnt : contours) {
        double area = contourArea(cnt);
        if (area < MIN_BAR_AREA || area > MAX_BAR_AREA)
            continue;

        Rect bbox = boundingRect(cnt);
        double aspect = (double)bbox.width / max(bbox.height, 1);

        if (aspect < MIN_ASPECT_RATIO)
            continue;

        // Compute average saturation and value within contour
        Mat roi_mask = Mat::zeros(mask.size(), CV_8UC1);
        drawContours(roi_mask, vector<vector<Point>>{cnt}, -1, Scalar(255), FILLED);
        Scalar avg_s = mean(s_channel, roi_mask);
        Scalar avg_v = mean(v_channel, roi_mask);

        BarCandidate bar;
        bar.cx = bbox.x + bbox.width / 2;
        bar.cy = bbox.y + bbox.height / 2;
        bar.x = bbox.x;
        bar.y = bbox.y;
        bar.w = bbox.width;
        bar.h = bbox.height;
        bar.area = area;
        bar.sat = avg_s[0];
        bar.val = avg_v[0];
        bar.aspect = aspect;
        bars.push_back(bar);
    }

    return bars;
}

/**
 * Find the best left-right pair of LED bars.
 * Returns true if a valid pair was found; populates left/right and midpoint.
 */
static bool find_best_pair(const vector<BarCandidate>& bars,
                           BarCandidate& out_left,
                           BarCandidate& out_right) {
    if (bars.size() < 2) return false;

    double best_score = -1.0;
    bool found = false;

    for (size_t i = 0; i < bars.size(); i++) {
        for (size_t j = i + 1; j < bars.size(); j++) {
            const auto& b1 = bars[i];
            const auto& b2 = bars[j];

            double vgap = abs(b1.cy - b2.cy);
            double hsep = abs(b1.cx - b2.cx);

            if (vgap > MAX_VERTICAL_GAP || hsep < MIN_HORIZONTAL_SEP)
                continue;

            double area_ratio = max(b1.area, b2.area) / max(min(b1.area, b2.area), 1.0);
            if (area_ratio > MAX_AREA_RATIO)
                continue;

            double sat_avg = (b1.sat + b2.sat) / 2.0;
            double area_sim = 1.0 / (1.0 + area_ratio);
            double vert_score = 1.0 / (1.0 + vgap);
            double min_area = min(b1.area, b2.area);

            double score = pow(sat_avg, 10.0) * area_sim * vert_score * min_area;

            if (score > best_score) {
                best_score = score;
                if (b1.cx < b2.cx) {
                    out_left = b1;
                    out_right = b2;
                } else {
                    out_left = b2;
                    out_right = b1;
                }
                found = true;
            }
        }
    }

    return found;
}

// ──────────────────────────────────────────────────────────────────
// ROS2 NODE
// ──────────────────────────────────────────────────────────────────

class CVNode : public rclcpp::Node {
public:
    CVNode() : Node("CVNode"), writer_initialized(false), last_frame_time(this->now()) {
        printf("Initializing subscriber...\n");
        this->declare_parameter<bool>("debug", false);
        this->declare_parameter<bool>("write_video", false);
        this->declare_parameter<string>("output_path", "output.mp4");
        this->get_parameter("debug", flag_debug_);
        this->get_parameter("write_video", flag_write_video_);
        this->get_parameter("output_path", output_video_path_);
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/robot/rs2/color/image_raw/compressed", 10, std::bind(&CVNode::topic_callback, this, _1));

        detections_publisher_ = this->create_publisher<Detection2DArray>("detections", 10);
    }

private:
    rclcpp::Publisher<Detection2DArray>::SharedPtr detections_publisher_;
    bool writer_initialized;
    cv::VideoWriter writer_;
    rclcpp::Time last_frame_time;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool flag_debug_;
    bool flag_write_video_;
    std::string output_video_path_;

    int frame_number = 0;

    void topic_callback(const sensor_msgs::msg::CompressedImage &msg) {
        try {
            frame_number++;

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            Mat frame = cv_ptr->image;

            // Convert to HSV once
            Mat hsv;
            cvtColor(frame, hsv, COLOR_BGR2HSV);

            // Detect LED bar candidates
            vector<BarCandidate> bars = detect_color_bars(hsv);

            // Find the best left-right pair
            BarCandidate left, right;
            bool pair_found = find_best_pair(bars, left, right);

            // Build detections message
            Detection2DArray detections_msg;
            detections_msg.header = msg.header;

            if (pair_found) {
                int mid_x = (left.cx + right.cx) / 2;
                int mid_y = (left.cy + right.cy) / 2;
                double pixel_sep = sqrt(pow(right.cx - left.cx, 2) + pow(right.cy - left.cy, 2));
                double dist_m = estimate_distance(pixel_sep);

                Detection2D detection;
                detection.bbox.center.position.x = static_cast<double>(mid_x);
                detection.bbox.center.position.y = static_cast<double>(mid_y);
                // Store pixel separation in bbox size_x and distance in size_y
                detection.bbox.size_x = pixel_sep;
                detection.bbox.size_y = dist_m;
                detections_msg.detections.push_back(detection);

                RCLCPP_INFO(this->get_logger(),
                    "Frame %d: target=(%d,%d) sep=%.0fpx dist=%.2fm",
                    frame_number, mid_x, mid_y, pixel_sep, dist_m);

                // Draw on frame for video output
                if (flag_write_video_) {
                    rectangle(frame, Point(left.x, left.y),
                              Point(left.x + left.w, left.y + left.h),
                              Scalar(255, 255, 0), 2);
                    rectangle(frame, Point(right.x, right.y),
                              Point(right.x + right.w, right.y + right.h),
                              Scalar(255, 255, 0), 2);
                    line(frame, Point(left.cx, left.cy),
                         Point(right.cx, right.cy), Scalar(0, 255, 255), 1);
                    circle(frame, Point(mid_x, mid_y), 5, Scalar(255, 0, 255), -1);
                    circle(frame, Point(mid_x, mid_y), 7, Scalar(255, 255, 255), 2);
                    int cs = 20;
                    line(frame, Point(mid_x - cs, mid_y), Point(mid_x + cs, mid_y),
                         Scalar(255, 0, 255), 1);
                    line(frame, Point(mid_x, mid_y - cs), Point(mid_x, mid_y + cs),
                         Scalar(255, 0, 255), 1);
                    putText(frame, format("%.2fm", dist_m),
                            Point(mid_x + 15, mid_y - 15),
                            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 255), 2);
                }
            }

            detections_publisher_->publish(detections_msg);

            last_frame_time = this->now();

            // Write video only if flag is true
            if (flag_write_video_) {
                if (!writer_initialized) {
                    writer_.open(output_video_path_,
                                 cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                 30, frame.size(), true);
                    if (!writer_.isOpened()) {
                        RCLCPP_ERROR(this->get_logger(), "Could not open the output video for write");
                    }
                    writer_initialized = true;
                }
                writer_.write(frame);
            }

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CVNode>());
    rclcpp::shutdown();
    return 0;
}