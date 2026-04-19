#include <cstdio>
#include <cmath>
#include <vector>
#include <set>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/parameter.hpp>

using std::placeholders::_1;
using namespace cv;
using namespace std;
using vision_msgs::msg::Detection3DArray;
using vision_msgs::msg::Detection3D;
using vision_msgs::msg::ObjectHypothesisWithPose;

enum Color { RED, BLUE };

// ──────────────────────────────────────────────────────────────────
// TUNABLE DETECTION PARAMETERS
// ──────────────────────────────────────────────────────────────────

static const Scalar RED_LOW1(0, 80, 90);
static const Scalar RED_HIGH1(8, 255, 255);
static const Scalar RED_LOW2(172, 80, 90);
static const Scalar RED_HIGH2(180, 255, 255);

static const Scalar BLUE_LOW(90, 90, 90);
static const Scalar BLUE_HIGH(125, 255, 255);

static const double MIN_BAR_AREA = 10.0;
static const double MAX_BAR_AREA = 80000.0;
static const double MIN_ASPECT_RATIO = 0.15;
static const double MAX_AREA_RATIO = 3.0;
static const double MAX_VERTICAL_GAP = 55.0;
static const double MIN_HORIZONTAL_SEP = 5.0;

// Distance estimation: distance = K / pixel_separation
static const double K_DISTANCE = 116.5;

// Robot physical dimensions (mm)
static const double ROBOT_SEP_SMALL_MM = 130.0;  // center-to-center, small config
static const double ROBOT_SEP_BIG_MM   = 226.0;  // center-to-center, big config
static const double FOCAL_LENGTH_PX    = 922.0;  // RealSense D435 color @ 1280x720

static const double K_SMALL = (ROBOT_SEP_SMALL_MM / 1000.0) * FOCAL_LENGTH_PX;  // ~119.9
static const double K_BIG   = (ROBOT_SEP_BIG_MM   / 1000.0) * FOCAL_LENGTH_PX;  // ~208.4

// Geometric constraint: pixel_sep / avg_bar_pixel_width must be below this.
// Real pairs are ~1.5 to 3.0; junk pairs (tiny blobs far apart) blow up to 100+.
static const double MAX_SEP_TO_SIZE_RATIO = 8.0;

// Multi-pair parameters
static const int MAX_PAIRS = 3;
static const double SAT_EXPONENT = 10.0;
static const double SCORE_THRESHOLD = 1e21;  // tune empirically, e.g. 1e22

// ──────────────────────────────────────────────────────────────────
// BAR CANDIDATE STRUCT
// ──────────────────────────────────────────────────────────────────

struct BarCandidate {
    int idx;  // unique index for greedy tracking
    int cx, cy, x, y, w, h;
    double area, sat, val, aspect;
    Color color;
};

// ──────────────────────────────────────────────────────────────────
// SCORED PAIR (for sorting + greedy consume)
// ──────────────────────────────────────────────────────────────────

struct ScoredPair {
    BarCandidate left;
    BarCandidate right;
    double score;
};

// ──────────────────────────────────────────────────────────────────
// DETECTION FUNCTIONS
// ──────────────────────────────────────────────────────────────────

static double estimate_distance(double pixel_separation) {
    if (pixel_separation <= 0.0) return -1.0;
    return K_DISTANCE / pixel_separation;
}

static vector<BarCandidate> detect_color_bars(const Mat& hsv) {
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);
    const Mat& s_channel = hsv_channels[1];
    const Mat& v_channel = hsv_channels[2];

    vector<BarCandidate> bars;
    int bar_idx = 0;

    // Detect red bars
    Mat mask_r1, mask_r2, mask_red;
    inRange(hsv, RED_LOW1, RED_HIGH1, mask_r1);
    inRange(hsv, RED_LOW2, RED_HIGH2, mask_r2);
    mask_red = mask_r1 | mask_r2;

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask_red, mask_red, MORPH_CLOSE, kernel, Point(-1, -1), 2);
    morphologyEx(mask_red, mask_red, MORPH_OPEN, kernel, Point(-1, -1), 1);

    vector<vector<Point>> contours_red;
    findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (auto& cnt : contours_red) {
        double area = contourArea(cnt);
        if (area < MIN_BAR_AREA || area > MAX_BAR_AREA) continue;

        Rect bbox = boundingRect(cnt);
        double aspect = (double)bbox.width / max(bbox.height, 1);
        if (aspect < MIN_ASPECT_RATIO) continue;

        Mat roi_mask = Mat::zeros(mask_red.size(), CV_8UC1);
        drawContours(roi_mask, vector<vector<Point>>{cnt}, -1, Scalar(255), FILLED);
        Scalar avg_s = mean(s_channel, roi_mask);
        Scalar avg_v = mean(v_channel, roi_mask);

        BarCandidate bar;
        bar.idx = bar_idx++;
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
        bar.color = RED;
        bars.push_back(bar);
    }

    // Detect blue bars
    Mat mask_blue;
    inRange(hsv, BLUE_LOW, BLUE_HIGH, mask_blue);
    morphologyEx(mask_blue, mask_blue, MORPH_CLOSE, kernel, Point(-1, -1), 2);
    morphologyEx(mask_blue, mask_blue, MORPH_OPEN, kernel, Point(-1, -1), 1);

    vector<vector<Point>> contours_blue;
    findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (auto& cnt : contours_blue) {
        double area = contourArea(cnt);
        if (area < MIN_BAR_AREA || area > MAX_BAR_AREA) continue;

        Rect bbox = boundingRect(cnt);
        double aspect = (double)bbox.width / max(bbox.height, 1);
        if (aspect < MIN_ASPECT_RATIO) continue;

        Mat roi_mask = Mat::zeros(mask_blue.size(), CV_8UC1);
        drawContours(roi_mask, vector<vector<Point>>{cnt}, -1, Scalar(255), FILLED);
        Scalar avg_s = mean(s_channel, roi_mask);
        Scalar avg_v = mean(v_channel, roi_mask);

        BarCandidate bar;
        bar.idx = bar_idx++;
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
        bar.color = BLUE;
        bars.push_back(bar);
    }

    return bars;
}

/**
 * Find up to MAX_PAIRS left-right pairs using greedy consume + threshold.
 *
 * 1. Score every valid (i,j) candidate pair
 * 2. Sort by score descending
 * 3. Greedily pick the top pair, mark both bars as consumed
 * 4. Pick next highest-scoring pair whose bars are both unconsumed
 * 5. Repeat until MAX_PAIRS selected or no valid pairs remain
 * 6. Reject any pair below SCORE_THRESHOLD
 */
static vector<ScoredPair> find_pairs(const vector<BarCandidate>& bars,
                                     double score_threshold = SCORE_THRESHOLD) {
    // Build and score all valid candidate pairs
    vector<ScoredPair> candidates;

    for (size_t i = 0; i < bars.size(); i++) {
        for (size_t j = i + 1; j < bars.size(); j++) {
            const auto& b1 = bars[i];
            const auto& b2 = bars[j];

            if (b1.color != b2.color) continue;

            double vgap = abs(b1.cy - b2.cy);
            double hsep = abs(b1.cx - b2.cx);

            if (vgap > MAX_VERTICAL_GAP || hsep < MIN_HORIZONTAL_SEP)
                continue;

            double area_ratio = max(b1.area, b2.area) / max(min(b1.area, b2.area), 1.0);
            if (area_ratio > MAX_AREA_RATIO)
                continue;

            // Geometric constraint: pixel_sep / avg_bar_width must be reasonable.
            // On a real robot this equals real_sep / real_bar_length (fixed constant).
            // Two tiny blobs far apart blow this ratio up and get rejected.
            double pixel_sep = sqrt(pow(b1.cx - b2.cx, 2) + pow(b1.cy - b2.cy, 2));
            double avg_bar_w = (b1.w + b2.w) / 2.0;
            if (avg_bar_w < 1.0) avg_bar_w = 1.0;
            double sep_size_ratio = pixel_sep / avg_bar_w;
            if (sep_size_ratio > MAX_SEP_TO_SIZE_RATIO)
                continue;

            double sat_avg = (b1.sat + b2.sat) / 2.0;
            double area_sim = 1.0 / (1.0 + area_ratio);
            double vert_score = 1.0 / (1.0 + vgap);
            double min_area = min(b1.area, b2.area);

            double score = pow(sat_avg, SAT_EXPONENT) * area_sim * vert_score * min_area;

            if (score < score_threshold)
                continue;

            ScoredPair sp;
            if (b1.cx < b2.cx) {
                sp.left = b1;
                sp.right = b2;
            } else {
                sp.left = b2;
                sp.right = b1;
            }
            sp.score = score;
            candidates.push_back(sp);
        }
    }

    // Sort by score descending
    sort(candidates.begin(), candidates.end(),
         [](const ScoredPair& a, const ScoredPair& b) {
             return a.score > b.score;
         });

    // Greedy consume
    set<int> used;
    vector<ScoredPair> selected;

    for (const auto& sp : candidates) {
        if ((int)selected.size() >= MAX_PAIRS)
            break;

        if (used.count(sp.left.idx) || used.count(sp.right.idx))
            continue;

        used.insert(sp.left.idx);
        used.insert(sp.right.idx);
        selected.push_back(sp);
    }

    return selected;
}

// ──────────────────────────────────────────────────────────────────
// PAIR COLORS (for video overlay, up to 3 pairs)
// ──────────────────────────────────────────────────────────────────

static const Scalar PAIR_COLORS[] = {
    Scalar(255, 0, 255),  // magenta
    Scalar(0, 255, 0),    // green
    Scalar(0, 165, 255),  // orange
};

static const Scalar BOX_COLORS[] = {
    Scalar(255, 255, 0),  // cyan
    Scalar(0, 255, 128),  // spring green
    Scalar(0, 128, 255),  // dark orange
};

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
        this->declare_parameter<double>("score_threshold", SCORE_THRESHOLD);
        this->declare_parameter<std::string>("team_color", "red");
        this->get_parameter("debug", flag_debug_);
        this->get_parameter("write_video", flag_write_video_);
        this->get_parameter("output_path", output_video_path_);
        this->get_parameter("score_threshold", score_threshold_);
        this->get_parameter("team_color", team_color_);
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/robot/rs2/color/image_raw/compressed", 10, std::bind(&CVNode::topic_callback, this, _1));

        detections_publisher_ = this->create_publisher<Detection3DArray>("detections", 10);
        detections_all_publisher_ = this->create_publisher<Detection3DArray>("detections_all", 10);
    }

private:
    rclcpp::Publisher<Detection3DArray>::SharedPtr detections_publisher_;
    rclcpp::Publisher<Detection3DArray>::SharedPtr detections_all_publisher_;
    bool writer_initialized;
    cv::VideoWriter writer_;
    rclcpp::Time last_frame_time;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool flag_debug_;
    bool flag_write_video_;
    std::string output_video_path_;
    double score_threshold_;
    std::string team_color_;

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

            // Find up to MAX_PAIRS pairs via greedy consume
            vector<ScoredPair> pairs = find_pairs(bars, score_threshold_);

            // Build detections messages
            Detection3DArray detections_all_msg;
            detections_all_msg.header = msg.header;
            Detection3DArray detections_msg;
            detections_msg.header = msg.header;
            Color opponent_color = (team_color_ == "red") ? BLUE : RED;

            for (int rank = 0; rank < (int)pairs.size(); rank++) {
                const auto& sp = pairs[rank];
                int mid_x = (sp.left.cx + sp.right.cx) / 2;
                int mid_y = (sp.left.cy + sp.right.cy) / 2;
                double pixel_sep = sqrt(pow(sp.right.cx - sp.left.cx, 2) +
                                        pow(sp.right.cy - sp.left.cy, 2));
                double dist_m = estimate_distance(pixel_sep);

                Detection3D detection;
                detection.bbox.center.position.x = static_cast<double>(mid_x);
                detection.bbox.center.position.y = static_cast<double>(mid_y);
                detection.bbox.center.position.z = dist_m;
                detection.bbox.size.x = pixel_sep;
                detections_all_msg.detections.push_back(detection);

                if (sp.left.color == opponent_color) {
                    detections_msg.detections.push_back(detection);
                }

                RCLCPP_INFO(this->get_logger(),
                    "Frame %d: pair #%d target=(%d,%d) sep=%.0fpx dist=%.2fm score=%.1e",
                    frame_number, rank + 1, mid_x, mid_y, pixel_sep, dist_m, sp.score);

                // Draw on frame for video output
                if (flag_write_video_) {
                    Scalar pair_color = PAIR_COLORS[rank % 3];
                    Scalar box_color = BOX_COLORS[rank % 3];

                    rectangle(frame, Point(sp.left.x, sp.left.y),
                              Point(sp.left.x + sp.left.w, sp.left.y + sp.left.h),
                              box_color, 2);
                    rectangle(frame, Point(sp.right.x, sp.right.y),
                              Point(sp.right.x + sp.right.w, sp.right.y + sp.right.h),
                              box_color, 2);
                    line(frame, Point(sp.left.cx, sp.left.cy),
                         Point(sp.right.cx, sp.right.cy), Scalar(0, 255, 255), 1);
                    circle(frame, Point(mid_x, mid_y), 5, pair_color, -1);
                    circle(frame, Point(mid_x, mid_y), 7, Scalar(255, 255, 255), 2);
                    int cs = 20;
                    line(frame, Point(mid_x - cs, mid_y), Point(mid_x + cs, mid_y),
                         pair_color, 1);
                    line(frame, Point(mid_x, mid_y - cs), Point(mid_x, mid_y + cs),
                         pair_color, 1);
                    putText(frame, format("#%d %.2fm", rank + 1, dist_m),
                            Point(mid_x + 15, mid_y - 15),
                            FONT_HERSHEY_SIMPLEX, 0.6, pair_color, 2);
                }
            }

            detections_all_publisher_->publish(detections_all_msg);
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