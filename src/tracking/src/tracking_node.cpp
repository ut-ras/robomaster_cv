#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/video/tracking.hpp"
#include <cfloat>
#include <set>
#include <vector>
#include <iostream>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/parameter.hpp>

using namespace std;
using namespace cv;


using std::placeholders::_1;
using std::placeholders::_2;

using vision_msgs::msg::Detection2DArray;
using sensor_msgs::msg::CompressedImage;
using vision_msgs::msg::Detection2D;
using vision_msgs::msg::ObjectHypothesisWithPose;

using vision_msgs::msg::Detection3DArray;
using vision_msgs::msg::Detection3D;

using namespace std; 
using namespace cv;

// ──────────────── Kalman Filter Class ────────────────
class Kalman3D {
public:
    KalmanFilter kf;
    Kalman3D(geometry_msgs::msg::Point32 pt) {
        kf.init(6, 3, 0);
        int dt = 1; 
        kf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
            1,0,0,dt,0,0,
            0,1,0,0,dt,0,
            0,0,1,0,0,dt,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1);
        kf.measurementMatrix = Mat::eye(3, 6, CV_32F);
        setIdentity(kf.processNoiseCov, Scalar::all(1e-2));         // Q
        setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));     // R
        setIdentity(kf.errorCovPost, Scalar::all(1));               // P 

        kf.statePost.at<float>(0) = pt.x;
        kf.statePost.at<float>(1) = pt.y;
        kf.statePost.at<float>(2) = pt.z;
        kf.statePost.at<float>(3) = 0;
        kf.statePost.at<float>(4) = 0;
        kf.statePost.at<float>(5) = 0;
    }

    Mat predict() {
        return kf.predict();
    }

    Mat correct(const geometry_msgs::msg::Point32& pt) {
        Mat meas(3, 1, CV_32F);
        meas.at<float>(0) = pt.x;
        meas.at<float>(1) = pt.y;
        meas.at<float>(2) = pt.z;
        Mat estimate = kf.correct(meas);
        return estimate;
    }

    Mat getState() const {
        return kf.statePost;
    }
};

// ──────────────── Track Management ────────────────
struct Track {
    Kalman3D filter;
    int id;
    int unseen;
    Track(geometry_msgs::msg::Point32 pt, int id_) : filter(pt), id(id_), unseen(0) {}
};

// ──────────────── Assignment Algorithm ────────────────
class AssignmentSolver {
public:
    void Solve(const vector<vector<float>>& costMatrix, vector<int>& assignment, float maxCost = FLT_MAX) {
        size_t n = costMatrix.size(); 
        size_t m = costMatrix[0].size();
        
        vector<bool> usedRows(n, false), usedCols(m, false);
        
        assignment.assign(n, -1);

        for (size_t i = 0; i < n; ++i) {
            float minCost = std::numeric_limits<float>::max();
            int bestJ = -1;
            for (size_t j = 0; j < m; ++j) {
                if (!usedCols[j] && costMatrix[i][j] < minCost) {
                    minCost = costMatrix[i][j];
                    bestJ = j;
                }
            }
            if (bestJ != -1 && minCost < maxCost) {
                assignment[i] = bestJ;
                usedCols[bestJ] = true;
            }
        }
    }
};

class DetectionListener : public rclcpp::Node {
public:
    DetectionListener() : Node("detection_listener"), tracks_(), next_id_(0) {
        publisher_ = this->create_publisher<Detection3DArray>("predicted_points", 10); 

        detection_sub_.subscribe(this, "detections");
        image_sub_.subscribe(this, "/robot/rs2/color/image_raw/compressed");

        sync_ = std::make_shared<Sync>(SyncPolicy(10), detection_sub_, image_sub_);
        sync_->registerCallback(std::bind(&DetectionListener::callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "DetectionListener node has been started.");
    }

private:
    bool writer_initialized = false; 
    cv::VideoWriter writer_; 
    rclcpp::Publisher<Detection3DArray>::SharedPtr publisher_;

    std::vector<Track> tracks_;
    int next_id_;

    geometry_msgs::msg::Point32 toPoint32(const Point3f point) const {
        geometry_msgs::msg::Point32 pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        return pt;
    }

    void callback(const Detection3DArray::ConstSharedPtr msg, const CompressedImage::ConstSharedPtr image_msg) {
        vector<Point3f> detections; 
        
        for (const auto &detection : msg->detections) {
            float x = detection.bbox.center.position.x;
            float y = detection.bbox.center.position.y;
            float z = detection.bbox.center.position.z;

            Point3f point;
            
            point.x = x; 
            point.y = y; 
            point.z = z;

            detections.push_back(point); 
        }

        vector<Point3f> predictions;
        for (auto& t : tracks_) {
            Mat val = t.filter.predict(); 
            predictions.push_back(Point3f(val.at<float>(0), val.at<float>(1), val.at<float>(2)));
        }

        vector<vector<float>> cost_matrix(tracks_.size(), vector<float>(detections.size(), FLT_MAX));
        for (size_t i = 0; i < predictions.size(); ++i) {
            for (size_t j = 0; j < detections.size(); ++j) {
                float dx = predictions[i].x - detections[j].x;
                float dy = predictions[i].y - detections[j].y;
                float dz = predictions[i].z - detections[j].z;
                cost_matrix[i][j] = dx * dx + dy * dy + dz * dz;
            }
        }

        vector<int> assignment;
        AssignmentSolver().Solve(cost_matrix, assignment, 2500.0f);

        RCLCPP_INFO(this->get_logger(), "Tracks: %zu, Detections: %zu", tracks_.size(), detections.size());

        vector<bool> matched(detections.size(), false);
        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (assignment[i] != -1) {
                tracks_[i].filter.correct(toPoint32(detections[assignment[i]]));
                tracks_[i].unseen = 0;
                matched[assignment[i]] = true;
            } else {
                tracks_[i].unseen++;
            }
        }

        for (size_t j = 0; j < detections.size(); ++j) {
            if (!matched[j]) {
                tracks_.emplace_back(toPoint32(detections[j]), next_id_++);
            }
        }

        tracks_.erase(remove_if(tracks_.begin(), tracks_.end(), [](const Track& t) {
            return t.unseen > 2;
        }), tracks_.end());

        Detection3DArray out;
        out.header = msg->header; 

        for (auto& t : tracks_) {
            Mat m = t.filter.getState();
            Point3f pt(m.at<float>(0), m.at<float>(1), m.at<float>(2));
            
            Detection3D detection;
            detection.bbox.center.position.x = pt.x;
            detection.bbox.center.position.y = pt.y;
            detection.bbox.center.position.z = pt.z;
            out.detections.push_back(detection); 
        }

        publisher_->publish(out);

        // try {
        //     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        //     Mat raw_frame = cv_ptr->image;
        //     // Downsample frame for efficiency
        //     Mat frame;
        //     cv::resize(raw_frame, frame, Size(), 0.5, 0.5, INTER_LINEAR);
            
        //     if (!writer_initialized) {
        //         writer_.open("oneFile_predicted_video.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 30, frame.size(), true);
        //         if (!writer_.isOpened()) {
        //             RCLCPP_ERROR(this->get_logger(), "Could not open the output video for write");
        //         }
        //         writer_initialized = true;
        //         RCLCPP_INFO(this->get_logger(), "Video writer initialized.");
        //     }
            
        //     for (auto& det : detections) {
        //         auto detectedPoint = cv::Point();
        //         detectedPoint.x = det.x; 
        //         detectedPoint.y = det.y; 
        //         circle(frame, detectedPoint, 10, Scalar(255, 0, 255), -1);
        //         RCLCPP_INFO(this->get_logger(), "\tDetection - x: %.2f, y: %.2f", det.x, det.y);
        //     }

        //     for (auto& pred : out.detections) {
        //         auto predictedPoint = cv::Point();
        //         predictedPoint.x = (int) pred.bbox.center.position.x;
        //         predictedPoint.y = (int) pred.bbox.center.position.y;
        //         circle(frame, predictedPoint, 10, Scalar(0, 255, 0), -1);
        //         RCLCPP_INFO(this->get_logger(), "\tPrediction - x: %.2f, y: %.2f", pred.bbox.center.position.x, pred.bbox.center.position.y);
        //     }

        //     writer_.write(frame);
        // } catch (cv_bridge::Exception &e) {
        //     RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        // }
    }

    message_filters::Subscriber<Detection3DArray> detection_sub_;
    message_filters::Subscriber<CompressedImage> image_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Detection3DArray, CompressedImage>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionListener>());
    rclcpp::shutdown();
    return 0;
}