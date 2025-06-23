#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
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




#define STATE_NUM 7
#define MEASURE_NUM 4

using namespace std; 
using namespace cv;


/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |               SortRect.hpp                  |
* |                                             |
* -----------------------------------------------
*/
struct SortRect {

    int id;
    float centerX;
    float centerY;
    float width;
    float height;
};




/*
* -----------------------------------------------
* |                                             |
* |                                             |
* |               KalmanFilter.cpp                  |
* |                                             |
* -----------------------------------------------
*/
// File: src/multi_object_tracker_node.cpp



// ──────────────── Kalman Filter Class ────────────────
class Kalman2D {
public:
    KalmanFilter kf;
    Kalman2D(geometry_msgs::msg::Point32 pt) {
        kf.init(4, 2, 0);
        kf.transitionMatrix = (Mat_<float>(4, 4) <<
            1, 0, 3, 0,
            0, 1, 0, 3,
            0, 0, 1, 0,
            0, 0, 0, 1);
        kf.measurementMatrix = Mat::eye(2, 4, CV_32F);
        setIdentity(kf.processNoiseCov, Scalar::all(1e-2) * 0.2);         // Q
        setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1) * 2);     // R
        setIdentity(kf.errorCovPost, Scalar::all(1) * 10000);               // P 

        kf.statePost.at<float>(0) = pt.x;
        kf.statePost.at<float>(1) = pt.y;
        kf.statePost.at<float>(2) = 0;
        kf.statePost.at<float>(3) = 0;
    }

    Point2f predict() {

        Mat prediction = kf.predict();
        return Point2f(prediction.at<float>(0), prediction.at<float>(1));
    }

    Point2f correct(const geometry_msgs::msg::Point32& pt) {
        Mat meas(2, 1, CV_32F);
        meas.at<float>(0) = pt.x;
        meas.at<float>(1) = pt.y;
        Mat estimate = kf.correct(meas);
        return Point2f(estimate.at<float>(0), estimate.at<float>(1));
    }
};

// ──────────────── Track Management ────────────────
struct Track {
    Kalman2D filter;
    int id;
    int unseen = 0;
    Track(geometry_msgs::msg::Point32 pt, int id_) : filter(pt), id(id_) {}
};

// ──────────────── Hungarian Algorithm ────────────────
class HungarianAlgorithm {
public:
    void Solve(const vector<vector<float>>& costMatrix, vector<int>& assignment) {

        
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
            if (bestJ != -1) {
                assignment[i] = bestJ;
                usedCols[bestJ] = true;
            }
        }
    }
};

class DetectionListener : public rclcpp::Node {
public:
    DetectionListener() : Node("detection_listener") {
        publisher_ = this->create_publisher<Detection2DArray>("predicted_points", 10); 
        // subscription_ = this->create_subscription<Detection2DArray>(
        //     "detections", 10, std::bind(&DetectionListener::callback, this, _1));

        detection_sub_.subscribe(this, "detections");
        image_sub_.subscribe(this, "/robot/rs2/color/image_raw/compressed");

        sync_ = std::make_shared<Sync>(SyncPolicy(10), detection_sub_, image_sub_);
        sync_->registerCallback(std::bind(&DetectionListener::callback, this, _1, _2));
    }

private:
    bool writer_initialized = false; 
    cv::VideoWriter writer_; 
    // rclcpp::Subscription<Detection2DArray>::SharedPtr subscription_;
    rclcpp::Publisher<Detection2DArray>::SharedPtr publisher_;

    std::vector<Track> tracks_;
    int next_id_;
    int frame_number = 0;


    geometry_msgs::msg::Point32 toPoint(const Point2f point) const {
        geometry_msgs::msg::Point32 pt;
        pt.x = point.x;
        pt.y = point.y;
        return pt;
    }

    Mat applyCanny(Mat& frame) {
        Mat edges;
        GaussianBlur(frame, frame, Size(5, 5), 1.5);
        // Canny with thresholds 100 and 200
        Canny(frame, edges, 100, 200);
        Mat edgesColor;
        cvtColor(edges, edgesColor, COLOR_GRAY2BGR);
        return edgesColor;
    }

    void callback(const Detection2DArray::ConstSharedPtr msg,
                  const CompressedImage::ConstSharedPtr image_msg) {
        frame_number++;



        vector<Point2f> detections; 


        
        for (const auto &detection : msg->detections) {
            float x = detection.bbox.center.position.x;
            float y = detection.bbox.center.position.y;
            // RCLCPP_INFO(this->get_logger(), "Frame Number: %d, Detection - x: %.2f, y: %.2f", frame_number, x, y);

            // geometry_msgs::msg::Pose pose; 

            Point2f point;
            
            point.x = x; 
            point.y = y; 


            //geometry_msgs::msg::PoseArray::SharedPtr
            
            detections.push_back(point); 

        }




        // const auto& detections = msg->poses;
        vector<Point2f> predictions;
        int i = 0;
        for (auto& t : tracks_) {
            if (!detections.empty()){
                Point2f val = t.filter.predict(); 
                predictions.push_back(val);

                RCLCPP_INFO(this->get_logger(), "Frame Number: %d, prediction - x: %.2f, y: %.2f || detection - x: %.2f, y: %.2f", frame_number, predictions.back().x, predictions.back().y, detections[i].x, detections[i].y);

                i++;
            }
        }

        vector<vector<float>> cost_matrix(tracks_.size(), vector<float>(detections.size(), 1e6));
        for (size_t i = 0; i < tracks_.size(); ++i) {
            for (size_t j = 0; j < detections.size(); ++j) {
                float dx = predictions[i].x - detections[j].x;
                float dy = predictions[i].y - detections[j].y;
                cost_matrix[i][j] = sqrt(dx * dx + dy * dy);
            }
        }


        vector<int> assignment;
        if (!tracks_.empty()) {
            HungarianAlgorithm().Solve(cost_matrix, assignment);
        }

        vector<bool> matched(detections.size(), false);
        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (assignment[i] != -1 && cost_matrix[i][assignment[i]] < 50.0f) {
                tracks_[i].filter.correct(toPoint(detections[assignment[i]]));
                tracks_[i].unseen = 0;
                matched[assignment[i]] = true;
            } else {
                tracks_[i].unseen++;
            }
        }

        if (tracks_.empty()) {
            for (const auto& det : detections) {
                tracks_.emplace_back(toPoint(det), next_id_++);
            }
            return;
        }

        tracks_.erase(remove_if(tracks_.begin(), tracks_.end(), [](const Track& t) {
            return t.unseen > 5;
        }), tracks_.end());

        Detection2DArray out;
        out.header = msg->header; 


        for (auto& t : tracks_) {
            Point2f pt = t.filter.predict();
            
            Detection2D detection;
            detection.bbox.center.position.x = pt.x;
            detection.bbox.center.position.y = pt.y;
            out.detections.push_back(detection); 

        }

       

        publisher_->publish(out);



        try {

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            Mat frame = cv_ptr->image;

            Mat edges = applyCanny(frame);
            
            if (!writer_initialized) {
                writer_.open("oneFile_predicted_video.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 30, frame.size(), true);
                if (!writer_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Could not open the output video for write");
                }
                writer_initialized = true;
            }
            

            float predictedX; 
            float predictedY;
            float detectedX;
            float detectedY;
            
            if (detections.size() >= 1)
            {
                predictedX = predictions[0].x;
                predictedY = predictions[0].y;

                detectedX = detections[0].x;
                detectedY = detections[0].y;

                 // draw two circles right here :) O O (like that <- :) )
                auto actualPoint = cv::Point();
                actualPoint.x = detectedX; 
                actualPoint.y = detectedY; 

                circle(frame, actualPoint, 10, Scalar(255, 0, 255), -1);

                auto predictedPoint = cv::Point();
                predictedPoint.x = predictedX; 
                predictedPoint.y = predictedY; 

                circle(frame, predictedPoint, 10, Scalar(0, 255, 0), -1);
                RCLCPP_INFO(this->get_logger(), "Framer Number: %d, Detection - x: %.2f, y: %.2f, Predicted x: %.2f, predicted y: %.2f", frame_number, detectedX, detectedY, predictedX, predictedY);
              
                

            }
            
            if (predictions.size() >= 1){
                predictedX = predictions[0].x;
                predictedY = predictions[0].y;

                auto predictedPoint = cv::Point();
                predictedPoint.x = predictedX; 
                predictedPoint.y = predictedY; 

                circle(frame, predictedPoint, 10, Scalar(0, 255, 0), -1);
                RCLCPP_INFO(this->get_logger(), "Framer Number: %d, Predicted x: %.2f, predicted y: %.2f", frame_number, predictedX, predictedY);
            } 


            writer_.write(frame);

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        


        

        //sort ros code here:
    }


    message_filters::Subscriber<Detection2DArray> detection_sub_;
    message_filters::Subscriber<CompressedImage> image_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Detection2DArray, CompressedImage>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;



};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionListener>());
    rclcpp::shutdown();
    return 0;
}