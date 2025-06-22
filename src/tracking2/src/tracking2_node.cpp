#include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world tracking2 package\n");
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

using std::placeholders::_1;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::Detection2D;




#include "cv_bridge/cv_bridge.h"
#include "opencv2/video/tracking.hpp"
#include <cfloat>
#include <set>
#include <vector>
#include <iostream>


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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <unordered_map>

using std::placeholders::_1;
using namespace std;
using namespace cv;

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
        subscription_ = this->create_subscription<Detection2DArray>(
            "detections", 10, std::bind(&DetectionListener::callback, this, _1));
    }

private:
    rclcpp::Subscription<Detection2DArray>::SharedPtr subscription_;
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

    void callback(const Detection2DArray::SharedPtr msg) {
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

        // add acutal points here
        Detection2D actualPoint; 
        if (detections.size() >= 1){
            actualPoint.bbox.center.position.x = detections[0].x;
            actualPoint.bbox.center.position.y = detections[0].y;
            
            out.detections.push_back(actualPoint);
        }
       

        publisher_->publish(out);


        

        //sort ros code here:
    }



};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionListener>());
    rclcpp::shutdown();
    return 0;
}