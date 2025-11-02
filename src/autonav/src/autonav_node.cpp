#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <opencv2/core.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <random>

// at what health should the robot retreat
#define HEALTH_THRESHOLD 67.0f 

class AutonavNode : public rclcpp::Node {
public:
    AutonavNode() : Node("AutonavNode") {
        /*Examples of the types of subscriptions needed for the autonavigation behaviors.*/
        localization_ = this->create_subscription<vision_msgs::msg::Detection3D>("localization_topic", 10, 
                        std::bind(&AutonavNode::localization_callback, this, std::placeholders::_1));

        odometry_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("odometry_topic", 10, 
                        std::bind(&AutonavNode::odometry_callback, this, std::placeholders::_1));

        robots_ = this->create_subscription<vision_msgs::msg::Detection3DArray>("robots_topic", 10,  //what????
                        std::bind(&AutonavNode::odometry_callback, this, std::placeholders::_1));

        health_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("health", 10, 
                        std::bind(&AutonavNode::health_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&AutonavNode::update_state_machine, this));

    }


    float get_latest_health() const {
        return latest_health;
    }

private:
    void localization_callback(const vision_msgs::msg::Detection3D::SharedPtr msg) {
        localization_pos.x = msg->bbox.center.position.x;
        localization_pos.y = msg->bbox.center.position.y;
        localization_pos.z = msg->bbox.center.position.z;
        RCLCPP_INFO(this->get_logger(), "Received: x = %d, y = %d, z = %d", localization_pos.x, localization_pos.y, localization_pos.z);
    }
 
    void odometry_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        odometry_pos = msg->data;
        RCLCPP_INFO(this->get_logger(), 
                "Received: x = %d, y = %d, z = %d, chassis_pitch = %d, chassis_yaw = %d, chassis_roll = %d, turret_pitch = %d, turret_yaw = %d", 
                odometry_pos[0], odometry_pos[1], odometry_pos[2], odometry_pos[3], odometry_pos[4], odometry_pos[5], odometry_pos[6], odometry_pos[7]);
    }

    void robots_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {        
        // converting detections msg data into points in the robots vector
        for (const auto &robot : msg->detections) {
            cv::Point3f point;
            point.x = robot.bbox.center.position.x;
            point.y = robot.bbox.center.position.y;
            point.z = robot.bbox.center.position.z;
            robots.push_back(point);
            RCLCPP_INFO(this->get_logger(), "Robot at: x = %d, y = %d, z = %d", point.x, point.y, point.z);
        }
    }

    void health_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        latest_health = msg->data[0];
        RCLCPP_INFO(this->get_logger(), "Recieved: health = %d", latest_health);
        if (latest_health != -1) {
            update_state_machine();
        }
    }

    double BOX_WIDTH = 10.0;
    double BOX_HEIGHT = 10.0;

    cv::Point2d cellCenter(int i, int j,
                        double px_min, double py_min,
                        double Hx, double Hy) const
    {
        double x = px_min + (static_cast<double>(i) + 0.5) * Hx;
        double y = py_min + (static_cast<double>(j) + 0.5) * Hy;
        return {x, y};
    }

    double distToNearestWall(const cv::Point2d &p) const
    {
        double dx = BOX_WIDTH / 2.0 - std::abs(p.x);
        double dy = BOX_HEIGHT / 2.0 - std::abs(p.y);
        return std::min(dx, dy);
    }


    void update_state_machine() {
        /*If statement to Pick Behavior*/


        if (latest_health == 0.0f){
            // retreat with out shooting 
        }

        else if (latest_health <= HEALTH_THRESHOLD ) {
            // retreat with shooting
            // CHANGE THE TARGET THE SPAWN POINT
            // MAYBE SHOOT IF POSSIBLE 
        }

        // patrol needs second condition checking that there are enemies on the point
        else if (latest_health > HEALTH_THRESHOLD && robots.size() > 0){
            // patrol 
        }
        
        // capturing the point needs second condition checking that there are no enemies on the point
        else if (latest_health > HEALTH_THRESHOLD && robots.size() == 0){
            // capture

            double boxWidth = BOX_WIDTH;
            double boxHeight = BOX_HEIGHT;
            double g = 10.0;
            double rSafe = 10.0;
            double minHoppingDistance = 5.0;
            double maxHoppingDistance = 50.0;
            double wallMargin = 5.0;
            double w_edge = 1.0;
            double w_center = 1.0;
            double w_safe = 1.0;

            cv::Point3f startingPosition = localization_pos;
            double px_min = -boxWidth / 2.0;
            double px_max = boxWidth / 2.0;

            double d_max = std::min(boxWidth / 2.0f, boxHeight / 2.0f);
            double R = std::max(boxWidth / 2.0f, boxHeight / 2.0f);

            const int G = static_cast<int>(g);

            double w_step = 1.0;
            double dangerThreshold = 0.7;
            int numCandidates = 5;
            double beta = 1.5;
            double eps = 1e-6;
            const double pi = std::acos(-1.0);

            std::vector<std::vector<double>> edge_cost(G, std::vector<double>(G, 0.0));
            std::vector<std::vector<double>> center_cost(G, std::vector<double>(G, 0.0));
            std::vector<std::vector<double>> robots_cost(G, std::vector<double>(G, 0.0));
            std::vector<std::vector<double>> final_cost(G, std::vector<double>(G, 0.0)); // tHIS IS THE THINGIE WE GOTTA PUBLISH

            double py_min = -boxHeight / 2.0;
            double Hx = boxWidth / static_cast<double>(G);
            double Hy = boxHeight / static_cast<double>(G);

            cv::Point2d reference(startingPosition.x, startingPosition.y);

            for (int i = 0; i < G; ++i) {
                for (int j = 0; j < G; ++j) {
                    cv::Point2d center = cellCenter(i, j, px_min, py_min, Hx, Hy);
                    double d = std::min(
                        boxWidth / 2.0 - std::abs(center.x),
                        boxHeight / 2.0 - std::abs(center.y));

                    edge_cost[i][j] = 1.0 - (d / d_max);
                    center_cost[i][j] = cv::norm(center - reference) / R;
                }
            }

            for (int i = 0; i < G; ++i) {
                for (int j = 0; j < G; ++j) {
                    robots_cost[i][j] = 0.0;
                }
            }

            for (const auto &robot : robots) {
                cv::Point2d robotPos(robot.x, robot.y);

                for (int i = 0; i < G; ++i) {
                    for (int j = 0; j < G; ++j) {
                        cv::Point2d center = cellCenter(i, j, px_min, py_min, Hx, Hy);
                        double dist = cv::norm(center - robotPos);
                        double c = rSafe / (dist + eps);

                        c = std::min(c, 1.0);
                        robots_cost[i][j] = std::max(robots_cost[i][j], c);
                    }
                }
            }

            cv::Point2d robotPose(startingPosition.x, startingPosition.y);
            std::vector<cv::Point2d> candidatePoints;
            candidatePoints.reserve(numCandidates);

            static std::mt19937 rng(std::random_device{}());
            std::uniform_real_distribution<double> stepDist(
                minHoppingDistance, maxHoppingDistance);
            std::uniform_real_distribution<double> angleDist(-pi, pi);

            int attempts = 0;
            int maxAttempts = numCandidates * 10;

            while (candidatePoints.size() < static_cast<size_t>(numCandidates) &&
                attempts < maxAttempts) {
                ++attempts;

                double r_step = stepDist(rng);
                double theta = angleDist(rng);

                cv::Point2d futureRobotPose = robotPose +
                    cv::Point2d(r_step * std::cos(theta), r_step * std::sin(theta));

                if (std::abs(futureRobotPose.x) > boxWidth / 2.0 ||
                    std::abs(futureRobotPose.y) > boxHeight / 2.0) {
                    continue;
                }

                if (distToNearestWall(futureRobotPose) < wallMargin) {
                    continue;
                }

                candidatePoints.push_back(futureRobotPose);
            }

            std::vector<double> scoresForCandidates;
            scoresForCandidates.reserve(candidatePoints.size());

            for (const auto &candidate : candidatePoints) {
                int ci = static_cast<int>((candidate.x - px_min) / Hx);
                int cj = static_cast<int>((candidate.y - py_min) / Hy);

                if (ci < 0 || ci >= G || cj < 0 || cj >= G) {
                    scoresForCandidates.push_back(1.0);
                    continue;
                }

                double step_cost = cv::norm(candidate - robotPose) / maxHoppingDistance;

                if (robots_cost[ci][cj] > dangerThreshold) {
                    final_cost[ci][cj] = 1.0;
                    scoresForCandidates.push_back(1.0);
                } else {
                    double combined = w_edge * edge_cost[ci][cj] +
                                    w_center * center_cost[ci][cj] +
                                    w_step * step_cost;

                    final_cost[ci][cj] = combined;
                    scoresForCandidates.push_back(combined);
                }
            }

            if (!scoresForCandidates.empty()) {
                std::vector<double> weights(scoresForCandidates.size());
                double maxScore = *std::max_element(
                    scoresForCandidates.begin(), scoresForCandidates.end());
                double sum = 0.0;

                for (size_t k = 0; k < scoresForCandidates.size(); ++k) {
                    weights[k] = std::exp(-beta * (scoresForCandidates[k] - maxScore));
                    sum += weights[k];
                }

                if (sum > 0.0) {
                    for (auto &w : weights) {
                        w /= sum;
                    }

                    std::discrete_distribution<size_t> chooser(
                        weights.begin(), weights.end());
                    size_t selectedIndex = chooser(rng);
                    cv::Point2d chosenPoint = candidatePoints[selectedIndex];
                    (void)chosenPoint;
                }
            }


        }

        /*Publish the goal posisiton, cost map, and anything else thats relevant to any other system here.*/
    }

    // subscriptions. Assume localization and odemetry are subscription datatypes are correct. 
    rclcpp::Subscription<vision_msgs::msg::Detection3D>::SharedPtr localization_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odometry_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr robots_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr health_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<cv::Point3f> robots; // a vector containing point3fs of detection data. each entry is a detection data
    std::vector<float> odometry_pos; // a vector of floats, containing position of the robot, x,y,z, and all turret/chassis data
    cv::Point3f localization_pos; // a 3d point of the position of the robot, gotten from the localization data
    float latest_health = -1.0f; // health of the robot

};



/*Put all behabiors in methods here! Call them in the if statement inside update_state_machine, which picks a behavior
  based on preconditions.*/




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonavNode>());
  rclcpp::shutdown();
  return 0;
}
