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
#include <numbers>

// at what health should the robot retreat
#define HEALTH_THRESHOLD 67.0f 
#define PI 3.14159265358979323846

enum PointState {EMPTY = 0, ENEMY = 1, FRIENDLY = 2, CONTESTED = 3};

//change with actual positions
std::vector<float> retreat_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> camp_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


class AutonavNode : public rclcpp::Node {
public:
    AutonavNode() : Node("AutonavNode") {
        /*Examples of the types of subscriptions needed for the autonavigation behaviors.*/
        localization_ = this->create_subscription<vision_msgs::msg::Detection3D>("localization_topic", 10, 
                        std::bind(&AutonavNode::localization_callback, this, std::placeholders::_1));

        odometry_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("odometry_topic", 10, 
                        std::bind(&AutonavNode::odometry_callback, this, std::placeholders::_1));

        enemies_ = this->create_subscription<vision_msgs::msg::Detection3DArray>("robots_topic", 10,
                        std::bind(&AutonavNode::enemies_callback, this, std::placeholders::_1));

        //Note change this with the actual friendlies node once created
        friendlies_ = this->create_subscription<vision_msgs::msg::Detection3DArray>("robots_topic", 10,
                        std::bind(&AutonavNode::friendlies_callback, this, std::placeholders::_1));

        health_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("health", 10, 
                        std::bind(&AutonavNode::health_callback, this, std::placeholders::_1));

        //Someone please check if this is right
        target_pos_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_pos", 10);

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
        RCLCPP_INFO(this->get_logger(), "Received: x = %.2f, y = %.2f, z = %.2f", localization_pos.x, localization_pos.y, localization_pos.z);
    }
 
    void odometry_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        odometry_pos = msg->data;
        RCLCPP_INFO(this->get_logger(), 
                "Received: x = %.2f, y = %.2f, z = %.2f, chassis_pitch = %.2f, chassis_yaw = %.2f, chassis_roll = %.2f, turret_pitch = %.2f, turret_yaw = %.2f", 
                odometry_pos[0], odometry_pos[1], odometry_pos[2], odometry_pos[3], odometry_pos[4], odometry_pos[5], odometry_pos[6], odometry_pos[7]);
    }

    void friendlies_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {        
        // Clear previous detections to get fresh data each time
        friendlies.clear();
        // converting detections msg data into points in the robots vector
        for (const auto &robot : msg->detections) {
            cv::Point3f point;
            point.x = robot.bbox.center.position.x;
            point.y = robot.bbox.center.position.y;
            point.z = robot.bbox.center.position.z;
            friendlies.push_back(point);
            RCLCPP_INFO(this->get_logger(), "Friendly detected at: x = %.2f, y = %.2f, z = %.2f", point.x, point.y, point.z);
        }
        RCLCPP_INFO(this->get_logger(), "Total friendlies detected: %zu", friendlies.size());
    }

    void enemies_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {        
        // Clear previous detections to get fresh data each time
        enemies.clear();
        // converting detections msg data into points in the robots vector
        for (const auto &robot : msg->detections) {
            cv::Point3f point;
            point.x = robot.bbox.center.position.x;
            point.y = robot.bbox.center.position.y;
            point.z = robot.bbox.center.position.z;
            enemies.push_back(point);
            RCLCPP_INFO(this->get_logger(), "Enemy detected at: x = %.2f, y = %.2f, z = %.2f", point.x, point.y, point.z);
        }
        RCLCPP_INFO(this->get_logger(), "Total enemies detected: %zu", enemies.size());
    }

    void health_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        latest_health = msg->data[0];
        RCLCPP_INFO(this->get_logger(), "Recieved: health = %.2f", latest_health);
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

    PointState point_taken(){
        //Assuming absolute turret rotation (i.e. in relation to field)
        //Assuming rotation = 0 at x axis
        //Assuming rotation dir matches atan2 dir
        //Might want to swap out odom_pos[1] for [2] (y, for z)
        //Might need offset from center of robot to came pos but might be neglibable

        //x, y, angle
        float pos_info[] = {odometry_pos[0], odometry_pos[1], odometry_pos[6]};

        bool friendly_found = false;
        bool enemy_found = false;

        for(cv::Point3f pos : enemies){

            /* calculating the detected robots position*/
            float theta_cam = std::atan2(pos.x, pos.z); 
            float theta_pos = pos_info[2] * PI / 180;   //degree -> radians
            float theta_det = pos_info[2] - theta_cam; //affirm both angles in same unit
            float x_det = pos_info[0] + pos.z * cos(theta_det);
            float y_det = pos_info[1] + pos.z * sin(theta_det);

            /*assuming pos in mm*/
            float capture_center_x = 6000;
            float capture_center_y = 6000;
            
            /* within 1/2 meter of capture zone*/
            if(std::fabs(x_det - capture_center_x) < 1000 && 
                std::fabs(y_det - capture_center_y) < 1000){
                enemy_found = true;
                break;
            }
        }
        for(cv::Point3f pos : friendlies){

            /* calculating the detected robots position*/
            float theta_cam = std::atan2(pos.x, pos.z); 
            float theta_pos = pos_info[2] * PI / 180;   //degree -> radians
            float theta_det = pos_info[2] - theta_cam; //affirm both angles in same unit
            float x_det = pos_info[0] + pos.z * cos(theta_det);
            float y_det = pos_info[1] + pos.z * sin(theta_det);

            /*assuming pos in mm*/
            float capture_center_x = 6000;
            float capture_center_y = 6000;
            
            /* within 1/2 meter of capture zone*/
            if(std::fabs(x_det - capture_center_x) < 1000 && 
                std::fabs(y_det - capture_center_y) < 1000){
                friendly_found = true;
                break;
            }
        }
        if(friendly_found && enemy_found){
            return CONTESTED;
        }else if(friendly_found){
            return FRIENDLY;
        }else if(enemy_found){
            return ENEMY;
        }else{
            return EMPTY;
        }
    }


    void update_state_machine() {
        /*If statement to Pick Behavior*/


        if (latest_health == 0.0f){
            // retreat with out shooting 
            // Can autoaim handle not shooting cmd?
        }

        else if (latest_health <= HEALTH_THRESHOLD ) {
            // retreat with shooting
            // CHANGE THE TARGET THE SPAWN POINT
            // MAYBE SHOOT IF POSSIBLE 
        }
        
        // capturing the point needs second condition checking that there are no enemies on the point
        else if (latest_health){
            RCLCPP_INFO(this->get_logger(), "=== CAPTURE STATE ACTIVATED ===");
            RCLCPP_INFO(this->get_logger(), "Current health: %.2f", latest_health);
            RCLCPP_INFO(this->get_logger(), "Current position (localization): x=%.2f, y=%.2f, z=%.2f", 
                       localization_pos.x, localization_pos.y, localization_pos.z);
            
            bool point_taken = false;
            enum PointState state = EMPTY;

            if(enemies.size() + friendlies.size() > 0){
                RCLCPP_INFO(this->get_logger(), "Checking %zu detected robots for capture point occupancy", enemies.size() + friendlies.size());
                //Assuming absolute turret rotation (i.e. in relation to field)
                //Assuming rotation = 0 at x axis

                //x, y, angle
                if(odometry_pos.size() < 7) {
                    RCLCPP_WARN(this->get_logger(), "Odometry data incomplete, skipping robot position check");
                } else {
                    state = point_taken();
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "No robots detected, capture point is clear");
            }
            
            if(point_taken == CONTESTED) {
                RCLCPP_INFO(this->get_logger(), "Capture point is contested, helping friendly");
            }else if(point_taken == ENEMY){
                RCLCPP_INFO(this->get_logger(), "Capture point is taken by enemy, spin attack!");
            }else if(point_taken == FRIENDLY){
                RCLCPP_INFO(this->get_logger(), "Capture Point is taken by friendly, camp and protect");
            }else{
                RCLCPP_INFO(this->get_logger(), "Capture Point Empty, Starting capture logic...");
            }
            
            // capture

            // Constants adjusted for testing (field is typically 12m x 12m)
            double boxWidth = BOX_WIDTH;
            double boxHeight = BOX_HEIGHT;
            double g = 20.0;  // Increased grid resolution for better visualization
            double rSafe = 1.0;  // Safe radius around robots (meters) - reduced for better testing
            double minHoppingDistance = 0.5;  // Minimum movement distance (meters)
            double maxHoppingDistance = 3.0;  // Maximum movement distance (meters)
            double wallMargin = 0.5;  // Margin from walls (meters)
            double w_edge = 1.0;
            double w_center = 1.0;

            RCLCPP_INFO(this->get_logger(), "Capture parameters: box=%.1fx%.1f, grid=%d, safe_radius=%.1f, hop_range=[%.1f,%.1f]", 
                       boxWidth, boxHeight, static_cast<int>(g), rSafe, minHoppingDistance, maxHoppingDistance);

            cv::Point3f startingPosition = localization_pos;
            double px_min = -boxWidth / 2.0;

            double d_max = std::min(boxWidth / 2.0f, boxHeight / 2.0f);
            double R = std::max(boxWidth / 2.0f, boxHeight / 2.0f);

            const int G = static_cast<int>(g);

            double w_step = 1.0;
            double dangerThreshold = 0.8;  // Increased threshold to reduce false positives
            int numCandidates = 10;  // Increased for better selection
            double beta = 1.5;
            double eps = 1e-6;
            const double pi = std::acos(-1.0);
            
            RCLCPP_INFO(this->get_logger(), "Starting position: x=%.2f, y=%.2f", startingPosition.x, startingPosition.y);

            std::vector<std::vector<double>> edge_cost(G, std::vector<double>(G, 0.0));
            std::vector<std::vector<double>> center_cost(G, std::vector<double>(G, 0.0));
            std::vector<std::vector<double>> robots_cost(G, std::vector<double>(G, 0.0));
            std::vector<std::vector<double>> final_cost(G, std::vector<double>(G, 0.0)); // tHIS IS THE THINGIE WE GOTTA PUBLISH

            double py_min = -boxHeight / 2.0;
            double Hx = boxWidth / static_cast<double>(G);
            double Hy = boxHeight / static_cast<double>(G);
            
            RCLCPP_INFO(this->get_logger(), "Grid cell size: Hx=%.3f, Hy=%.3f", Hx, Hy);

            cv::Point2d reference(startingPosition.x, startingPosition.y);
            RCLCPP_INFO(this->get_logger(), "Reference point (starting position): x=%.2f, y=%.2f", reference.x, reference.y);

            RCLCPP_INFO(this->get_logger(), "Computing edge and center costs...");
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
            RCLCPP_INFO(this->get_logger(), "Edge and center costs computed");

            for (int i = 0; i < G; ++i) {
                for (int j = 0; j < G; ++j) {
                    robots_cost[i][j] = 0.0;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Computing robot avoidance costs for %zu robots...", robots.size());
            for (const auto &robot : robots) {
                cv::Point2d robotPos(robot.x, robot.y);
                RCLCPP_INFO(this->get_logger(), "  Processing robot at: x=%.2f, y=%.2f", robotPos.x, robotPos.y);

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
            RCLCPP_INFO(this->get_logger(), "Robot avoidance costs computed");

            cv::Point2d robotPose(startingPosition.x, startingPosition.y);
            RCLCPP_INFO(this->get_logger(), "Current robot pose: x=%.2f, y=%.2f", robotPose.x, robotPose.y);
            
            std::vector<cv::Point2d> candidatePoints;
            candidatePoints.reserve(numCandidates);

            static std::mt19937 rng(std::random_device{}());
            std::uniform_real_distribution<double> stepDist(
                minHoppingDistance, maxHoppingDistance);
            std::uniform_real_distribution<double> angleDist(-pi, pi);

            int attempts = 0;
            int maxAttempts = numCandidates * 20;  // Increased attempts for better coverage

            RCLCPP_INFO(this->get_logger(), "Generating %d candidate points...", numCandidates);
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
            RCLCPP_INFO(this->get_logger(), "Generated %zu candidate points after %d attempts", candidatePoints.size(), attempts);

            std::vector<double> scoresForCandidates;
            scoresForCandidates.reserve(candidatePoints.size());

            RCLCPP_INFO(this->get_logger(), "Evaluating candidate points...");
            int candidate_idx = 0;
            for (const auto &candidate : candidatePoints) {
                int ci = static_cast<int>((candidate.x - px_min) / Hx);
                int cj = static_cast<int>((candidate.y - py_min) / Hy);

                if (ci < 0 || ci >= G || cj < 0 || cj >= G) {
                    RCLCPP_WARN(this->get_logger(), "  Candidate %d at (%.2f, %.2f) is out of bounds", 
                               candidate_idx, candidate.x, candidate.y);
                    scoresForCandidates.push_back(1.0);
                    candidate_idx++;
                    continue;
                }

                double step_cost = cv::norm(candidate - robotPose) / maxHoppingDistance;

                if (robots_cost[ci][cj] > dangerThreshold) {
                    // Dangerous candidates get a high penalty score (lower is better, but we want to avoid these)
                    final_cost[ci][cj] = 1.0;
                    scoresForCandidates.push_back(10.0);  // High penalty for dangerous candidates
                    RCLCPP_WARN(this->get_logger(), "  Candidate %d at (%.2f, %.2f) is DANGEROUS (robot_cost=%.3f)", 
                               candidate_idx, candidate.x, candidate.y, robots_cost[ci][cj]);
                } else {
                    double combined = w_edge * edge_cost[ci][cj] +
                                    w_center * center_cost[ci][cj] +
                                    w_step * step_cost;

                    final_cost[ci][cj] = combined;
                    scoresForCandidates.push_back(combined);
                    RCLCPP_INFO(this->get_logger(), "  Candidate %d at (%.2f, %.2f): edge=%.3f, center=%.3f, step=%.3f, combined=%.3f", 
                               candidate_idx, candidate.x, candidate.y, 
                               edge_cost[ci][cj], center_cost[ci][cj], step_cost, combined);
                }
                candidate_idx++;
            }

            if (!scoresForCandidates.empty()) {
                RCLCPP_INFO(this->get_logger(), "Selecting best candidate from %zu options...", scoresForCandidates.size());
                std::vector<double> weights(scoresForCandidates.size());
                double maxScore = *std::max_element(
                    scoresForCandidates.begin(), scoresForCandidates.end());
                double minScore = *std::min_element(
                    scoresForCandidates.begin(), scoresForCandidates.end());
                RCLCPP_INFO(this->get_logger(), "Score range: min=%.3f, max=%.3f (lower is better)", minScore, maxScore);
                
                // Count safe vs dangerous candidates
                int safe_count = 0, dangerous_count = 0;
                for (double score : scoresForCandidates) {
                    if (score >= 10.0) {
                        dangerous_count++;
                    } else {
                        safe_count++;
                    }
                }
                RCLCPP_INFO(this->get_logger(), "Safe candidates: %d, Dangerous candidates: %d", safe_count, dangerous_count);
                
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
                    
                    RCLCPP_INFO(this->get_logger(), "=== SELECTED GOAL POINT ===");
                    RCLCPP_INFO(this->get_logger(), "Chosen point: x=%.2f, y=%.2f", chosenPoint.x, chosenPoint.y);
                    RCLCPP_INFO(this->get_logger(), "Distance from current position: %.2f meters", 
                               cv::norm(chosenPoint - robotPose));
                    RCLCPP_INFO(this->get_logger(), "Selected candidate index: %zu, score: %.3f", 
                               selectedIndex, scoresForCandidates[selectedIndex]);
                    
                    // Log cost map summary
                    double min_cost = 1.0, max_cost = 0.0, avg_cost = 0.0;
                    double min_safe_cost = 1.0, max_safe_cost = 0.0, avg_safe_cost = 0.0;
                    int valid_cells = 0;
                    int safe_cells = 0;
                    int dangerous_cells = 0;
                    for (int i = 0; i < G; ++i) {
                        for (int j = 0; j < G; ++j) {
                            min_cost = std::min(min_cost, final_cost[i][j]);
                            max_cost = std::max(max_cost, final_cost[i][j]);
                            avg_cost += final_cost[i][j];
                            valid_cells++;
                            
                            if (final_cost[i][j] < 1.0) {  // Safe cells (not dangerous)
                                min_safe_cost = std::min(min_safe_cost, final_cost[i][j]);
                                max_safe_cost = std::max(max_safe_cost, final_cost[i][j]);
                                avg_safe_cost += final_cost[i][j];
                                safe_cells++;
                            } else {
                                dangerous_cells++;
                            }
                        }
                    }
                    if (valid_cells > 0) {
                        avg_cost /= valid_cells;
                        RCLCPP_INFO(this->get_logger(), "Cost map stats (all cells): min=%.3f, max=%.3f, avg=%.3f", 
                                   min_cost, max_cost, avg_cost);
                        RCLCPP_INFO(this->get_logger(), "Safe cells: %d, Dangerous cells: %d", safe_cells, dangerous_cells);
                        if (safe_cells > 0) {
                            avg_safe_cost /= safe_cells;
                            RCLCPP_INFO(this->get_logger(), "Safe cell costs: min=%.3f, max=%.3f, avg=%.3f", 
                                       min_safe_cost, max_safe_cost, avg_safe_cost);
                        }
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to normalize weights!");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "No valid candidates generated!");
            }
            
            RCLCPP_INFO(this->get_logger(), "=== CAPTURE LOGIC COMPLETE ===");


        }

        /*Publish the goal posisiton, cost map, and anything else thats relevant to any other system here.*/
    }

    // subscriptions. Assume localization and odemetry are subscription datatypes are correct. 
    rclcpp::Subscription<vision_msgs::msg::Detection3D>::SharedPtr localization_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odometry_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr enemies_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr friendlies_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr health_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<cv::Point3f> enemies;
    std::vector<cv::Point3f> friendlies; // a vector containing point3fs of detection data. each entry is a detection data
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
