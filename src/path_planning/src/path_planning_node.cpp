#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <random>
#include <sstream>
#include <cmath> // Added for M_PI, std::sin, std::cos, std::atan2
#include <opencv2/core.hpp>

#include <vision_msgs/msg/detection3_d_array.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "../include/Grid.h"
#include "../include/Path.h"

class PathPlanningNode : public rclcpp::Node
{
public:
  PathPlanningNode() : Node("PathPlanningNode")
  {
    printf("hello world from PathPlanning package\n");
    nextPos_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("next_pos", 10);

    robotPos_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "robotPos", 10, std::bind(&PathPlanningNode::robotPos_callback, this, std::placeholders::_1));

    targetPos_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "targetPos", 10, std::bind(&PathPlanningNode::targetPos_callback, this, std::placeholders::_1));

    enemies_subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        "robots_topic", 10, std::bind(&PathPlanningNode::enemies_callback, this, std::placeholders::_1));

    // Note: change this with the actual friendlies node once created
    friendlies_subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        "robots_topic", 10, std::bind(&PathPlanningNode::friendlies_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&PathPlanningNode::periodic_callback, this));
  }

private:
  void periodic_callback()
  {
    std_msgs::msg::Int32MultiArray msg;

    if (targetPosChanged_)
    {
      grid_.setTarget(targetPos_);
    }

    if (otherRobotPosesChanged_)
    {
      otherRobotPoses_.clear(); // Clear to rebuild the list of dynamic obstacles

      // Process enemies into grid obstacles
      for (const cv::Point3f &pos : enemies)
      {
        float theta_cam = std::atan2(pos.x, pos.z);
        float theta_pos = rawRobotPos_[2] * M_PI / 180.0f; // degree -> radians
        float theta_det = theta_pos - theta_cam;           // both angles in radians now

        float x_det = rawRobotPos_[0] + pos.z * std::cos(theta_det);
        float y_det = rawRobotPos_[1] + pos.z * std::sin(theta_det);

        // Convert detected continuous position to grid coordinates
        otherRobotPoses_.push_back({static_cast<int>(x_det / TILE_SIZE), static_cast<int>(y_det / TILE_SIZE)});

        // [[maybe_unused]] float capture_center_x = 6000;
        // [[maybe_unused]] float capture_center_y = 6000;
      }

      // Process friendlies into grid obstacles (Optional: mirror the logic above if needed)
      // for(const cv::Point3f& pos : friendlies) { ... }

      for (Pos p : otherRobotPoses_)
      {
        grid_.addCostBox(p.x, p.y, COST_OTHER_ROBOT, COST_OTHER_ROBOT_ADJACENT);
      }
    }

    std::stringstream ss;
    ss << "From (" << robotPos_.x << "," << robotPos_.y << ") to (" << targetPos_.x << "," << targetPos_.y << "): \n";

    if (targetPosChanged_ || robotPosChanged_ || otherRobotPosesChanged_)
    {
      path_.reloadFromGrid(grid_);
      path_.calculate(robotPos_, grid_);
      pathPoints_ = path_.getPath();

      // Reset flags after recalculating
      otherRobotPosesChanged_ = false;
      robotPosChanged_ = false;
      targetPosChanged_ = false;
    }

    if (pathPoints_.size() > 1)
    {
      msg.data.push_back(pathPoints_[1].x);
      msg.data.push_back(pathPoints_[1].y);
      ss << "Next pos: (" << pathPoints_[1].x << ", " << pathPoints_[1].y << ")";
    }
    else if (pathPoints_.size() > 0)
    {
      msg.data.push_back(pathPoints_[0].x);
      msg.data.push_back(pathPoints_[0].y);
      ss << "Next pos: (" << pathPoints_[0].x << ", " << pathPoints_[0].y << ")";
    }
    else
    {
      ss << "No next points available";
    }

    std::stringstream full_path_ss;
    full_path_ss << "Full Path (" << pathPoints_.size() << " points): ";
    for (const auto &point : pathPoints_)
    {
      full_path_ss << "[" << point.x << ", " << point.y << "] ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", full_path_ss.str().c_str());

    ss << "\nPath used " << pathPoints_.size() << " points";

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    nextPos_publisher_->publish(msg);
  }

  void robotPos_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::vector<float> out = msg->data;

    // Save the raw pose (assumes out is [x, y, theta] based on your math logic)
    if (out.size() >= 3)
    {
      rawRobotPos_ = out;
    }

    Pos newRobotPos = {(int)(out[0] / TILE_SIZE), (int)(out[1] / TILE_SIZE)};
    if (newRobotPos.x != robotPos_.x || newRobotPos.y != robotPos_.y)
    {
      robotPosChanged_ = true;
      robotPos_ = newRobotPos;
    }

    RCLCPP_INFO(this->get_logger(), "Setting robotPos to: %d, %d", robotPos_.x, robotPos_.y);
  }

  void targetPos_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::vector<float> out = msg->data;

    Pos newTargetPos = {(int)(out[0] / TILE_SIZE), (int)(out[1] / TILE_SIZE)};
    if (newTargetPos.x != targetPos_.x || newTargetPos.y != targetPos_.y)
    {
      targetPosChanged_ = true;
      targetPos_ = newTargetPos;
    }

    RCLCPP_INFO(this->get_logger(), "Setting targetPos to: %d, %d", targetPos_.x, targetPos_.y);
  }

  void friendlies_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
  {
    otherRobotPosesChanged_ = true;
    friendlies.clear();
    for (const auto &robot : msg->detections)
    {
      cv::Point3f point;
      point.x = robot.bbox.center.position.x;
      point.y = robot.bbox.center.position.y;
      point.z = robot.bbox.center.position.z;
      friendlies.push_back(point);
    }
    RCLCPP_INFO(this->get_logger(), "Total friendlies detected: %zu", friendlies.size());
  }

  void enemies_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
  {
    otherRobotPosesChanged_ = true;
    enemies.clear();
    for (const auto &robot : msg->detections)
    {
      cv::Point3f point;
      point.x = robot.bbox.center.position.x;
      point.y = robot.bbox.center.position.y;
      point.z = robot.bbox.center.position.z;
      enemies.push_back(point);
    }
    RCLCPP_INFO(this->get_logger(), "Total enemies detected: %zu", enemies.size());
  }

  std::random_device rd;
  std::mt19937 gen{rd()};
  // std::uniform_int_distribution<> distribX{0, TILE_COUNT_X}; // Uncomment if TILE_COUNT_X is defined
  // std::uniform_int_distribution<> distribY{0, TILE_COUNT_Y};

  // Pos getRandomPoint()
  // {
  //   return Pos({distribX(gen), distribY(gen)});
  // }

  Pos robotPos_;
  std::vector<float> rawRobotPos_{0.0f, 0.0f, 0.0f}; // Stores continuous x, y, theta
  bool robotPosChanged_{false};

  Pos targetPos_;
  bool targetPosChanged_{false};

  std::vector<Pos> otherRobotPoses_;
  bool otherRobotPosesChanged_{false};

  Grid grid_;
  Path path_;
  std::vector<Pos> pathPoints_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr robotPos_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr targetPos_subscription_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr enemies_subscription_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr friendlies_subscription_;

  std::vector<cv::Point3f> enemies;
  std::vector<cv::Point3f> friendlies;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr nextPos_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanningNode>());
  rclcpp::shutdown();
  return 0;
}