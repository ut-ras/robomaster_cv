#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <random>
#include <sstream>

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

    // Find actual name/type for this subscription
    robotPos_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("robotPos", 10, std::bind(&PathPlanningNode::robotPos_callback, this, std::placeholders::_1));
    targetPos_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("targetPos", 10, std::bind(&PathPlanningNode::targetPos_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PathPlanningNode::periodic_callback, this));
  }

private:
  void periodic_callback()
  {
    std_msgs::msg::Int32MultiArray msg;
    // robotPos_ = getRandomPoint();
    // targetPos_ = getRandomPoint();

    if (targetPosChanged_)
    {
      grid_.setTarget(targetPos_);
      path_.reloadFromGrid(grid_);
    }

    std::stringstream ss;

    ss << "From (" << robotPos_.x << "," << robotPos_.y << ") to (" << targetPos_.x << "," << targetPos_.y << "): \n";

    if(targetPosChanged_ || robotPosChanged_){
      pathPoints_ = path_.calculate(robotPos_);
    }

    if (pathPoints_.size() > 1)
    {
      msg.data[0] = pathPoints_[1].x;
      msg.data[1] = pathPoints_[1].y;
      ss << "Next pos: (" << pathPoints_[1].x << pathPoints_[1].y << ")";
    }else if (pathPoints_.size() > 0){
      msg.data[0] = pathPoints_[0].x;
      msg.data[1] = pathPoints_[0].y;
      ss << "Next pos: (" << pathPoints_[0].x << pathPoints_[0].y << ")";
    }else{
      ss << "Next points";
    }

    ss << "\nPath used " << pathPoints_.size() << " points";

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    nextPos_publisher_->publish(msg);
  }

  void robotPos_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::vector<float> out = msg->data;

    // Do conversion from whatever unit to mm or something

    robotPos_ = {(int)(out[0] / TILE_SIZE), (int)(out[1] / TILE_SIZE)};
    robotPosChanged_ = true;

    RCLCPP_INFO(this->get_logger(), "Setting robotPos to: %d, %d,", robotPos_.x, robotPos_.y);
  }
  void targetPos_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::vector<float> out = msg->data;

    // Do conversion from whatever unit to mm or something

    targetPos_ = {(int)(out[0] / TILE_SIZE), (int)(out[1] / TILE_SIZE)};
    targetPosChanged_ = true;

    RCLCPP_INFO(this->get_logger(), "Setting targetPos to: %d, %d,", targetPos_.x, targetPos_.y);
  }

  std::random_device rd;                                                                       // Provides a non-deterministic seed
  std::mt19937 gen = std::mt19937(rd());                                                       // Seed the engine with a truly random value
  std::uniform_int_distribution<> distribX = std::uniform_int_distribution<>(0, TILE_COUNT_X); // Range [0, TILE_COUNT_Y]
  std::uniform_int_distribution<> distribY = std::uniform_int_distribution<>(0, TILE_COUNT_Y); // Range [0, TILE_COUNT_X]
  Pos getRandomPoint()
  {
    return Pos({distribX(gen), distribY(gen)});
  }

  Pos robotPos_;
  bool robotPosChanged_{false};
  Pos targetPos_;
  bool targetPosChanged_{false};

  Grid grid_;
  Path path_;
  std::vector<Pos> pathPoints_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr robotPos_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr targetPos_subscription_;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr nextPos_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PathPlanningNode>());

  rclcpp::shutdown();

  return 0;
}
