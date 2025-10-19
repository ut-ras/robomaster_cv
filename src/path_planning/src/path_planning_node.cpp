#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <random>
#include <sstream>

#include "std_msgs/msg/int32_multi_array.hpp"

#include "../include/Grid.h"
#include "../include/Path.h"

class PathPlanningNode : public rclcpp::Node
{
public:
  PathPlanningNode() : Node("PathPlanningNode")
  {
    printf("hello world from PathPlanning package\n");
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("int_array", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("int_arrays", 10, std::bind(&PathPlanningNode::data_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PathPlanningNode::callback, this));
  }

private:
  void callback()
  {
    std_msgs::msg::Int32MultiArray msg;
    Pos p = getRandomPoint();
    Pos p2 = getRandomPoint();
    // msg.data = {p.x, p.y, p2.x, p2.y};

    grid_.setTarget(p2);
    path_.reloadFromGrid(grid_);

    std::stringstream ss;
    
    ss << "From (" << p.x << "," << p.y << ") to ("<< p2.x << "," << p2.y << "): \n";

    std::vector<Pos> points = path_.calculate(p);
    for (Pos p : points)
    {
      msg.data.push_back(p.x);
      msg.data.push_back(p.y);
      ss << "(" << p.x << "," << p.y << "), ";
    }

    ss << "\nPath used " << points.size() << " points";

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    publisher_->publish(msg);
  }

  void data_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        std::vector<int> out = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received: %d, %d, %d, %d, %d", out[0], out[1], out[2], out[3], out[4]);
    }

  std::random_device rd;  // Provides a non-deterministic seed
  std::mt19937 gen = std::mt19937(rd()); // Seed the engine with a truly random value
  std::uniform_int_distribution<> distribX = std::uniform_int_distribution<>(0, TILE_COUNT_X); // Range [0, TILE_COUNT_Y]
  std::uniform_int_distribution<> distribY = std::uniform_int_distribution<>(0, TILE_COUNT_Y); // Range [0, TILE_COUNT_X]
  Pos getRandomPoint()
  {
    return Pos({distribX(gen), distribY(gen)});
  }

  Grid grid_;
  Path path_;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
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
