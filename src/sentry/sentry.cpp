#include <memory>
#include <chrono>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

// Define a simple enum for our states.
enum class RobotState {
  STARTUP,
  PATROL,
  DEAD
};

// A simple structure to hold 3D coordinates.
struct Point3D {
  double x;
  double y;
  double z;
};

class Sentry : public rclcpp::Node
{
public:
  Sentry() : Node("sentry_state_machine")
  {
    // Declare parameters for the startup point.
    this->declare_parameter<double>("startup_x", 0.0);
    this->declare_parameter<double>("startup_y", 0.0);
    this->declare_parameter<double>("startup_z", 0.0);

    // Get startup point parameters.
    startup_point_.x = this->get_parameter("startup_x").as_double();
    startup_point_.y = this->get_parameter("startup_y").as_double();
    startup_point_.z = this->get_parameter("startup_z").as_double();

    // Define patrol points (you could also load these from parameters or a file).
    patrol_points_.push_back({1.0, 1.0, 0.0});
    patrol_points_.push_back({2.0, 1.0, 0.0});
    patrol_points_.push_back({2.0, 2.0, 0.0});
    patrol_points_.push_back({1.0, 2.0, 0.0});
    current_patrol_index_ = 0;

    // Initialize the state to STARTUP.
    current_state_ = RobotState::STARTUP;

    // Subscriber: receive localization data (PoseStamped messages).
    localization_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "localization", 10,
      std::bind(&Sentry::localization_callback, this, std::placeholders::_1));

    // Publisher: send desired position commands to the dev board.
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("desired_position", 10);

    // Timer: update the state machine and publish target positions at 1 Hz.
    timer_ = this->create_wall_timer(1s, std::bind(&Sentry::update_state_machine, this));

    RCLCPP_INFO(this->get_logger(), "Sentry state machine node started.");
  }

private:
  // Callback to receive the robot's current position.
  void localization_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_position_ = msg->pose.position;
    RCLCPP_INFO(this->get_logger(),
      "Localization: (%.2f, %.2f, %.2f)",
      current_position_.x, current_position_.y, current_position_.z);
  }

  // This function checks the current state, publishes the appropriate target, and handles state transitions.
  void update_state_machine()
  {
    // Create a target message to send to the dev board.
    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = "map";  // Change as needed.

    switch (current_state_)
    {
      case RobotState::STARTUP:
      {
        // Command the robot to the startup point.
        target_msg.pose.position.x = startup_point_.x;
        target_msg.pose.position.y = startup_point_.y;
        target_msg.pose.position.z = startup_point_.z;
        setNeutralOrientation(target_msg);
        target_pub_->publish(target_msg);
        RCLCPP_INFO(this->get_logger(), "[STARTUP] Sending target: (%.2f, %.2f, %.2f)",
          startup_point_.x, startup_point_.y, startup_point_.z);

        // Transition to PATROL when the robot is near the startup point.
        if (distance(current_position_, startup_point_) < 0.5)
        {
          RCLCPP_INFO(this->get_logger(), "Transitioning from STARTUP to PATROL.");
          current_state_ = RobotState::PATROL;
        }
        break;
      }
      case RobotState::PATROL:
      {
        if (patrol_points_.empty())
        {
          RCLCPP_WARN(this->get_logger(), "No patrol points defined.");
          break;
        }

        // Select the current patrol point.
        Point3D patrol_target = patrol_points_[current_patrol_index_];
        target_msg.pose.position.x = patrol_target.x;
        target_msg.pose.position.y = patrol_target.y;
        target_msg.pose.position.z = patrol_target.z;
        setNeutralOrientation(target_msg);
        target_pub_->publish(target_msg);
        RCLCPP_INFO(this->get_logger(), "[PATROL] Sending target: (%.2f, %.2f, %.2f)",
          patrol_target.x, patrol_target.y, patrol_target.z);

        // If the robot is near the patrol point, advance to the next one.
        if (distance(current_position_, patrol_target) < 0.5)
        {
          current_patrol_index_ = (current_patrol_index_ + 1) % patrol_points_.size();
          RCLCPP_INFO(this->get_logger(), "Reached patrol point. Advancing to next.");
        }

        // Example condition to transition to DEAD state.
        // Here we check a ROS parameter (set externally) that, when true, kills the robot.
        if (this->has_parameter("kill_robot"))
        {
          bool kill_robot;
          this->get_parameter("kill_robot", kill_robot);
          if (kill_robot)
          {
            RCLCPP_WARN(this->get_logger(), "Transitioning from PATROL to DEAD.");
            current_state_ = RobotState::DEAD;
          }
        }
        break;
      }
      case RobotState::DEAD:
      {
        // In DEAD state, command the robot to hold its current position (i.e. stop).
        target_msg.pose.position.x = current_position_.x;
        target_msg.pose.position.y = current_position_.y;
        target_msg.pose.position.z = current_position_.z;
        setNeutralOrientation(target_msg);
        target_pub_->publish(target_msg);
        RCLCPP_INFO(this->get_logger(), "[DEAD] Robot is dead. Stopping movement.");
        break;
      }
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown state encountered!");
        break;
    }
  }

  // Helper to set an identity quaternion (no rotation) in a PoseStamped.
  void setNeutralOrientation(geometry_msgs::msg::PoseStamped &msg)
  {
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
  }

  // Helper to compute Euclidean distance between the current position and a target.
  double distance(const geometry_msgs::msg::Point &p, const Point3D &pt)
  {
    double dx = p.x - pt.x;
    double dy = p.y - pt.y;
    double dz = p.z - pt.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  // Overloaded helper to compute distance using two Point3D.
  double distance(const Point3D &p1, const Point3D &p2)
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  // ROS2 subscriber and publisher.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr localization_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Current state of the robot.
  RobotState current_state_;

  // The startup target position.
  Point3D startup_point_;

  // A list of patrol points.
  std::vector<Point3D> patrol_points_;
  size_t current_patrol_index_;

  // The latest known robot position.
  geometry_msgs::msg::Point current_position_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sentry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
