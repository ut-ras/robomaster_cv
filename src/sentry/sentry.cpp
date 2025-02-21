#include <memory>
#include <chrono>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

// Define the three states.
enum class RobotState {
  STARTUP,
  PATROL,
  DEAD
};

// Simple structure to hold a 3D point.
struct Pose2D { // top left of field is 0,0
  double x;
  double y;
  double theta; // Rotation in radians    - 0 is same as coordinate plane
};


class Sentry : public rclcpp::Node
{
public:
  Sentry() : Node("sentry_state_machine")
  {
    // Initialize health to full (100%).
    current_health_ = 100.0;

    // Define the three startup points.
    // These could also be loaded from parameters.
    std::vector<Pose2D> startup_points_ = {
      {0.0, 0.0, 0.0},   // First point with 0 rotation
      {0.0, -4, M_PI/2}, // Facing 90 degrees
      {4, -4, M_PI},     // Facing 180 degrees
      {4, -2, -M_PI/2}   // Facing -90 degrees
  };


    // Initialize indices:
    // - current_startup_index_ tracks our progress forward through the startup points.
    // - current_reverse_index_ is used in DEAD state (starting at the last point).
    current_startup_index_ = 0;
    current_reverse_index_ = startup_points_.size() - 1;

    // Initial state: STARTUP.
    current_state_ = RobotState::STARTUP;

    // Subscribe to localization data (PoseStamped).
    localization_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "localization", 10,
      std::bind(&Sentry::localization_callback, this, std::placeholders::_1));

    // Subscribe to health data (Float32, representing percentage).
    health_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "health", 10,
      std::bind(&Sentry::health_callback, this, std::placeholders::_1));

    // Publisher to send the desired target position.
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("desired_position", 10);

    // Timer to update the state machine (runs at 1 Hz).
    timer_ = this->create_wall_timer(1s, std::bind(&Sentry::update_state_machine, this));

    RCLCPP_INFO(this->get_logger(), "Sentry state machine node started.");
  }

private:
  // Callback to update the robot's current position.
  void localization_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_position_ = msg->pose.position;
    RCLCPP_INFO(this->get_logger(), "Localization: (%.2f, %.2f, %.2f)",
                current_position_.x, current_position_.y, current_position_.z);
  }

  // Callback to update the robot's current health.
  void health_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_health_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Health: %.2f%%", current_health_);
  }

  // Main state machine update function.
  void update_state_machine()
  {
    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = "map";  // Adjust the frame if needed.
    setNeutralOrientation(target_msg);

    // Distance threshold for considering that a target point has been reached.
    const double threshold = 0.5;

    switch (current_state_)
    {
      case RobotState::STARTUP:
      {
        // Navigate through startup points in forward order.
        Pose2D target_point = startup_points_[current_startup_index_];
        target_msg.pose.position.x = target_point.x;
        target_msg.pose.position.y = target_point.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, target_pose.theta); // Convert theta to quaternion
        target_msg.pose.orientation.x = q.x();
        target_msg.pose.orientation.y = q.y();
        target_msg.pose.orientation.z = q.z();
        target_msg.pose.orientation.w = q.w();

        target_pub_->publish(target_msg);

        RCLCPP_INFO(this->get_logger(),
                    "[STARTUP] Sending target: (%.2f, %.2f, %.2f), index: %zu",
                    target_point.x, target_point.y, target_point.z, current_startup_index_);

        // If the robot is near the current startup point, move to the next.
        if (distance(current_position_, target_point) < threshold)
        {
          if (current_startup_index_ < startup_points_.size() - 1)
          {
            current_startup_index_++;
          }
          else
          {
            // Finished all startup points; transition to PATROL.
            current_state_ = RobotState::PATROL;
            RCLCPP_INFO(this->get_logger(), "Transitioning from STARTUP to PATROL.");
          }
        }
        break;
      }
      case RobotState::PATROL:
      {
        // In PATROL state, hold position at the final startup point.
        Pose2D patrol_target = startup_points_.back();
        target_msg.pose.position.x = patrol_target.x;
        target_msg.pose.position.y = patrol_target.y;
        target_msg.pose.position.z = patrol_target.z;
        target_pub_->publish(target_msg);

        RCLCPP_INFO(this->get_logger(),
                    "[PATROL] Holding at: (%.2f, %.2f, %.2f)",
                    patrol_target.x, patrol_target.y, patrol_target.z);

        // If health drops below 20%, transition to DEAD state.
        if (current_health_ < 20.0)
        {
          current_state_ = RobotState::DEAD;
          current_reverse_index_ = startup_points_.size() - 1;  // Start from the last startup point.
          RCLCPP_WARN(this->get_logger(),
                      "Health low (%.2f%%). Transitioning from PATROL to DEAD.",
                      current_health_);
        }
        break;
      }
      case RobotState::DEAD:
      {
        // Navigate the startup points in reverse order.
        Pose2D target_point = startup_points_[current_reverse_index_];
        target_msg.pose.position.x = target_point.x;
        target_msg.pose.position.y = target_point.y;
        target_msg.pose.position.z = target_point.z;
        target_pub_->publish(target_msg);

        RCLCPP_INFO(this->get_logger(),
                    "[DEAD] Sending target: (%.2f, %.2f, %.2f), reverse index: %zu",
                    target_point.x, target_point.y, target_point.z, current_reverse_index_);

        // If the robot is near the target, move to the previous point (if available).
        if (distance(current_position_, target_point) < threshold)
        {
          if (current_reverse_index_ > 0)
          {
            current_reverse_index_--;
          }
        }

        // If health recovers above 70%, transition back to STARTUP.
        if (current_health_ > 70.0)
        {
          current_state_ = RobotState::STARTUP;
          current_startup_index_ = 0;  // Reset to start the forward sequence again.
          RCLCPP_INFO(this->get_logger(),
                      "Health recovered (%.2f%%). Transitioning from DEAD to STARTUP.",
                      current_health_);
        }
        break;
      }
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown state encountered!");
        break;
    }
  }

  // Helper: set an identity quaternion (no rotation) for the target message.
  void setNeutralOrientation(geometry_msgs::msg::PoseStamped &msg)
  {
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
  }

  // Helper: compute Euclidean distance between current position and a target point.
  double distance(const geometry_msgs::msg::Point &p, const Pose2D &pt)
{
    double dx = p.x - pt.x;
    double dy = p.y - pt.y;
    return std::sqrt(dx * dx + dy * dy);  // Ignore z-component
}

  // Subscribers and publisher.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr health_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State machine variables.
  RobotState current_state_;
  std::vector<Pose2D> startup_points_;
  size_t current_startup_index_;
  size_t current_reverse_index_;

  // Latest known robot position and health.
  geometry_msgs::msg::Point current_position_;
  float current_health_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sentry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
