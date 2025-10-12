#include <memory>
#include <chrono>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

// Constants for health thresholds
constexpr float HEALTH_THRESHOLD_LOW = 20.0;    // Transition to DEAD
constexpr float HEALTH_THRESHOLD_RECOVERED = 70.0; // Transition back to STARTUP

// Define the three states.
enum class RobotState {
  STARTUP,
  PATROL,
  DEAD
};

// Simple structure to hold a 2D pose with orientation.
struct Pose2D { // top left of field is 0,0
  double x;
  double y;
  double theta; // Rotation in radians - 0 is same as coordinate plane
};

class Sentry : public rclcpp::Node {
public:
  Sentry() : Node("sentry_state_machine"), 
             startup_points_({
                 {0.0, 0.0, 0.0},   // TODO: Review these positions and orientations for tuning
                 {0.0, -4, M_PI/2},  // TODO: Adjust path if needed
                 {4, -4, M_PI},      // TODO: Confirm final patrol route
                 {4, -2, -M_PI/2}    // TODO: Add more waypoints if required
             }),
             current_startup_index_(0),
             current_reverse_index_(startup_points_.size() - 1),
             current_health_(100.0),
             current_state_(RobotState::STARTUP) 
  {
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
  //Localization callback: Updates pose and prints coordinates
  void localization_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_position_ = msg->pose.position;
    RCLCPP_INFO(this->get_logger(), "Localization: (%.2f, %.2f)",
                current_position_.x, current_position_.y);
  }

  //Health callback: Updates health and prints it
  void health_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_health_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Health: %.2f%%", current_health_);
  }

  void update_state_machine() {

    /*Creating a message containing target pos, setting header time and pose format*/
    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = "map";  // Adjust the frame if needed.

    const double threshold = 0.5;

    switch (current_state_) {

      /*Go through start points ascending until reach end, then switch to patrol*/
      case RobotState::STARTUP: {
        /*Update targets position to startup points*/
        Pose2D target_point = startup_points_[current_startup_index_];
        target_msg.pose.position.x = target_point.x;
        target_msg.pose.position.y = target_point.y;
        target_msg.pose.position.z = 0.0; // Fixed: No z-component in Pose2D

        /*Update target rotation to startupt point*/
        tf2::Quaternion q;
        q.setRPY(0, 0, target_point.theta);
        target_msg.pose.orientation.x = q.x();
        target_msg.pose.orientation.y = q.y();
        target_msg.pose.orientation.z = q.z();
        target_msg.pose.orientation.w = q.w();

        /*Publishh the target pose*/
        target_pub_->publish(target_msg);

        /*Print 'target sent' message*/
        RCLCPP_INFO(this->get_logger(),
                    "[STARTUP] Sending target: (%.2f, %.2f), index: %zu",
                    target_point.x, target_point.y, current_startup_index_);

        /*If our current position is within tolerance of target*/
        if (distance(current_position_, target_point) < threshold) {

          /*If not done with startup, update index if not at end of startup pints; 
          else switch to patrol and print*/
          if (current_startup_index_ < startup_points_.size() - 1) {
            current_startup_index_++;
          } else {
            current_state_ = RobotState::PATROL;
            RCLCPP_INFO(this->get_logger(), "Transitioning from STARTUP to PATROL.");
          }
        }
        break;
      }

      /*Go to back of startup points, set state to dead if low enough*/
      case RobotState::PATROL: {

        /*Set patrol target to the back element of the startup points list and publish/print*/
        Pose2D patrol_target = startup_points_.back();
        target_msg.pose.position.x = patrol_target.x;
        target_msg.pose.position.y = patrol_target.y;
        target_msg.pose.position.z = 0.0;
        target_pub_->publish(target_msg);

        RCLCPP_INFO(this->get_logger(), "[PATROL] Holding at: (%.2f, %.2f)",
                    patrol_target.x, patrol_target.y);

        /*If health below low threshold, set state to dead, reset reverse index, print*/
        if (current_health_ < HEALTH_THRESHOLD_LOW) {
          current_state_ = RobotState::DEAD;
          current_reverse_index_ = startup_points_.size() - 1;
          RCLCPP_WARN(this->get_logger(), "Health low (%.2f%%). Transitioning to DEAD.", current_health_);
        }
        break;
      }

      /*Reatreat; go through startup points descending until reach start,
      change to startup state once healed*/
      case RobotState::DEAD: {

        /*Set target to startup point reverse index and publish/print*/
        Pose2D target_point = startup_points_[current_reverse_index_];
        target_msg.pose.position.x = target_point.x;
        target_msg.pose.position.y = target_point.y;
        target_msg.pose.position.z = 0.0;
        target_pub_->publish(target_msg);

        RCLCPP_INFO(this->get_logger(), "[DEAD] Sending target: (%.2f, %.2f), reverse index: %zu",
                    target_point.x, target_point.y, current_reverse_index_);

        /*If within threshold of target point, update reverse index if reverse index > 0*/
        if (distance(current_position_, target_point) < threshold) {
          if (current_reverse_index_ > 0) {
            current_reverse_index_--;
          }
        }

        /*If health above recovered threshold, set state to startup, reset startup index to 0, and print*/
        if (current_health_ > HEALTH_THRESHOLD_RECOVERED) {
          current_state_ = RobotState::STARTUP;
          current_startup_index_ = 0;
          RCLCPP_INFO(this->get_logger(), "Health recovered (%.2f%%). Transitioning to STARTUP.", current_health_);
        }
        break;
      }

      /*Something went wrong*/
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown state encountered!");
        break;
    }
  }

  /*Grabs distance between two points*/
  double distance(const geometry_msgs::msg::Point &p, const Pose2D &pt) {
    double dx = p.x - pt.x;
    double dy = p.y - pt.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  /*Subscription/Publisher members + timer member*/
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr health_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  RobotState current_state_;                    //Robot behavior
  std::vector<Pose2D> startup_points_;          //Sentry Path
  size_t current_startup_index_;                //Startup index
  size_t current_reverse_index_;                //Retreat index
  geometry_msgs::msg::Point current_position_;  //Where we are
  float current_health_;                        //Health
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sentry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
