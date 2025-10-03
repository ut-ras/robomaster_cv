#include <memory>
#include <chrono>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

//TODO finish class
class PatrolSentry : public rclcpp::Node{
  public:
    PatrolSentry() : Node("patrol_sentry_state_machine"){
      //TODO define subs/pubs and timer
      /*
      Required input:
      1)Location
      2)Health
      3)Ammo
      4)Detection

      Required output:
      1) State?
      2) Target pose?
      3) Target aim?
      */

      timer_ = this->create_wall_timer(1s, std::bind(&PatrolSentry::patrol, this));
    }
  private:
    //TODO define behavior
    void patrol(){
      /*
      Design questions:
      1) Where patrol?
      2) How scan (camera/chassis movement)?
      3) When shoot?
      4) Chase enemies?
      5) Stay still or zigzag?
      6) Take cover?
      7) Focus/split fire?
      */
    }

    //TODO declare subs/pubs/timer
    rclcpp::TimerBase::SharedPtr timer_;

    //TODO declare other fields
};

int main(int argc, char **argv) {
  //TODO update main
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolSentry>();   //Don't know what this does
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}