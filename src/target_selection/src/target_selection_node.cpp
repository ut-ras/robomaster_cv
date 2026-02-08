#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>


class TargetSelectionNode : public rclcpp::Node
{
public: 
  TargetSelectionNode() : Node("TargetSelection") {
    predicted_points_sub = this->create_subscription<vision_msgs::msg::Detection3DArray>("predicted_points", 10, 
                            std::bind(&TargetSelectionNode::callback, this, std::placeholders::_1));
    
    target_pub = this->create_publisher<vision_msgs::msg::Detection3D>("target_selected", 10);
  }
private:
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr predicted_points_sub;
  rclcpp::Publisher<vision_msgs::msg::Detection3D>::SharedPtr target_pub;

  void callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
    if (msg->detections.empty()) {
      RCLCPP_INFO(this->get_logger(), "No detections received.");
      return;
    }

    vision_msgs::msg::Detection3D target;

    double min_z = -1.0;
    for (const auto &detection : msg->detections) {
      double z = detection.bbox.center.position.z;

      if (z < min_z) {
        min_z = z;
        target = detection;
      }     
    }

    target_pub->publish(target);
  }
};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetSelectionNode>());
  rclcpp::shutdown();

  return 0;
}
