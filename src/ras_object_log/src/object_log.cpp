#include <chrono>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "stampede_msgs/msg/object_log_input.hpp"
#include "stampede_msgs/msg/bounding_box.hpp"
#include "stampede_msgs/msg/turret_data.hpp"
#include "object-log/ObjectLog.h"
#include "object-log/BoundingBox.h"
#include "object-log/ArmorPlate.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{

public:
  MinimalSubscriber()
      : Node("object_log")
  {
    _objectlog = ObjectLog();
    publisher_ = this->create_publisher<stampede_msgs::msg::TurretData>("turret_target", 10);
    subscription_ = this->create_subscription<stampede_msgs::msg::ObjectLogInput>(
        "object_log_input", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void constructMessage(std::vector<BoundingBox> boxList) {
    _objectlog.boxesInput(boxList, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    auto message = stampede_msgs::msg::TurretData();
    std::vector<float> target = _objectlog.getFinalArmorPlateState();
    message.xpos = target[0];
    message.ypos = target[1];
    message.zpos = target[2];
    message.xvel = target[3];
    message.yvel = target[4];
    message.zvel = target[5];
    message.xacc = target[6];
    message.yacc = target[7];
    message.zacc = target[8];
    message.has_target = true;
    RCLCPP_INFO(this->get_logger(), "Publishing: %f %f %f %f %f %f %f %f %f", message.xpos,
                message.ypos,
                message.zpos,
                message.xvel,
                message.yvel,
                message.zvel,
                message.xacc,
                message.yacc,
                message.zacc);
    publisher_->publish(message);
  }
  void topic_callback(const stampede_msgs::msg::ObjectLogInput &msg) 
  {
    std::vector<BoundingBox> boxList;
    for (int i = 0; i < msg.boxes.size(); i++)
    {
      BoundingBox box = BoundingBox();
      box.setXCenter(msg.boxes[i].center_x);
      box.setYCenter(msg.boxes[i].center_y);
      box.setDepthVal(msg.boxes[i].depth);
      box.setWidth(msg.boxes[i].width);
      box.setHeight(msg.boxes[i].height);
      boxList.push_back(box);
    }
    constructMessage(boxList);
  }
  rclcpp::Subscription<stampede_msgs::msg::ObjectLogInput>::SharedPtr subscription_;
  rclcpp::Publisher<stampede_msgs::msg::TurretData>::SharedPtr publisher_;
  ObjectLog _objectlog;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
