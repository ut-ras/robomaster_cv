#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class BradenClass : public rclcpp::Node {
public: 
  BradenClass() : Node("braden_node") {
    printf("Hello World braden_package package\n");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BradenClass>());
  rclcpp::shutdown();
  return 0;
}
