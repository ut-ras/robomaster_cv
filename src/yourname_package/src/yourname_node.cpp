#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class ChrisNode : public rclcpp::Node {
public:
  ChrisNode() : Node("ChrisNode") {
    printf("hello world chris_package package\n");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChrisNode>());
  rclcpp::shutdown();

  return 0;
}