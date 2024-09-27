#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class AravNode : public rclcpp::Node {
public:
  AravNode() : Node("arav_node") {
    printf("hello world arav_package package\n");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AravNode>());
  rclcpp::shutdown();

  return 0;
}