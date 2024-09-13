#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class AdipuNode : public rclcpp::Node {

public:
  AdipuNode() : Node("Adipu_node") {
    printf("hello world Adipu_package package\n");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdipuNode>());
  rclcpp::shutdown();
  
  return 0;
}
