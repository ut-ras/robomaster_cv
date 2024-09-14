#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class jose_pedrazaNode : public rclcpp::Node {
public:
  jose_pedrazaNode() : Node("jose_pedrazaNode") {
    printf("hello world jose_pedraza_package package\n");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jose_pedrazaNode>());
  rclcpp::shutdown();

  return 0;
}