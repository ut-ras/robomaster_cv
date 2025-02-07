#include <cstdio>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

class DominicNode : public rclcpp::Node {

public:
  DominicNode() : Node("DominicNode") {
    printf("hello world dominic_package package\n");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DominicNode>());
  rclcpp::shutdown();

  return 0;
}
