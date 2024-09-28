#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class DevaNode : public rclcpp::Node {
  public:
    DevaNode() : Node("DevaNode") {
      printf("hello world deva_package package\n");
    }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world deva_package package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DevaNode>());
  rclcpp::shutdown(); 

  return 0;
}
