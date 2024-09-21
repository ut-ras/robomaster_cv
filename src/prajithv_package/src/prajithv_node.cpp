#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class PrajithNode : public rclcpp::Node {
  public:
    PrajithNode() : Node("PrajithNode") {
      printf("hello world prajithv_package package\n");
    }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrajithNode>());
  rclcpp::shutdown();

  return 0;
}
