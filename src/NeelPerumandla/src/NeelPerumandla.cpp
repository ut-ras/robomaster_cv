#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class Neel : public rclcpp::Node{
  
public:
  Neel() : Node("Neel"){
    printf("hello world NeelPerumandla package\n");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Neel>());
  rclcpp::shutdown();

  
  return 0;
}
