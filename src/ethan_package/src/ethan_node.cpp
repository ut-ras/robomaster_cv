#include <cstdio>
#include <rclcpp/rclcpp.hpp>


class PreprocessingNode : public rclcpp::Node{
  public: 
    PreprocessingNode() : Node("preprocessing_node"){
      printf("hello world ethan_package package\n");
    }
};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  // running an instance of the class forever (or until the user stops the script)
  rclcpp::spin(std::make_shared<PreprocessingNode>());
  rclcpp::shutdown();

  return 0;
}
