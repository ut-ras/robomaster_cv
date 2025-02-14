#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
using std::placeholders::_1;


class PreprocessingNode : public rclcpp::Node{
  public: 
    PreprocessingNode() : Node("preprocessing_node"){
      printf("hello world ethan_package package\n");

      // create a subscriber 
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, std::bind(&PreprocessingNode::topic_callback, this, _1));
    }

  private:
  // method will run anytime a message is recieved from the publisher
  void topic_callback(const sensor_msgs::msg::Image &msg) const {
    //printing out the data
    RCLCPP_INFO(this->get_logger(), "Message recieved: width: %d, height: %d\n", msg.width, msg.height);
  }
  // creating subscription variable 
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
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
