#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


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
      imageCb(msg);
    }
    // creating subscription variable 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  
  private: 
    // function to convert and draw on image
    void imageCb(const sensor_msgs::msg::Image &msg) const {
      cv_bridge::CvImagePtr cv_ptr; 

      try{
        // making a copy of the image to modify 
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      }catch(cv_bridge::Exception &e){
        printf("cv_brdige exception: %s", e.what());
        return; 
      }

      // writing old image to file
      cv::imwrite("old_image.jpeg", cv_ptr->image);

      // draw somthing on the image
      // circle(img, location, size, color)
      cv::Mat mat; 
      cv::cvtColor(cv_ptr->image, mat, cv::COLOR_BGR2GRAY);
      cv::circle(cv_ptr->image, cv::Point(400, 200), 100, cv::Scalar(255, 0, 0));
      cv::circle(cv_ptr->image, cv::Point(400, 400), 100, cv::Scalar(0, 255, 0));

      cv::circle(mat, cv::Point(400, 200), 100, cv::Scalar(255, 0, 0));
      cv::circle(mat, cv::Point(400, 400), 100, cv::Scalar(0, 255, 0));

      

      // writing new image to file 
      cv::imwrite("new_image.jpeg", cv_ptr->image);
      cv::imwrite("mat_image.jpeg", mat); 
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
