#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "analysis.h"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("id_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "id", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    tnode_ = this->get_clock()->now();
  }

private:
  void topic_callback(const std_msgs::msg::String::UniquePtr msg) 
  {

    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());


    unsigned int count = analysis::countCharacters(msg->data);


    RCLCPP_INFO_STREAM (this->get_logger(), "We count " << count << " characters");

    //This is how to get time, and time lapsed in ROS
    // ts_ = this->get_clock()->now();
    // rclcpp::Duration delta = ts_ - tnode_;
    // double dt = delta.seconds();
     
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Time ts_;
  rclcpp::Time tnode_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
