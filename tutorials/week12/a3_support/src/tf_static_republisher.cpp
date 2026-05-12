#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class TFStaticRepublisher : public rclcpp::Node {
public:
  TFStaticRepublisher()
  : Node("tf_static_republisher")
  {
    // QoS for subscribing (volatile)
    auto qos_sub = rclcpp::QoS(1).reliable().durability(rclcpp::DurabilityPolicy::Volatile);

    // QoS for publishing (transient_local)
    auto qos_pub = rclcpp::QoS(1).reliable().durability(rclcpp::DurabilityPolicy::TransientLocal);

    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", qos_sub,
      std::bind(&TFStaticRepublisher::listener_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", qos_pub);
  }

private:
  void listener_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    publisher_->publish(*msg);
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFStaticRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}