#include <rclcpp/rclcpp.hpp>
#include <functional>

#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <set>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class SingleMessageRecorder : public rclcpp::Node
{
public:
    SingleMessageRecorder()
    : Node("single_message_recorder")
    {
      //
      //ros2 run a3_support single_message_recorder --ros-args --params-file /path/to/params.yaml

      // Declare and get the bag_name parameter
      std::string bag_name = this->declare_parameter<std::string>("bag_name", generate_default_bag_name());

      // Declare and get the topics parameter
      topics_ = this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});

      // Check if topics are provided
      if (topics_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No topics provided to record. Please set the 'topics' parameter.");
        rclcpp::shutdown();
        return;
      }

      // Initialize the writer and open the bag file
      writer_ = std::make_shared<rosbag2_cpp::Writer>();
      writer_->open(bag_name);

      RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name.c_str());

      // Get all available topics and their types
      auto topics_all = this->get_topic_names_and_types();
 
      for (const auto & topic : topics_all) {
        // RCLCPP_INFO(this->get_logger(), "Topic: %s, Type: %s", topic.first.c_str(), topic.second[0].c_str());
    
        // Check if the topic is in the list of topics to record
        if (std::find(topics_.begin(), topics_.end(), topic.first) != topics_.end()) {
            // RCLCPP_INFO(this->get_logger(), "Recording topic: %s", topic.first.c_str());
    
            // Dynamically create subscriptions based on the message type
            if (topic.second[0] == "tf2_msgs/msg/TFMessage" && topic.first == "/tf_static") {
                auto subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
                    topic.first, rclcpp::QoS(1).transient_local(), // Use transient_local QoS
                    [this, topic_name = topic.first](tf2_msgs::msg::TFMessage::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "tf2_msgs/msg/TFMessage");
                    });
                subscriptions_.push_back(subscription);
            } else if (topic.second[0] == "std_msgs/msg/String") {
                auto subscription = this->create_subscription<std_msgs::msg::String>(
                    topic.first, 10,
                    [this, topic_name = topic.first](std_msgs::msg::String::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<std_msgs::msg::String> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "std_msgs/msg/String");
                    });
                subscriptions_.push_back(subscription);
            } else if (topic.second[0] == "sensor_msgs/msg/Range") {
                auto subscription = this->create_subscription<sensor_msgs::msg::Range>(
                    topic.first, 10,
                    [this, topic_name = topic.first](sensor_msgs::msg::Range::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<sensor_msgs::msg::Range> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "sensor_msgs/msg/Range");
                    });
                subscriptions_.push_back(subscription);
            } else if (topic.second[0] == "tf2_msgs/msg/TFMessage") {
              auto subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
                  topic.first, 10,
                  [this, topic_name = topic.first](tf2_msgs::msg::TFMessage::SharedPtr msg) {
                      auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                      rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
                      serializer.serialize_message(msg.get(), serialized_msg.get());
                      this->record_message(serialized_msg, topic_name, "tf2_msgs/msg/TFMessage");
                  });
              subscriptions_.push_back(subscription);
            } else if (topic.second[0] == "geometry_msgs/msg/PoseArray") {
                auto subscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
                    topic.first, 10,
                    [this, topic_name = topic.first](geometry_msgs::msg::PoseArray::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<geometry_msgs::msg::PoseArray> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "geometry_msgs/msg/PoseArray");
                    });
                subscriptions_.push_back(subscription);
            } else if (topic.second[0] == "geometry_msgs/msg/PoseStamped") {
                auto subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    topic.first, 10,
                    [this, topic_name = topic.first](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "geometry_msgs/msg/PoseStamped");
                    });
                subscriptions_.push_back(subscription);
              } else if (topic.second[0] == "nav_msgs/msg/Odometry") {
                auto subscription = this->create_subscription<nav_msgs::msg::Odometry>(
                    topic.first, 10,
                    [this, topic_name = topic.first](nav_msgs::msg::Odometry::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "nav_msgs/msg/Odometry");
                    });
                subscriptions_.push_back(subscription);
             } else if (topic.second[0] == "sensor_msgs/msg/LaserScan") {
                auto subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    topic.first, 10,
                    [this, topic_name = topic.first](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "sensor_msgs/msg/LaserScan");
                    });
                subscriptions_.push_back(subscription);                
              } else if (topic.second[0] == "std_msgs/msg/Empty") {
                auto subscription = this->create_subscription<std_msgs::msg::Empty>(
                    topic.first, 10,
                    [this, topic_name = topic.first](std_msgs::msg::Empty::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<std_msgs::msg::Empty> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "std_msgs/msg/Empty");
                    });
                subscriptions_.push_back(subscription);
              } else if (topic.second[0] == "std_msgs/msg/Float64") {
                auto subscription = this->create_subscription<std_msgs::msg::Float64>(
                    topic.first, 10,
                    [this, topic_name = topic.first](std_msgs::msg::Float64::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<std_msgs::msg::Float64> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "std_msgs/msg/Float64");
                    });
                subscriptions_.push_back(subscription);
              } else if (topic.second[0] == "grid_map_msgs/msg/GridMap") {
                auto subscription = this->create_subscription<grid_map_msgs::msg::GridMap>(
                    topic.first, 10,
                    [this, topic_name = topic.first](grid_map_msgs::msg::GridMap::SharedPtr msg) {
                        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                        rclcpp::Serialization<grid_map_msgs::msg::GridMap> serializer;
                        serializer.serialize_message(msg.get(), serialized_msg.get());
                        this->record_message(serialized_msg, topic_name, "grid_map_msgs/msg/GridMap");
                    });
                subscriptions_.push_back(subscription);
              } else {
                RCLCPP_WARN(this->get_logger(), "Unsupported message type: %s for topic: %s",
                            topic.second[0].c_str(), topic.first.c_str());
            }
        }
    }

  }

  ~SingleMessageRecorder()
  {
      if (writer_ && !writer_closed_) {
          writer_->close();
          writer_closed_ = true;
          RCLCPP_INFO(this->get_logger(), "Recorded %d total messages. Bag file closed.", message_count_);
      }
  }
 
private:    
  void record_message(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg, const std::string & topic, const std::string & type)
  {
      // Create topic in bag if not already created
      if (created_topics_.find(topic) == created_topics_.end()) {
          rosbag2_storage::TopicMetadata topic_metadata;
          topic_metadata.name = topic;
          topic_metadata.type = type;
          topic_metadata.serialization_format = "cdr";
          writer_->create_topic(topic_metadata);
          created_topics_.insert(topic);
      }
      
      // Write the serialized message to the bag
      writer_->write(serialized_msg, topic, type, this->now());

      message_count_++;
      
      // Throttle logging for /tf to avoid spam (log every 0.5s)
      if (topic == "/tf") {
          auto now = this->now();
          if ((now - last_tf_log_time_).seconds() >= 0.5) {
              RCLCPP_INFO(this->get_logger(), "Recording /tf messages... (%d total)", message_count_);
              last_tf_log_time_ = now;
          }
      } else {
          RCLCPP_INFO(this->get_logger(), "Recorded message from %s", topic.c_str());
      }

      // Special handling for /tf - keep recording for a few seconds
      if (topic == "/tf") {
          // Don't remove /tf subscription, let timer handle it
          return;
      }

      // For all other topics - record once and remove subscription
      subscriptions_.erase(
          std::remove_if(subscriptions_.begin(), subscriptions_.end(),
                        [&topic](const auto & sub) { return sub->get_topic_name() == topic; }),
          subscriptions_.end());

      // When first non-tf message is recorded, start shutdown timer for /tf
      if (!shutdown_timer_ && topic != "/tf_static") {
          RCLCPP_INFO(this->get_logger(), "First snapshot recorded, will record /tf for 2 more seconds...");
          shutdown_timer_ = this->create_wall_timer(
              std::chrono::seconds(2),
              [this]() {
                  if (writer_ && !writer_closed_) {
                      writer_->close();
                      writer_closed_ = true;
                  }
                  RCLCPP_INFO(this->get_logger(), "Finished recording %d messages", message_count_);
                  rclcpp::shutdown();
              });
      }

      // If all non-tf subscriptions are done and no timer yet, shutdown now
      bool has_non_tf = false;
      for (const auto& sub : subscriptions_) {
          std::string topic_name = sub->get_topic_name();
          if (topic_name != "/tf" && topic_name != "/tf_static") {
              has_non_tf = true;
              break;
          }
      }
      if (!has_non_tf && !shutdown_timer_) {
          if (writer_ && !writer_closed_) {
              writer_->close();
              writer_closed_ = true;
          }
          RCLCPP_INFO(this->get_logger(), "Finished recording %d messages", message_count_);
          rclcpp::shutdown();
      }
  }
  
  int message_count_ = 0;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  rclcpp::Time last_tf_log_time_{0, 0, RCL_ROS_TIME};
  std::set<std::string> created_topics_;
  bool writer_closed_ = false;
  
  std::string generate_default_bag_name()
  {
      // Get the current time
      auto now = std::chrono::system_clock::now();
      auto time_t_now = std::chrono::system_clock::to_time_t(now);

      // Format the time as YYYY-MM-DD_HH-MM-SS
      std::ostringstream oss;
      oss << "sample_" << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d_%H-%M-%S");
      return oss.str();
  }  
 
    std::shared_ptr<rosbag2_cpp::Writer> writer_;
    std::vector<std::string> topics_;
    //std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};
 
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleMessageRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}