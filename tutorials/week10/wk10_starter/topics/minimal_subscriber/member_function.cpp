// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <string>
#include <sstream>
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
    vizPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker",3);  
    marker_counter_=0;
    msg_str_ = "Hello World";
        timer_ = this->create_wall_timer(
          50ms, std::bind(&MinimalSubscriber::send, this));
    // rclcpp::Time t = this->get_clock()->now();
    // RCLCPP_INFO(this->get_logger(), "Node started at: %l", t.nanoseconds());
    ts_ = this->get_clock()->now();
    tnode_ = this->get_clock()->now();
  }

private:
  void topic_callback(const std_msgs::msg::String::UniquePtr msg) 
  {
    std::stringstream ss;
    ss << msg->data;
    msg_str_ = ss.str();  

    ts_ = this->get_clock()->now();
    rclcpp::Duration delta = ts_ - tnode_;
    tnode_=ts_;
    double dt = delta.seconds();
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    std::cout << msg->data << "," << dt << std::endl;
     
  }

  void send(void){
    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker marker;

    //We need to set the frame
    // Set the frame ID and time stamp.
    marker.header.frame_id = "world";
    //marker.header.seq = seq;
    rclcpp::Time t = this->get_clock()->now();
    marker.header.stamp = t;


    //We set lifetime (it will dissapear in this many seconds)
    marker.lifetime = rclcpp::Duration(30,0);
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "student_id";
    marker.id = marker_counter_;

    // The marker type, we use a cylinder in this example
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

    // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
    marker.action = visualization_msgs::msg::Marker::ADD;

    //As an example, we are setting it
    marker.pose.position.x = -4.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.5;

    //Orientation, can we orientate it?
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;


    // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;

    //Alpha is stransparency (50% transparent)
    marker.color.a = 0.5f;

    //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
    marker.color.r = 0.0;
    marker.color.g = static_cast<float>(177.0/255.0);
    marker.color.b = 0.0;

    rclcpp::Duration delta = t - ts_ -2s;
    double dt = delta.seconds();
    std::stringstream ss;
    ss << msg_str_ << " " << std::setfill('0') << std::setw(2) << std::fixed << std::setprecision(2) << dt << "s";
    marker.text = ss.str();
    markerArray.markers.push_back(marker);
    vizPub_->publish(markerArray); 

  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vizPub_;
  unsigned int marker_counter_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string msg_str_; 
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
