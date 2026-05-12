#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include <deque>
#include <mutex>
#include <thread>

class Mission : public rclcpp::Node
{

public:
  Mission();

  ~Mission();

  visualization_msgs::msg::Marker produceMarker(geometry_msgs::msg::Point pt);

  void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);

  void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

private:
  void run();//!< Function that will run continously in the thread
  void progress();//!< Function will update of progress of the mission (triggered by the timer)
  double distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt);//<! distance between the arguments

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;//!< 2D Goal Pose subscriber
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;//!< Visualisation Marker publsiher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_goals_;//<! Publisher of goals

  std::thread* thread_; //!< Thread object pointer
  rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals

  visualization_msgs::msg::MarkerArray markerArray_; //!< Marker Array
  unsigned int ct_; //!< Marker Count

  std::mutex mtxOdo_;//<! Mutex for odo
  nav_msgs::msg::Odometry odo_; //!< Storage of odometry 


  std::mutex mtxGoals_;//<! Mutex for goals_ and init_dist_to_goal_;
  std::deque<geometry_msgs::msg::Point> goals_; //!< Storage of goals 
  double init_dist_to_goal_;

  double tolerance_;

  std::atomic<bool> chasing_;//<! Indicates if we are chasing (pursuing goal)
};

