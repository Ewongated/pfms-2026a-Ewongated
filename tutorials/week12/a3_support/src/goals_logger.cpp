#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <string>
#include <fstream>
// #include "logger.h"

class GoalsLogger : public rclcpp::Node
{
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;//!< 2D Goal Pose subscriber
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;//!< Visualisation Marker publsiher

  std::string filename_; //!< Filename to save goals to
  std::ofstream file_; //!< Handle to file
  visualization_msgs::msg::MarkerArray markerArray_; //!< Marker Array
  unsigned int ct_; //!< Marker Count


public:
  GoalsLogger() : 
    Node("goals_logger"), ct_(0)
  {

    //! By default this code will save goals from this package to GOALS.txt where you run the code <br>
    //! rosrun project_setup goals_logger
    //!
    //! You can also supply a file of goals yourself <br>
    //! ros2 run a3_support goals_logger --ros-args -p filename:=$HOME/GOALS.TXT
    //! In above this saves the goals to GOALS.TXT in your home directory
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Filename to save goals";
    this->declare_parameter("filename", "GOALS.txt", param_desc);
    std::string filename_ = this->get_parameter("filename").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(),"file name with goals to be saved:" << filename_);


    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 100, std::bind(&GoalsLogger::goalsCallback,this,std::placeholders::_1));
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",1000); 

    file_.open(filename_ , std::ios::out);

    if (!file_.is_open()){
        RCLCPP_FATAL_STREAM (this->get_logger(), "Can not open " << filename_  << " to save data");
        rclcpp::shutdown();
    }
  }


  ~GoalsLogger(){
      //! Closes file (via handle) if it is open
      if (file_.is_open()){
          file_.close();
      }
  }

private:
  /*! @brief Obtain MarkerArray of CUBES from geometry_msgs::Point
   * The markers are reported in world coordinate frames, namespace goals, type CUBE, colour green
   *
   *  @param goals - vector of geometry_msgs::Point
   *  @return
   */
   visualization_msgs::msg::Marker produceMarker(geometry_msgs::msg::Point pt){


      visualization_msgs::msg::Marker marker;

      //We need to set the frame
      // Set the frame ID and time stamp.
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      //We set lifetime (it will dissapear in this many seconds)
      marker.lifetime = rclcpp::Duration(1000,0); //zero is forever

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "goals_clicked"; //This is namespace, markers can be in diofferent namespace
      marker.id = ct_++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

      // The marker type
      marker.type = visualization_msgs::msg::Marker::CUBE;

      // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = pt.x;
      marker.pose.position.y = pt.y;
      marker.pose.position.z = pt.z;


      //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;


      // Set the scale of the marker -- 1m side
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      //Let's send a marker with color (green for reachable, red for now)
      std_msgs::msg::ColorRGBA color;
      color.a=0.5;//a is alpha - transparency 0.5 is 50%;
      color.r=230.0/255.0;
      color.g=230.0/255.0;
      color.b=250.0/255.0;

      marker.color = color;

      return marker;
  }

  void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
  {

      //! On each callback will save message to file, space seperated x y z values (row for each message)
      geometry_msgs::msg::Point pt = msg->pose.position;
      std::stringstream ss;

      ss << pt.x << " " << pt.y << " " << pt.z << std::endl;

      RCLCPP_INFO_STREAM(this->get_logger(),"Saving: " << ss.str());
      file_ << ss.str();

      visualization_msgs::msg::Marker marker = produceMarker(pt);
      markerArray_.markers.push_back(marker);

      pub_->publish(markerArray_);

  }




};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalsLogger>());
  rclcpp::shutdown();
  return 0;
}
