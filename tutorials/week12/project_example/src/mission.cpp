#include "mission.h"
#include <iostream>

Mission::Mission() : 
    Node("mission"), ct_(0), chasing_(false), tolerance_(0.5)
{

    //! By default this code will save goals from this package to GOALS.txt where you run the code <br>
    //! rosrun project_setup goals_logger
    //!
    //! You can also supply a file of goals yourself <br>
    //! ros2 run a3_support goals_logger --ros-args -p filename:=$HOME/GOALS.TXT
    //! In above this saves the goals to GOALS.TXT in your home directory
    // auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    // param_desc.description = "Filename to save goals";
    // node_handle_->declare_parameter("filename", "GOALS.txt", param_desc);
    // std::string filename = node_handle_->get_parameter("filename").as_string();
    // RCLCPP_INFO_STREAM(node_handle_->get_logger(),"file name with goals to be saved:" << filename);


    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 100, std::bind(&Mission::goalsCallback,this,std::placeholders::_1));
    sub_odo_ = this->create_subscription<nav_msgs::msg::Odometry>("drone/gt_odom", 100, std::bind(&Mission::odomCallback,this,std::placeholders::_1));
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",1000); 
    pub_goals_ = this->create_publisher<geometry_msgs::msg::Point>("/drone/goal", 10);

    // We create as an example here a function that is called via a timer at a set interval
    // Create a timer that calls the timerCallback function every 500ms
    timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&Mission::progress, this));

    // Finally we create a thread that will run the function threadFunction
    // The thread could start before any data is recieved via callbacks as we have not called spin yet
    // So consider this when you write your code
    thread_ = new std::thread(&Mission::run, this);

}


Mission::~Mission(){

    thread_->join();
    std::cout << __func__ << " finishing" << std::endl;
}

visualization_msgs::msg::Marker Mission::produceMarker(geometry_msgs::msg::Point pt){


      visualization_msgs::msg::Marker marker;

      //We need to set the frame
      // Set the frame ID and time stamp.
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      //We set lifetime (it will dissapear in this many seconds)
      marker.lifetime = rclcpp::Duration(1000,0); //zero is forever

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "goals"; //This is namespace, markers can be in diofferent namespace
      marker.id = ct_++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

      // The marker type
      marker.type = visualization_msgs::msg::Marker::CYLINDER;

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
      color.r=0;
      color.g=0;
      color.b=250.0/255.0;

      marker.color = color;

      return marker;
}

void Mission::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
{

    //! On each callback will save message to file, space seperated x y z values (row for each message)
    geometry_msgs::msg::Point pt = msg->pose.position;


    visualization_msgs::msg::Marker marker = produceMarker(pt);
    markerArray_.markers.push_back(marker);

    pub_->publish(markerArray_);

    //Let's also send the goal to quacdopter

    goals_.push_back(pt);

    RCLCPP_INFO_STREAM(this->get_logger(),"Have: " << goals_.size() << " goals");

    //pub_goals_->publish(pt);
}

void Mission::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    std::unique_lock<std::mutex> lck(mtxOdo_);
    odo_ = *msg;
    lck.unlock();
}

void Mission::progress(){

    if(chasing_){
        nav_msgs::msg::Odometry odo;
        geometry_msgs::msg::Point goal;

        std::unique_lock<std::mutex> lck1(mtxOdo_);
        odo = odo_;
        lck1.unlock();
            
        std::unique_lock<std::mutex> lck2(mtxGoals_);
        goal = goals_.front();
        lck2.unlock();

        double dist = distance(odo,goal);

        unsigned int progress = (unsigned int)(100.0*((init_dist_to_goal_-dist)/init_dist_to_goal_));

        RCLCPP_INFO_STREAM(this->get_logger(),"Progress:"<< progress);
    }
}

double Mission::distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt){

    return std::pow( ( std::pow(odo.pose.pose.position.x-pt.x,2) + std::pow(odo.pose.pose.position.y-pt.y,2)  ) ,0.5);
}


void Mission::run(){

    // This function runs in a separate thread of execution, it is started in the constructor
    // We can call any function from here, but we need to be careful with the shared resources
    // We need to make sure that we are thread safe
    // This function needs to terminate when the node is destroyed and we can do so by checking if the node is still alive
    while(rclcpp::ok()){

        nav_msgs::msg::Odometry odo;
        geometry_msgs::msg::Point goal;

        if(goals_.size()>0){

            std::unique_lock<std::mutex> lck1(mtxOdo_);
            odo = odo_;
            lck1.unlock();
            
            std::unique_lock<std::mutex> lck2(mtxGoals_);
            goal = goals_.front();
            lck2.unlock();
        }

        if(chasing_){
            double dist = distance(odo,goal);
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),2.0,"dist:"<< dist );

            if(dist<tolerance_){
                goals_.pop_front();//Removoing element at front
                RCLCPP_INFO(this->get_logger(),"Reached and going for new goal");
                //Now lets get stats of the new goal
                if(goals_.size()>0){                    
                    goal = goals_.front();
                    init_dist_to_goal_=distance(odo,goal);
                    pub_goals_->publish(goal);
                }
            }

            if(goals_.size()==0){
                chasing_=false;
            }
        }
        else{
            if(goals_.size()>0){
                chasing_=true;
                goal = goals_.front();
                init_dist_to_goal_=distance(odo,goal);
                pub_goals_->publish(goal);
                RCLCPP_INFO(this->get_logger(),"Sending first goal");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));// we sleep for 10 miliseconds
    }    

    std::cout << __func__ << " finishing" << std::endl;

}