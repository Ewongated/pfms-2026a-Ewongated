#ifndef RACINGNODE_H
#define RACINGNODE_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include "laserprocessing.h"

enum class MissionState {
    IDLE,
    NAVIGATING,
    COMPLETE
};

class RacingNode : public rclcpp::Node
{
public:
    RacingNode();
    ~RacingNode();

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void missionService(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    // Control
    void controlLoop();
    double computeSteering(const nav_msgs::msg::Odometry& odom,
                           const geometry_msgs::msg::Point& lookAhead) const;
    void publishStop();
    void publishCommand(double throttle, double brake, double steering);
    void publishWaypoints();

    // Helpers
    static double yawFromOdom(const nav_msgs::msg::Odometry& odom);
    static double normaliseAngle(double angle);
    static double euclidean(const nav_msgs::msg::Odometry& odom,
                            const geometry_msgs::msg::Point& pt);

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       subOdom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   subLaser_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subGoals_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              pubThrottle_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              pubBrake_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              pubSteering_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarkers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr       pubWaypoints_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    // Shared state
    nav_msgs::msg::Odometry                odom_;
    bool                                   hasOdom_;
    sensor_msgs::msg::LaserScan            laser_;
    bool                                   hasLaser_;
    std::vector<geometry_msgs::msg::Point> goals_;
    std::size_t                            currentGoal_;
    MissionState                           state_;
    std::vector<geometry_msgs::msg::Point> waypoints_;
    mutable std::mutex                     mutex_;

    // Control thread
    std::thread       controlThread_;
    std::atomic<bool> running_;

    // Parameters
    double goalTolerance_;
    bool   advanced_;

    // Constants
    static constexpr double CONTROL_HZ         = 20.0;
    static constexpr double CRUISE_THROTTLE    = 0.25;
    static constexpr double MAX_BRAKE          = 8000.0;
    static constexpr double MAX_STEER          = 1;
    static constexpr double SLOW_ZONE_M        = 10.0;
    static constexpr double WAYPOINT_SPACING_M = 3.0;
    static constexpr double LOOKAHEAD_M        = 3.0;
    static constexpr double WHEELBASE_M        = 2.65;
};

#endif // RACINGNODE_H