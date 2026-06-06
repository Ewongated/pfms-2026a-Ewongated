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
    IDLE,       //!< Stopped, waiting for mission start
    CRUISING,   //!< Driving forward on a straight / gentle curve -- full throttle
    TURNING,    //!< Sharp corner -- corner braking + feathered throttle
    COMPLETE    //!< All goals reached or mission aborted
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
    void publishStop();
    void publishCommand(double throttle, double brake, double steering);
    void publishWaypoints();

    // Helpers
    double computeAlpha(const nav_msgs::msg::Odometry& odom,
                        const geometry_msgs::msg::Point& target) const;
    static double yawFromOdom(const nav_msgs::msg::Odometry& odom);
    static double normaliseAngle(double angle);
    static double euclidean(const nav_msgs::msg::Odometry& odom,
                            const geometry_msgs::msg::Point& pt);
    static const char* stateName(MissionState s);

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       subOdom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   subLaser_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subGoals_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               pubThrottle_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               pubBrake_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               pubSteering_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarkers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr        pubWaypoints_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    // Laser processing
    std::shared_ptr<LaserProcessing> laserProc_;

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

    // Steering state
    double prevAlpha_ = 0.0;  //!< Previous heading error for derivative damping

    // Constants -- shared
    static constexpr double      CONTROL_HZ            = 20.0;
    static constexpr double      MAX_BRAKE              = 8000.0;
    static constexpr double      MAX_STEER              = 5.0;
    static constexpr double      WHEELBASE_M            = 2.65;
    static constexpr double      V_MAX                  = 15.0;
    static constexpr double      CORNER_ALPHA_RAD       = 0.4;  //!< Threshold to enter TURNING [rad] (~23 deg)
    static constexpr double      MIN_SPEED_FACTOR       = 0.3;

    // Constants -- CRUISING state
    static constexpr double      CRUISE_THROTTLE        = 0.5;
    static constexpr double      STEER_K                = 1.2;  //!< Nonlinear steering gain
    static constexpr double      STEER_KD               = 0.5;  //!< Derivative damping gain
    static constexpr std::size_t GOAL_LOOKAHEAD         = 6;    //!< Goals ahead to steer toward

    // Constants -- TURNING state
    static constexpr double      CORNER_BRAKE           = 8000.0;
    static constexpr double      TURN_THROTTLE          = 0.3;       //!< Feathered throttle mid-corner
    static constexpr double      TURN_V_MAX             = 4.5;       //!< Hard speed cap in TURNING [m/s]
    static constexpr double      TURN_STEER_K           = 7.0;       //!< Tighter steering gain when corner imminent
    static constexpr double      TURN_STEER_KD          = 0.2;       //!< Derivative damping when corner imminent
    static constexpr std::size_t TURN_GOAL_LOOKAHEAD    = 3;         //!< Closer steer target when corner imminent
    static constexpr double      BRAKE_PREVIEW_DIST_M   = 6.0;       //!< Pre-corner braking distance [m]
};
#endif // RACINGNODE_H