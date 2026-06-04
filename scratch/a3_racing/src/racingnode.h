#ifndef RACINGNODE_H
#define RACINGNODE_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>

#include "laserprocessing.h"

/**
 * @brief Mission state machine states.
 *
 * Transitions:
 * @code
 *  IDLE ──(service data=true)──► NAVIGATING
 *  NAVIGATING ──(obstacle)──────► STOPPED
 *  NAVIGATING ──(goal reached)──► NAVIGATING (next goal) or COMPLETE
 *  STOPPED ──(obstacle clear)──► NAVIGATING
 *  COMPLETE ──(service data=true)──► NAVIGATING (restart)
 *  Any ──(service data=false)──► IDLE
 * @endcode
 */
enum class MissionState {
    IDLE,        ///< Waiting for service call to start mission
    NAVIGATING,  ///< Driving towards the current active goal
    STOPPED,     ///< Obstacle detected — holding position until clear
    COMPLETE     ///< All goals visited; mission finished
};

/**
 * @brief ROS2 node for the Project 3 Racing Track mission.
 *
 * Subscribes to odometry, laser scan and goals. Publishes throttle, brake,
 * steering, visualisation markers and detected waypoints. Responds to the
 * /orange/mission service (std_srvs/SetBool) to start/stop the mission.
 *
 * ### Architecture
 * - **Callbacks** (laser, odom, goals) run on the ROS executor thread and
 *   update shared state under mutex_.
 * - **Control thread** runs at CONTROL_HZ and implements the state machine.
 *   It is the only writer to all actuator publishers.
 * - **LaserProcessing** library is used for all sensor algorithms; it is
 *   independently testable without the ROS node.
 *
 * ### Parameters
 * | Name            | Type   | Default | Description                        |
 * |-----------------|--------|---------|------------------------------------|
 * | goal_tolerance  | double | 1.5     | Distance to consider goal reached [m] |
 * | road_gradient   | double | 3.0     | Max % gradient (unused in P3 P/C)  |
 * | advanced        | bool   | false   | Enable D/HD laser-waypoint mode    |
 */
class RacingNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the node, sets up all publishers, subscribers,
     *        service, and launches the control thread.
     */
    RacingNode();

    /**
     * @brief Destructor — signals the control thread to stop and joins it.
     */
    ~RacingNode();

private:
    // ── Callbacks ────────────────────────────────────────────────────────────

    /**
     * @brief Stores the latest odometry and forwards it to LaserProcessing.
     * @param msg Incoming odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Stores the latest laser scan and updates LaserProcessing.
     * @param msg Incoming laser scan message.
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Receives the goal list from /orange/goals.
     *
     * Goals are appended to goals_ if the mission is IDLE so that additional
     * goals can be sent at any time before activation. Replaces goals_ if
     * already IDLE and no mission is running.
     *
     * @param msg PoseArray of goal poses.
     */
    void goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * @brief Handles /orange/mission service requests.
     *
     * data=true  → transitions IDLE/COMPLETE → NAVIGATING.
     * data=false → transitions any state → IDLE, stops the car.
     *
     * Response:
     * - success: whether the current active goal is within 0.5 m of road centre.
     * - message: "X% complete (goal N/M)"
     *
     * @param req Service request.
     * @param res Service response.
     */
    void missionService(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    // ── Control thread ────────────────────────────────────────────────────────

    /**
     * @brief Main control loop running at CONTROL_HZ.
     *
     * Reads state_ and dispatches to the appropriate handler:
     * - IDLE: publishes zero commands.
     * - NAVIGATING: checks for obstacles, validates goal corridor, computes
     *   steering, publishes throttle/brake/steering.
     * - STOPPED: publishes brake; resumes when obstacle clears.
     * - COMPLETE: publishes zero commands, logs mission summary.
     */
    void controlLoop();

    // ── Motion helpers ────────────────────────────────────────────────────────

    /**
     * @brief Computes required steering angle towards a goal.
     *
     * Uses a simple proportional heading controller. The heading error is the
     * angle from the current yaw to the bearing towards the goal, normalised
     * to [-π, π]. Steering is clamped to [-MAX_STEER, MAX_STEER].
     *
     * @param odom  Current platform odometry.
     * @param goal  Target goal in world coordinates.
     * @return Steering command in [-MAX_STEER, MAX_STEER].
     */
    double computeSteering(const nav_msgs::msg::Odometry& odom,
                           const geometry_msgs::msg::Point& goal) const;

    /**
     * @brief Publishes a combined stop command (full brake, zero throttle/steer).
     */
    void publishStop();

    /**
     * @brief Publishes actuator commands.
     * @param throttle Throttle in [0, 1].
     * @param brake    Brake torque [Nm].
     * @param steering Steering angle [rad].
     */
    void publishCommand(double throttle, double brake, double steering);

    // ── Visualisation helpers ─────────────────────────────────────────────────

    /**
     * @brief Publishes /visualisation_marker and /orange/waypoints for all
     *        currently known waypoints.
     *
     * Each waypoint is rendered as a CYLINDER (radius 0.2 m, height 0.5 m)
     * in namespace "road". Orientation of each waypoint points toward the
     * next waypoint.
     */
    void publishWaypoints();

    /**
     * @brief Creates a single CYLINDER marker for the "road" namespace.
     * @param pt       World-frame position.
     * @param id       Unique marker ID.
     * @param yaw_rad  Orientation yaw [rad].
     * @return Configured Marker message.
     */
    visualization_msgs::msg::Marker makeWaypointMarker(
        const geometry_msgs::msg::Point& pt,
        int id,
        double yaw_rad,
        const rclcpp::Time& stamp) const;
    /**
     * @brief Extracts yaw from an odometry quaternion.
     * @param odom Odometry message.
     * @return Yaw in [-π, π] [rad].
     */
    static double yawFromOdom(const nav_msgs::msg::Odometry& odom);

    /**
     * @brief Normalises an angle to [-π, π].
     * @param angle Input angle [rad].
     * @return Normalised angle [rad].
     */
    static double normaliseAngle(double angle);

    /**
     * @brief Euclidean distance from odometry position to a 2-D point.
     * @param odom Platform odometry.
     * @param pt   Target point.
     * @return Distance [m].
     */
    static double euclidean(const nav_msgs::msg::Odometry& odom,
                            const geometry_msgs::msg::Point& pt);

private:
    // ── ROS interfaces ────────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        subOdom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    subLaser_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr  subGoals_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr            pubThrottle_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr            pubBrake_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr            pubSteering_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarkers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     pubWaypoints_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr              service_;

    // ── Shared state (protected by mutex_) ───────────────────────────────────
    nav_msgs::msg::Odometry                 odom_;         ///< Latest odometry
    bool                                    hasOdom_;      ///< True once odom received
    std::vector<geometry_msgs::msg::Point>  goals_;        ///< Goal list from /orange/goals
    std::size_t                             currentGoal_;  ///< Index of active goal
    MissionState                            state_;        ///< Current state machine state
    std::vector<geometry_msgs::msg::Point>  waypoints_;    ///< Accumulated road waypoints

    mutable std::mutex mutex_; ///< Guards all shared state above

    // ── LaserProcessing library ───────────────────────────────────────────────
    std::unique_ptr<LaserProcessing> laserProc_; ///< Laser algorithm object

    // ── Control thread ────────────────────────────────────────────────────────
    std::thread        controlThread_; ///< Background control loop thread
    std::atomic<bool>  running_;       ///< Set false to terminate controlThread_

    // ── Parameters ────────────────────────────────────────────────────────────
    double goalTolerance_; ///< Distance threshold for goal reached [m]
    bool   advanced_;      ///< D/HD mode: generate waypoints from laser

    // ── Control constants ─────────────────────────────────────────────────────
    static constexpr double CONTROL_HZ         = 20.0;    ///< Control loop rate [Hz]
    static constexpr double MAX_THROTTLE       = 0.6;     ///< Maximum throttle [0–1]
    static constexpr double CRUISE_THROTTLE    = 0.2;    ///< Steady-state throttle [0–1]
    static constexpr double MAX_BRAKE          = 8000.0;  ///< Full brake torque [Nm]
    static constexpr double MAX_STEER          = 0.6;     ///< Maximum steering angle [rad]
    static constexpr double STEER_GAIN         = 0.8;     ///< Proportional steering gain
    static constexpr double SLOW_ZONE_M        = 10.0;     ///< Distance at which throttle begins tapering [m]
    static constexpr double WAYPOINT_SPACING_M = 3.0;     ///< Minimum spacing between stored waypoints [m]
    static constexpr double CENTRE_CHECK_M     = 0.5;     ///< Radius for service "goal near centre" check [m]
};

#endif // RACINGNODE_H
