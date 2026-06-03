#ifndef RACING_TRACK_NODE_H
#define RACING_TRACK_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "racing_track_pkg/track_analyser.h"

/**
 * @brief Mission states for the racing track node.
 *
 * Transitions are driven by the control thread.  The service callback and ROS
 * subscriber callbacks only write sensor data and trigger atomic state changes
 * — they never block or loop.  This ensures the control thread can always be
 * terminated cleanly (no hanging in loops, as required by the assessment criteria).
 *
 * @dot
 * digraph MissionState {
 *   IDLE -> ACQUIRING  [label="service data=true"]
 *   ACQUIRING -> NAVIGATING [label="goal validated"]
 *   NAVIGATING -> ACQUIRING [label="goal reached, goals remain"]
 *   NAVIGATING -> OBSTACLE_STOP [label="obstacle ahead"]
 *   NAVIGATING -> COMPLETE [label="all goals reached"]
 *   OBSTACLE_STOP -> NAVIGATING [label="path clear"]
 *   COMPLETE -> IDLE
 *   NAVIGATING -> IDLE [label="service data=false", style=dashed]
 *   OBSTACLE_STOP -> IDLE [label="service data=false", style=dashed]
 * }
 * @enddot
 */
enum class MissionState : int
{
    IDLE          = 0, //!< Waiting for service activation
    ACQUIRING     = 1, //!< Reading laser, validating/computing next goal
    NAVIGATING    = 2, //!< Publishing steering/throttle/brake at CONTROL_HZ
    OBSTACLE_STOP = 3, //!< Full brake; waiting for path to clear
    COMPLETE      = 4  //!< All goals reached; publishing final waypoints then → IDLE
};

/**
 * @brief ROS 2 node for the A3 Project 3 racing track mission.
 *
 * ### Architecture
 * Two threads run concurrently:
 *  - **ROS executor thread** (rclcpp::spin): handles all subscriber callbacks
 *    and service callbacks.  Callbacks only copy data under mutex_ and return
 *    immediately — they never block.
 *  - **Control thread** (controlLoop()): runs the state machine at 20 Hz,
 *    reads sensor data under mutex_, calls TrackAnalyser functions, and
 *    publishes commands.
 *
 * ### ROS interfaces
 *  Subscribe:  /orange/odom, /orange/laserscan, /orange/goals
 *  Publish:    /orange/brake_cmd, /orange/steering_cmd, /orange/throttle_cmd,
 *              /orange/waypoints, /visualisation_marker
 *  Service:    /orange/mission  (std_srvs/srv/SetBool)
 *
 * ### Parameters
 *  - track_width     (double, default 8.0)  — nominal track width [m]
 *  - goal_tolerance  (double, default 0.2)  — goal-in-corridor tolerance [m]
 *  - obstacle_range  (double, default 6.0)  — forward obstacle detection range [m]
 *  - goal_tolerance_reached (double, default 0.5) — distance to declare goal reached [m]
 *  - advanced        (bool,   default false) — if true, use laser-derived waypoints (D/HD)
 */
class RacingTrackNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the node, declares parameters, creates all pub/sub/service,
     *        and starts the control thread.
     */
    RacingTrackNode();

    /**
     * @brief Destructor — signals the control thread to stop and joins it.
     */
    ~RacingTrackNode();

private:
    // ── ROS subscriber callbacks ──────────────────────────────────────────────

    /**
     * @brief Stores the latest odometry message.  Non-blocking, protected by mutex_.
     */
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Stores the latest laser scan message.  Non-blocking, protected by mutex_.
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Appends new goals to goals_.  Protected by mutex_.
     *
     * Goals can be sent at any time, including while a mission is running —
     * new goals are appended to the current list per the assignment FAQ.
     */
    void goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    // ── Service callback ──────────────────────────────────────────────────────

    /**
     * @brief Starts or stops the mission in response to /orange/mission.
     *
     * data=true:  transitions state_ to ACQUIRING (begins mission).
     * data=false: transitions state_ to IDLE (stops mission).
     *
     * Response:
     *  - success: true if the current active goal is within 0.5 m of the
     *             detected corridor centre.
     *  - message: "% completion: N%"
     */
    void missionService(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // ── Control thread ────────────────────────────────────────────────────────

    /**
     * @brief Main control loop.  Runs at CONTROL_PERIOD_MS until controlRunning_ is false.
     *
     * Implements the MissionState machine:
     *  IDLE:          publish stop commands.
     *  ACQUIRING:     copy sensor data, run analyseCorridor(), validate/select next goal.
     *  NAVIGATING:    compute steering, publish commands, check for goal reached or obstacle.
     *  OBSTACLE_STOP: publish stop commands, re-check for obstacle.
     *  COMPLETE:      publish final waypoints and markers, transition to IDLE.
     */
    void controlLoop();

    // ── Publishers ────────────────────────────────────────────────────────────

    /** @brief Publishes brake=MAX, throttle=0, steering=0. */
    void publishStop();

    /**
     * @brief Publishes brake, throttle, and steering commands.
     * @param brake    Brake torque [0–8000 Nm].
     * @param throttle Throttle [0–1].
     * @param steering Steering angle [-1–1].
     */
    void publishControl(double brake, double throttle, double steering);

    /**
     * @brief Publishes the current waypoint list as a PoseArray and MarkerArray.
     *
     * Each waypoint is published as a CYLINDER marker with namespace "road",
     * radius 0.2 m, height 0.5 m, as specified.
     */
    void publishWaypoints();

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * @brief Returns the percentage of goals completed [0–100].
     * @return (goalsCompleted_ / goals_.size()) * 100, capped at 100.
     */
    unsigned int completionPercent() const;

    /**
     * @brief Advances goalIndex_ to the next goal, or transitions to COMPLETE.
     */
    void advanceGoal();

    // ── Publishers / subscribers / service ───────────────────────────────────

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odomSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr        brakePub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr        steeringPub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr        throttlePub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypointsPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr missionSrv_;

    // ── Shared state (protected by mutex_) ────────────────────────────────────

    mutable std::mutex mutex_;

    nav_msgs::msg::Odometry       latestOdom_;   //!< Most recent odometry message
    sensor_msgs::msg::LaserScan   latestScan_;   //!< Most recent laser scan
    bool                          odomReady_ = false;  //!< True once first odom received
    bool                          scanReady_ = false;  //!< True once first scan received

    std::vector<geometry_msgs::msg::Pose> goals_;      //!< All received goals
    std::size_t                           goalIndex_   = 0; //!< Index of current active goal
    std::size_t                           goalsCompleted_ = 0; //!< How many goals have been reached

    std::vector<Point2D> waypoints_; //!< Current accumulated waypoints for publication

    // ── Atomic state — readable without mutex ─────────────────────────────────

    std::atomic<MissionState> state_{MissionState::IDLE};
    std::atomic<bool>         controlRunning_{false};

    // ── Control thread ────────────────────────────────────────────────────────

    std::thread controlThread_;

    // ── Algorithm library ─────────────────────────────────────────────────────

    TrackAnalyser analyser_; //!< Pure-C++ track analysis library

    // ── Parameters (set from ROS parameter server at construction) ────────────

    double goalReachedTolerance_; //!< Distance to declare goal reached [m]
    bool   advanced_;             //!< D/HD: use laser-derived waypoints

    // ── Control constants ─────────────────────────────────────────────────────

    static constexpr int    CONTROL_PERIOD_MS = 50;      //!< Control loop period [ms]
    static constexpr double MAX_BRAKE         = 8000.0;  //!< Full brake torque [Nm]
    static constexpr double MAX_THROTTLE      = 0.4;     //!< Maximum throttle [0–1]
    static constexpr double COAST_THROTTLE    = 0.2;     //!< Reduced throttle near goal [0–1]
    static constexpr double COAST_DISTANCE    = 5.0;     //!< Distance at which throttle reduces [m]
};

#endif // RACING_TRACK_NODE_H
