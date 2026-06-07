/**
 * @file racingnode.h
 * @brief RacingNode class — ROS2 control node for the A3 Racing Track project.
 */
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
#include "geometry_utils.h"

/**
 * @brief State machine states for the racing mission.
 */
enum class MissionState {
    IDLE,     //!< Waiting for mission start via service call
    CRUISING, //!< Straight / gentle curve — full throttle
    TURNING,  //!< Sharp corner — reduced speed, tighter steering gain
    COMPLETE  //!< All goals reached or mission aborted
};

/**
 * @brief ROS2 node controlling the Ackerman (Audi R8) around a walled racing track.
 *
 * ### Architecture
 * Three ROS callbacks (odom, laser, goals) update shared state under `mutex_`.
 * A dedicated control thread runs at CONTROL_HZ and drives the state machine:
 * IDLE → CRUISING ↔ TURNING → COMPLETE.
 * LaserProcessing is constructed on the first laser scan and is accessed from
 * both threads via its own internal mutex.
 *
 * ### P/C behaviour
 * - Starts/stops on `/orange/mission` service call.
 * - Validates each goal against the laser-detected corridor.
 * - Stops immediately on non-wall obstacle detection.
 * - Publishes steering, throttle, brake commands and waypoint markers.
 *
 * ### D/HD behaviour (`advanced:=true`)
 * - Each control tick derives a track centre point from the laser and appends
 *   it to `advancedGoals_` when it is at least WAYPOINT_SPACING_M from the last.
 * - Advanced waypoints are merged with the provided goal list at each tick.
 *
 * ### How to run
 * @code
 * ros2 run a3_racing racing_node
 * ros2 run a3_racing racing_node --ros-args -p goal_tolerance:=1.5
 * ros2 run a3_racing racing_node --ros-args -p advanced:=true
 * @endcode
 */
class RacingNode : public rclcpp::Node
{
public:
    /** @brief Constructs the node, declares parameters, wires up all
     *         publishers/subscribers/service, and starts the control thread. */
    RacingNode();

    /** @brief Signals the control thread to stop and joins it. */
    ~RacingNode();

private:
    // ── Callbacks ────────────────────────────────────────────────────────────

    /** @brief Stores latest odometry and forwards it to LaserProcessing. */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /** @brief Stores latest scan; constructs LaserProcessing on first call. */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Stores the goal list from `/orange/goals`.
     * Ignored while a mission is active; resets `currentGoal_` to 0.
     */
    void goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * @brief Handles `/orange/mission` service calls.
     * @param req `data=true` starts, `data=false` stops the mission.
     * @param res `success` = current goal in corridor; `message` = % complete.
     */
    void missionService(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    // ── Control loop ─────────────────────────────────────────────────────────

    /**
     * @brief Main control loop running at CONTROL_HZ in a dedicated thread.
     *
     * Each tick snapshots shared state, evaluates the state machine, issues
     * drive commands via publishCommand() or publishStop(), and sleeps for
     * the remainder of the period.
     */
    void controlLoop();

    /** @brief Publishes zero throttle and full brake to stop the car. */
    void publishStop();

    /**
     * @brief Publishes throttle, brake, and steering commands.
     * @param throttle Normalised throttle [0, 1].
     * @param brake    Brake force [0, MAX_BRAKE].
     * @param steering Steering angle [-MAX_STEER, MAX_STEER] rad.
     */
    void publishCommand(double throttle, double brake, double steering);

    /**
     * @brief Publishes accumulated waypoints as a MarkerArray and PoseArray.
     *
     * Markers use namespace "road", CYLINDER type, 0.2 m radius (scale 0.4),
     * 0.5 m height. Each pose orientation points toward the next waypoint.
     */
    void publishWaypoints();

    /**
     * @brief Appends a laser-derived centre-track waypoint when far enough from the last.
     *
     * Called each tick in advanced mode. Uses trackCentreAhead() and only
     * appends when the new point is at least WAYPOINT_SPACING_M from the
     * last stored advanced waypoint.
     *
     * @param laserProc Active LaserProcessing instance.
     * @param odom      Current odometry snapshot.
     */
    void generateAdvancedWaypoints(const std::shared_ptr<LaserProcessing>& laserProc,
                                   const nav_msgs::msg::Odometry& odom);

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * @brief Signed heading error from the car to a target point.
     * @param odom   Current odometry.
     * @param target Target in world coordinates.
     * @return Angle in [-pi, pi] rad.
     */
    double computeAlpha(const nav_msgs::msg::Odometry& odom,
                        const geometry_msgs::msg::Point& target) const;

    /// @brief Human-readable label for a MissionState value.
    static const char* stateName(MissionState s);

    // ── ROS interfaces ────────────────────────────────────────────────────────

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       subOdom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   subLaser_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subGoals_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               pubThrottle_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               pubBrake_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               pubSteering_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarkers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr        pubWaypoints_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                 service_;

    // ── Shared state (protected by mutex_) ────────────────────────────────────

    std::shared_ptr<LaserProcessing>       laserProc_;
    nav_msgs::msg::Odometry                odom_;
    bool                                   hasOdom_;
    sensor_msgs::msg::LaserScan            laser_;
    bool                                   hasLaser_;
    std::vector<geometry_msgs::msg::Point> goals_;         //!< Goals from /orange/goals
    std::vector<geometry_msgs::msg::Point> advancedGoals_; //!< Laser-derived waypoints (advanced mode)
    std::vector<geometry_msgs::msg::Point> waypoints_;     //!< Reached waypoints for publishing
    std::size_t                            currentGoal_;
    MissionState                           state_;
    mutable std::mutex                     mutex_;

    // ── Control thread ────────────────────────────────────────────────────────

    std::thread       controlThread_;
    std::atomic<bool> running_;

    // ── Parameters ────────────────────────────────────────────────────────────

    double goalTolerance_; //!< Distance to consider a goal reached [m] (default 1.5)
    bool   advanced_;      //!< Enable D/HD laser-derived waypoint mode

    double prevAlpha_; //!< Previous heading error for derivative steering damping

    // ── Tuning constants ─────────────────────────────────────────────────────
    // All physical / tuning values are named constants; none are magic numbers.

    static constexpr double CONTROL_HZ            = 20.0;    //!< Control loop rate [Hz]
    static constexpr double MAX_BRAKE             = 8000.0;  //!< Maximum brake force
    static constexpr double MAX_STEER             = 10.0;    //!< Maximum steering angle [rad]
    static constexpr double WHEELBASE_M           = 2.65;    //!< Audi R8 wheelbase [m]
    static constexpr double V_MAX                 = 12.0;    //!< Top speed in CRUISING [m/s]
    static constexpr double WAYPOINT_SPACING_M    = 3.0;     //!< Min spacing between laser-derived waypoints [m]
    static constexpr double CORNER_ALPHA_RAD      = 0.4;     //!< Heading error threshold to enter TURNING (~23 deg)
    static constexpr double MIN_SPEED_FACTOR      = 0.3;     //!< Throttle floor in CRUISING
    static constexpr double MIN_SPEED_FACTOR_TURN = 0.5;     //!< Higher floor in TURNING to retain arc momentum

    static constexpr double CRUISE_THROTTLE         = 0.5;
    static constexpr double STEER_K                 = 1.2;   //!< Nonlinear steering gain (CRUISING)
    static constexpr double STEER_KD                = 0.5;   //!< Derivative damping (CRUISING)
    static constexpr double CRUISE_LOOKAHEAD_DIST_M = 15.0;  //!< Steer-target distance on straights [m]

    static constexpr double      CORNER_BRAKE         = 8000.0;
    static constexpr double      TURN_THROTTLE        = 0.3;  //!< Feathered throttle mid-corner
    static constexpr double      TURN_V_MAX           = 7.0;  //!< Hard speed cap in TURNING [m/s]
    static constexpr double      TURN_STEER_K         = 10.0; //!< Tighter steering gain at corners
    static constexpr double      TURN_STEER_KD        = 0.1;  //!< Derivative damping at corners
    static constexpr double      BRAKE_PREVIEW_DIST_M = 6.0;  //!< Distance at which pre-corner braking begins [m]

    static constexpr std::size_t PREVIEW_GOAL_LOOKAHEAD = 3;  //!< Goals ahead used for corner-detection peek
    static constexpr double      TURN_LD_MAX            = 8.0; //!< Steer lookahead at gentle corner entry [m]
    static constexpr double      TURN_LD_MIN            = 1.0; //!< Steer lookahead at sharp U-turn [m]
    //!< Lookahead interpolates linearly between TURN_LD_MIN and TURN_LD_MAX
    //!< with |alphaPreview| / pi, tightening the steer target as the bend sharpens.
};

#endif // RACINGNODE_H
