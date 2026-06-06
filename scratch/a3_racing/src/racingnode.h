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

/**
 * @brief State machine states for the racing mission.
 */
enum class MissionState {
    IDLE,       //!< Stopped, waiting for mission start
    CRUISING,   //!< Driving forward on a straight / gentle curve -- full throttle
    TURNING,    //!< Sharp corner -- corner braking + feathered throttle
    COMPLETE    //!< All goals reached or mission aborted
};

/**
 * @brief ROS2 node that controls the Ackerman (Audi R8) platform around a
 *        walled racing track.
 *
 * ### Architecture
 * - Three ROS callbacks (odom, laser, goals) update shared state under mutex_.
 * - A dedicated control thread (controlLoop()) runs at CONTROL_HZ and drives
 *   the state machine: IDLE → CRUISING ↔ TURNING → COMPLETE.
 * - LaserProcessing is constructed on first laser scan and accessed from both
 *   the callback thread and the control thread via its own internal mutex.
 *
 * ### P/C behaviour
 * - Starts/stops on /orange/mission service call.
 * - Validates each goal against the laser-detected corridor (goalInCorridor).
 * - Detects non-wall obstacles and halts the mission.
 * - Publishes steering, throttle, brake commands; waypoints; markers.
 *
 * ### D/HD behaviour (advanced:=true)
 * - Each control tick calls generateAdvancedWaypoints() to derive a centre-track
 *   point from the laser, spacing them every WAYPOINT_SPACING_M.
 * - Laser-derived waypoints are appended to the goal list so the car is guided
 *   precisely even on sections between the provided goals.
 *
 * ### How to run
 * @code
 * ros2 run a3_racing racing_node
 * ros2 run a3_racing racing_node --ros-args -p advanced:=true
 * @endcode
 */
class RacingNode : public rclcpp::Node
{
public:
    /** @brief Constructs the node, declares parameters, creates all publishers,
     *         subscribers, and the service, then launches the control thread. */
    RacingNode();

    /** @brief Stops the control thread and joins it before destruction. */
    ~RacingNode();
private:
    /** @brief Stores latest odometry and forwards it to LaserProcessing. */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /** @brief Stores latest scan; constructs LaserProcessing on first call. */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Receives the goal list from /orange/goals.
     *
     * Ignored while a mission is active.  Resets currentGoal_ to 0.
     */
    void goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * @brief Handles /orange/mission service calls.
     *
     * @param req  data=true starts the mission; data=false stops it.
     * @param res  success=whether current goal is within corridor;
     *             message=percentage completion string.
     */
    void missionService(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    /**
     * @brief Main control loop running at CONTROL_HZ in a dedicated thread.
     *
     * Each tick: snapshots shared state, runs the state machine switch,
     * calls publishCommand() or publishStop(), and sleeps for the remainder
     * of the period.
     */
    void controlLoop();

    /** @brief Publishes zero throttle and MAX_BRAKE to halt the car. */
    void publishStop();

    /**
     * @brief Publishes throttle, brake, and steering commands.
     * @param throttle Normalised throttle [0, 1].
     * @param brake    Brake force [0, MAX_BRAKE].
     * @param steering Steering angle [−MAX_STEER, MAX_STEER] rad.
     */
    void publishCommand(double throttle, double brake, double steering);

    /**
     * @brief Publishes the accumulated waypoints as MarkerArray and PoseArray.
     *
     * Each marker uses namespace "road", CYLINDER type, radius 0.2 m, height 0.5 m.
     * Orientation of each pose points toward the next waypoint.
     */
    void publishWaypoints();

    /**
     * @brief Generates laser-derived centre-track waypoints ahead of the car.
     *
     * Called each control tick when advanced_=true. Uses trackCentreAhead() to
     * compute the midpoint between the two wall segments and, if the result is
     * at least WAYPOINT_SPACING_M ahead of the last stored advanced waypoint,
     * appends it to advancedGoals_. These are then merged with the goals from
     * /orange/goals so the car has fine-grained steering targets on sections
     * not covered by the provided goal list.
     *
     * @param laserProc  Shared pointer to the active LaserProcessing instance.
     * @param odom       Current platform odometry snapshot.
     */
    void generateAdvancedWaypoints(
        const std::shared_ptr<LaserProcessing>& laserProc,
        const nav_msgs::msg::Odometry& odom);

    /**
     * @brief Computes the heading error (alpha) from the car to a target point.
     * @param odom   Current odometry (provides position and yaw).
     * @param target Target point in world coordinates.
     * @return Signed angle in [-pi, pi] from the car's heading to the target.
     */
    double computeAlpha(const nav_msgs::msg::Odometry& odom,
                        const geometry_msgs::msg::Point& target) const;

    /**
     * @brief Extracts yaw from an odometry quaternion.
     * @param odom Odometry message.
     * @return Yaw in [-pi, pi] radians.
     */
    static double yawFromOdom(const nav_msgs::msg::Odometry& odom);

    /**
     * @brief Euclidean distance from the odometry position to a point.
     * @param odom Odometry providing current position.
     * @param pt   Target point.
     * @return Distance in metres.
     */
    static double euclidean(const nav_msgs::msg::Odometry& odom,
                            const geometry_msgs::msg::Point& pt);

    /**
     * @brief Returns a human-readable string for a MissionState value.
     * @param s State to stringify.
     * @return C-string label.
     */
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
    std::vector<geometry_msgs::msg::Point> goals_;          //!< Goals from /orange/goals
    std::vector<geometry_msgs::msg::Point> advancedGoals_;  //!< Laser-derived waypoints (advanced mode)
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
    static constexpr double      MAX_BRAKE             = 8000.0;
    static constexpr double      MAX_STEER             = 10.0;
    static constexpr double      WHEELBASE_M           = 2.65;
    static constexpr double      V_MAX                 = 12.0;
    static constexpr double      WAYPOINT_SPACING_M    = 3.0;  //!< Min spacing between laser-derived waypoints [m]
    static constexpr double      CORNER_ALPHA_RAD      = 0.4;  //!< Threshold to enter TURNING [rad] (~23 deg)
    static constexpr double      MIN_SPEED_FACTOR      = 0.3;  //!< Throttle floor in CRUISING
    static constexpr double      MIN_SPEED_FACTOR_TURN = 0.5;  //!< Throttle floor in TURNING -- higher so
                                                                //!< the car retains momentum to drive back
                                                                //!< out to the arc when it has cut inside

    // Constants -- CRUISING state
    static constexpr double      CRUISE_THROTTLE         = 0.5;
    static constexpr double      STEER_K                 = 1.2;  //!< Nonlinear steering gain
    static constexpr double      STEER_KD                = 0.5;  //!< Derivative damping gain
    static constexpr double      CRUISE_LOOKAHEAD_DIST_M = 15.0; //!< Steer-target distance on straights [m]

    // Constants -- TURNING state
    static constexpr double      CORNER_BRAKE           = 8000.0;
    static constexpr double      TURN_THROTTLE          = 0.3;   //!< Feathered throttle mid-corner
    static constexpr double      TURN_V_MAX             = 7.0;   //!< Hard speed cap in TURNING [m/s]
    static constexpr double      TURN_STEER_K           = 10.0;  //!< Tighter steering gain when corner imminent
    static constexpr double      TURN_STEER_KD          = 0.1;   //!< Derivative damping when corner imminent
    static constexpr double      BRAKE_PREVIEW_DIST_M   = 6.0;   //!< Pre-corner braking distance [m]

    // Lookahead -- corner preview (fixed small index, used only for
    // state-transition detection, not for the steer target itself)
    static constexpr std::size_t PREVIEW_GOAL_LOOKAHEAD = 3;     //!< Goals ahead for corner-detection peek

    // Lookahead -- steer target (distance-based, scales with turn severity)
    static constexpr double      TURN_LD_MAX            = 8.0;  //!< Steer-target dist at corner entry [m]
    static constexpr double      TURN_LD_MIN            = 1.0;  //!< Steer-target dist at full U-turn [m]
    //!< Between these limits the lookahead distance is interpolated linearly
    //!< with |alphaPreview| / pi, so a 180-deg U-turn gets TURN_LD_MIN and a
    //!< 23-deg entry bend gets TURN_LD_MAX.  This keeps the steer target
    //!< pointing outward (back toward the arc) when the car has cut inside,
    //!< rather than tangentially along the inside of the curve.
};
#endif // RACINGNODE_H