#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <math.h>
#include <mutex>
#include <vector>
#include <optional>

/**
 * @brief Processes 180 deg laser scans for the Ackerman (Audi R8) racing platform.
 *
 * The laser is mounted 3.725 m ahead of the reported odometry origin.
 * All output points are expressed in the **world frame** when an odometry
 * pose is supplied; otherwise they are in the laser frame.
 *
 * ### Key responsibilities
 * - Detect whether an obstacle (non-wall object) blocks the forward corridor.
 * - Validate whether a goal lies within the free corridor between the two wall segments.
 * - (D/HD) Compute waypoints equidistant between the two detected wall segments.
 */
class LaserProcessing
{
public:
    /**
     * @brief Constructs a LaserProcessing object with an initial scan.
     * @param laserScan First laser scan message.
     */
    explicit LaserProcessing(sensor_msgs::msg::LaserScan laserScan);

    /**
     * @brief Replaces the stored scan with a new one. Thread-safe.
     * @param laserScan New laser scan.
     */
    void newScan(sensor_msgs::msg::LaserScan laserScan);

    /**
     * @brief Updates the current platform odometry. Thread-safe.
     * @param odom Latest odometry message.
     */
    void newOdom(nav_msgs::msg::Odometry odom);

    /**
     * @brief Detects whether a non-wall obstacle blocks the forward arc.
     *
     * Examines rays within a forward cone (?OBSTACLE_HALF_ANGLE_DEG of the
     * scan centre). A reading is flagged as an obstacle when its range is
     * shorter than WALL_MIN_RANGE_M, indicating an object closer than the
     * typical wall distance.
     *
     * Both positive (obstacle present) and negative (clear) results are
     * meaningful -- unit tests must cover both cases.
     *
     * @return true if a non-wall obstacle is detected in the forward corridor.
     */
    bool obstacleInFront() const;

    /**
     * @brief Checks whether a goal lies inside the free corridor between walls.
     *
     * Transforms @p goal into the laser frame, identifies the left and right
     * wall segment midpoints in the forward direction, and tests whether the
     * goal's lateral position is between them (within CORRIDOR_TOLERANCE_M).
     *
     * @param goal         Goal position in world coordinates.
     * @return true if the goal is within the observed free corridor.
     */
    bool goalInCorridor(const geometry_msgs::msg::Point& goal) const;

    /**
     * @brief Computes the track centre point directly ahead of the laser.
     *
     * Finds the median left-wall and right-wall range in the forward sector
     * and returns their midpoint in **world coordinates**. Returns an empty
     * optional when fewer than MIN_WALL_READINGS_PER_SIDE valid readings are
     * found on either side.
     *
     * @return World-frame centre point, or std::nullopt if walls cannot be detected.
     */
    std::optional<geometry_msgs::msg::Point> trackCentreAhead() const;

    /**
     * @brief Counts the number of distinct laser segments (continuous valid readings).
     * @return Segment count.
     */
    unsigned int countSegments() const;

private:
    /**
     * @brief Converts a scan ray index to a Cartesian point in the **laser frame**.
     * @param index Ray index.
     * @return 2-D Cartesian point (z = 0).
     */
    geometry_msgs::msg::Point polarToCart(unsigned int index) const;

    /**
     * @brief Transforms a laser-frame point into the world frame using odom_.
     *
     * Accounts for the 3.725 m forward offset of the laser from the reported
     * odometry origin.
     *
     * @param laserPt Point in laser frame.
     * @return Corresponding world-frame point.
     */
    geometry_msgs::msg::Point laserToWorld(const geometry_msgs::msg::Point& laserPt) const;

    /**
     * @brief Returns the yaw extracted from an odometry quaternion [rad].
     * @param odom Odometry message.
     * @return Yaw angle in [-pi, pi].
     */
    static double yawFromOdom(const nav_msgs::msg::Odometry& odom);

private:
    sensor_msgs::msg::LaserScan laserScan_; //!< Most recent laser scan
    nav_msgs::msg::Odometry     odom_;      //!< Most recent platform odometry
    bool                        hasOdom_;   //!< True once at least one odom message received
    mutable std::mutex          mtx_;       //!< Protects laserScan_ and odom_

    // ?? Tuning constants ?????????????????????????????????????????????????????
    static constexpr double LASER_OFFSET_M           = 3.725; //!< Laser forward offset from odom origin [m]
    static constexpr double OBSTACLE_HALF_ANGLE_DEG  = 15.0;  //!< Half-width of forward obstacle-check cone [deg]
    static constexpr double OBSTACLE_WALL_FRACTION   = 0.55;  //!< Obstacle threshold as fraction of estimated wall range
    static constexpr double OBSTACLE_MIN_RANGE_M      = 1.0;   //!< Absolute minimum obstacle detection range [m]
    static constexpr double CORRIDOR_TOLERANCE_M     = 0.2;   //!< Allowed lateral deviation from corridor centre [m]
    static constexpr unsigned int MIN_WALL_READINGS_PER_SIDE = 3;  //!< Minimum valid rays needed per wall side
    static constexpr unsigned int MIN_OBSTACLE_RAYS          = 3;  //!< Min consecutive close rays to confirm obstacle
};

#endif // LASERPROCESSING_H