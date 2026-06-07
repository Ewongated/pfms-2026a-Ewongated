/**
 * @file laserprocessing.h
 * @brief LaserProcessing class — laser scan analysis for the A3 Racing Track project.
 */
#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <mutex>
#include <vector>
#include <optional>

#include "geometry_utils.h"

/**
 * @brief Analyses 180° laser scans for the Ackerman (Audi R8) racing platform.
 *
 * The laser is mounted 3.725 m ahead of the reported odometry origin.
 * Output points are expressed in the world frame when odometry is available.
 *
 * Responsibilities:
 * - Detect non-wall obstacles blocking the forward corridor.
 * - Validate whether a goal lies within the free corridor between the walls.
 * - (D/HD) Compute the track centre point ahead for laser-derived waypoints.
 *
 * Thread safety: newScan() and newOdom() may be called from any thread;
 * all public methods snapshot shared state under an internal mutex before
 * processing.
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
     * @brief Updates the stored odometry. Thread-safe.
     * @param odom Latest odometry message.
     */
    void newOdom(nav_msgs::msg::Odometry odom);

    /**
     * @brief Returns true if a non-wall obstacle blocks the forward arc.
     *
     * Examines rays within ±OBSTACLE_HALF_ANGLE_DEG of the scan centre.
     * A return shorter than OBSTACLE_MAX_RANGE_M (= half track width minus
     * tolerance) cannot be a wall. A cluster of MIN_OBSTACLE_RAYS consecutive
     * close returns is required to reject single noisy readings.
     *
     * @return true if an obstacle is detected.
     */
    bool obstacleInFront() const;

    /**
     * @brief Returns true if @p goal lies inside the free corridor between walls.
     *
     * Transforms the goal into the car body frame, collects forward-facing
     * wall readings from the laser, and tests whether the goal's lateral
     * position falls between the median left and right wall positions
     * (within CORRIDOR_TOLERANCE_M). Falls back to track-width bounds when
     * fewer than MIN_WALL_READINGS_PER_SIDE valid readings exist on either side.
     *
     * @param goal Goal position in world coordinates.
     * @return true if the goal is within the observed corridor.
     */
    bool goalInCorridor(const geometry_msgs::msg::Point& goal) const;

    /**
     * @brief Returns the track centre point ahead in world coordinates.
     *
     * Computes the median lateral position of left and right wall readings
     * in the forward sector and places the centre point 5 m ahead in the
     * laser frame, then transforms to the world frame.
     *
     * @return World-frame centre point, or std::nullopt when walls cannot
     *         be detected (fewer than MIN_WALL_READINGS_PER_SIDE on either side).
     */
    std::optional<geometry_msgs::msg::Point> trackCentreAhead() const;

    /**
     * @brief Counts the number of distinct continuous segments in the scan.
     * @return Segment count.
     */
    unsigned int countSegments() const;

private:
    /// @brief Converts ray index to a 2-D Cartesian point in the laser frame.
    static geometry_msgs::msg::Point polarToCart(
        const sensor_msgs::msg::LaserScan& scan, unsigned int index);

    /**
     * @brief Transforms a laser-frame point to the world frame.
     *
     * Accounts for the LASER_OFFSET_M forward offset of the laser.
     * @param laserPt Point in laser frame.
     * @param odom    Odometry snapshot.
     * @return Corresponding world-frame point.
     */
    static geometry_msgs::msg::Point laserToWorld(
        const geometry_msgs::msg::Point& laserPt,
        const nav_msgs::msg::Odometry&   odom);

    /// @brief Returns the median of a non-empty vector, sorting it in place.
    static double median(std::vector<double>& v);

    /**
     * @brief Collects lateral (Y) positions of forward wall readings from a scan.
     *
     * Shared by goalInCorridor() and trackCentreAhead() to avoid code duplication.
     * Populates @p leftY (positive Y) and @p rightY (negative Y).
     *
     * @param scan   Scan snapshot to read.
     * @param leftY  Output: lateral positions of left-wall returns.
     * @param rightY Output: lateral positions of right-wall returns.
     */
    static void collectWallReadings(const sensor_msgs::msg::LaserScan& scan,
                                    std::vector<double>& leftY,
                                    std::vector<double>& rightY);

    sensor_msgs::msg::LaserScan laserScan_; //!< Most recent laser scan
    nav_msgs::msg::Odometry     odom_;      //!< Most recent odometry
    bool                        hasOdom_;   //!< True once at least one odom has been received
    mutable std::mutex          mtx_;       //!< Guards laserScan_ and odom_

    // Physical constants (from assignment spec)
    static constexpr double LASER_OFFSET_M          = 3.725; //!< Laser is 3.725 m ahead of odom origin
    static constexpr double HALF_TRACK_WIDTH_M      = 4.0;   //!< Half the nominal ~8 m track width
    static constexpr double TRACK_TOLERANCE_M       = 0.5;   //!< Width tolerance from spec

    // Derived / tuning constants
    static constexpr double OBSTACLE_HALF_ANGLE_DEG = 8.0;
    //!< Obstacle max range: anything closer than this in the forward cone is not a wall
    static constexpr double OBSTACLE_MAX_RANGE_M    = HALF_TRACK_WIDTH_M - TRACK_TOLERANCE_M;
    static constexpr double CORRIDOR_TOLERANCE_M    = 0.2;   //!< Lateral tolerance for goalInCorridor

    static constexpr unsigned int MIN_WALL_READINGS_PER_SIDE = 3; //!< Minimum readings per side to trust wall estimate
    static constexpr unsigned int MIN_OBSTACLE_RAYS          = 3; //!< Consecutive close rays required to confirm obstacle
};

#endif // LASERPROCESSING_H
