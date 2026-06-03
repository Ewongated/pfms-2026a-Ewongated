#ifndef TRACK_ANALYSER_H
#define TRACK_ANALYSER_H

#include <vector>
#include <cmath>

/**
 * @brief Point in 2-D world frame.
 */
struct Point2D
{
    double x = 0.0; //!< World-frame x [m]
    double y = 0.0; //!< World-frame y [m]
};

/**
 * @brief Result of a single corridor analysis pass.
 *
 * Populated by TrackAnalyser::analyseCorridor() and consumed by the ROS node's
 * control loop.  All coordinates are in world frame.
 */
struct CorridorResult
{
    bool   valid       = false; //!< true if both walls were detected
    double centreX     = 0.0;  //!< World-frame x of computed track centre [m]
    double centreY     = 0.0;  //!< World-frame y of computed track centre [m]
    double dLeft       = 0.0;  //!< Lateral distance to left wall from sensor [m]
    double dRight      = 0.0;  //!< Lateral distance to right wall from sensor [m]
    double trackWidth  = 0.0;  //!< Estimated full track width dLeft + dRight [m]
};

/**
 * @brief Pure library for racing-track laser analysis.
 *
 * Contains no ROS types — all inputs are plain C++ (raw range vectors,
 * sensor pose scalars) and all outputs are plain structs.  This allows the
 * library to be unit-tested with recorded data without a live ROS graph.
 *
 * ### Algorithm overview
 *  1. toWorldPoints()   — convert raw ranges → world-frame hit points.
 *  2. analyseCorridor() — split hits into left/right walls, compute the median
 *                         lateral distance to each wall, return corridor centre.
 *  3. isGoalInCorridor() — check a PoseArray goal against the corridor.
 *  4. obstacleAhead()   — scan the forward cone for non-wall returns.
 *  5. computeWaypoints() — D/HD: derive 3 m-spaced centre waypoints from laser
 *                          by slicing the scan longitudinally.
 *
 * ### Coordinate conventions
 * Sensor origin is at (sx, sy) with heading sensorYaw (radians, world frame).
 * "Forward" is the +lx direction in the sensor local frame.
 * "Left" is the +ly direction (positive lateral offset).
 */
class TrackAnalyser
{
public:
    /**
     * @brief Constructs the analyser with configurable track parameters.
     *
     * @param nominalTrackWidth   Expected track width [m].  Default 8.0 m;
     *                            configured via ROS parameter track_width.
     * @param goalTolerance       Maximum lateral error for goal-in-corridor check [m].
     *                            Default 0.2 m per spec.
     * @param obstacleRange       Maximum forward range to treat as an obstacle [m].
     * @param waypointSpacing     Longitudinal spacing between D/HD waypoints [m].
     */
    explicit TrackAnalyser(double nominalTrackWidth = 8.0,
                           double goalTolerance     = 0.2,
                           double obstacleRange     = 6.0,
                           double waypointSpacing   = 3.0);

    ~TrackAnalyser() = default;

    // ── Configuration setters (called from ROS parameter reads) ──────────────

    /** @brief Override nominal track width from ROS parameter. */
    void setNominalTrackWidth(double w);

    /** @brief Override goal lateral tolerance from ROS parameter. */
    void setGoalTolerance(double t);

    // ── Core analysis ─────────────────────────────────────────────────────────

    /**
     * @brief Converts raw laser ranges to world-frame hit points.
     *
     * Rays with range == 0.0 (marked invalid by the ROS wrapper) are skipped.
     * The sensor origin is offset 3.725 m forward of the car's odometry-reported
     * centre along the heading axis — this offset is applied here so callers
     * only need to pass the raw odometry values.
     *
     * @param ranges     Raw range vector from LaserScan message.
     * @param carX       Car centre x from odometry [m].
     * @param carY       Car centre y from odometry [m].
     * @param carYaw     Car heading from odometry [rad].
     * @param angleMin   scan.angle_min from LaserScan message [rad].
     * @param angleInc   scan.angle_increment from LaserScan message [rad].
     * @return Vector of valid hit points in world frame.
     */
    std::vector<Point2D> toWorldPoints(const std::vector<double>& ranges,
                                       double carX,   double carY,   double carYaw,
                                       double angleMin, double angleInc) const;

    /**
     * @brief Finds the corridor centre from a set of world-frame hit points.
     *
     * Separates hits into left and right wall groups using the car's local
     * lateral axis.  Computes the median lateral distance to each wall group
     * and returns the world-frame corridor centre midpoint.
     *
     * Requires at least MIN_WALL_HITS hits on each side to declare a valid
     * result; otherwise CorridorResult::valid is false.
     *
     * @param hits    World-frame hit points (output of toWorldPoints()).
     * @param carX    Car centre x [m].
     * @param carY    Car centre y [m].
     * @param carYaw  Car heading [rad].
     * @return Populated CorridorResult; valid=false if walls not detected.
     */
    CorridorResult analyseCorridor(const std::vector<Point2D>& hits,
                                   double carX, double carY, double carYaw) const;

    /**
     * @brief Returns true if the goal lies within the detected corridor.
     *
     * Transforms the goal into the car's local frame and checks its lateral
     * component against the corridor centre within goalTolerance_.
     *
     * @param goalX   Goal x in world frame [m].
     * @param goalY   Goal y in world frame [m].
     * @param corridor Result from analyseCorridor().
     * @param carX    Car centre x [m].
     * @param carY    Car centre y [m].
     * @param carYaw  Car heading [rad].
     * @return true if the goal is within goalTolerance_ of corridor centre.
     */
    bool isGoalInCorridor(double goalX, double goalY,
                          const CorridorResult& corridor,
                          double carX, double carY, double carYaw) const;

    /**
     * @brief Returns true if a non-wall obstacle is detected in the forward cone.
     *
     * Examines rays within OBSTACLE_HALF_ANGLE_RAD of the car's heading.  A hit
     * is classified as an obstacle (not a wall) when its lateral offset in the
     * car frame is less than WALL_LATERAL_MIN_M and its forward distance is less
     * than obstacleRange_.
     *
     * @param ranges    Raw range vector from LaserScan message.
     * @param angleMin  scan.angle_min [rad].
     * @param angleInc  scan.angle_increment [rad].
     * @return true if an obstacle is detected ahead.
     */
    bool obstacleAhead(const std::vector<double>& ranges,
                       double angleMin, double angleInc) const;

    /**
     * @brief Computes the pure-pursuit steering angle toward a goal.
     *
     * Implements a simple proportional heading controller.  The steering output
     * is clamped to [-1, 1] (normalised Ackerman steering range).
     *
     * @param carX    Car x [m].
     * @param carY    Car y [m].
     * @param carYaw  Car heading [rad].
     * @param goalX   Goal x [m].
     * @param goalY   Goal y [m].
     * @return Steering command in [-1, 1].
     */
    double computeSteering(double carX, double carY, double carYaw,
                           double goalX, double goalY) const;

    // ── D/HD — laser-derived waypoints ────────────────────────────────────────

    /**
     * @brief Derives centre-of-track waypoints spaced every waypointSpacing_ m.
     *
     * For each longitudinal distance 3 m, 6 m, 9 m, … up to maxLongitudinal,
     * collects all hit points whose local forward (lx) component falls within
     * ±SLICE_HALF_WIDTH_M of the target distance, separates them into left/right
     * groups, and places a waypoint at the lateral midpoint converted back to
     * world frame.  Slices with insufficient hits on either side are skipped.
     *
     * @param hits              World-frame hit points from toWorldPoints().
     * @param carX              Car x [m].
     * @param carY              Car y [m].
     * @param carYaw            Car heading [rad].
     * @param maxLongitudinal   How far ahead to generate waypoints [m].
     * @return Vector of world-frame waypoints along the track centre.
     */
    std::vector<Point2D> computeWaypoints(const std::vector<Point2D>& hits,
                                          double carX, double carY, double carYaw,
                                          double maxLongitudinal = 20.0) const;

    // ── Utility ───────────────────────────────────────────────────────────────

    /** @brief Euclidean distance between two 2-D points [m]. */
    static double euclidean(double ax, double ay, double bx, double by);

    /** @brief Wraps angle to (-π, π]. */
    static double normaliseAngle(double angle);

private:
    double nominalTrackWidth_;   //!< Expected track width [m]
    double goalTolerance_;       //!< Max lateral error for goal validation [m]
    double obstacleRange_;       //!< Forward range threshold for obstacle detection [m]
    double waypointSpacing_;     //!< Longitudinal spacing for D/HD waypoints [m]

    // ── Constants ──────────────────────────────────────────────────────────
    static constexpr double ACKERMAN_LASER_OFFSET   = 3.725; //!< Laser forward offset from car centre [m]
    static constexpr double WALL_LATERAL_MIN_M      = 2.0;   //!< Min lateral offset to be classified as a wall hit [m]
    static constexpr double OBSTACLE_HALF_ANGLE_RAD = 5.0 * M_PI / 180.0; //!< Half-width of obstacle cone [rad]
    static constexpr int    MIN_WALL_HITS           = 3;     //!< Minimum hits per side to declare wall detected
    static constexpr double SLICE_HALF_WIDTH_M      = 0.5;   //!< Half-width of each longitudinal slice for waypoint computation [m]
    static constexpr double STEERING_GAIN           = 1.2;   //!< Proportional gain for steering controller
    static constexpr double STEERING_MAX            = 1.0;   //!< Steering clamp [normalised]

    /**
     * @brief Computes the sensor world-frame position from car odometry.
     *
     * Applies the 3.725 m forward offset along the car heading to convert the
     * rear-axle odometry position to the laser sensor origin.
     *
     * @param carX   Car centre x [m].
     * @param carY   Car centre y [m].
     * @param carYaw Car heading [rad].
     * @param[out] sx Sensor x [m].
     * @param[out] sy Sensor y [m].
     */
    void sensorOrigin(double carX, double carY, double carYaw,
                      double& sx, double& sy) const;

    /**
     * @brief Computes the median of a vector (modifies a copy, preserving original).
     * @param vals Values to find the median of.
     * @return Median value, or 0.0 if the vector is empty.
     */
    static double median(std::vector<double> vals);
};

#endif // TRACK_ANALYSER_H
