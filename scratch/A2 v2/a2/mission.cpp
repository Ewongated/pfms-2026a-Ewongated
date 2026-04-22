#include "mission.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

// ============================================================
// Constructors / Destructor
// ============================================================

/**
 * @brief Constructs a Mission for BASIC mode (no rangers).
 *
 * Mission objective defaults to BASIC. Rangers vector is left empty;
 * obstacle detection is disabled.
 *
 * @param controllers Platforms to coordinate
 */
Mission::Mission(std::vector<ControllerInterface*> controllers)
    : controllers_(controllers)
    , objective_(pfms::MissionObjective::BASIC)
    , running_(false)
    , abandoned_(false)
{
    abandonedControllers_.assign(controllers_.size(), false);
    totalDistance_.assign(controllers_.size(), 0.0);
}

/**
 * @brief Constructs a Mission with rangers for ADVANCED mode.
 *
 * Rangers must be provided in the same order as controllers (index i of
 * rangers_ corresponds to index i of controllers_).
 *
 * @param controllers Platforms to coordinate
 * @param rangers     Laser/Sonar sensors for obstacle detection
 */
Mission::Mission(std::vector<ControllerInterface*> controllers,
                 std::vector<RangerInterface*> rangers)
    : controllers_(controllers)
    , rangers_(rangers)
    , objective_(pfms::MissionObjective::BASIC)
    , running_(false)
    , abandoned_(false)
{
    abandonedControllers_.assign(controllers_.size(), false);
    totalDistance_.assign(controllers_.size(), 0.0);
}

/**
 * @brief Destructor — stops the mission thread cleanly.
 */
Mission::~Mission()
{
    execute(false);
}

// ============================================================
// MissionInterface
// ============================================================

/**
 * @brief Sets the goal list for a specific platform type.
 *
 * If goals for this platform type were previously set, they are replaced.
 * The goals are stored and will be dispatched to the matching controller
 * when execute(true) is called.
 *
 * @param goals    Ordered waypoint list for this platform
 * @param platform The platform type these goals are intended for
 */
void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals,
                       pfms::PlatformType platform)
{
    std::lock_guard<std::mutex> lock(mutex_);
    goalMap_[platform] = goals;
}

/**
 * @brief Sets the mission objective (BASIC or ADVANCED).
 * @param objective The desired mode
 * @return true always
 */
bool Mission::setMissionObjective(pfms::MissionObjective objective)
{
    objective_ = objective;
    return true;
}

/**
 * @brief Starts or stops mission execution.
 *
 * execute(true): Dispatches goals to each controller, pre-computes total
 * distances, launches the background coordination thread, and returns
 * immediately (non-blocking).
 *
 * execute(false): Signals the coordination thread and all controllers to
 * stop, then joins the thread.
 *
 * @param start true to start, false to stop
 * @return true if the mission can proceed (all controllers have goals)
 */
bool Mission::execute(bool start)
{
    if (!start) {
        running_ = false;
        for (auto* ctrl : controllers_) {
            ctrl->execute(false);
        }
        if (missionThread_.joinable()) {
            missionThread_.join();
        }
        return true;
    }

    // Dispatch goals to each controller
    for (unsigned int i = 0; i < controllers_.size(); ++i) {
        pfms::PlatformType type = controllers_.at(i)->getPlatformType();
        std::lock_guard<std::mutex> lock(mutex_);
        if (goalMap_.count(type)) {
            controllers_.at(i)->setGoals(goalMap_.at(type));
        }
    }

    // Pre-compute total distances for % status calculation
    for (unsigned int i = 0; i < controllers_.size(); ++i) {
        totalDistance_.at(i) = computeTotalDistance(i);
    }

    // Reset abandoned state
    {
        std::lock_guard<std::mutex> lock(mutex_);
        abandoned_ = false;
        abandonedControllers_.assign(controllers_.size(), false);
    }

    running_ = true;
    missionThread_ = std::thread(&Mission::runMission, this);
    return true;
}

/**
 * @brief Returns mission completion status for each platform.
 *
 * Each value is a percentage [0–100] of distance travelled vs total
 * mission distance. Reaches 100 only when the controller is IDLE
 * (i.e. has finished its goal list or been stopped).
 *
 * @return Vector with one element per controller
 */
std::vector<unsigned int> Mission::status()
{
    std::vector<unsigned int> result;
    result.reserve(controllers_.size());

    for (unsigned int i = 0; i < controllers_.size(); ++i) {
        unsigned int goalIdx = 0;
        pfms::PlatformStatus ps = controllers_.at(i)->status(goalIdx);

        if (ps == pfms::PlatformStatus::IDLE && totalDistance_.at(i) > 0.0) {
            // Controller finished — check if it actually completed all goals
            // by comparing distanceTravelled to totalDistance
            double travelled = controllers_.at(i)->distanceTravelled();
            unsigned int pct = static_cast<unsigned int>(
                std::min(100.0, 100.0 * travelled / totalDistance_.at(i)));

            // If IDLE and goals are exhausted → 100%
            pfms::PlatformType type = controllers_.at(i)->getPlatformType();
            std::vector<pfms::geometry_msgs::Point> goals;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goalMap_.count(type)) goals = goalMap_.at(type);
            }
            if (goalIdx >= goals.size()) {
                pct = 100;
            }
            result.push_back(pct);
        } else if (totalDistance_.at(i) <= 0.0) {
            result.push_back(0);
        } else {
            double travelled = controllers_.at(i)->distanceTravelled();
            unsigned int pct = static_cast<unsigned int>(
                std::min(99.0, 100.0 * travelled / totalDistance_.at(i)));
            result.push_back(pct);
        }
    }
    return result;
}

/**
 * @brief Returns distance travelled by each platform.
 * @return Vector of distances [m], one per controller
 */
std::vector<double> Mission::getDistanceTravelled()
{
    std::vector<double> result;
    result.reserve(controllers_.size());
    for (auto* ctrl : controllers_) {
        result.push_back(ctrl->distanceTravelled());
    }
    return result;
}

/**
 * @brief Returns time in motion for each platform.
 * @return Vector of times [s], one per controller
 */
std::vector<double> Mission::getTimeMoving()
{
    std::vector<double> result;
    result.reserve(controllers_.size());
    for (auto* ctrl : controllers_) {
        result.push_back(ctrl->timeTravelled());
    }
    return result;
}

/**
 * @brief Returns true if any controller was stopped due to an obstacle.
 */
bool Mission::isAbandoned() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return abandoned_;
}

/**
 * @brief Returns per-controller abandonment flags.
 * @return Vector of bools (true = that controller was stopped by obstacle)
 */
std::vector<bool> Mission::getAbandonedControllers() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return abandonedControllers_;
}

// ============================================================
// Private — background thread
// ============================================================

/**
 * @brief Background mission coordination loop.
 *
 * BASIC mode:
 *   Starts all controllers concurrently and waits for all to reach IDLE.
 *
 * ADVANCED mode:
 *   Monitors each controller's currentGoalIndex. When a controller
 *   completes a goal and is about to move to the next, uses isPathObstructed()
 *   to check the path. If obstructed, stops that controller and marks it
 *   as abandoned. Other controllers continue unaffected.
 *
 * Polls at 100ms intervals to avoid busy-waiting.
 */
void Mission::runMission()
{
    // Start all controllers concurrently
    for (auto* ctrl : controllers_) {
        ctrl->execute(true);
    }

    if (objective_ == pfms::MissionObjective::BASIC) {
        // BASIC: just wait for all to finish
        while (running_.load()) {
            bool allDone = true;
            for (auto* ctrl : controllers_) {
                unsigned int idx = 0;
                if (ctrl->status(idx) != pfms::PlatformStatus::IDLE) {
                    allDone = false;
                    break;
                }
            }
            if (allDone) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return;
    }

    // ADVANCED: monitor goal transitions and check for obstacles
    // Track the last known goal index per controller
    std::vector<unsigned int> lastGoalIdx(controllers_.size(), 0);

    while (running_.load()) {
        bool allDone = true;

        for (unsigned int i = 0; i < controllers_.size(); ++i) {
            // Skip already-abandoned controllers
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (abandonedControllers_.at(i)) continue;
            }

            unsigned int currentIdx = 0;
            pfms::PlatformStatus ps = controllers_.at(i)->status(currentIdx);

            if (ps != pfms::PlatformStatus::IDLE) {
                allDone = false;
            }

            // Detect when the controller has advanced to a new goal
            if (currentIdx > lastGoalIdx.at(i)) {
                lastGoalIdx.at(i) = currentIdx;

                // Get the next goal this controller will pursue
                pfms::PlatformType type = controllers_.at(i)->getPlatformType();
                std::vector<pfms::geometry_msgs::Point> goals;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (goalMap_.count(type)) goals = goalMap_.at(type);
                }

                if (currentIdx < goals.size()) {
                    // Check if the path to the next goal is clear
                    if (!rangers_.empty() && i < rangers_.size()) {
                        if (isPathObstructed(i, goals[currentIdx])) {
                            // Path blocked — abandon this controller
                            controllers_.at(i)->execute(false);
                            std::lock_guard<std::mutex> lock(mutex_);
                            abandonedControllers_.at(i) = true;
                            abandoned_ = true;
                        }
                    }
                }
            }
        }

        if (allDone) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ============================================================
// Private — obstacle detection
// ============================================================

/**
 * @brief Checks whether the straight-line path to nextGoal is obstructed.
 *
 * Algorithm:
 *  1. Read a laser scan from rangers_.at(controllerIdx).
 *  2. Get the current sensor pose (world frame).
 *  3. Compute the bearing from the sensor to nextGoal.
 *  4. For each laser ray whose angle is within BEAM_HALF_WIDTH of that
 *     bearing, compare its range to the distance to the goal.
 *  5. If any such ray returns a range shorter than (dist_to_goal - tolerance),
 *     an obstacle is present on the path.
 *
 * @param controllerIdx Index into controllers_ and rangers_
 * @param nextGoal      Target point to check
 * @return true if an obstacle is detected
 */
bool Mission::isPathObstructed(unsigned int controllerIdx,
                                const pfms::geometry_msgs::Point& nextGoal)
{
    if (controllerIdx >= rangers_.size()) return false;

    RangerInterface* ranger = rangers_.at(controllerIdx);

    // Read a fresh laser scan (this updates sensorPose internally)
    std::vector<double> ranges = ranger->getData();
    if (ranges.empty()) return false;

    pfms::nav_msgs::Odometry sensorPose = ranger->getSensorPose();
    double fov        = ranger->getFieldOfView() * M_PI / 180.0; // convert to rad
    double angRes     = ranger->getAngularResolution() * M_PI / 180.0;
    double maxRange   = ranger->getMaxRange();

    // Bearing from sensor to next goal (world frame)
    double dx          = nextGoal.x - sensorPose.position.x;
    double dy          = nextGoal.y - sensorPose.position.y;
    double distToGoal  = std::sqrt(dx*dx + dy*dy);
    double bearingToGoal = std::atan2(dy, dx);

    // Bearing relative to sensor heading
    double relBearing = bearingToGoal - sensorPose.yaw;
    while (relBearing >  M_PI) relBearing -= 2.0 * M_PI;
    while (relBearing < -M_PI) relBearing += 2.0 * M_PI;

    // Laser rays span from -fov/2 to +fov/2 relative to sensor heading
    double angleMin = -fov / 2.0;

    // Angular window to check — check rays within ±BEAM_HALF_WIDTH of goal bearing
    const double BEAM_HALF_WIDTH = 5.0 * M_PI / 180.0; // ±5 degrees

    bool obstructed = false;
    for (unsigned int r = 0; r < ranges.size(); ++r) {
        double rayAngle = angleMin + r * angRes;

        if (std::abs(rayAngle - relBearing) > BEAM_HALF_WIDTH) continue;
        if (ranges[r] <= 0.0 || ranges[r] >= maxRange) continue;

        // If this ray hits something closer than the goal, path is blocked
        // Use a small margin so we don't flag the goal itself
        double tolerance = controllers_.at(controllerIdx)->distanceToGoal() > 0.0
                         ? 1.0 : 0.5;
        if (ranges[r] < (distToGoal - tolerance)) {
            obstructed = true;
            break;
        }
    }

    return obstructed;
}

// ============================================================
// Private — distance pre-computation
// ============================================================

/**
 * @brief Computes the total estimated mission distance for a controller.
 *
 * Chains checkOriginToDestination() across all goals in order, starting
 * from the controller's current odometry. Used to normalise the status()
 * percentage.
 *
 * @param controllerIdx Index into controllers_
 * @return Total estimated distance [m], or 0 if no goals
 */
double Mission::computeTotalDistance(unsigned int controllerIdx)
{
    pfms::PlatformType type = controllers_.at(controllerIdx)->getPlatformType();
    std::vector<pfms::geometry_msgs::Point> goals;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!goalMap_.count(type)) return 0.0;
        goals = goalMap_.at(type);
    }

    if (goals.empty()) return 0.0;

    double total = 0.0;
    pfms::nav_msgs::Odometry origin = controllers_.at(controllerIdx)->getOdometry();

    for (const auto& goal : goals) {
        double dist = 0.0, time = 0.0;
        pfms::nav_msgs::Odometry estPose;
        controllers_.at(controllerIdx)->checkOriginToDestination(origin, goal, dist, time, estPose);
        if (dist > 0.0) total += dist;
        origin = estPose;
    }
    return total;
}