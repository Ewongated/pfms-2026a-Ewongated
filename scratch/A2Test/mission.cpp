#include "mission.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

// ============================================================
// Constructors / Destructor
// ============================================================

Mission::Mission(std::vector<ControllerInterface*> controllers)
    : controllers_(controllers)
    , objective_(pfms::MissionObjective::BASIC)
    , running_(false)
    , abandoned_(false)
{
    goalsByIndex_.resize(controllers_.size());
    abandonedControllers_.assign(controllers_.size(), false);
    totalDistance_.assign(controllers_.size(), 0.0);
}

Mission::Mission(std::vector<ControllerInterface*> controllers,
                 std::vector<RangerInterface*> rangers)
    : controllers_(controllers)
    , rangers_(rangers)
    , objective_(pfms::MissionObjective::BASIC)
    , running_(false)
    , abandoned_(false)
{
    goalsByIndex_.resize(controllers_.size());
    abandonedControllers_.assign(controllers_.size(), false);
    totalDistance_.assign(controllers_.size(), 0.0);
}

Mission::~Mission()
{
    execute(false);
}

// ============================================================
// MissionInterface
// ============================================================

/**
 * @brief Associates goals with the first unassigned controller matching platform.
 *
 * Uses index-based storage so two controllers of the same PlatformType each
 * receive their own independent goal list.
 */
void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals,
                       pfms::PlatformType platform)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (unsigned int i = 0; i < controllers_.size(); ++i) {
        if (controllers_[i]->getPlatformType() == platform) {
            goalsByIndex_[i] = goals;
            return;
        }
    }
}

bool Mission::setMissionObjective(pfms::MissionObjective objective)
{
    objective_ = objective;
    return true;
}

/**
 * @brief Starts or stops mission execution.
 *
 * execute(true): Dispatches goals to controllers, pre-computes distances,
 * resets abandonment state, launches runMission() thread, returns immediately.
 * Controllers are started inside runMission() — this keeps execute() non-blocking
 * and prevents double-execute if a test also calls controller->execute(true).
 *
 * execute(false): Clears running_, stops all controllers, joins thread.
 */
bool Mission::execute(bool start)
{
    if (!start) {
        running_ = false;
        for (auto* ctrl : controllers_) ctrl->execute(false);
        if (missionThread_.joinable()) missionThread_.join();
        return true;
    }

    // Dispatch stored goals to each controller
    {
        for (unsigned int i = 0; i < controllers_.size(); ++i) {
            std::vector<pfms::geometry_msgs::Point> goals;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                goals = goalsByIndex_[i];
            }
            if (!goals.empty()) {
                controllers_[i]->setGoals(goals);
            }
        }
    }

    for (unsigned int i = 0; i < controllers_.size(); ++i) {
        totalDistance_[i] = computeTotalDistance(i);
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        abandoned_ = false;
        abandonedControllers_.assign(controllers_.size(), false);
    }

    running_       = true;
    missionThread_ = std::thread(&Mission::runMission, this);
    return true;
}

/**
 * @brief Returns per-controller completion percentage [0–100].
 *
 * Returns 100 only when the controller is IDLE and all goals are exhausted.
 * Caps at 99 while still RUNNING to avoid premature 100%.
 */
std::vector<unsigned int> Mission::status()
{
    std::vector<unsigned int> result;
    result.reserve(controllers_.size());

    for (unsigned int i = 0; i < controllers_.size(); ++i) {
        unsigned int goalIdx = 0;
        const pfms::PlatformStatus ps = controllers_[i]->status(goalIdx);
        const double travelled = controllers_[i]->distanceTravelled();

        std::vector<pfms::geometry_msgs::Point> goals;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (i < goalsByIndex_.size()) goals = goalsByIndex_[i];
        }

        std::cout << "[status] controller " << i
                  << " ps=" << ps
                  << " goalIdx=" << goalIdx
                  << " goals.size()=" << goals.size()
                  << " travelled=" << travelled
                  << " totalDist=" << totalDistance_[i] << std::endl;

        if (totalDistance_[i] <= 0.0) {
            result.push_back(0u);
            continue;
        }

        // Return 100 if IDLE and all goals exhausted, or if goalIdx has
        // passed all goals regardless of status
        if ((ps == pfms::PlatformStatus::IDLE &&
             goalIdx >= static_cast<unsigned int>(goals.size())) ||
            goalIdx >= static_cast<unsigned int>(goals.size())) {
            result.push_back(100u);
        } else if (ps == pfms::PlatformStatus::IDLE) {
            result.push_back(static_cast<unsigned int>(
                std::min(100.0, 100.0 * travelled / totalDistance_[i])));
        } else {
            result.push_back(static_cast<unsigned int>(
                std::min(99.0, 100.0 * travelled / totalDistance_[i])));
        }
    }
    return result;
}

std::vector<double> Mission::getDistanceTravelled()
{
    std::vector<double> result;
    result.reserve(controllers_.size());
    for (auto* ctrl : controllers_) result.push_back(ctrl->distanceTravelled());
    return result;
}

std::vector<double> Mission::getTimeMoving()
{
    std::vector<double> result;
    result.reserve(controllers_.size());
    for (auto* ctrl : controllers_) result.push_back(ctrl->timeTravelled());
    return result;
}

bool Mission::isAbandoned() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return abandoned_;
}

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
 * Starts all controllers concurrently then polls at 100 ms.
 * BASIC: waits for all IDLE.
 * ADVANCED: also checks for obstacles at each goal transition and stops
 * any controller whose next path is obstructed.
 */
void Mission::runMission()
{
    for (auto* ctrl : controllers_) ctrl->execute(true);

    if (objective_ == pfms::MissionObjective::BASIC) {
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
    std::vector<unsigned int> lastGoalIdx(controllers_.size(), 0);

    while (running_.load()) {
        bool allDone = true;

        for (unsigned int i = 0; i < controllers_.size(); ++i) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (abandonedControllers_[i]) continue;
            }

            unsigned int currentIdx = 0;
            const pfms::PlatformStatus ps = controllers_[i]->status(currentIdx);
            if (ps != pfms::PlatformStatus::IDLE) allDone = false;

            if (currentIdx > lastGoalIdx[i]) {
                lastGoalIdx[i] = currentIdx;

                std::vector<pfms::geometry_msgs::Point> goals;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (i < goalsByIndex_.size()) goals = goalsByIndex_[i];
                }

                if (currentIdx < static_cast<unsigned int>(goals.size())) {
                    if (!rangers_.empty() && i < rangers_.size()) {
                        if (isPathObstructed(i, goals[currentIdx])) {
                            controllers_[i]->execute(false);
                            std::lock_guard<std::mutex> lock(mutex_);
                            abandonedControllers_[i] = true;
                            abandoned_               = true;
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
 * @brief Returns true if the path to nextGoal is obstructed.
 *
 * Reads laser, computes bearing to goal, checks every ray within
 * ±BEAM_HALF_WIDTH_RAD. If any ray returns a range shorter than the
 * goal distance (minus a 1 m clearance margin), an obstacle is present.
 */
bool Mission::isPathObstructed(unsigned int controllerIdx,
                                const pfms::geometry_msgs::Point& nextGoal)
{
    if (controllerIdx >= rangers_.size()) return false;

    RangerInterface* ranger = rangers_[controllerIdx];
    const std::vector<double> ranges = ranger->getData();
    if (ranges.empty()) return false;

    const pfms::nav_msgs::Odometry sensorPose = ranger->getSensorPose();
    const double fov      = ranger->getFieldOfView() * M_PI / 180.0;
    const double angRes   = ranger->getAngularResolution() * M_PI / 180.0;
    const double maxRange = ranger->getMaxRange();

    const double dx           = nextGoal.x - sensorPose.position.x;
    const double dy           = nextGoal.y - sensorPose.position.y;
    const double distToGoal   = std::sqrt(dx * dx + dy * dy);
    const double bearingToGoal = std::atan2(dy, dx);

    double relBearing = bearingToGoal - sensorPose.yaw;
    while (relBearing >  M_PI) relBearing -= 2.0 * M_PI;
    while (relBearing < -M_PI) relBearing += 2.0 * M_PI;

    const double angleMin     = -fov / 2.0;
    const double clearanceM   = 1.0; //!< obstacle must be this much closer than goal

    for (unsigned int r = 0; r < ranges.size(); ++r) {
        const double rayAngle = angleMin + static_cast<double>(r) * angRes;
        if (std::abs(rayAngle - relBearing) > BEAM_HALF_WIDTH_RAD) continue;
        if (ranges[r] <= 0.0 || ranges[r] >= maxRange)              continue;
        if (ranges[r] < (distToGoal - clearanceM))                  return true;
    }
    return false;
}

// ============================================================
// Private — distance pre-computation
// ============================================================

double Mission::computeTotalDistance(unsigned int controllerIdx)
{
    std::vector<pfms::geometry_msgs::Point> goals;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (controllerIdx >= goalsByIndex_.size()) return 0.0;
        goals = goalsByIndex_[controllerIdx];
    }
    if (goals.empty()) return 0.0;

    double total = 0.0;
    pfms::nav_msgs::Odometry origin = controllers_[controllerIdx]->getOdometry();

    for (const auto& goal : goals) {
        double dist = 0.0, time = 0.0;
        pfms::nav_msgs::Odometry estPose;
        controllers_[controllerIdx]->checkOriginToDestination(
            origin, goal, dist, time, estPose);
        if (dist > 0.0) total += dist;
        origin = estPose;
    }
    return total;
}
