#include "mission.h"
#include "ranger.h"
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
    goalsAssigned_.assign(controllers_.size(), false);
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
    goalsAssigned_.assign(controllers_.size(), false);
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

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals,
                       pfms::PlatformType platform)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Assign to the first controller that matches the platform type and has
    // not yet been assigned goals in this mission cycle.
    for (unsigned int i = 0; i < controllers_.size(); ++i) {
        if (controllers_[i]->getPlatformType() == platform && !goalsAssigned_[i]) {
            goalsByIndex_[i]  = goals;
            goalsAssigned_[i] = true;
            return;
        }
    }
}

bool Mission::setMissionObjective(pfms::MissionObjective objective)
{
    objective_ = objective;
    return true;
}

bool Mission::execute(bool start)
{
    if (!start) {
        running_ = false;
        for (auto* ctrl : controllers_) ctrl->execute(false);
        if (missionThread_.joinable()) missionThread_.join();
        // Reset assignment flags so setGoals() can be called again for a new mission.
        std::lock_guard<std::mutex> lock(mutex_);
        goalsAssigned_.assign(controllers_.size(), false);
        return true;
    }

    // Dispatch the stored goal lists to each controller before starting the thread.
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

    // Pre-compute total distances for the status() percentage calculation.
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

        if (totalDistance_[i] <= 0.0) {
            result.push_back(0u);
            continue;
        }

        // Report 100% only when the controller is IDLE and has exhausted its goal list.
        // Cap at 99% while still RUNNING to prevent premature 100% reports.
        if (goalIdx >= static_cast<unsigned int>(goals.size())) {
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

void Mission::runMission()
{
    // Start all controllers concurrently.
    for (auto* ctrl : controllers_) ctrl->execute(true);

    if (objective_ == pfms::MissionObjective::BASIC) {
        // BASIC mode: poll until all controllers reach IDLE.
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

    // ADVANCED mode: monitor goal transitions and check for obstacles.
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

            // A goal transition has occurred — check whether the next goal is clear.
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

    // Compute the bearing from the sensor to the next goal in world frame,
    // then convert to a relative bearing in the sensor frame.
    const double dx            = nextGoal.x - sensorPose.position.x;
    const double dy            = nextGoal.y - sensorPose.position.y;
    const double distToGoal    = std::sqrt(dx * dx + dy * dy);
    const double bearingToGoal = std::atan2(dy, dx);

    double relBearing = bearingToGoal - sensorPose.yaw;
    while (relBearing >  M_PI) relBearing -= 2.0 * M_PI;
    while (relBearing < -M_PI) relBearing += 2.0 * M_PI;

    // Use the actual scan angle_min from the ranger rather than -fov/2 so the
    // ray indexing is correct for non-symmetric scan configurations.
    const double angleMin   = static_cast<Ranger*>(ranger)->getAngleMin();
    const double clearanceM = 1.0; //!< Obstacle must be at least this much closer than the goal [m]

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
