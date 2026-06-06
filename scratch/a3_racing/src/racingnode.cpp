#include "racingnode.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

RacingNode::RacingNode()
    : Node("racing_node"),
      hasOdom_(false),
      hasLaser_(false),
      currentGoal_(0),
      state_(MissionState::IDLE),
      running_(true)
{
    this->declare_parameter<double>("goal_tolerance", 0.03);
    this->declare_parameter<bool>("advanced", false);

    goalTolerance_ = this->get_parameter("goal_tolerance").as_double();
    advanced_      = this->get_parameter("advanced").as_bool();

    RCLCPP_INFO(this->get_logger(),
        "RacingNode starting -- tolerance=%.2f advanced=%s",
        goalTolerance_, advanced_ ? "true" : "false");

    subOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/orange/odom", 10,
        std::bind(&RacingNode::odomCallback, this, std::placeholders::_1));

    subLaser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/orange/laserscan", 10,
        std::bind(&RacingNode::laserCallback, this, std::placeholders::_1));

    subGoals_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orange/goals", 10,
        std::bind(&RacingNode::goalsCallback, this, std::placeholders::_1));

    pubThrottle_ = this->create_publisher<std_msgs::msg::Float64>("/orange/throttle_cmd", 10);
    pubBrake_    = this->create_publisher<std_msgs::msg::Float64>("/orange/brake_cmd", 10);
    pubSteering_ = this->create_publisher<std_msgs::msg::Float64>("/orange/steering_cmd", 10);
    pubMarkers_  = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualisation_marker", 10);
    pubWaypoints_= this->create_publisher<geometry_msgs::msg::PoseArray>("/orange/waypoints", 10);

    service_ = this->create_service<std_srvs::srv::SetBool>(
        "/orange/mission",
        std::bind(&RacingNode::missionService, this,
                  std::placeholders::_1, std::placeholders::_2));

    controlThread_ = std::thread(&RacingNode::controlLoop, this);

    RCLCPP_INFO(this->get_logger(), "RacingNode ready.");
}

RacingNode::~RacingNode()
{
    running_.store(false);
    if (controlThread_.joinable()) controlThread_.join();
}

// --- Callbacks ----------------------------------------------------------------

void RacingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    odom_    = *msg;
    hasOdom_ = true;

    if (laserProc_) laserProc_->newOdom(*msg);
}

void RacingNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    laser_    = *msg;
    hasLaser_ = true;

    if (!laserProc_) {
        laserProc_ = std::make_shared<LaserProcessing>(*msg);
        RCLCPP_INFO(this->get_logger(), "LaserProcessing initialised.");
    } else {
        laserProc_->newScan(*msg);
    }
}

void RacingNode::goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == MissionState::CRUISING || state_ == MissionState::TURNING) {
        RCLCPP_WARN(this->get_logger(), "Goals received while active -- ignored.");
        return;
    }

    goals_.clear();
    for (const auto& pose : msg->poses) {
        goals_.push_back(pose.position);
    }
    currentGoal_ = 0;
    RCLCPP_INFO(this->get_logger(), "Received %zu goals.", goals_.size());
}

void RacingNode::missionService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (req->data) {
        if (goals_.empty()) {
            res->success = false;
            res->message = "No goals available.";
            return;
        }
        if (state_ == MissionState::CRUISING || state_ == MissionState::TURNING) {
            res->success = true;
            res->message = "Already running.";
            return;
        }
        currentGoal_ = 0;
        waypoints_.clear();
        state_ = MissionState::CRUISING;
        RCLCPP_INFO(this->get_logger(), "Mission STARTED -- %zu goals.", goals_.size());
    } else {
        state_ = MissionState::IDLE;
        RCLCPP_INFO(this->get_logger(), "Mission STOPPED.");
    }

    const std::size_t total     = goals_.size();
    const std::size_t completed = currentGoal_;
    const double pct = total > 0 ? (static_cast<double>(completed) / total * 100.0) : 0.0;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << pct
       << "% complete (" << completed << "/" << total << ")";

    res->success = true;
    res->message = ss.str();
}

// --- Control loop -------------------------------------------------------------

void RacingNode::controlLoop()
{
    rclcpp::Rate rate(CONTROL_HZ);

    while (running_.load() && rclcpp::ok()) {

        nav_msgs::msg::Odometry                odom;
        MissionState                           state;
        std::vector<geometry_msgs::msg::Point> goals;
        std::size_t                            currentGoal;
        bool                                   haveOdom;
        bool                                   haveLaser;
        std::shared_ptr<LaserProcessing>       laserProc;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            odom        = odom_;
            state       = state_;
            goals       = goals_;
            currentGoal = currentGoal_;
            haveOdom    = hasOdom_;
            haveLaser   = hasLaser_;
            laserProc   = laserProc_;
        }

        MissionState pendingState   = state;
        std::size_t  pendingGoal    = currentGoal;
        bool         doPublishWaypoints = false;

        switch (state) {

        case MissionState::IDLE:
            publishStop();
            break;

        case MissionState::COMPLETE:
            publishStop();
            break;

        case MissionState::CRUISING:
        case MissionState::TURNING: {
            if (!haveOdom) {
                publishStop();
                break;
            }

            // -- All goals exhausted ------------------------------------------
            if (currentGoal >= goals.size()) {
                RCLCPP_INFO(this->get_logger(), "All goals reached -- COMPLETE.");
                publishStop();
                pendingState        = MissionState::COMPLETE;
                doPublishWaypoints  = true;
                break;
            }

            // -- Obstacle check -----------------------------------------------
            if (haveLaser && laserProc && laserProc->obstacleInFront()) {
                RCLCPP_WARN(this->get_logger(),
                    "Obstacle detected ahead -- ending mission.");
                publishStop();
                pendingState        = MissionState::COMPLETE;
                doPublishWaypoints  = true;
                break;
            }

            const geometry_msgs::msg::Point& goal = goals[currentGoal];
            const double dist = euclidean(odom, goal);

            // -- Overshot check -----------------------------------------------
            // Use the path-tangent direction (previous goal -> current goal)
            // as the reference instead of the vehicle's live yaw.  This
            // prevents the dot product from firing spuriously as the car's
            // heading rotates through a U-turn, which caused apex goals to be
            // skipped en masse and the car to miss the hairpin entirely.
            bool overshot = false;
            if (currentGoal > 0) {
                const geometry_msgs::msg::Point& prevGoal = goals[currentGoal - 1];
                const double pathDx  = goal.x - prevGoal.x;
                const double pathDy  = goal.y - prevGoal.y;
                const double pathLen = std::hypot(pathDx, pathDy);
                if (pathLen > 1e-6) {
                    const double toGoalX = goal.x - odom.pose.pose.position.x;
                    const double toGoalY = goal.y - odom.pose.pose.position.y;
                    const double pathDot = toGoalX * (pathDx / pathLen)
                                         + toGoalY * (pathDy / pathLen);
                    overshot = (pathDot < 0.0) && (dist > goalTolerance_);
                }
            } else {
                // No previous goal: fall back to vehicle-heading dot product
                // for goal 0 only, where no path tangent exists yet.
                const double yaw     = yawFromOdom(odom);
                const double toGoalX = goal.x - odom.pose.pose.position.x;
                const double toGoalY = goal.y - odom.pose.pose.position.y;
                const double dot     = toGoalX * std::cos(yaw) + toGoalY * std::sin(yaw);
                overshot = (dot < 0.0) && (dist > goalTolerance_);
            }

            if (dist < goalTolerance_ || overshot) {
                if (overshot)
                    RCLCPP_WARN(this->get_logger(),
                        "Goal %zu overshot -- skipping.", currentGoal);
                else
                    RCLCPP_INFO(this->get_logger(), "Goal %zu reached.", currentGoal);

                pendingGoal        = currentGoal + 1;
                doPublishWaypoints = true;
                prevAlpha_ = 0.0;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    waypoints_.push_back(goal);
                }
                break;
            }

            // -- Current speed ------------------------------------------------
            const double currentSpeed = std::hypot(
                odom.twist.twist.linear.x, odom.twist.twist.linear.y);

            // -- Pre-corner lookahead alpha (used for transition only) ---------
            // Peek at a close fixed-index target to detect an upcoming corner
            // early.  PREVIEW_GOAL_LOOKAHEAD is intentionally small and separate
            // from the steer-target lookahead so corner detection is responsive
            // even when the steer target is far ahead on a straight.
            const std::size_t previewIdx =
                std::min(currentGoal + PREVIEW_GOAL_LOOKAHEAD, goals.size() - 1);
            const double alphaPreview = computeAlpha(odom, goals[previewIdx]);

            // -- State transition: CRUISING <-> TURNING -----------------------
            // Transition into TURNING when either:
            //   (a) the close-lookahead alpha already exceeds the threshold, or
            //   (b) the car is within BRAKE_PREVIEW_DIST_M of the current goal
            //       and the corner alpha at that goal is large.
            const bool cornerImminent =
                std::abs(alphaPreview) > CORNER_ALPHA_RAD ||
                (dist < BRAKE_PREVIEW_DIST_M &&
                 std::abs(computeAlpha(odom, goal)) > CORNER_ALPHA_RAD * 0.7);

            // FIX (Bug 2): suppress exit from TURNING when the preview index is
            // clamped to the last goal -- alphaPreview would point nearly
            // straight ahead even mid-corner, causing a spurious transition.
            const bool previewClamped =
                (currentGoal + PREVIEW_GOAL_LOOKAHEAD) >= goals.size();

            // 20% hysteresis band on exit; blocked when preview is clamped.
            const bool exitCorner =
                !previewClamped &&
                std::abs(alphaPreview) < CORNER_ALPHA_RAD * 0.8;

            const MissionState nextState =
                (state == MissionState::TURNING)
                    ? (exitCorner ? MissionState::CRUISING : MissionState::TURNING)
                    : (cornerImminent ? MissionState::TURNING : MissionState::CRUISING);

            if (nextState != state) {
                RCLCPP_INFO(this->get_logger(),
                    "State transition: %s -> %s  (alphaPreview=%.3f dist=%.2f"
                    " previewClamped=%s)",
                    stateName(state), stateName(nextState),
                    alphaPreview, dist,
                    previewClamped ? "true" : "false");
                pendingState = nextState;
            }

            // -- [1] Lookahead: angle-scaled distance when turning -------------
            // In CRUISING a fixed long distance gives smooth straight-line
            // tracking.  In TURNING the lookahead distance shrinks continuously
            // with |alphaPreview| so the steer target tightens as the bend
            // sharpens.  At a full U-turn (pi rad) it reaches TURN_LD_MIN,
            // which pulls the car back out toward the arc when it has cut
            // inside rather than pointing tangentially along the inside wall.
            double lookaheadDist;
            if (cornerImminent) {
                const double alphaClamped = std::clamp(
                    std::abs(alphaPreview), 0.0, M_PI);
                const double turnFraction = alphaClamped / M_PI;
                lookaheadDist = TURN_LD_MAX
                    - (TURN_LD_MAX - TURN_LD_MIN) * turnFraction;
            } else {
                lookaheadDist = CRUISE_LOOKAHEAD_DIST_M;
            }

            // Walk forward through goals to find the first one at or beyond
            // lookaheadDist.  When the car is inside the arc, chord distances
            // are shorter so this automatically reaches further around the curve.
            std::size_t steerGoalIdx = goals.size() - 1;
            for (std::size_t k = currentGoal; k < goals.size(); ++k) {
                steerGoalIdx = k;
                if (euclidean(odom, goals[k]) >= lookaheadDist) break;
            }
            const geometry_msgs::msg::Point& steerGoal = goals[steerGoalIdx];

            // -- Alpha against chosen steer target ----------------------------
            const double alpha  = computeAlpha(odom, steerGoal);
            const double dAlpha = alpha - prevAlpha_;
            prevAlpha_          = alpha;

            // -- [2] Steering gain: tighten as soon as corner is imminent -----
            // Apply the higher gain whenever a corner is detected, not only
            // after the state has fully transitioned to TURNING.
            const double steerK  = cornerImminent ? TURN_STEER_K  : STEER_K;
            const double steerKd = cornerImminent ? TURN_STEER_KD : STEER_KD;

            const double steerRaw = std::atan2(
                2.0 * WHEELBASE_M * std::sin(alpha), dist);
            const double gain     = 1.0 + steerK * std::abs(alpha);
            const double steering = std::clamp(
                steerRaw * gain + steerKd * dAlpha, -MAX_STEER, MAX_STEER);

            // -- Speed planning -----------------------------------------------
            // In TURNING, raise the speedFactor floor so the car retains enough
            // momentum to drive back out to the arc when it has cut inside.
            // The default MIN_SPEED_FACTOR (0.3) gives only ~9% throttle at
            // large alpha, which stalls the car mid-corner.
            const double minFactor   = (nextState == MissionState::TURNING)
                ? MIN_SPEED_FACTOR_TURN
                : MIN_SPEED_FACTOR;
            const double speedFactor = std::max(std::cos(alpha), minFactor);

            // -- [3] Hard speed cap: lower ceiling in TURNING -----------------
            const double vMax       = (nextState == MissionState::TURNING)
                ? TURN_V_MAX : V_MAX;
            const double targetSpeed = vMax * speedFactor;

            // -- [4] Throttle / brake -----------------------------------------
            double throttle = 0.0;
            double brake    = 0.0;

            if (nextState == MissionState::TURNING) {
                if (currentSpeed > targetSpeed)
                    brake = CORNER_BRAKE;
                else
                    throttle = TURN_THROTTLE * speedFactor;
            } else {
                if (cornerImminent && currentSpeed > TURN_V_MAX)
                    brake = CORNER_BRAKE * 0.5;
                else
                    throttle = (currentSpeed < targetSpeed) ? CRUISE_THROTTLE : 0.0;
            }

            // -- Debug logging ------------------------------------------------
            RCLCPP_INFO(this->get_logger(),
                "[%s] goal=%zu steer=%zu ld=%.1f pos=(%.2f,%.2f) target=(%.2f,%.2f) "
                "dist=%.2f a=%.3f ap=%.3f steer=%.3f thr=%.2f brk=%.0f spd=%.2f vmax=%.1f",
                stateName(nextState),
                currentGoal, steerGoalIdx, lookaheadDist,
                odom.pose.pose.position.x, odom.pose.pose.position.y,
                steerGoal.x, steerGoal.y,
                dist, alpha, alphaPreview, steering, throttle, brake,
                currentSpeed, vMax);

            publishCommand(throttle, brake, steering);
            break;
        }

        } // switch

        if (doPublishWaypoints)
            publishWaypoints();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pendingState != state)
                state_ = pendingState;
            if (pendingGoal != currentGoal)
                currentGoal_ = pendingGoal;
        }

        rate.sleep();
    }
}

// --- Heading error ------------------------------------------------------------

double RacingNode::computeAlpha(const nav_msgs::msg::Odometry& odom,
                                 const geometry_msgs::msg::Point& target) const
{
    const double yaw = yawFromOdom(odom);
    const double dx  = target.x - odom.pose.pose.position.x;
    const double dy  = target.y - odom.pose.pose.position.y;
    const double ld  = std::hypot(dx, dy);
    if (ld < 1e-6) return 0.0;

    const double localX =  dx * std::cos(-yaw) - dy * std::sin(-yaw);
    const double localY =  dx * std::sin(-yaw) + dy * std::cos(-yaw);
    return std::atan2(localY, localX);
}

// --- Publish helpers ----------------------------------------------------------

void RacingNode::publishStop()
{
    publishCommand(0.0, MAX_BRAKE, 0.0);
}

void RacingNode::publishCommand(double throttle, double brake, double steering)
{
    std_msgs::msg::Float64 t, b, s;
    t.data = throttle;
    b.data = brake;
    s.data = steering;
    pubThrottle_->publish(t);
    pubBrake_->publish(b);
    pubSteering_->publish(s);
}

// --- Waypoints ----------------------------------------------------------------

void RacingNode::publishWaypoints()
{
    std::vector<geometry_msgs::msg::Point> wps;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        wps = waypoints_;
    }
    if (wps.empty()) return;

    const rclcpp::Time now = this->get_clock()->now();
    visualization_msgs::msg::MarkerArray markerArray;
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.stamp    = now;
    poseArray.header.frame_id = "world";

    for (std::size_t i = 0; i < wps.size(); ++i) {
        double wpYaw = 0.0;
        if (i + 1 < wps.size()) {
            wpYaw = std::atan2(wps[i+1].y - wps[i].y, wps[i+1].x - wps[i].x);
        } else if (i > 0) {
            wpYaw = std::atan2(wps[i].y - wps[i-1].y, wps[i].x - wps[i-1].x);
        }

        visualization_msgs::msg::Marker m;
        m.header.frame_id = "world";
        m.header.stamp    = now;
        m.ns              = "road";
        m.id              = static_cast<int>(i);
        m.type            = visualization_msgs::msg::Marker::CYLINDER;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.lifetime        = rclcpp::Duration(0, 0);
        m.pose.position   = wps[i];
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, wpYaw);
        m.pose.orientation = tf2::toMsg(q);
        m.scale.x = 0.4;
        m.scale.y = 0.4;
        m.scale.z = 0.5;
        m.color.r = 0.0f;
        m.color.g = 0.8f;
        m.color.b = 0.2f;
        m.color.a = 0.8f;
        markerArray.markers.push_back(m);

        geometry_msgs::msg::Pose pose;
        pose.position    = wps[i];
        pose.orientation = tf2::toMsg(q);
        poseArray.poses.push_back(pose);
    }

    pubMarkers_->publish(markerArray);
    pubWaypoints_->publish(poseArray);
}

// --- Static helpers -----------------------------------------------------------

double RacingNode::yawFromOdom(const nav_msgs::msg::Odometry& odom)
{
    const auto& q     = odom.pose.pose.orientation;
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

double RacingNode::normaliseAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double RacingNode::euclidean(const nav_msgs::msg::Odometry& odom,
                             const geometry_msgs::msg::Point& pt)
{
    const double dx = pt.x - odom.pose.pose.position.x;
    const double dy = pt.y - odom.pose.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

const char* RacingNode::stateName(MissionState s)
{
    switch (s) {
        case MissionState::IDLE:      return "IDLE";
        case MissionState::CRUISING:  return "CRUISING";
        case MissionState::TURNING:   return "TURNING";
        case MissionState::COMPLETE:  return "COMPLETE";
        default:                      return "UNKNOWN";
    }
}