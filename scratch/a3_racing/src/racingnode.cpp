/**
 * @file racingnode.cpp
 * @brief Implementation of RacingNode.
 */
#include "racingnode.h"

#include <cmath>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

// Construction / destruction

RacingNode::RacingNode()
    : Node("racing_node"),
      hasOdom_(false),
      hasLaser_(false),
      currentGoal_(0),
      state_(MissionState::IDLE),
      running_(true),
      prevAlpha_(0.0)
{
    this->declare_parameter<double>("goal_tolerance", 1.5);
    this->declare_parameter<bool>("advanced", false);

    goalTolerance_ = this->get_parameter("goal_tolerance").as_double();
    advanced_      = this->get_parameter("advanced").as_bool();

    RCLCPP_INFO(get_logger(), "RacingNode starting tolerance=%.2f advanced=%s",
        goalTolerance_, advanced_ ? "true" : "false");

    subOdom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/orange/odom", 10,
        std::bind(&RacingNode::odomCallback, this, std::placeholders::_1));

    subLaser_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/orange/laserscan", 10,
        std::bind(&RacingNode::laserCallback, this, std::placeholders::_1));

    subGoals_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "/orange/goals", 10,
        std::bind(&RacingNode::goalsCallback, this, std::placeholders::_1));

    pubThrottle_ = create_publisher<std_msgs::msg::Float64>("/orange/throttle_cmd", 10);
    pubBrake_    = create_publisher<std_msgs::msg::Float64>("/orange/brake_cmd", 10);
    pubSteering_ = create_publisher<std_msgs::msg::Float64>("/orange/steering_cmd", 10);
    pubMarkers_  = create_publisher<visualization_msgs::msg::MarkerArray>("/visualisation_marker", 10);
    pubWaypoints_= create_publisher<geometry_msgs::msg::PoseArray>("/orange/waypoints", 10);

    service_ = create_service<std_srvs::srv::SetBool>(
        "/orange/mission",
        std::bind(&RacingNode::missionService, this,
                  std::placeholders::_1, std::placeholders::_2));

    controlThread_ = std::thread(&RacingNode::controlLoop, this);
    RCLCPP_INFO(get_logger(), "RacingNode ready.");
}

RacingNode::~RacingNode()
{
    running_.store(false);
    if (controlThread_.joinable()) controlThread_.join();
}

// Callbacks 

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
        RCLCPP_INFO(get_logger(), "LaserProcessing initialised.");
    } else {
        laserProc_->newScan(*msg);
    }
}

void RacingNode::goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == MissionState::CRUISING || state_ == MissionState::TURNING) {
        RCLCPP_WARN(get_logger(), "Goals received while active ignored.");
        return;
    }
    if (msg->poses.empty()) {
        RCLCPP_WARN(get_logger(), "Empty goal list ignored.");
        return;
    }
    goals_.clear();
    for (const auto& pose : msg->poses) goals_.push_back(pose.position);
    currentGoal_ = 0;
    RCLCPP_INFO(get_logger(), "Received %zu goals.", goals_.size());
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
            // Report corridor status for the current goal while already running.
            res->success = laserProc_ && currentGoal_ < goals_.size()
                           && laserProc_->goalInCorridor(goals_[currentGoal_]);
            res->message = "Already running.";
            return;
        }
        currentGoal_ = 0;
        waypoints_.clear();
        advancedGoals_.clear();
        state_ = MissionState::CRUISING;
        RCLCPP_INFO(get_logger(), "Mission STARTED %zu goals.", goals_.size());
    } else {
        state_ = MissionState::IDLE;
        RCLCPP_INFO(get_logger(), "Mission STOPPED.");
    }

    // Build progress string.
    const double pct = goals_.empty() ? 0.0
        : static_cast<double>(currentGoal_) / goals_.size() * 100.0;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << pct
       << "% complete (" << currentGoal_ << "/" << goals_.size() << ")";

    res->success = laserProc_ && currentGoal_ < goals_.size()
                   && laserProc_->goalInCorridor(goals_[currentGoal_]);
    res->message = ss.str();
}

//Control loop 

void RacingNode::controlLoop()
{
    rclcpp::Rate rate(CONTROL_HZ);

    while (running_.load() && rclcpp::ok()) {

        // Snapshot shared state
        nav_msgs::msg::Odometry                odom;
        MissionState                           state;
        std::vector<geometry_msgs::msg::Point> goals;
        std::size_t                            currentGoal;
        bool                                   haveOdom, haveLaser;
        std::shared_ptr<LaserProcessing>       laserProc;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            odom        = odom_;
            state       = state_;
            currentGoal = currentGoal_;
            haveOdom    = hasOdom_;
            haveLaser   = hasLaser_;
            laserProc   = laserProc_;

            goals = goals_;
            // In advanced mode, append laser-derived waypoints after the provided goals.
            if (advanced_) {
                goals.insert(goals.end(), advancedGoals_.begin(), advancedGoals_.end());
            }
        }

        // Generate a new advanced waypoint each tick while actively driving.
        if (advanced_ && haveOdom && laserProc &&
            (state == MissionState::CRUISING || state == MissionState::TURNING))
        {
            generateAdvancedWaypoints(laserProc, odom);
        }

        MissionState pendingState      = state;
        std::size_t  pendingGoal       = currentGoal;
        bool         doPublishWaypoints = false;

        switch (state) {

        case MissionState::IDLE:
        case MissionState::COMPLETE:
            publishStop();
            break;

        case MissionState::CRUISING:
        case MissionState::TURNING: {
            if (!haveOdom) { publishStop(); break; }

            // All goals exhausted 
            if (currentGoal >= goals.size()) {
                RCLCPP_INFO(get_logger(), "All goals reached COMPLETE.");
                publishStop();
                pendingState       = MissionState::COMPLETE;
                doPublishWaypoints = true;
                break;
            }

            // Obstacle check 
            if (haveLaser && laserProc && laserProc->obstacleInFront()) {
                RCLCPP_WARN(get_logger(), "Obstacle detected ending mission.");
                publishStop();
                pendingState       = MissionState::COMPLETE;
                doPublishWaypoints = true;
                break;
            }

            const geometry_msgs::msg::Point& goal = goals[currentGoal];

            // Corridor validation (log only; do not skip the goal) 
            const bool inCorridor = !haveLaser || !laserProc
                                    || laserProc->goalInCorridor(goal);
            if (!inCorridor) {
                RCLCPP_DEBUG(get_logger(),
                    "Goal %zu outside current laser corridor.", currentGoal);
            }

            const double dist = geom::euclidean(odom, goal);

            // Overshot check 
            // Use the path-tangent direction rather than the vehicle yaw so that
            // apex goals are not spuriously skipped mid-U-turn.
            bool overshot = false;
            if (currentGoal > 0) {
                const geometry_msgs::msg::Point& prev = goals[currentGoal - 1];
                const double pathDx  = goal.x - prev.x;
                const double pathDy  = goal.y - prev.y;
                const double pathLen = std::hypot(pathDx, pathDy);
                if (pathLen > 1e-6) {
                    const double dot = (goal.x - odom.pose.pose.position.x) * (pathDx / pathLen)
                                     + (goal.y - odom.pose.pose.position.y) * (pathDy / pathLen);
                    overshot = (dot < 0.0) && (dist > goalTolerance_);
                }
            } else {
                // Goal 0: fall back to vehicle heading dot product.
                const double yaw = geom::yawFromOdom(odom);
                const double dot = (goal.x - odom.pose.pose.position.x) * std::cos(yaw)
                                 + (goal.y - odom.pose.pose.position.y) * std::sin(yaw);
                overshot = (dot < 0.0) && (dist > goalTolerance_);
            }

            if (dist < goalTolerance_ || overshot) {
                if (overshot)
                    RCLCPP_WARN(get_logger(), "Goal %zu overshot skipping.", currentGoal);
                else
                    RCLCPP_INFO(get_logger(), "Goal %zu reached.", currentGoal);

                pendingGoal        = currentGoal + 1;
                doPublishWaypoints = true;
                prevAlpha_         = 0.0;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    waypoints_.push_back(goal);
                }
                break;
            }

            // Speed 
            const double currentSpeed = std::hypot(
                odom.twist.twist.linear.x, odom.twist.twist.linear.y);

            // Corner preview 
            // Peek a few goals ahead for early corner detection; kept separate
            // from the steer-target lookahead to stay responsive on straights.
            const std::size_t previewIdx =
                std::min(currentGoal + PREVIEW_GOAL_LOOKAHEAD, goals.size() - 1);
            const double alphaPreview = computeAlpha(odom, goals[previewIdx]);

            const bool cornerImminent =
                std::abs(alphaPreview) > CORNER_ALPHA_RAD ||
                (dist < BRAKE_PREVIEW_DIST_M &&
                 std::abs(computeAlpha(odom, goal)) > CORNER_ALPHA_RAD * 0.7);

            // Suppress exit from TURNING when preview index is clamped to the
            // last goal (alphaPreview would appear small even mid-corner).
            const bool previewClamped =
                (currentGoal + PREVIEW_GOAL_LOOKAHEAD) >= goals.size();

            const bool exitCorner =
                !previewClamped && std::abs(alphaPreview) < CORNER_ALPHA_RAD * 0.8;

            const MissionState nextState =
                (state == MissionState::TURNING)
                    ? (exitCorner ? MissionState::CRUISING : MissionState::TURNING)
                    : (cornerImminent ? MissionState::TURNING : MissionState::CRUISING);

            if (nextState != state) {
                RCLCPP_INFO(get_logger(),
                    "State: %s � %s (ap=%.3f dist=%.2f clamped=%s)",
                    stateName(state), stateName(nextState),
                    alphaPreview, dist, previewClamped ? "Y" : "N");
                pendingState = nextState;
            }

            // Lookahead distance 
            // On straights use a long fixed distance for smooth tracking.
            // At corners shrink lookahead continuously with bend severity so
            // the steer target pulls the car back toward the arc.
            double lookaheadDist;
            if (cornerImminent) {
                const double frac = std::clamp(std::abs(alphaPreview), 0.0, M_PI) / M_PI;
                lookaheadDist = TURN_LD_MAX - (TURN_LD_MAX - TURN_LD_MIN) * frac;
            } else {
                lookaheadDist = CRUISE_LOOKAHEAD_DIST_M;
            }

            // Walk forward to find the first goal at or beyond lookaheadDist.
            std::size_t steerIdx = goals.size() - 1;
            for (std::size_t k = currentGoal; k < goals.size(); ++k) {
                steerIdx = k;
                if (geom::euclidean(odom, goals[k]) >= lookaheadDist) break;
            }
            const geometry_msgs::msg::Point& steerGoal = goals[steerIdx];

            //Steering
            const double alpha  = computeAlpha(odom, steerGoal);
            const double dAlpha = alpha - prevAlpha_;
            prevAlpha_          = alpha;

            const double steerK  = cornerImminent ? TURN_STEER_K  : STEER_K;
            const double steerKd = cornerImminent ? TURN_STEER_KD : STEER_KD;

            const double steerRaw = std::atan2(2.0 * WHEELBASE_M * std::sin(alpha), dist);
            const double steering = std::clamp(
                steerRaw * (1.0 + steerK * std::abs(alpha)) + steerKd * dAlpha,
                -MAX_STEER, MAX_STEER);

            //Throttle / brake 
            const double minFactor   = (nextState == MissionState::TURNING)
                ? MIN_SPEED_FACTOR_TURN : MIN_SPEED_FACTOR;
            const double speedFactor = std::max(std::cos(alpha), minFactor);
            const double vMax        = (nextState == MissionState::TURNING)
                ? TURN_V_MAX : V_MAX;
            const double targetSpeed = vMax * speedFactor;

            double throttle = 0.0, brake = 0.0;
            if (nextState == MissionState::TURNING) {
                if (currentSpeed > targetSpeed) brake    = CORNER_BRAKE;
                else                            throttle = TURN_THROTTLE * speedFactor;
            } else {
                if (cornerImminent && currentSpeed > TURN_V_MAX)
                    brake    = CORNER_BRAKE * 0.5;
                else
                    throttle = (currentSpeed < targetSpeed) ? CRUISE_THROTTLE : 0.0;
            }

            RCLCPP_INFO(get_logger(),
                "[%s] g=%zu st=%zu ld=%.1f pos=(%.2f,%.2f) tgt=(%.2f,%.2f) "
                "d=%.2f a=%.3f ap=%.3f str=%.3f thr=%.2f brk=%.0f v=%.2f corr=%s",
                stateName(nextState), currentGoal, steerIdx, lookaheadDist,
                odom.pose.pose.position.x, odom.pose.pose.position.y,
                steerGoal.x, steerGoal.y,
                dist, alpha, alphaPreview, steering, throttle, brake,
                currentSpeed, inCorridor ? "Y" : "N");

            publishCommand(throttle, brake, steering);
            break;
        }

        } // switch

        if (doPublishWaypoints) publishWaypoints();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pendingState != state)      state_       = pendingState;
            if (pendingGoal  != currentGoal) currentGoal_ = pendingGoal;
        }

        rate.sleep();
    }
}

// Helpers 

double RacingNode::computeAlpha(const nav_msgs::msg::Odometry& odom,
                                const geometry_msgs::msg::Point& target) const
{
    const double yaw    = geom::yawFromOdom(odom);
    const double dx     = target.x - odom.pose.pose.position.x;
    const double dy     = target.y - odom.pose.pose.position.y;
    if (std::hypot(dx, dy) < 1e-6) return 0.0;

    const double localX =  dx * std::cos(-yaw) - dy * std::sin(-yaw);
    const double localY =  dx * std::sin(-yaw) + dy * std::cos(-yaw);
    return std::atan2(localY, localX);
}

void RacingNode::publishStop()
{
    publishCommand(0.0, MAX_BRAKE, 0.0);
}

void RacingNode::publishCommand(double throttle, double brake, double steering)
{
    std_msgs::msg::Float64 t, b, s;
    t.data = throttle; b.data = brake; s.data = steering;
    pubThrottle_->publish(t);
    pubBrake_->publish(b);
    pubSteering_->publish(s);
}

void RacingNode::publishWaypoints()
{
    std::vector<geometry_msgs::msg::Point> wps;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        wps = waypoints_;
    }
    if (wps.empty()) return;

    const rclcpp::Time now = get_clock()->now();
    visualization_msgs::msg::MarkerArray markerArray;
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.stamp    = now;
    poseArray.header.frame_id = "world";

    for (std::size_t i = 0; i < wps.size(); ++i) {
        // Orientation points toward the next waypoint (or continues from the previous).
        const double wpYaw = (i + 1 < wps.size())
            ? std::atan2(wps[i+1].y - wps[i].y, wps[i+1].x - wps[i].x)
            : (i > 0 ? std::atan2(wps[i].y - wps[i-1].y, wps[i].x - wps[i-1].x) : 0.0);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, wpYaw);

        visualization_msgs::msg::Marker m;
        m.header.frame_id  = "world";
        m.header.stamp     = now;
        m.ns               = "road";
        m.id               = static_cast<int>(i);
        m.type             = visualization_msgs::msg::Marker::CYLINDER;
        m.action           = visualization_msgs::msg::Marker::ADD;
        m.lifetime         = rclcpp::Duration(0, 0);
        m.pose.position    = wps[i];
        m.pose.orientation = tf2::toMsg(q);
        m.scale.x = 0.4;  // diameter = 2 � 0.2 m radius (spec)
        m.scale.y = 0.4;
        m.scale.z = 0.5;
        m.color.r = 0.0f; m.color.g = 0.8f; m.color.b = 0.2f; m.color.a = 0.8f;
        markerArray.markers.push_back(m);

        geometry_msgs::msg::Pose pose;
        pose.position    = wps[i];
        pose.orientation = tf2::toMsg(q);
        poseArray.poses.push_back(pose);
    }

    pubMarkers_->publish(markerArray);
    pubWaypoints_->publish(poseArray);
}

void RacingNode::generateAdvancedWaypoints(
    const std::shared_ptr<LaserProcessing>& laserProc,
    const nav_msgs::msg::Odometry& odom)
{
    const auto centre = laserProc->trackCentreAhead();
    if (!centre.has_value()) return;

    const geometry_msgs::msg::Point& pt = centre.value();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!advancedGoals_.empty()) {
        const geometry_msgs::msg::Point& last = advancedGoals_.back();
        if (std::hypot(pt.x - last.x, pt.y - last.y) < WAYPOINT_SPACING_M) return;
    } else {
        // Accept the first point only if it is ahead of the car.
        const double yaw = geom::yawFromOdom(odom);
        const double dot = (pt.x - odom.pose.pose.position.x) * std::cos(yaw)
                         + (pt.y - odom.pose.pose.position.y) * std::sin(yaw);
        if (dot <= 0.0) return;
    }

    advancedGoals_.push_back(pt);
    RCLCPP_INFO(get_logger(), "[advanced] Waypoint #%zu at (%.2f, %.2f)",
        advancedGoals_.size(), pt.x, pt.y);
}

const char* RacingNode::stateName(MissionState s)
{
    switch (s) {
        case MissionState::IDLE:     return "IDLE";
        case MissionState::CRUISING: return "CRUISING";
        case MissionState::TURNING:  return "TURNING";
        case MissionState::COMPLETE: return "COMPLETE";
        default:                     return "UNKNOWN";
    }
}
