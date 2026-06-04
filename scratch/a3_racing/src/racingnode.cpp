#include "racingnode.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

// ── Constructor / Destructor ─────────────────────────────────────────────────

RacingNode::RacingNode()
    : Node("racing_node"),
      hasOdom_(false),
      currentGoal_(0),
      state_(MissionState::IDLE),
      running_(true)
{
    // ── Declare parameters ───────────────────────────────────────────────────
    this->declare_parameter<double>("goal_tolerance", 1.5);
    this->declare_parameter<bool>  ("advanced",       false);

    goalTolerance_ = this->get_parameter("goal_tolerance").as_double();
    advanced_      = this->get_parameter("advanced").as_bool();

    RCLCPP_INFO(this->get_logger(),
        "RacingNode starting — tolerance=%.2f advanced=%s",
        goalTolerance_, advanced_ ? "true" : "false");

    // ── Subscribers ──────────────────────────────────────────────────────────
    subOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/orange/odom", 10,
        std::bind(&RacingNode::odomCallback, this, std::placeholders::_1));

    subLaser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/orange/laserscan", 10,
        std::bind(&RacingNode::laserCallback, this, std::placeholders::_1));

    subGoals_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orange/goals", 10,
        std::bind(&RacingNode::goalsCallback, this, std::placeholders::_1));

    // ── Publishers ───────────────────────────────────────────────────────────
    pubThrottle_ = this->create_publisher<std_msgs::msg::Float64>(
        "/orange/throttle_cmd", 10);

    pubBrake_ = this->create_publisher<std_msgs::msg::Float64>(
        "/orange/brake_cmd", 10);

    pubSteering_ = this->create_publisher<std_msgs::msg::Float64>(
        "/orange/steering_cmd", 10);

    pubMarkers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualisation_marker", 10);

    pubWaypoints_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/orange/waypoints", 10);

    // ── Service ──────────────────────────────────────────────────────────────
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "/orange/mission",
        std::bind(&RacingNode::missionService, this,
                  std::placeholders::_1, std::placeholders::_2));

    // ── Control thread ────────────────────────────────────────────────────────
    controlThread_ = std::thread(&RacingNode::controlLoop, this);

    RCLCPP_INFO(this->get_logger(), "RacingNode ready.");
}

RacingNode::~RacingNode()
{
    running_.store(false);
    if (controlThread_.joinable()) controlThread_.join();
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void RacingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_    = *msg;
        hasOdom_ = true;
    }
    // Forward to laser processing — laserProc_ may not exist yet on first call.
    if (laserProc_) {
        laserProc_->newOdom(*msg);
    }
}

void RacingNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!laserProc_) {
        // First scan — create the LaserProcessing object.
        laserProc_ = std::make_unique<LaserProcessing>(*msg);
        // If we already have odom, give it to the new object.
        std::lock_guard<std::mutex> lock(mutex_);
        if (hasOdom_) laserProc_->newOdom(odom_);
    } else {
        laserProc_->newScan(*msg);
    }
}

void RacingNode::goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Only update goals when IDLE so a live mission is not disrupted.
    if (state_ != MissionState::IDLE && state_ != MissionState::COMPLETE) {
        RCLCPP_WARN(this->get_logger(),
            "Goals received while mission active — ignored. Stop mission first.");
        return;
    }

    goals_.clear();
    for (const auto& pose : msg->poses) {
        goals_.push_back(pose.position);
    }
    currentGoal_ = 0;

    RCLCPP_INFO(this->get_logger(),
        "Received %zu goals.", goals_.size());
}

void RacingNode::missionService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (req->data) {
        // ── Start mission ────────────────────────────────────────────────────
        if (goals_.empty()) {
            res->success = false;
            res->message = "No goals available. Publish goals to /orange/goals first.";
            return;
        }

        if (state_ == MissionState::NAVIGATING || state_ == MissionState::STOPPED) {
            res->success = true;
            res->message = "Mission already running.";
            return;
        }

        // Reset goal index when starting fresh (not resuming from STOPPED).
        if (state_ == MissionState::IDLE) {
            currentGoal_ = 0;
            waypoints_.clear();
        }

        state_ = MissionState::NAVIGATING;
        RCLCPP_INFO(this->get_logger(),
            "Mission STARTED — %zu goals.", goals_.size());

    } else {
        // ── Stop mission ─────────────────────────────────────────────────────
        state_ = MissionState::IDLE;
        RCLCPP_INFO(this->get_logger(), "Mission STOPPED by service call.");
    }

    // Build response.
    const bool nearCentre = [&]() -> bool {
        if (!hasOdom_ || goals_.empty() || currentGoal_ >= goals_.size()) return false;
        if (!laserProc_) return false;
        return laserProc_->goalInCorridor(goals_[currentGoal_]);
    }();

    const std::size_t total     = goals_.size();
    const std::size_t completed = (state_ == MissionState::COMPLETE) ? total : currentGoal_;
    const double pct = total > 0 ? (static_cast<double>(completed) / total * 100.0) : 0.0;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << pct
       << "% complete (goal " << completed << "/" << total << ")";

    res->success = nearCentre;
    res->message = ss.str();
}

// ── Control thread ────────────────────────────────────────────────────────────

void RacingNode::controlLoop()
{
    rclcpp::Rate rate(CONTROL_HZ);

    while (running_.load() && rclcpp::ok()) {

        // Snapshot shared state for this iteration.
        nav_msgs::msg::Odometry            odom;
        MissionState                       state;
        std::vector<geometry_msgs::msg::Point> goals;
        std::size_t                        currentGoal;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            odom        = odom_;
            state       = state_;
            goals       = goals_;
            currentGoal = currentGoal_;
        }

        switch (state) {

        // ── IDLE ─────────────────────────────────────────────────────────────
        case MissionState::IDLE:
            publishStop();
            break;

        // ── NAVIGATING ───────────────────────────────────────────────────────
        case MissionState::NAVIGATING: {
            if (!hasOdom_ || !laserProc_) {
                publishStop();
                break;
            }

            // Check for non-wall obstacle in the forward corridor.
            if (laserProc_->obstacleInFront()) {
                RCLCPP_WARN(this->get_logger(), "Obstacle detected — stopping.");
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    state_ = MissionState::STOPPED;
                }
                publishStop();
                break;
            }

            if (currentGoal >= goals.size()) {
                // All goals visited.
                RCLCPP_INFO(this->get_logger(), "All goals reached — mission COMPLETE.");
                publishStop();
                publishWaypoints();
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = MissionState::COMPLETE;
                break;
            }

            const geometry_msgs::msg::Point& goal = goals[currentGoal];

            // In D/HD mode, also check for laser-derived waypoints ahead.
            if (advanced_ && laserProc_) {
                auto centre = laserProc_->trackCentreAhead();
                if (centre.has_value()) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    // Store waypoint if it is far enough from the last stored one.
                    bool addWp = waypoints_.empty();
                    if (!addWp) {
                        const auto& last = waypoints_.back();
                        const double dx = centre->x - last.x;
                        const double dy = centre->y - last.y;
                        addWp = std::sqrt(dx*dx + dy*dy) >= WAYPOINT_SPACING_M;
                    }
                    if (addWp) {
                        waypoints_.push_back(*centre);
                    }
                }
            }

            // Validate goal is within the free corridor (P/C requirement).
            const bool inCorridor = laserProc_->goalInCorridor(goal);
            if (!inCorridor) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Goal %zu not in corridor — continuing towards it.", currentGoal);
            }

            // Distance to goal.
            const double dist = euclidean(odom, goal);

            if (dist < goalTolerance_) {
                RCLCPP_INFO(this->get_logger(),
                    "Goal %zu reached (dist=%.2f m).", currentGoal, dist);

                // Store waypoint at goal position.
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    waypoints_.push_back(goal);
                    currentGoal_++;
                }
                publishWaypoints();
                break;
            }

            // Compute steering and publish motion command.
            const double steering = computeSteering(odom, goal);

            // Throttle tapers as we approach the goal.
            double throttle = CRUISE_THROTTLE;
            if (dist < SLOW_ZONE_M) {
                throttle = CRUISE_THROTTLE * (dist / SLOW_ZONE_M);
                throttle = std::max(throttle, 0.1); // keep minimum creep
            }

            publishCommand(throttle, 0.0, steering);
            break;
        }

        // ── STOPPED (obstacle) ────────────────────────────────────────────────
        case MissionState::STOPPED:
            publishStop();
            if (laserProc_ && !laserProc_->obstacleInFront()) {
                RCLCPP_INFO(this->get_logger(), "Obstacle cleared — resuming.");
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = MissionState::NAVIGATING;
            }
            break;

        // ── COMPLETE ──────────────────────────────────────────────────────────
        case MissionState::COMPLETE:
            publishStop();
            break;
        }

        rate.sleep();
    }
}

// ── Motion helpers ────────────────────────────────────────────────────────────

double RacingNode::computeSteering(const nav_msgs::msg::Odometry& odom,
                                   const geometry_msgs::msg::Point& goal) const
{
    const double yaw     = yawFromOdom(odom);
    const double bearing = std::atan2(goal.y - odom.pose.pose.position.y,
                                      goal.x - odom.pose.pose.position.x);
    const double error   = normaliseAngle(bearing - yaw);
    const double steer   = std::clamp(STEER_GAIN * error, -MAX_STEER, MAX_STEER);
    return steer;
}

void RacingNode::publishStop()
{
    publishCommand(0.0, MAX_BRAKE, 0.0);
}

void RacingNode::publishCommand(double throttle, double brake, double steering)
{
    std_msgs::msg::Float64 tMsg, bMsg, sMsg;
    tMsg.data = throttle;
    bMsg.data = brake;
    sMsg.data = steering;
    pubThrottle_->publish(tMsg);
    pubBrake_->publish(bMsg);
    pubSteering_->publish(sMsg);
}

// ── Visualisation ─────────────────────────────────────────────────────────────

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
    geometry_msgs::msg::PoseArray        poseArray;
    poseArray.header.stamp    = now;
    poseArray.header.frame_id = "world";

    for (std::size_t i = 0; i < wps.size(); ++i) {
        // Orientation of this waypoint points toward the next one.
        double yaw = 0.0;
        if (i + 1 < wps.size()) {
            yaw = std::atan2(wps[i+1].y - wps[i].y,
                             wps[i+1].x - wps[i].x);
        } else if (i > 0) {
            yaw = std::atan2(wps[i].y - wps[i-1].y,
                             wps[i].x - wps[i-1].x);
        }

        // Marker.
        markerArray.markers.push_back(
            makeWaypointMarker(wps[i], static_cast<int>(i), yaw, now));

        // Pose for PoseArray.
        geometry_msgs::msg::Pose pose;
        pose.position = wps[i];
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.orientation = tf2::toMsg(q);
        poseArray.poses.push_back(pose);
    }

    pubMarkers_->publish(markerArray);
    pubWaypoints_->publish(poseArray);
}

visualization_msgs::msg::Marker RacingNode::makeWaypointMarker(
    const geometry_msgs::msg::Point& pt,
    int id,
    double yaw_rad,
    const rclcpp::Time& stamp) const
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp    = stamp;
    m.ns              = "road";
    m.id              = id;
    m.type            = visualization_msgs::msg::Marker::CYLINDER;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.lifetime        = rclcpp::Duration(0, 0); // persist until replaced

    m.pose.position = pt;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    m.pose.orientation = tf2::toMsg(q);

    m.scale.x = 0.4;  // diameter = 2 × radius 0.2 m
    m.scale.y = 0.4;
    m.scale.z = 0.5;  // height 0.5 m

    m.color.r = 0.0f;
    m.color.g = 0.8f;
    m.color.b = 0.2f;
    m.color.a = 0.8f;

    return m;
}

// ── Static utilities ──────────────────────────────────────────────────────────

double RacingNode::yawFromOdom(const nav_msgs::msg::Odometry& odom)
{
    const auto& q      = odom.pose.pose.orientation;
    const double siny  = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
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