#include "racing_track_pkg/racing_track_node.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <cmath>
#include <sstream>

// ============================================================
// Constructor / Destructor
// ============================================================

RacingTrackNode::RacingTrackNode()
    : Node("racing_track_node")
{
    // ── Declare parameters (all configurable from launch / command line) ──────
    this->declare_parameter("track_width",          8.0);
    this->declare_parameter("goal_tolerance",        0.2);
    this->declare_parameter("obstacle_range",        6.0);
    this->declare_parameter("waypoint_spacing",      3.0);
    this->declare_parameter("goal_tolerance_reached",0.5);
    this->declare_parameter("advanced",              false);

    analyser_.setNominalTrackWidth(this->get_parameter("track_width").as_double());
    analyser_.setGoalTolerance(    this->get_parameter("goal_tolerance").as_double());
    goalReachedTolerance_ = this->get_parameter("goal_tolerance_reached").as_double();
    advanced_             = this->get_parameter("advanced").as_bool();

    // ── Subscribers ───────────────────────────────────────────────────────────
    odomSub_  = this->create_subscription<nav_msgs::msg::Odometry>(
        "/orange/odom", 10,
        std::bind(&RacingTrackNode::odometryCallback, this, std::placeholders::_1));

    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/orange/laserscan", 10,
        std::bind(&RacingTrackNode::laserCallback, this, std::placeholders::_1));

    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orange/goals", 10,
        std::bind(&RacingTrackNode::goalsCallback, this, std::placeholders::_1));

    // ── Publishers ────────────────────────────────────────────────────────────
    brakePub_    = this->create_publisher<std_msgs::msg::Float64>("/orange/brake_cmd",    10);
    steeringPub_ = this->create_publisher<std_msgs::msg::Float64>("/orange/steering_cmd", 10);
    throttlePub_ = this->create_publisher<std_msgs::msg::Float64>("/orange/throttle_cmd", 10);
    waypointsPub_= this->create_publisher<geometry_msgs::msg::PoseArray>("/orange/waypoints", 10);
    markerPub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualisation_marker", 10);

    // ── Service ───────────────────────────────────────────────────────────────
    missionSrv_ = this->create_service<std_srvs::srv::SetBool>(
        "/orange/mission",
        std::bind(&RacingTrackNode::missionService, this,
                  std::placeholders::_1, std::placeholders::_2));

    // ── Start control thread ──────────────────────────────────────────────────
    controlRunning_ = true;
    controlThread_  = std::thread(&RacingTrackNode::controlLoop, this);

    RCLCPP_INFO(this->get_logger(),
        "RacingTrackNode started. advanced=%s", advanced_ ? "true" : "false");
}

RacingTrackNode::~RacingTrackNode()
{
    controlRunning_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
}

// ============================================================
// Subscriber callbacks — copy data under mutex, return immediately
// ============================================================

void RacingTrackNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latestOdom_ = *msg;
    odomReady_  = true;
}

void RacingTrackNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latestScan_ = *msg;
    scanReady_  = true;
}

void RacingTrackNode::goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Append new goals — consistent with FAQ: goals can arrive at any time
    for (const auto& pose : msg->poses) {
        goals_.push_back(pose);
    }
    RCLCPP_INFO(this->get_logger(),
        "Received %zu goals, total now %zu", msg->poses.size(), goals_.size());
}

// ============================================================
// Service callback
// ============================================================

void RacingTrackNode::missionService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        // Start mission — transition to ACQUIRING only if we were IDLE.
        MissionState expected = MissionState::IDLE;
        if (state_.compare_exchange_strong(expected, MissionState::ACQUIRING)) {
            std::lock_guard<std::mutex> lock(mutex_);
            goalIndex_      = 0;
            goalsCompleted_ = 0;
            waypoints_.clear();
            RCLCPP_INFO(this->get_logger(), "Mission started.");
        }
    } else {
        // Stop mission unconditionally.
        state_ = MissionState::IDLE;
        publishStop();
        RCLCPP_INFO(this->get_logger(), "Mission stopped by service call.");
    }

    // Build response — check if current goal is within 0.5 m of corridor centre.
    bool goalInCorridor = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (scanReady_ && odomReady_ && !goals_.empty() && goalIndex_ < goals_.size()) {
            const auto& odom = latestOdom_;
            const auto& scan = latestScan_;

            // Extract yaw from quaternion.
            tf2::Quaternion q(odom.pose.pose.orientation.x,
                              odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z,
                              odom.pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            const double carX = odom.pose.pose.position.x;
            const double carY = odom.pose.pose.position.y;

            std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
            // Convert zero/inf to 0.0 (invalid marker).
            for (auto& r : ranges) {
                if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) r = 0.0;
            }

            auto hits     = analyser_.toWorldPoints(ranges, carX, carY, yaw,
                                                    scan.angle_min, scan.angle_increment);
            auto corridor = analyser_.analyseCorridor(hits, carX, carY, yaw);

            const double gx = goals_[goalIndex_].position.x;
            const double gy = goals_[goalIndex_].position.y;
            goalInCorridor  = analyser_.isGoalInCorridor(gx, gy, corridor,
                                                         carX, carY, yaw);
        }
    }

    response->success = goalInCorridor;
    std::ostringstream oss;
    oss << "% completion: " << completionPercent() << "%";
    response->message = oss.str();
}

// ============================================================
// Control loop — runs on dedicated thread at CONTROL_PERIOD_MS
// ============================================================

void RacingTrackNode::controlLoop()
{
    while (controlRunning_.load()) {

        switch (state_.load()) {

        // ── IDLE ──────────────────────────────────────────────────────────────
        case MissionState::IDLE:
            publishStop();
            break;

        // ── ACQUIRING ─────────────────────────────────────────────────────────
        case MissionState::ACQUIRING: {
            // Snapshot sensor data under mutex.
            nav_msgs::msg::Odometry   odom;
            sensor_msgs::msg::LaserScan scan;
            bool ready = false;
            std::size_t goalIdx = 0;
            std::vector<geometry_msgs::msg::Pose> goalsCopy;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ready    = odomReady_ && scanReady_ && !goals_.empty();
                odom     = latestOdom_;
                scan     = latestScan_;
                goalIdx  = goalIndex_;
                goalsCopy = goals_;
            }

            if (!ready || goalIdx >= goalsCopy.size()) {
                publishStop();
                break; // stay in ACQUIRING until data and goals arrive
            }

            // Extract yaw from quaternion.
            tf2::Quaternion q(odom.pose.pose.orientation.x,
                              odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z,
                              odom.pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            const double carX = odom.pose.pose.position.x;
            const double carY = odom.pose.pose.position.y;

            std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
            for (auto& r : ranges) {
                if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) r = 0.0;
            }

            auto hits     = analyser_.toWorldPoints(ranges, carX, carY, yaw,
                                                    scan.angle_min, scan.angle_increment);
            auto corridor = analyser_.analyseCorridor(hits, carX, carY, yaw);

            if (advanced_) {
                // D/HD path: derive laser waypoints and prepend to goal list.
                auto laserWps = analyser_.computeWaypoints(hits, carX, carY, yaw);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    for (const auto& wp : laserWps) {
                        waypoints_.push_back(wp);
                    }
                }
                publishWaypoints();
            }

            const double gx = goalsCopy[goalIdx].position.x;
            const double gy = goalsCopy[goalIdx].position.y;

            if (!corridor.valid) {
                // Cannot validate without wall data — proceed cautiously.
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "Corridor not detected, proceeding to goal without validation.");
                state_ = MissionState::NAVIGATING;
            } else if (analyser_.isGoalInCorridor(gx, gy, corridor, carX, carY, yaw)) {
                state_ = MissionState::NAVIGATING;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "Goal [%.2f, %.2f] not in corridor, skipping.", gx, gy);
                advanceGoal(); // skip invalid goal
            }
            break;
        }

        // ── NAVIGATING ────────────────────────────────────────────────────────
        case MissionState::NAVIGATING: {
            nav_msgs::msg::Odometry     odom;
            sensor_msgs::msg::LaserScan scan;
            std::size_t goalIdx = 0;
            std::vector<geometry_msgs::msg::Pose> goalsCopy;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                odom      = latestOdom_;
                scan      = latestScan_;
                goalIdx   = goalIndex_;
                goalsCopy = goals_;
            }

            if (goalIdx >= goalsCopy.size()) {
                state_ = MissionState::COMPLETE;
                break;
            }

            tf2::Quaternion q(odom.pose.pose.orientation.x,
                              odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z,
                              odom.pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            const double carX = odom.pose.pose.position.x;
            const double carY = odom.pose.pose.position.y;
            const double goalX = goalsCopy[goalIdx].position.x;
            const double goalY = goalsCopy[goalIdx].position.y;

            // Build range vector for obstacle check.
            std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
            for (auto& r : ranges) {
                if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) r = 0.0;
            }

            // Check for obstacle before computing steering.
            if (analyser_.obstacleAhead(ranges, scan.angle_min, scan.angle_increment)) {
                RCLCPP_WARN(this->get_logger(), "Obstacle detected — stopping.");
                state_ = MissionState::OBSTACLE_STOP;
                publishStop();
                break;
            }

            const double dist     = TrackAnalyser::euclidean(carX, carY, goalX, goalY);
            const double steering = analyser_.computeSteering(carX, carY, yaw, goalX, goalY);

            // Reduce throttle as we approach the goal.
            const double throttle = (dist < COAST_DISTANCE) ? COAST_THROTTLE : MAX_THROTTLE;

            publishControl(0.0, throttle, steering);

            // Log a waypoint if advanced and waypoint store exists.
            if (advanced_) {
                std::lock_guard<std::mutex> lock(mutex_);
                // Avoid accumulating duplicates within 1 m of each other.
                bool tooClose = false;
                for (const auto& wp : waypoints_) {
                    if (TrackAnalyser::euclidean(wp.x, wp.y, carX, carY) < 1.0) {
                        tooClose = true; break;
                    }
                }
                if (!tooClose) {
                    waypoints_.push_back({carX, carY});
                }
            }

            if (dist < goalReachedTolerance_) {
                RCLCPP_INFO(this->get_logger(), "Goal %zu reached.", goalIdx);
                advanceGoal();
            }
            break;
        }

        // ── OBSTACLE_STOP ─────────────────────────────────────────────────────
        case MissionState::OBSTACLE_STOP: {
            publishStop();
            sensor_msgs::msg::LaserScan scan;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                scan = latestScan_;
            }
            std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
            for (auto& r : ranges) {
                if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) r = 0.0;
            }
            if (!analyser_.obstacleAhead(ranges, scan.angle_min, scan.angle_increment)) {
                RCLCPP_INFO(this->get_logger(), "Path clear — resuming.");
                state_ = MissionState::NAVIGATING;
            }
            break;
        }

        // ── COMPLETE ──────────────────────────────────────────────────────────
        case MissionState::COMPLETE:
            publishStop();
            publishWaypoints();
            RCLCPP_INFO(this->get_logger(), "Mission complete — all goals reached.");
            state_ = MissionState::IDLE;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(CONTROL_PERIOD_MS));
    }
}

// ============================================================
// Publishers
// ============================================================

void RacingTrackNode::publishStop()
{
    publishControl(MAX_BRAKE, 0.0, 0.0);
}

void RacingTrackNode::publishControl(double brake, double throttle, double steering)
{
    std_msgs::msg::Float64 brakeMsg, throttleMsg, steeringMsg;
    brakeMsg.data    = brake;
    throttleMsg.data = throttle;
    steeringMsg.data = steering;
    brakePub_->publish(brakeMsg);
    throttlePub_->publish(throttleMsg);
    steeringPub_->publish(steeringMsg);
}

void RacingTrackNode::publishWaypoints()
{
    std::vector<Point2D> wps;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        wps = waypoints_;
    }
    if (wps.empty()) return;

    // ── PoseArray ─────────────────────────────────────────────────────────────
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.stamp    = this->now();
    poseArray.header.frame_id = "world";

    for (std::size_t i = 0; i < wps.size(); ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = wps[i].x;
        pose.position.y = wps[i].y;
        pose.position.z = 0.0;

        // Orientation toward next waypoint (or same as last if final waypoint).
        double yaw = 0.0;
        if (i + 1 < wps.size()) {
            yaw = std::atan2(wps[i + 1].y - wps[i].y,
                             wps[i + 1].x - wps[i].x);
        } else if (i > 0) {
            yaw = std::atan2(wps[i].y - wps[i - 1].y,
                             wps[i].x - wps[i - 1].x);
        }
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        poseArray.poses.push_back(pose);
    }
    waypointsPub_->publish(poseArray);

    // ── MarkerArray — CYLINDER per waypoint, namespace "road" ─────────────────
    visualization_msgs::msg::MarkerArray markerArray;

    // First marker: DELETE_ALL to clear stale markers.
    visualization_msgs::msg::Marker clearMarker;
    clearMarker.header.frame_id = "world";
    clearMarker.header.stamp    = this->now();
    clearMarker.ns              = "road";
    clearMarker.action          = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(clearMarker);

    for (int i = 0; i < static_cast<int>(wps.size()); ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "world";
        m.header.stamp    = this->now();
        m.ns              = "road";
        m.id              = i;
        m.type            = visualization_msgs::msg::Marker::CYLINDER;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = wps[i].x;
        m.pose.position.y = wps[i].y;
        m.pose.position.z = 0.25; // half height so base sits on ground
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.4; // diameter = radius * 2
        m.scale.y = 0.4;
        m.scale.z = 0.5; // height per spec
        m.color.r = 0.0f;
        m.color.g = 0.8f;
        m.color.b = 0.2f;
        m.color.a = 0.8f;
        markerArray.markers.push_back(m);
    }
    markerPub_->publish(markerArray);
}

// ============================================================
// Helpers
// ============================================================

unsigned int RacingTrackNode::completionPercent() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (goals_.empty()) return 0u;
    return static_cast<unsigned int>(
        100.0 * static_cast<double>(goalsCompleted_) / static_cast<double>(goals_.size()));
}

void RacingTrackNode::advanceGoal()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++goalsCompleted_;
    ++goalIndex_;
    if (goalIndex_ >= goals_.size()) {
        state_ = MissionState::COMPLETE;
    } else {
        state_ = MissionState::ACQUIRING;
    }
}
