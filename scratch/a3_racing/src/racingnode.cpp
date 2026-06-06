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
    this->declare_parameter<double>("goal_tolerance", 1.5);
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

    if (state_ != MissionState::IDLE && state_ != MissionState::COMPLETE) {
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
        if (state_ == MissionState::NAVIGATING) {
            res->success = true;
            res->message = "Already running.";
            return;
        }
        currentGoal_ = 0;
        waypoints_.clear();
        state_ = MissionState::NAVIGATING;
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

        switch (state) {

        case MissionState::IDLE:
            publishStop();
            break;

        case MissionState::NAVIGATING: {
            if (!haveOdom) {
                publishStop();
                break;
            }

            // -- All goals exhausted ------------------------------------------
            if (currentGoal >= goals.size()) {
                RCLCPP_INFO(this->get_logger(), "All goals reached -- COMPLETE.");
                publishStop();
                publishWaypoints();
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = MissionState::COMPLETE;
                break;
            }

            // -- Obstacle check -----------------------------------------------
            // If anything non-wall is directly ahead, end the mission.
            if (haveLaser && laserProc) {
                if (laserProc->obstacleInFront()) {
                    RCLCPP_WARN(this->get_logger(),
                        "Obstacle detected ahead -- ending mission.");
                    publishStop();
                    publishWaypoints();
                    std::lock_guard<std::mutex> lock(mutex_);
                    state_ = MissionState::COMPLETE;
                    break;
                }
            }

            const geometry_msgs::msg::Point& goal = goals[currentGoal];
            const double dist = euclidean(odom, goal);

            // -- Goal corridor validation -------------------------------------
            // Only check goals within CORRIDOR_CHECK_DIST_M. Beyond that the
            // laser geometry is unreliable around corners and valid goals may
            // be incorrectly rejected.
            if (haveLaser && laserProc && dist <= CORRIDOR_CHECK_DIST_M) {
                if (!laserProc->goalInCorridor(goals[currentGoal])) {
                    RCLCPP_WARN(this->get_logger(),
                        "Goal %zu is outside the track corridor -- skipping.",
                        currentGoal);
                    std::lock_guard<std::mutex> lock(mutex_);
                    currentGoal_++;
                    break;
                }
            }

            // -- Overshot check -----------------------------------------------
            const double yaw     = yawFromOdom(odom);
            const double toGoalX = goal.x - odom.pose.pose.position.x;
            const double toGoalY = goal.y - odom.pose.pose.position.y;
            const double dot     = toGoalX * std::cos(yaw) + toGoalY * std::sin(yaw);
            const bool   overshot = (dot < 0.0) && (dist > goalTolerance_);

            if (dist < goalTolerance_ || overshot) {
                if (overshot)
                    RCLCPP_WARN(this->get_logger(), "Goal %zu overshot -- skipping.", currentGoal);
                else
                    RCLCPP_INFO(this->get_logger(), "Goal %zu reached.", currentGoal);

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    waypoints_.push_back(goal);
                    currentGoal_++;
                }
                publishWaypoints();
                break;
            }

            // -- Steering -----------------------------------------------------
            // Compute heading error (alpha) from current pose to active goal,
            // then apply the bicycle-model Pure Pursuit formula.
            const double alpha    = computeAlpha(odom, goal);
            const double steerRaw = std::atan2(
                2.0 * WHEELBASE_M * std::sin(alpha), dist);
            const double steering = std::clamp(steerRaw, -MAX_STEER, MAX_STEER);

            // -- Speed planning -----------------------------------------------
            // Throttle and brake are mutually exclusive.
            // On sharp corners (|alpha| > CORNER_ALPHA_RAD) apply light brake.
            // Otherwise scale throttle by cos(alpha) so tighter arcs get less
            // throttle, clamped to a minimum fraction to avoid stalling.
            double throttle = 0.0;
            double brake    = 0.0;

            if (std::abs(alpha) > CORNER_ALPHA_RAD) {
                brake = CORNER_BRAKE;
            } else {
                const double speedFactor = std::max(std::cos(alpha), MIN_SPEED_FACTOR);
                throttle = CRUISE_THROTTLE * speedFactor;
            }

            // -- Debug logging ------------------------------------------------
            RCLCPP_INFO(this->get_logger(),
                "[Nav] goal=%zu pos=(%.2f,%.2f) target=(%.2f,%.2f) "
                "dist=%.2f alpha=%.3f steer=%.3f thr=%.2f brk=%.0f",
                currentGoal,
                odom.pose.pose.position.x, odom.pose.pose.position.y,
                goal.x, goal.y,
                dist, alpha, steering, throttle, brake);

            publishCommand(throttle, brake, steering);
            break;
        }

        case MissionState::COMPLETE:
            publishStop();
            break;
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