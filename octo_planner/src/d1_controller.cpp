#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

class D1ControllerNode : public rclcpp::Node
{
public:
  D1ControllerNode()
  : Node("d1_controller"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    target_index_(0),
    pose_adjusting_(false),
    goal_reached_(false),
    goal_proximity_latched_(false)
  {
    declare_parameter<std::string>("robot_type", "go2");
    const std::string robot_type = toLower(trim(get_parameter("robot_type").as_string()));
    const bool is_go2 = (robot_type == "go2" || robot_type == "unitree_go2");

    declare_parameter<std::string>("path_topic", "/planned_path");
    declare_parameter<std::string>("start_navigation_topic", "/start_navigation");
    declare_parameter<std::string>("stop_navigation_topic", "/stop_navigation");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("manual_cmd_vel_topic", "/web_cmd_vel");
    declare_parameter<std::string>("tracking_point_marker_topic", "/tracking_point_marker");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("path_tracking_frame", is_go2 ? "odom" : "map");
    declare_parameter<std::string>(
      "base_frame", is_go2 ? "base_link" : "base_footprint");
    declare_parameter<std::string>(
      "base_frame_candidates",
      is_go2 ? "base_link,body,base_footprint,odom" : "body,base_link,base_footprint,camera_init");
    declare_parameter<std::string>(
      "robot_center_offset_frame", is_go2 ? "base_link" : "body");
    declare_parameter<double>("robot_center_offset_x", is_go2 ? 0.0 : -0.18);
    declare_parameter<double>("robot_center_offset_y", 0.0);
    declare_parameter<double>("robot_center_offset_z", 0.0);
    declare_parameter<double>("tf_lookup_timeout_sec", is_go2 ? 0.15 : 0.05);
    declare_parameter<bool>("require_start_command", true);
    declare_parameter<double>("control_frequency", 20.0);
    declare_parameter<double>("lookahead_distance", is_go2 ? 1.2 : 0.35);
    declare_parameter<double>("tracking_point_reached_xy_tolerance", is_go2 ? 0.30 : 0.20);
    declare_parameter<double>("tracking_point_marker_scale", 0.28);
    declare_parameter<bool>("enable_tracking_debug_view", true);
    declare_parameter<int>("tracking_debug_view_size_px", 640);
    declare_parameter<double>("tracking_debug_view_pixels_per_meter", 80.0);
    declare_parameter<double>("tracking_debug_view_frequency", 10.0);
    declare_parameter<double>("goal_position_tolerance", 0.05);
    declare_parameter<double>("goal_yaw_tolerance", 0.10);
    declare_parameter<double>("linear_gain", 1.5);
    declare_parameter<double>("lateral_gain", 1.5);
    declare_parameter<double>("heading_gain", 2.5);
    declare_parameter<double>("cross_track_angular_gain", 1.0);
    declare_parameter<double>("pure_pursuit_gain", 1.0);
    declare_parameter<double>("min_tracking_linear_speed", is_go2 ? 0.22 : 0.0);
    declare_parameter<double>("turn_coupling_factor", is_go2 ? 0.15 : 0.65);
    declare_parameter<double>("final_yaw_gain", 0.5);
    declare_parameter<bool>("use_unicycle_control", is_go2);
    declare_parameter<bool>("enable_lateral_motion", true);
    declare_parameter<double>("max_linear_speed", is_go2 ? 0.50 : 0.60);
    declare_parameter<double>("min_linear_speed", 0.0);
    declare_parameter<double>("max_lateral_speed", is_go2 ? 0.35 : 0.60);
    declare_parameter<double>("max_angular_speed", is_go2 ? 1.00 : 1.50);
    declare_parameter<bool>("align_final_yaw", true);
    declare_parameter<bool>("use_path_tangent_heading", true);
    declare_parameter<double>("linear_deadband", 0.05);
    declare_parameter<double>("lateral_deadband", 0.05);
    declare_parameter<double>("angular_deadband", 0.05);
    declare_parameter<bool>("enable_tracking_diagnostic_log", true);
    declare_parameter<double>("tracking_diagnostic_log_period_sec", 1.0);

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    const auto path_topic = get_parameter("path_topic").as_string();
    const auto start_navigation_topic = get_parameter("start_navigation_topic").as_string();
    const auto stop_navigation_topic = get_parameter("stop_navigation_topic").as_string();
    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    const auto manual_cmd_vel_topic = get_parameter("manual_cmd_vel_topic").as_string();
    const auto tracking_point_marker_topic =
      get_parameter("tracking_point_marker_topic").as_string();
    const double control_frequency = get_parameter("control_frequency").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&D1ControllerNode::onPath, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&D1ControllerNode::onGoalPose, this, std::placeholders::_1));
    start_navigation_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_navigation_topic, rclcpp::QoS(10).reliable(),
      std::bind(&D1ControllerNode::onStartNavigation, this, std::placeholders::_1));
    stop_navigation_sub_ = create_subscription<std_msgs::msg::Bool>(
      stop_navigation_topic, rclcpp::QoS(10).reliable(),
      std::bind(&D1ControllerNode::onStopNavigation, this, std::placeholders::_1));
    tf_ok_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/navigation/tf_ok", rclcpp::QoS(10),
      std::bind(&D1ControllerNode::onTfOk, this, std::placeholders::_1));
    manual_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      manual_cmd_vel_topic, rclcpp::QoS(10).reliable(),
      std::bind(&D1ControllerNode::onManualCmdVel, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    tracking_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      tracking_point_marker_topic, rclcpp::QoS(1).transient_local().reliable());

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency));
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&D1ControllerNode::onControlTimer, this));
    const double debug_view_frequency = get_parameter("tracking_debug_view_frequency").as_double();
    const auto debug_period = std::chrono::duration<double>(1.0 / std::max(1.0, debug_view_frequency));
    debug_view_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(debug_period),
      std::bind(&D1ControllerNode::renderTrackingDebugView, this));

    RCLCPP_INFO(
      get_logger(),
      "path controller started (robot_type=%s). path=%s cmd_vel=%s map_frame=%s "
      "base_frame=%s candidates=%s require_start_command=%s",
      robot_type.c_str(), path_topic.c_str(), cmd_vel_topic.c_str(),
      get_parameter("map_frame").as_string().c_str(),
      get_parameter("base_frame").as_string().c_str(),
      get_parameter("base_frame_candidates").as_string().c_str(),
      get_parameter("require_start_command").as_bool() ? "true" : "false");
  }

private:
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latched_goal_pose_ = *msg;
    has_latched_goal_pose_ = true;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      pending_plan_.clear();
      clearActivePlan();
      RCLCPP_WARN(get_logger(), "Received empty planned_path.");
      return;
    }

    if (get_parameter("require_start_command").as_bool()) {
      pending_plan_ = msg->poses;
      clearActivePlan();
      publishCmd(geometry_msgs::msg::Twist());
      RCLCPP_INFO(
        get_logger(),
        "Received planned_path with %zu poses. Waiting for /start_navigation confirmation.",
        pending_plan_.size());
      return;
    }

    activatePlan(msg->poses);
  }

  void onStartNavigation(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      stopNavigation("Navigation start denied/cancelled. Holding position.");
      return;
    }

    if (pending_plan_.empty()) {
      RCLCPP_WARN(get_logger(), "Start navigation requested, but no pending planned_path is available.");
      return;
    }

    try {
      activatePlan(pending_plan_);
      pending_plan_.clear();
    } catch (const std::exception & ex) {
      stopNavigation("Start navigation failed. Holding position.");
      RCLCPP_ERROR(get_logger(), "Start navigation exception: %s", ex.what());
    }
  }

  void onStopNavigation(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    stopNavigation("Stop navigation requested. Path tracking aborted and zero velocity sent.");
  }

  void onTfOk(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      tf_lost_stop_sent_ = false;
      return;
    }
    if (tf_lost_stop_sent_ || !isNavigationActive()) {
      return;
    }
    tf_lost_stop_sent_ = true;
    stopNavigation("TF_LOST: hold position (zero velocity, stop tracking).");
  }

  void onManualCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (isNavigationActive() && isNonZeroTwist(*msg)) {
      pending_plan_.clear();
      clearActivePlan();
      RCLCPP_INFO(
        get_logger(),
        "Manual web velocity received while navigating. Path tracking aborted without zero burst.");
    } else if (isNavigationActive()) {
      return;
    }

    publishCmd(*msg);
  }

  void activatePlan(const std::vector<geometry_msgs::msg::PoseStamped> & plan)
  {
    global_plan_ = plan;
    if (!snapPlanToTrackingFrame()) {
      global_plan_.clear();
      RCLCPP_ERROR(
        get_logger(),
        "Failed to snap planned_path to tracking frame '%s'. Navigation not started.",
        tracking_frame_id_.c_str());
      return;
    }
    if (!latchNavigationGoalInMap()) {
      global_plan_.clear();
      RCLCPP_ERROR(get_logger(), "Failed to latch navigation goal in map frame.");
      return;
    }
    target_index_ = findInitialTargetIndexOnPath();
    pose_adjusting_ = false;
    goal_proximity_latched_ = false;
    goal_reached_ = global_plan_.empty();
    publishTrackingPointMarker();
    RCLCPP_INFO(
      get_logger(),
      "Navigation execution started with %zu poses in frame '%s'. initial_target_index=%d",
      global_plan_.size(), tracking_frame_id_.c_str(), target_index_);
    if (get_parameter("enable_tracking_diagnostic_log").as_bool()) {
      RobotPose2D robot_pose;
      if (!lookupRobotPose2D(robot_pose)) {
        RCLCPP_WARN(get_logger(), "[NAV_DIAG] phase=START TF lookup failed; yaw fields may be invalid.");
      }
      logNavigationDiagnostic("START", robot_pose, TrackingTarget{}, geometry_msgs::msg::Twist{}, 0.0);
    }
  }

  void clearActivePlan()
  {
    global_plan_.clear();
    has_navigation_goal_ = false;
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_proximity_latched_ = false;
    goal_reached_ = true;
    clearTrackingPointMarker();
  }

  void stopNavigation(const char * log_message)
  {
    pending_plan_.clear();
    clearActivePlan();
    publishZeroBurst();
    RCLCPP_INFO(get_logger(), "%s", log_message);
  }

  bool isNavigationActive() const
  {
    return !global_plan_.empty() && !goal_reached_;
  }

  static bool isNonZeroTwist(const geometry_msgs::msg::Twist & twist)
  {
    constexpr double epsilon = 1.0e-6;
    return std::abs(twist.linear.x) > epsilon ||
           std::abs(twist.linear.y) > epsilon ||
           std::abs(twist.linear.z) > epsilon ||
           std::abs(twist.angular.x) > epsilon ||
           std::abs(twist.angular.y) > epsilon ||
           std::abs(twist.angular.z) > epsilon;
  }

  void publishZeroBurst()
  {
    const auto zero = geometry_msgs::msg::Twist();
    for (int i = 0; i < 5; ++i) {
      publishCmd(zero);
    }
  }

  void onControlTimer()
  {
    try {
      onControlTimerImpl();
    } catch (const std::exception & ex) {
      stopNavigation("Control loop exception. Path tracking aborted.");
      RCLCPP_ERROR(get_logger(), "Control loop exception: %s", ex.what());
    }
  }

  void onControlTimerImpl()
  {
    if (goal_reached_ || global_plan_.empty()) {
      return;
    }

    if (isNearNavigationGoalXY()) {
      goal_proximity_latched_ = true;
    }

    if (goal_proximity_latched_) {
      if (!pose_adjusting_) {
        RCLCPP_INFO(
          get_logger(),
          "Within goal_position_tolerance (%.3f m) of /goal_pose in map XY. Final pose adjustment.",
          get_parameter("goal_position_tolerance").as_double());
        target_index_ = static_cast<int>(global_plan_.size()) - 1;
        pose_adjusting_ = true;
      }
      trackFinalPose(navigation_goal_map_);
      return;
    }

    if (pose_adjusting_) {
      trackFinalPose(navigation_goal_map_);
      return;
    }

    TrackingTarget target;
    if (!selectTrackingTarget(target)) {
      return;
    }

    if (isFinalTrackingPointReached(target)) {
      pose_adjusting_ = true;
      goal_proximity_latched_ = true;
      RCLCPP_INFO(get_logger(), "Final tracking point reached. Switching to final pose adjustment.");
      trackFinalPose(navigation_goal_map_);
      return;
    }

    geometry_msgs::msg::Twist cmd_vel;
    double heading_error = 0.0;
    if (!computeTrackingHeadingError(target, heading_error)) {
      return;
    }
    if (get_parameter("use_unicycle_control").as_bool()) {
      if (!computeUnicycleCmd(target, heading_error, cmd_vel)) {
        return;
      }
    } else {
      const double linear_gain = get_parameter("linear_gain").as_double();
      const double lateral_gain = get_parameter("lateral_gain").as_double();
      const double heading_gain = get_parameter("heading_gain").as_double();
      const double cross_track_angular_gain =
        get_parameter("cross_track_angular_gain").as_double();
      const bool enable_lateral_motion = get_parameter("enable_lateral_motion").as_bool();
      cmd_vel.linear.x = clamp(
        target.base_x * linear_gain,
        -get_parameter("max_linear_speed").as_double(),
        get_parameter("max_linear_speed").as_double());
      if (enable_lateral_motion) {
        cmd_vel.linear.y = clamp(
          target.base_y * lateral_gain,
          -get_parameter("max_lateral_speed").as_double(),
          get_parameter("max_lateral_speed").as_double());
      } else {
        cmd_vel.linear.y = 0.0;
      }
      cmd_vel.angular.z = clamp(
        heading_error * heading_gain + target.base_y * cross_track_angular_gain,
        -get_parameter("max_angular_speed").as_double(),
        get_parameter("max_angular_speed").as_double());
      cmd_vel.linear.x = applyDeadband(
        cmd_vel.linear.x, get_parameter("linear_deadband").as_double());
      cmd_vel.linear.y = applyDeadband(
        cmd_vel.linear.y, get_parameter("lateral_deadband").as_double());
      cmd_vel.angular.z = applyDeadband(
        cmd_vel.angular.z, get_parameter("angular_deadband").as_double());
    }
    if (get_parameter("enable_tracking_diagnostic_log").as_bool()) {
      RobotPose2D robot_pose;
      if (lookupRobotPose2D(robot_pose)) {
        logNavigationDiagnostic("TRACK", robot_pose, target, cmd_vel, heading_error);
      }
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Track target in %s: x=%.3f y=%.3f heading_error=%.3f cmd=(%.3f, %.3f, %.3f)",
        get_parameter("base_frame").as_string().c_str(),
        target.base_x,
        target.base_y,
        heading_error,
        cmd_vel.linear.x,
        cmd_vel.linear.y,
        cmd_vel.angular.z);
    }
    publishCmd(cmd_vel);
  }

  struct RobotPose2D
  {
    double x;
    double y;
    double z;
    double yaw;
  };

  struct TrackingTarget
  {
    double base_x;
    double base_y;
  };

  bool isFinalTrackingPointReached(const TrackingTarget & target) const
  {
    if (global_plan_.empty() || target_index_ != static_cast<int>(global_plan_.size()) - 1) {
      return false;
    }

    const double target_xy_dist = std::hypot(target.base_x, target.base_y);
    return target_xy_dist < get_parameter("tracking_point_reached_xy_tolerance").as_double();
  }

  int findNearestPlanIndexXY(const RobotPose2D & robot_pose) const
  {
    int nearest_index = 0;
    double nearest_dist_sq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < global_plan_.size(); ++i) {
      const double d = xyDistanceToPlanPoint(robot_pose, static_cast<int>(i));
      const double dist_sq = d * d;
      if (dist_sq < nearest_dist_sq) {
        nearest_dist_sq = dist_sq;
        nearest_index = static_cast<int>(i);
      }
    }
    return nearest_index;
  }

  int findLookaheadTargetIndex(int from_index, const RobotPose2D & robot_pose) const
  {
    if (global_plan_.empty()) {
      return 0;
    }
    const int last = static_cast<int>(global_plan_.size()) - 1;
    from_index = std::clamp(from_index, 0, last);
    const double lookahead = get_parameter("lookahead_distance").as_double();
    for (int i = from_index; i <= last; ++i) {
      if (xyDistanceToPlanPoint(robot_pose, i) >= lookahead) {
        return i;
      }
    }
    return last;
  }

  int findInitialTargetIndexOnPath()
  {
    if (global_plan_.empty()) {
      return 0;
    }
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      RCLCPP_WARN(get_logger(), "Failed to get robot pose. Start tracking from path index 0.");
      return 0;
    }
    const int nearest = findNearestPlanIndexXY(robot_pose);
    return findLookaheadTargetIndex(nearest, robot_pose);
  }

  bool latchNavigationGoalInMap()
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    const double tf_timeout = get_parameter("tf_lookup_timeout_sec").as_double();

    if (has_latched_goal_pose_) {
      navigation_goal_map_ = latched_goal_pose_;
    } else if (!global_plan_.empty()) {
      navigation_goal_map_ = global_plan_.back();
    } else {
      return false;
    }

    if (navigation_goal_map_.header.frame_id.empty()) {
      navigation_goal_map_.header.frame_id = map_frame;
    }
    navigation_goal_map_.header.stamp = rclcpp::Time(0);

    if (navigation_goal_map_.header.frame_id != map_frame) {
      try {
        navigation_goal_map_ = tf_buffer_.transform(
          navigation_goal_map_, map_frame, tf2::durationFromSec(tf_timeout));
        navigation_goal_map_.header.frame_id = map_frame;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
          get_logger(), "Latch goal %s -> %s failed: %s",
          navigation_goal_map_.header.frame_id.c_str(), map_frame.c_str(), ex.what());
        return false;
      }
    }

    has_navigation_goal_ = true;
    const auto & p = navigation_goal_map_.pose.position;
    const double yaw = tf2::getYaw(navigation_goal_map_.pose.orientation);
    RCLCPP_INFO(
      get_logger(),
      "Navigation goal (map): x=%.3f y=%.3f yaw=%.1f deg (from %s)",
      p.x, p.y, yaw * 180.0 / M_PI,
      has_latched_goal_pose_ ? "/goal_pose" : "planned_path end");
    return true;
  }

  bool isNearNavigationGoalXY()
  {
    if (!has_navigation_goal_) {
      return false;
    }
    RobotPose2D robot_map;
    if (!lookupRobotPoseInFrame(get_parameter("map_frame").as_string(), robot_map)) {
      return false;
    }
    const auto & g = navigation_goal_map_.pose.position;
    const double dx = g.x - robot_map.x;
    const double dy = g.y - robot_map.y;
    return std::hypot(dx, dy) < get_parameter("goal_position_tolerance").as_double();
  }

  bool computeTrackingHeadingError(const TrackingTarget & target, double & heading_error)
  {
    if (!get_parameter("use_path_tangent_heading").as_bool()) {
      heading_error = std::atan2(target.base_y, std::max(1.0e-6, target.base_x));
      return true;
    }

    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      return false;
    }

    const int idx = target_index_;
    if (idx < 0 || global_plan_.empty()) {
      heading_error = std::atan2(target.base_y, std::max(1.0e-6, target.base_x));
      return true;
    }

    int from_idx = std::max(0, idx - 1);
    const auto & p_from = global_plan_[static_cast<std::size_t>(from_idx)].pose.position;
    const auto & p_to = global_plan_[static_cast<std::size_t>(idx)].pose.position;
    double dx = p_to.x - p_from.x;
    double dy = p_to.y - p_from.y;
    if (std::hypot(dx, dy) < 1e-3 && idx + 1 < static_cast<int>(global_plan_.size())) {
      const auto & p_next = global_plan_[static_cast<std::size_t>(idx + 1)].pose.position;
      dx = p_next.x - p_to.x;
      dy = p_next.y - p_to.y;
    }
    if (std::hypot(dx, dy) < 1e-3) {
      heading_error = std::atan2(target.base_y, std::max(1.0e-6, target.base_x));
    } else {
      const double path_yaw = std::atan2(dy, dx);
      heading_error = normalizeAngle(path_yaw - robot_pose.yaw);
    }
    return true;
  }

  bool computeUnicycleCmd(
    const TrackingTarget & target,
    double heading_error,
    geometry_msgs::msg::Twist & cmd_vel) const
  {
    // TEB/DWA 差速 (v, omega)：路径切线航向为主；横向只做弱修正，避免与航向反向导致原地蹭。
    const double v_max = get_parameter("max_linear_speed").as_double();
    const double w_max = get_parameter("max_angular_speed").as_double();
    const double k_h = get_parameter("heading_gain").as_double();
    const double k_ct = get_parameter("cross_track_angular_gain").as_double();
    const double pp_gain = get_parameter("pure_pursuit_gain").as_double();
    const double linear_gain = get_parameter("linear_gain").as_double();
    const double min_vx = get_parameter("min_tracking_linear_speed").as_double();
    const double turn_coupling = get_parameter("turn_coupling_factor").as_double();
    const bool path_tangent = get_parameter("use_path_tangent_heading").as_bool();
    const double Ld = std::max(
      0.3, std::max(get_parameter("lookahead_distance").as_double(),
        std::hypot(target.base_x, target.base_y)));
    const double cross_track = target.base_y;
    const double alpha = std::atan2(target.base_y, std::max(1.0e-6, target.base_x));
    const double lookahead_dist = std::hypot(target.base_x, target.base_y);

    double vx = v_max;
    if (std::abs(heading_error) > 0.52) {
      vx *= 0.55;
    } else {
      vx *= std::max(0.55, std::cos(heading_error));
    }
    vx *= 1.0 / (1.0 + 0.25 * std::abs(cross_track));
    const double forward_need = std::abs(target.base_x) * linear_gain;
    vx = std::max(vx, std::min(v_max, std::max(0.08, forward_need)));

    const double v_eps = 0.25;
    double wz = k_h * heading_error;
    if (path_tangent) {
      // 切线模式：横向项必须与航向同号或很弱，防止 bearing 与 segment_yaw 打架
      const double wz_ct = 0.25 * std::atan2(k_ct * cross_track, v_eps + v_max * 0.5);
      if (heading_error * wz_ct >= 0.0 || std::abs(heading_error) < 0.12) {
        wz += wz_ct;
      }
    } else {
      const double wz_pp = pp_gain * 2.0 * vx * std::sin(alpha) / Ld;
      const double wz_stanley = std::atan2(k_ct * cross_track, std::abs(vx) + v_eps);
      wz += wz_pp + wz_stanley;
    }

    if (target.base_x < -0.08) {
      vx = clamp(
        -std::min(v_max * 0.35, std::abs(target.base_x) * linear_gain),
        -v_max * 0.35, -0.05);
      wz = k_h * heading_error;
    }

    wz = clamp(wz, -w_max, w_max);
    if (w_max > 1e-6 && turn_coupling > 1e-6) {
      const double turn_scale =
        1.0 - turn_coupling * std::min(1.0, std::abs(wz) / w_max);
      vx *= std::max(0.75, turn_scale);
    }
    if (min_vx > 0.0 && lookahead_dist > 0.4 && target.base_x > 0.1) {
      const double sign = (vx >= 0.0) ? 1.0 : -1.0;
      if (std::abs(vx) < min_vx) {
        vx = sign * min_vx;
      }
    }

    cmd_vel.linear.x = clamp(vx, -v_max, v_max);
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = wz;
    cmd_vel.linear.x = applyDeadband(
      cmd_vel.linear.x, get_parameter("linear_deadband").as_double());
    cmd_vel.angular.z = applyDeadband(
      cmd_vel.angular.z, get_parameter("angular_deadband").as_double());
    return true;
  }

  bool selectTrackingTarget(TrackingTarget & target)
  {
    if (global_plan_.empty()) {
      return false;
    }

    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      return false;
    }

    // 前视跟踪：不逐点“啃”完整条路径，只追前方 lookahead 处的路径点
    const int nearest = findNearestPlanIndexXY(robot_pose);
    const int from = std::max(nearest, target_index_);
    const int lookahead_idx = findLookaheadTargetIndex(from, robot_pose);
    if (lookahead_idx != target_index_) {
      target_index_ = lookahead_idx;
      publishTrackingPointMarker();
    }

    const auto & target_point = global_plan_[static_cast<std::size_t>(target_index_)].pose.position;
    const double dx_map = target_point.x - robot_pose.x;
    const double dy_map = target_point.y - robot_pose.y;
    const double cos_yaw = std::cos(robot_pose.yaw);
    const double sin_yaw = std::sin(robot_pose.yaw);
    target.base_x = cos_yaw * dx_map + sin_yaw * dy_map;
    target.base_y = -sin_yaw * dx_map + cos_yaw * dy_map;
    return true;
  }

  bool snapPlanToTrackingFrame()
  {
    if (global_plan_.empty()) {
      return true;
    }
    const std::string map_frame = get_parameter("map_frame").as_string();
    tracking_frame_id_ = get_parameter("path_tracking_frame").as_string();
    if (tracking_frame_id_.empty()) {
      tracking_frame_id_ = map_frame;
    }
    if (tracking_frame_id_ == map_frame) {
      for (auto & pose : global_plan_) {
        if (pose.header.frame_id.empty()) {
          pose.header.frame_id = map_frame;
        }
      }
      return true;
    }

    const double tf_timeout = get_parameter("tf_lookup_timeout_sec").as_double();
    for (auto & pose : global_plan_) {
      geometry_msgs::msg::PoseStamped in = pose;
      if (in.header.frame_id.empty()) {
        in.header.frame_id = map_frame;
      }
      in.header.stamp = rclcpp::Time(0);
      try {
        pose = tf_buffer_.transform(
          in, tracking_frame_id_, tf2::durationFromSec(tf_timeout));
        pose.header.frame_id = tracking_frame_id_;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
          get_logger(), "Snap path pose %s -> %s failed: %s",
          in.header.frame_id.c_str(), tracking_frame_id_.c_str(), ex.what());
        return false;
      }
    }
    RCLCPP_INFO(
      get_logger(), "planned_path snapped to '%s' (%zu poses) for navigation",
      tracking_frame_id_.c_str(), global_plan_.size());
    return true;
  }

  bool lookupRobotPoseInFrame(const std::string & track_frame, RobotPose2D & robot_pose)
  {
    std::string last_error;
    for (const auto & base_frame : getBaseFrameCandidates()) {
      try {
        const double tf_timeout = get_parameter("tf_lookup_timeout_sec").as_double();
        const auto tf = tf_buffer_.lookupTransform(
          track_frame, base_frame, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout));
        if (active_base_frame_ != base_frame) {
          active_base_frame_ = base_frame;
          RCLCPP_INFO(get_logger(), "Using robot base frame for tracking: %s", base_frame.c_str());
        }
        robot_pose.x = tf.transform.translation.x;
        robot_pose.y = tf.transform.translation.y;
        robot_pose.z = tf.transform.translation.z;
        robot_pose.yaw = tf2::getYaw(tf.transform.rotation);
        applyRobotCenterOffset(base_frame, robot_pose);
        return true;
      } catch (const tf2::TransformException & ex) {
        last_error = ex.what();
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Lookup robot pose from %s failed for all base_frame candidates. Last error: %s",
      track_frame.c_str(), last_error.c_str());
    return false;
  }

  bool lookupRobotPose2D(RobotPose2D & robot_pose)
  {
    const std::string track_frame = tracking_frame_id_.empty()
      ? get_parameter("map_frame").as_string()
      : tracking_frame_id_;
    return lookupRobotPoseInFrame(track_frame, robot_pose);
  }

  double xyDistanceToPlanPoint(const RobotPose2D & robot_pose, int index) const
  {
    const auto & point = global_plan_[static_cast<std::size_t>(index)].pose.position;
    const double dx = point.x - robot_pose.x;
    const double dy = point.y - robot_pose.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  void publishTrackingPointMarker()
  {
    if (global_plan_.empty()) {
      clearTrackingPointMarker();
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = tracking_frame_id_.empty()
      ? get_parameter("map_frame").as_string()
      : tracking_frame_id_;
    marker.header.stamp = now();
    marker.ns = "d1_tracking_point";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = global_plan_[static_cast<std::size_t>(target_index_)].pose;
    const double scale = get_parameter("tracking_point_marker_scale").as_double();
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = 0.1f;
    marker.color.g = 0.65f;
    marker.color.b = 1.0f;
    marker.color.a = 0.95f;
    tracking_marker_pub_->publish(marker);
  }

  void clearTrackingPointMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = tracking_frame_id_.empty()
      ? get_parameter("map_frame").as_string()
      : tracking_frame_id_;
    marker.header.stamp = now();
    marker.ns = "d1_tracking_point";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    tracking_marker_pub_->publish(marker);
  }

  void renderTrackingDebugView()
  {
    try {
      renderTrackingDebugViewImpl();
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "OpenCV tracking debug view exception: %s", ex.what());
    }
  }

  void renderTrackingDebugViewImpl()
  {
    if (!get_parameter("enable_tracking_debug_view").as_bool() || debug_view_disabled_) {
      return;
    }
    if (global_plan_.empty()) {
      return;
    }

    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      return;
    }

    const int image_size =
      std::max(240, static_cast<int>(get_parameter("tracking_debug_view_size_px").as_int()));
    const double pixels_per_meter =
      std::max(10.0, get_parameter("tracking_debug_view_pixels_per_meter").as_double());
    const cv::Point center(image_size / 2, image_size / 2);
    cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(18, 24, 28));

    cv::line(image, cv::Point(center.x, 0), cv::Point(center.x, image_size), cv::Scalar(48, 64, 70), 1);
    cv::line(image, cv::Point(0, center.y), cv::Point(image_size, center.y), cv::Scalar(48, 64, 70), 1);
    cv::arrowedLine(
      image, center, cv::Point(center.x, center.y - 58), cv::Scalar(230, 230, 230), 2,
      cv::LINE_AA, 0, 0.25);
    cv::circle(image, center, 8, cv::Scalar(230, 230, 230), -1, cv::LINE_AA);
    cv::putText(
      image, "robot +X", cv::Point(center.x + 10, center.y - 62),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

    std::vector<cv::Point> projected_points;
    projected_points.reserve(global_plan_.size());
    for (const auto & pose : global_plan_) {
      projected_points.push_back(projectPlanPoint(robot_pose, pose, center, pixels_per_meter));
    }

    for (std::size_t i = 1; i < projected_points.size(); ++i) {
      cv::line(image, projected_points[i - 1], projected_points[i], cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
    }
    for (const auto & point : projected_points) {
      cv::circle(image, point, 3, cv::Scalar(90, 210, 90), -1, cv::LINE_AA);
    }

    if (target_index_ >= 0 && target_index_ < static_cast<int>(projected_points.size())) {
      const auto target_point = projected_points[static_cast<std::size_t>(target_index_)];
      cv::circle(image, target_point, 12, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      cv::circle(image, target_point, 4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }

    double final_yaw_error = 0.0;
    if (drawFinalGoalYaw(image, robot_pose, center, pixels_per_meter, final_yaw_error)) {
      char goal_yaw_text[160];
      std::snprintf(
        goal_yaw_text, sizeof(goal_yaw_text), "goal yaw err: %.1f deg",
        final_yaw_error * 180.0 / M_PI);
      cv::putText(
        image, goal_yaw_text,
        cv::Point(16, 108), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 230, 255), 1,
        cv::LINE_AA);
    }

    cv::putText(
      image, "tracking index: " + std::to_string(target_index_),
      cv::Point(16, 28), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(80, 190, 255), 2,
      cv::LINE_AA);
    char pose_text[160];
    std::snprintf(
      pose_text, sizeof(pose_text), "robot %s: x=%.2f y=%.2f yaw=%.1f deg",
      tracking_frame_id_.empty() ? "map" : tracking_frame_id_.c_str(),
      robot_pose.x, robot_pose.y, robot_pose.yaw * 180.0 / M_PI);
    cv::putText(
      image, pose_text,
      cv::Point(16, 56), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1,
      cv::LINE_AA);
    char cmd_text[160];
    std::snprintf(
      cmd_text, sizeof(cmd_text), "cmd vel: x=%.3f y=%.3f wz=%.3f",
      last_cmd_vel_.linear.x, last_cmd_vel_.linear.y, last_cmd_vel_.angular.z);
    cv::putText(
      image, cmd_text,
      cv::Point(16, 82), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(120, 230, 255), 1,
      cv::LINE_AA);
    cv::putText(
      image, "top = robot forward, red = current target",
      cv::Point(16, image_size - 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 200, 210), 1,
      cv::LINE_AA);

    try {
      cv::imshow("d1_controller_xy_tracking", image);
      cv::waitKey(1);
    } catch (const cv::Exception & ex) {
      debug_view_disabled_ = true;
      RCLCPP_WARN(get_logger(), "Disable OpenCV tracking debug view: %s", ex.what());
    }
  }

  cv::Point projectPlanPoint(
    const RobotPose2D & robot_pose,
    const geometry_msgs::msg::PoseStamped & pose,
    const cv::Point & center,
    double pixels_per_meter) const
  {
    const double dx_map = pose.pose.position.x - robot_pose.x;
    const double dy_map = pose.pose.position.y - robot_pose.y;
    const double cos_yaw = std::cos(robot_pose.yaw);
    const double sin_yaw = std::sin(robot_pose.yaw);
    const double base_x = cos_yaw * dx_map + sin_yaw * dy_map;
    const double base_y = -sin_yaw * dx_map + cos_yaw * dy_map;

    return cv::Point(
      static_cast<int>(std::round(center.x - base_y * pixels_per_meter)),
      static_cast<int>(std::round(center.y - base_x * pixels_per_meter)));
  }

  bool drawFinalGoalYaw(
    cv::Mat & image,
    const RobotPose2D & robot_pose,
    const cv::Point & center,
    double pixels_per_meter,
    double & yaw_error) const
  {
    if (global_plan_.empty()) {
      return false;
    }

    geometry_msgs::msg::PoseStamped final_pose =
      has_navigation_goal_ ? navigation_goal_map_ : global_plan_.back();
    const std::string track_frame = tracking_frame_id_.empty()
      ? get_parameter("map_frame").as_string()
      : tracking_frame_id_;
    if (!final_pose.header.frame_id.empty() && final_pose.header.frame_id != track_frame) {
      try {
        final_pose.header.stamp = rclcpp::Time(0);
        final_pose = tf_buffer_.transform(
          final_pose, track_frame,
          tf2::durationFromSec(get_parameter("tf_lookup_timeout_sec").as_double()));
      } catch (const tf2::TransformException &) {
        return false;
      }
    }
    const cv::Point final_point =
      projectPlanPoint(robot_pose, final_pose, center, pixels_per_meter);
    const double goal_yaw = tf2::getYaw(final_pose.pose.orientation);
    yaw_error = normalizeAngle(goal_yaw - robot_pose.yaw);
    const double goal_yaw_in_robot_frame = yaw_error;
    const double arrow_length_px = std::max(26.0, pixels_per_meter * 0.35);
    const cv::Point arrow_end(
      static_cast<int>(std::round(final_point.x - std::sin(goal_yaw_in_robot_frame) * arrow_length_px)),
      static_cast<int>(std::round(final_point.y - std::cos(goal_yaw_in_robot_frame) * arrow_length_px)));

    cv::circle(image, final_point, 10, cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
    cv::arrowedLine(
      image, final_point, arrow_end, cv::Scalar(0, 230, 255), 2,
      cv::LINE_AA, 0, 0.30);
    return true;
  }

  void trackFinalPose(const geometry_msgs::msg::PoseStamped & final_pose_map)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    RobotPose2D robot_map;
    if (!lookupRobotPoseInFrame(map_frame, robot_map)) {
      return;
    }

    geometry_msgs::msg::PoseStamped goal_map = final_pose_map;
    if (goal_map.header.frame_id.empty()) {
      goal_map.header.frame_id = map_frame;
    }
    goal_map.header.stamp = rclcpp::Time(0);
    try {
      if (goal_map.header.frame_id != map_frame) {
        goal_map = tf_buffer_.transform(
          goal_map, map_frame,
          tf2::durationFromSec(get_parameter("tf_lookup_timeout_sec").as_double()));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Transform goal %s -> %s failed: %s",
        goal_map.header.frame_id.c_str(), map_frame.c_str(), ex.what());
      return;
    }

    const double dx_map = goal_map.pose.position.x - robot_map.x;
    const double dy_map = goal_map.pose.position.y - robot_map.y;
    const double dist_map = std::hypot(dx_map, dy_map);
    const double pos_tol = get_parameter("goal_position_tolerance").as_double();
    const bool final_position_reached = dist_map < pos_tol;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.y = 0.0;
    const double linear_gain = get_parameter("linear_gain").as_double();
    const double heading_gain = get_parameter("heading_gain").as_double();
    const double cross_track_angular_gain =
      get_parameter("cross_track_angular_gain").as_double();
    const double final_yaw_gain = get_parameter("final_yaw_gain").as_double();
    const double max_linear = get_parameter("max_linear_speed").as_double();
    const double max_angular = get_parameter("max_angular_speed").as_double();

    if (!final_position_reached) {
      const double cos_y = std::cos(robot_map.yaw);
      const double sin_y = std::sin(robot_map.yaw);
      const double err_x_base = cos_y * dx_map + sin_y * dy_map;
      const double err_y_base = -sin_y * dx_map + cos_y * dy_map;
      const double goal_bearing = std::atan2(err_y_base, std::max(1.0e-6, err_x_base));
      const double Ld = std::max(0.3, std::hypot(err_x_base, err_y_base));
      double vx = clamp(err_x_base * linear_gain, -max_linear, max_linear);
      vx *= std::max(0.15, std::cos(goal_bearing));
      const double v_eps = 0.15;
      const double wz_pp = 2.0 * std::abs(vx) * std::sin(goal_bearing) / Ld;
      double wz = heading_gain * goal_bearing + wz_pp +
        cross_track_angular_gain * err_y_base / std::max(v_eps, std::abs(vx));
      cmd_vel.linear.x = applyDeadband(clamp(vx, -max_linear, max_linear),
        get_parameter("linear_deadband").as_double());
      cmd_vel.angular.z = applyDeadband(clamp(wz, -max_angular, max_angular),
        get_parameter("angular_deadband").as_double());
    }

    const bool align_final_yaw = get_parameter("align_final_yaw").as_bool();
    double final_yaw_error = 0.0;
    if (align_final_yaw) {
      if (!computeFinalYawErrorXY(final_pose_map, final_yaw_error)) {
        return;
      }
      if (final_position_reached) {
        cmd_vel.angular.z = clamp(final_yaw_error * final_yaw_gain, -max_angular, max_angular);
        cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, get_parameter("angular_deadband").as_double());
      }
    }

    const bool final_yaw_reached =
      !align_final_yaw || std::abs(final_yaw_error) < get_parameter("goal_yaw_tolerance").as_double();
    if (final_position_reached && final_yaw_reached) {
      finishNavigationAtGoal();
      return;
    }

    publishCmd(cmd_vel);
  }

  void finishNavigationAtGoal()
  {
    pending_plan_.clear();
    clearActivePlan();
    publishZeroBurst();
    RCLCPP_INFO(
      get_logger(),
      "Goal reached within position and yaw tolerances. Navigation finished and controller is idle.");
  }

  bool computeFinalYawErrorXY(
    const geometry_msgs::msg::PoseStamped & final_pose_in,
    double & yaw_error)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    RobotPose2D robot_map;
    if (!lookupRobotPoseInFrame(map_frame, robot_map)) {
      return false;
    }

    geometry_msgs::msg::PoseStamped goal_map = final_pose_in;
    if (goal_map.header.frame_id.empty()) {
      goal_map.header.frame_id = map_frame;
    }
    goal_map.header.stamp = rclcpp::Time(0);

    try {
      if (goal_map.header.frame_id != map_frame) {
        goal_map = tf_buffer_.transform(
          goal_map, map_frame,
          tf2::durationFromSec(get_parameter("tf_lookup_timeout_sec").as_double()));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Transform final pose yaw %s -> %s failed: %s",
        goal_map.header.frame_id.c_str(), map_frame.c_str(), ex.what());
      return false;
    }

    const double goal_yaw = tf2::getYaw(goal_map.pose.orientation);
    yaw_error = normalizeAngle(goal_yaw - robot_map.yaw);
    return true;
  }

  void publishCmd(geometry_msgs::msg::Twist cmd_vel)
  {
    const double min_linear = get_parameter("min_linear_speed").as_double();
    const double stop_thr = get_parameter("linear_deadband").as_double();
    if (min_linear > 0.0) {
      applyMinLinearSpeed(cmd_vel.linear.x, min_linear, stop_thr);
      applyMinLinearSpeed(cmd_vel.linear.y, min_linear, stop_thr);
    }
    last_cmd_vel_ = cmd_vel;
    cmd_pub_->publish(cmd_vel);
  }

  bool transformToBase(
    const geometry_msgs::msg::PoseStamped & pose_in,
    geometry_msgs::msg::PoseStamped & pose_out)
  {
    RobotPose2D unused_pose;
    if (active_base_frame_.empty() && !lookupRobotPose2D(unused_pose)) {
      return false;
    }
    const std::string base_frame =
      active_base_frame_.empty() ? get_parameter("base_frame").as_string() : active_base_frame_;
    const std::string track_frame = tracking_frame_id_.empty()
      ? get_parameter("map_frame").as_string()
      : tracking_frame_id_;

    geometry_msgs::msg::PoseStamped stamped = pose_in;
    if (stamped.header.frame_id.empty()) {
      stamped.header.frame_id = track_frame;
    }
    stamped.header.stamp = rclcpp::Time(0);

    try {
      pose_out = tf_buffer_.transform(
        stamped, base_frame,
        tf2::durationFromSec(get_parameter("tf_lookup_timeout_sec").as_double()));
      applyRobotCenterOffsetToRelativePose(base_frame, pose_out);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Transform %s -> %s failed: %s",
        stamped.header.frame_id.c_str(), base_frame.c_str(), ex.what());
      return false;
    }
  }

  std::vector<std::string> getBaseFrameCandidates() const
  {
    std::vector<std::string> candidates;
    const auto add_candidate = [&candidates](const std::string & frame) {
      if (!frame.empty() && std::find(candidates.begin(), candidates.end(), frame) == candidates.end()) {
        candidates.push_back(frame);
      }
    };

    add_candidate(get_parameter("base_frame").as_string());
    for (const auto & frame : splitCsv(get_parameter("base_frame_candidates").as_string())) {
      add_candidate(frame);
    }
    return candidates;
  }

  bool shouldApplyRobotCenterOffset(const std::string & frame) const
  {
    return frame == get_parameter("robot_center_offset_frame").as_string();
  }

  void applyRobotCenterOffset(const std::string & frame, RobotPose2D & robot_pose) const
  {
    if (!shouldApplyRobotCenterOffset(frame)) {
      return;
    }

    const double offset_x = get_parameter("robot_center_offset_x").as_double();
    const double offset_y = get_parameter("robot_center_offset_y").as_double();
    const double offset_z = get_parameter("robot_center_offset_z").as_double();
    const double cos_yaw = std::cos(robot_pose.yaw);
    const double sin_yaw = std::sin(robot_pose.yaw);
    robot_pose.x += cos_yaw * offset_x - sin_yaw * offset_y;
    robot_pose.y += sin_yaw * offset_x + cos_yaw * offset_y;
    robot_pose.z += offset_z;
  }

  void applyRobotCenterOffsetToRelativePose(
    const std::string & frame,
    geometry_msgs::msg::PoseStamped & pose) const
  {
    if (!shouldApplyRobotCenterOffset(frame)) {
      return;
    }

    pose.pose.position.x -= get_parameter("robot_center_offset_x").as_double();
    pose.pose.position.y -= get_parameter("robot_center_offset_y").as_double();
    pose.pose.position.z -= get_parameter("robot_center_offset_z").as_double();
  }

  static std::vector<std::string> splitCsv(const std::string & text)
  {
    std::vector<std::string> parts;
    std::string current;
    for (const char ch : text) {
      if (ch == ',') {
        const auto trimmed = trim(current);
        if (!trimmed.empty()) {
          parts.push_back(trimmed);
        }
        current.clear();
      } else {
        current.push_back(ch);
      }
    }

    const auto trimmed = trim(current);
    if (!trimmed.empty()) {
      parts.push_back(trimmed);
    }
    return parts;
  }

  static std::string trim(const std::string & text)
  {
    std::size_t first = 0;
    while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first]))) {
      ++first;
    }

    std::size_t last = text.size();
    while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1]))) {
      --last;
    }
    return text.substr(first, last - first);
  }

  static std::string toLower(std::string text)
  {
    for (char & ch : text) {
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
  }

  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(max_value, value));
  }

  static double applyDeadband(double value, double deadband)
  {
    return std::abs(value) < deadband ? 0.0 : value;
  }

  static void applyMinLinearSpeed(double & value, double min_speed, double stop_threshold)
  {
    if (min_speed <= 0.0) {
      return;
    }
    if (std::abs(value) <= stop_threshold) {
      value = 0.0;
      return;
    }
    if (std::abs(value) < min_speed) {
      value = (value >= 0.0 ? 1.0 : -1.0) * min_speed;
    }
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double radToDeg(double rad)
  {
    return rad * 180.0 / M_PI;
  }

  bool lookupFrameYawRad(const std::string & parent, const std::string & child, double & yaw_out)
  {
    try {
      const double tf_timeout = get_parameter("tf_lookup_timeout_sec").as_double();
      const auto tf = tf_buffer_.lookupTransform(
        parent, child, tf2::TimePointZero, tf2::durationFromSec(tf_timeout));
      yaw_out = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void logNavigationDiagnostic(
    const char * phase,
    const RobotPose2D & robot_pose,
    const TrackingTarget & target,
    const geometry_msgs::msg::Twist & cmd_vel,
    double heading_error_rad = 0.0)
  {
    const double period_sec = get_parameter("tracking_diagnostic_log_period_sec").as_double();
    const int throttle_ms = static_cast<int>(std::max(0.2, period_sec) * 1000.0);

    const std::string map_frame = get_parameter("map_frame").as_string();
    const std::string base_frame = active_base_frame_.empty() ?
      get_parameter("base_frame").as_string() : active_base_frame_;

    double map_odom_yaw = std::numeric_limits<double>::quiet_NaN();
    double odom_base_yaw = std::numeric_limits<double>::quiet_NaN();
    lookupFrameYawRad(map_frame, "odom", map_odom_yaw);
    lookupFrameYawRad("odom", base_frame, odom_base_yaw);

    if (global_plan_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "[NAV_DIAG] phase=%s plan=empty base_frame=%s",
        phase, base_frame.c_str());
      return;
    }

    const int plan_size = static_cast<int>(global_plan_.size());
    const int tgt_idx = std::clamp(target_index_, 0, plan_size - 1);
    const int seg_from = std::max(0, tgt_idx - 1);
    const int seg_to = (tgt_idx == 0 && plan_size > 1) ? 1 : tgt_idx;
    const auto & p_from = global_plan_[static_cast<std::size_t>(seg_from)].pose.position;
    const auto & p_to = global_plan_[static_cast<std::size_t>(seg_to)].pose.position;
    const auto & tgt = global_plan_[static_cast<std::size_t>(tgt_idx)].pose.position;

    const double path_seg_yaw = std::atan2(p_to.y - p_from.y, p_to.x - p_from.x);
    const double dx_map = tgt.x - robot_pose.x;
    const double dy_map = tgt.y - robot_pose.y;
    const double bearing = std::atan2(dy_map, dx_map);
    const double yaw_err_seg = normalizeAngle(path_seg_yaw - robot_pose.yaw);
    const double yaw_err_bearing = normalizeAngle(bearing - robot_pose.yaw);

    char map_odom_yaw_str[24] = "NA";
    char odom_base_yaw_str[24] = "NA";
    if (!std::isnan(map_odom_yaw)) {
      std::snprintf(
        map_odom_yaw_str, sizeof(map_odom_yaw_str), "%.1f", radToDeg(map_odom_yaw));
    }
    if (!std::isnan(odom_base_yaw)) {
      std::snprintf(
        odom_base_yaw_str, sizeof(odom_base_yaw_str), "%.1f", radToDeg(odom_base_yaw));
    }

    if (std::strcmp(phase, "START") == 0) {
      RCLCPP_INFO(
        get_logger(),
        "[NAV_DIAG] phase=START plan_pts=%d init_tgt=%d/%d "
        "robot_map=(%.3f,%.3f,%.3f) yaw_deg=%.1f "
        "path_seg_yaw_deg=%.1f bearing_deg=%.1f yaw_err_vs_seg_deg=%.1f yaw_err_vs_bearing_deg=%.1f "
        "tf map_odom_yaw_deg=%s odom_%s_yaw_deg=%s base_frame=%s | 复制本行发给我",
        plan_size, tgt_idx, plan_size - 1,
        robot_pose.x, robot_pose.y, robot_pose.z, radToDeg(robot_pose.yaw),
        radToDeg(path_seg_yaw), radToDeg(bearing), radToDeg(yaw_err_seg), radToDeg(yaw_err_bearing),
        map_odom_yaw_str, base_frame.c_str(), odom_base_yaw_str, base_frame.c_str());
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), throttle_ms,
      "[NAV_DIAG] phase=TRACK tgt=%d/%d robot_map=(%.3f,%.3f) yaw_deg=%.1f "
      "tgt_map=(%.3f,%.3f) dist=%.3f "
      "path_seg_yaw_deg=%.1f bearing_deg=%.1f yaw_err_vs_seg_deg=%.1f yaw_err_vs_bearing_deg=%.1f "
      "base_%s x=%.3f y=%.3f heading_err_deg=%.1f "
      "cmd_vel=(%.3f,%.3f,%.3f) tf map_odom_yaw_deg=%s odom_%s_yaw_deg=%s | 复制本行发给我",
      tgt_idx, plan_size - 1,
      robot_pose.x, robot_pose.y, radToDeg(robot_pose.yaw),
      tgt.x, tgt.y, std::hypot(dx_map, dy_map),
      radToDeg(path_seg_yaw), radToDeg(bearing), radToDeg(yaw_err_seg), radToDeg(yaw_err_bearing),
      base_frame.c_str(), target.base_x, target.base_y, radToDeg(heading_error_rad),
      cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
      map_odom_yaw_str, base_frame.c_str(), odom_base_yaw_str);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tf_ok_sub_;
  bool tf_lost_stop_sent_{false};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracking_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_view_timer_;

  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
  std::vector<geometry_msgs::msg::PoseStamped> pending_plan_;
  int target_index_;
  bool pose_adjusting_;
  bool goal_reached_;
  bool goal_proximity_latched_;
  bool debug_view_disabled_{false};
  std::string active_base_frame_;
  std::string tracking_frame_id_;
  geometry_msgs::msg::PoseStamped navigation_goal_map_;
  geometry_msgs::msg::PoseStamped latched_goal_pose_;
  bool has_latched_goal_pose_{false};
  bool has_navigation_goal_{false};
  geometry_msgs::msg::Twist last_cmd_vel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<D1ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
