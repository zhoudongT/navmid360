
#include <ego_planner/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    node_->declare_parameter("fsm/flight_type", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/thresh_path_replan_meter", 0.70);
    node_->declare_parameter("fsm/path_replan_cooldown_sec", 2.0);
    node_->declare_parameter("fsm/progress_replan_timeout_sec", 3.0);
    node_->declare_parameter("fsm/progress_replan_min_drop_m", 0.35);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);
    node_->declare_parameter("fsm/goal_arrival_z_tol", 0.15);

    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/thresh_path_replan_meter", path_replan_thresh_);
    node_->get_parameter("fsm/path_replan_cooldown_sec", path_replan_cooldown_sec_);
    node_->get_parameter("fsm/progress_replan_timeout_sec", progress_replan_timeout_sec_);
    node_->get_parameter("fsm/progress_replan_min_drop_m", progress_replan_min_drop_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);
    node_->get_parameter("fsm/goal_arrival_z_tol", goal_arrival_z_tol_);

    node_->declare_parameter("fsm/global_path_enable", true);
    node_->declare_parameter("fsm/global_path_topic", std::string("/ego_global_path"));
    global_path_enable_ = node_->get_parameter("fsm/global_path_enable").as_bool();
    global_path_topic_ = node_->get_parameter("fsm/global_path_topic").as_string();

    have_trigger_ = !flag_realworld_experiment_;
    last_goal_progress_time_ = node_->now();

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    viz_refresh_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&EGOReplanFSM::refreshVisualizationCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;

    if (global_path_enable_)
    {
      rclcpp::QoS path_qos(1);
      path_qos.transient_local();
      path_qos.reliable();
      global_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          global_path_topic_,
          path_qos,
          [this](const std::shared_ptr<const nav_msgs::msg::Path> &msg)
          {
            this->globalPathCallback(msg);
          });
      RCLCPP_INFO(
          node_->get_logger(),
          "Global path mode: subscribe %s -> planGlobalTrajWaypoints",
          global_path_topic_.c_str());
    }
  }

  void EGOReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = next_wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.35, 0);

      /*** FSM状态转换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          rclcpp::spin_some(node_);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.15, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::planGlobalPath(const nav_msgs::msg::Path &path_msg)
  {
    if (path_msg.poses.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Received empty global path");
      return;
    }
    if (!have_odom_)
    {
      pending_global_path_ = path_msg;
      have_pending_global_path_ = true;
      RCLCPP_WARN(node_->get_logger(), "Global path queued: no odometry yet (%zu poses)",
                  path_msg.poses.size());
      return;
    }

    const auto &path_start = path_msg.poses.front().pose.position;
    const auto &path_end = path_msg.poses.back().pose.position;
    const Eigen::Vector3d incoming_start(path_start.x, path_start.y, path_start.z);
    const Eigen::Vector3d incoming_end(path_end.x, path_end.y, path_end.z);
    // 同一路径重复发布（心跳/重发）时不重算全局轨迹、不触发 REPLAN
    if (have_target_ && path_msg.poses.size() == last_global_path_n_ &&
        (incoming_end - last_global_path_end_).norm() < 0.05 &&
        (incoming_start - last_global_path_start_).norm() < 0.05)
    {
      return;
    }
    last_global_path_n_ = path_msg.poses.size();
    last_global_path_start_ = incoming_start;
    last_global_path_end_ = incoming_end;

    std::vector<Eigen::Vector3d> raw_wps;
    raw_wps.reserve(path_msg.poses.size());
    for (const auto &ps : path_msg.poses)
    {
      const double gz = ps.pose.position.z;
      raw_wps.emplace_back(ps.pose.position.x, ps.pose.position.y, gz);
    }

    size_t start_i = 0;
    double best_d = 1e9;
    const size_t search_end = std::max(size_t(2), raw_wps.size() / 2);
    for (size_t i = 0; i < search_end; ++i)
    {
      const double d = (raw_wps[i] - odom_pos_).norm();
      if (d < best_d)
      {
        best_d = d;
        start_i = i;
      }
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(raw_wps.size() - start_i);
    for (size_t i = start_i; i < raw_wps.size(); ++i)
      waypoints.push_back(raw_wps[i]);
    if (waypoints.empty())
      waypoints.push_back(raw_wps.back());

    constexpr size_t kMaxPts = 200;
    if (waypoints.size() > kMaxPts)
    {
      std::vector<Eigen::Vector3d> sub;
      sub.reserve(kMaxPts);
      const double step = double(waypoints.size() - 1) / double(kMaxPts - 1);
      for (size_t i = 0; i < kMaxPts; ++i)
        sub.push_back(waypoints[size_t(std::round(i * step))]);
      waypoints = std::move(sub);
    }

    init_pt_ = odom_pos_;
    planner_manager_->global_data_.last_progress_time_ = 0.0;

    const bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), waypoints,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from path!");
      return;
    }

    end_pt_ = waypoints.back();
    last_goal_dist_ = (end_pt_ - odom_pos_).head<2>().norm();
    last_goal_progress_time_ = node_->now();
    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;

    if (planner_manager_->pp_.planar_mode_)
    {
      const double ref_z = planner_manager_->global_data_.getPosition(0.0)(2);
      planner_manager_->setPlanarReferenceZ(ref_z);
    }

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.35, 0);

    constexpr double step_size_t = 0.1;
    const int i_end = std::max(
        1, int(std::floor(planner_manager_->global_data_.global_duration_ / step_size_t)));
    vector<Eigen::Vector3d> global_traj(i_end);
    for (int i = 0; i < i_end; i++)
      global_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    visualization_->displayGlobalPathList(global_traj, 0.15, 0);

    RCLCPP_INFO(
        node_->get_logger(),
        "Global path loaded: %zu poses (from idx %zu), duration %.2fs, end=(%.2f, %.2f, z=%.2f)",
        waypoints.size(), start_i, planner_manager_->global_data_.global_duration_,
        end_pt_(0), end_pt_(1), end_pt_(2));

    pending_global_replan_ = true;

    // 勿在 INIT 中直接 GEN_NEW_TRAJ（grid_map 可能尚未收到 odom）
    if (exec_state_ == INIT)
      return;

    if (exec_state_ == EXEC_TRAJ && !planner_manager_->pp_.use_raw_global_waypoints_)
      changeFSMExecState(REPLAN_TRAJ, "GLOBAL_PATH");
    else if (exec_state_ == EXEC_TRAJ)
      pending_global_replan_ = false;
    else
      changeFSMExecState(GEN_NEW_TRAJ, "GLOBAL_PATH");
  }

  bool EGOReplanFSM::isGoalReached(const Eigen::Vector3d &pos) const
  {
    const double xy_err = (end_pt_.head<2>() - pos.head<2>()).norm();
    const double z_err = std::abs(end_pt_(2) - pos(2));
    if (planner_manager_->pp_.preserve_global_path_z_)
      return xy_err < no_replan_thresh_ && z_err < goal_arrival_z_tol_;
    return (end_pt_ - pos).norm() < no_replan_thresh_;
  }

  bool EGOReplanFSM::obstacleNearAhead() const
  {
    if (!planner_manager_ || !planner_manager_->grid_map_)
      return false;

    auto map = planner_manager_->grid_map_;
    Eigen::Vector2d delta = (local_target_pt_ - odom_pos_).head<2>();
    const double dist = delta.norm();
    if (dist < 0.08)
      return false;

    const double step = 0.08;
    const int steps = std::min(40, (int)std::ceil(dist / step));
    Eigen::Vector3d dir(delta(0) / dist, delta(1) / dist, 0.0);

    for (int i = 1; i <= steps; ++i)
    {
      Eigen::Vector3d p = odom_pos_ + (step * i) * dir;
      if (planner_manager_->pp_.planar_mode_)
        p(2) = planner_manager_->pp_.cruise_z_;
      if (map->getInflateOccupancy(p))
        return true;
    }
    return false;
  }

  void EGOReplanFSM::globalPathCallback(const std::shared_ptr<const nav_msgs::msg::Path> &msg)
  {
    planGlobalPath(*msg);
  }

  void EGOReplanFSM::refreshGlobalPathVisualization()
  {
    if (!have_target_)
      return;
    const double duration = planner_manager_->global_data_.global_duration_;
    if (duration < 1e-3)
      return;

    constexpr double step_size_t = 0.12;
    const int i_end = std::max(2, int(std::floor(duration / step_size_t)));
    vector<Eigen::Vector3d> global_traj(i_end);
    for (int i = 0; i < i_end; i++)
      global_traj[i] = planner_manager_->global_data_.getPosition(i * step_size_t);
    visualization_->displayGlobalPathList(global_traj, 0.15, 0);
  }

  void EGOReplanFSM::refreshVisualizationCallback()
  {
    if (!have_target_)
      return;
    refreshGlobalPathVisualization();
    visualization_->republishCachedVisualizations(end_pt_);
  }

  void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (global_path_enable_)
      return;

    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    init_pt_ = odom_pos_;

    double gz = msg->pose.position.z;
    if (gz < 0.05)
      gz = odom_pos_(2);
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, gz);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;

    // 栅格位姿由 grid_map/odom（/drone_N_grid_odom，与点云同 stamp）更新，勿在此用 /state_estimation 覆盖

    if (have_pending_global_path_ && global_path_enable_)
    {
      nav_msgs::msg::Path pending = pending_global_path_;
      have_pending_global_path_ = false;
      planGlobalPath(pending);
    }
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    rclcpp::Clock clock(RCL_SYSTEM_TIME);  // 确保使用当前节点的时间源
    auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", rclcpp::Clock().now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((rclcpp::Clock().now() - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (rclcpp::Clock().now() - msg_time).seconds());
      return;
    }

    // 路径缓冲区初始化
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    // 计算路径持续时间
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // 生成bspline并存储
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    // 处理每条路径
    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      // 存储路径控制点和节点
      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      // 计算路径持续时间
      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      if (global_path_enable_ && (have_target_ || pending_global_replan_))
      {
        pending_global_replan_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {
      if (!have_odom_ || !have_target_)
      {
        goto force_return;
      }
      if (!planner_manager_ || !planner_manager_->grid_map_)
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "GEN_NEW_TRAJ waiting for grid_map");
        goto force_return;
      }
      if (!planner_manager_->grid_map_->odomValid())
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "GEN_NEW_TRAJ waiting for grid_map odom (cloud map not ready)");
        goto force_return;
      }

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      bool success =
          (global_path_enable_ || planner_manager_->pp_.use_raw_global_waypoints_)
              ? planFromGlobalTraj(5)
              : planFromCurrentTraj(3);

      if (!success && global_path_enable_)
      {
        syncGlobalProgressToRobot();
        success = callReboundReplan(true, false);
        if (!success)
          success = callReboundReplan(true, true);
      }

      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      const double dist_to_goal_xy = (end_pt_ - odom_pos_).head<2>().norm();
      if (dist_to_goal_xy + progress_replan_min_drop_ < last_goal_dist_)
      {
        last_goal_dist_ = dist_to_goal_xy;
        last_goal_progress_time_ = node_->now();
      }
      else if (dist_to_goal_xy > no_replan_thresh_ * 2.0 &&
               (node_->now() - last_goal_progress_time_).seconds() > progress_replan_timeout_sec_)
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "goal progress stalled: dist=%.2fm for %.1fs, force REPLAN",
            dist_to_goal_xy, (node_->now() - last_goal_progress_time_).seconds());
        changeFSMExecState(REPLAN_TRAJ, "PROGRESS_STALL");
        break;
      }

      if (shouldReplanForPathDeviation())
      {
        changeFSMExecState(REPLAN_TRAJ, "PATH_DEV");
        break;
      }

      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = rclcpp::Clock().now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3 ||
               (isGoalReached(odom_pos_) && (local_target_pt_ - end_pt_).head<2>().norm() < no_replan_thresh_))
      {
        if (t_cur > info->duration_ - 1e-2 || isGoalReached(odom_pos_))
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if (!isGoalReached(pos) && (end_pt_ - pos).head<2>().norm() > no_replan_thresh_ &&
                 t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        const double dist_to_final = (end_pt_ - odom_pos_).head<2>().norm();
        const double path_dev =
            global_path_enable_ ? globalPathLateralError() : 0.0;
        // 近终点且仍贴在全局路径上时才少 REPLAN
        if (global_path_enable_ && dist_to_final < no_replan_thresh_ * 2.0 &&
            path_dev < path_replan_thresh_ * 0.85)
        {
          /* hold EXEC_TRAJ */
        }
        else if (!planner_manager_->pp_.planar_mode_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        else
        {
          const double traj_dev =
              (odom_pos_ - info->position_traj_.evaluateDeBoorT(t_cur)).norm();
          const bool robot_moving = odom_vel_.norm() > 0.06;
          const bool obs_ahead = obstacleNearAhead();
          if (robot_moving || traj_dev > no_replan_thresh_ * 0.35 || obs_ahead ||
              path_dev > path_replan_thresh_)
            changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = rclcpp::Clock().now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    if (planner_manager_->pp_.planar_mode_)
    {
      // 局部段仍用 ref_z；起点保留里程计真实高度，避免与全局 3D 路径脱节
      if (!planner_manager_->pp_.preserve_global_path_z_)
        start_pt_(2) = planner_manager_->pp_.cruise_z_;
      start_vel_(2) = 0.0;
      start_acc_(2) = 0.0;
    }

    bool flag_random_poly_init = false;
    if (!planner_manager_->pp_.use_raw_global_waypoints_ &&
        timesOfConsecutiveStateCalls().first > 1)
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = rclcpp::Clock().now();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = (time_now - info->start_time_).seconds();

    if (planner_manager_->pp_.planar_mode_)
    {
      // 四足/平面：replan 起点始终用真实 odom，避免狗未动但轨迹时间前进导致局部路径前移
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
    }
    else
    {
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    }

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    // double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_cur = (rclcpp::Clock().now() - info->start_time_).seconds();

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    // double t_cur_global = ros::Time::now().toSec();
    double t_cur_global = rclcpp::Clock().now().seconds();

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();

      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
      refreshGlobalPathVisualization();
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  double EGOReplanFSM::globalPathLateralError()
  {
    auto &gd = planner_manager_->global_data_;
    if (gd.global_duration_ < 1e-3)
      return 0.0;

    double d_min = 1e9;
    for (double t = 0.0; t <= gd.global_duration_ + 1e-4; t += 0.05)
    {
      const double d = (gd.getPosition(t) - odom_pos_).head<2>().norm();
      if (d < d_min)
        d_min = d;
    }
    return d_min;
  }

  bool EGOReplanFSM::shouldReplanForPathDeviation()
  {
    if (!global_path_enable_ || !have_target_ || !have_odom_)
      return false;
    if (planner_manager_->global_data_.global_duration_ < 1e-3)
      return false;

    const double path_dev = globalPathLateralError();
    if (path_dev <= path_replan_thresh_)
      return false;

    const double since_last =
        (node_->now() - last_path_dev_replan_time_).seconds();
    if (since_last < path_replan_cooldown_sec_)
      return false;

    last_path_dev_replan_time_ = node_->now();
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "odom 偏离全局路径 %.2fm (>%.2fm)，触发 REPLAN",
        path_dev, path_replan_thresh_);
    return true;
  }

  void EGOReplanFSM::syncGlobalProgressToRobot()
  {
    auto &gd = planner_manager_->global_data_;
    if (gd.global_duration_ < 1e-3)
      return;

    double t_best = 0.0;
    double d_min = 1e9;
    for (double t = 0.0; t <= gd.global_duration_ + 1e-4; t += 0.05)
    {
      const double d = (gd.getPosition(t) - start_pt_).head<2>().norm();
      if (d < d_min)
      {
        d_min = d;
        t_best = t;
      }
    }
    if (d_min > path_replan_thresh_ * 0.85)
      gd.last_progress_time_ = t_best;
    else
      gd.last_progress_time_ = std::max(gd.last_progress_time_, t_best);
  }

  void EGOReplanFSM::getLocalTarget()
  {
    syncGlobalProgressToRobot();

    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ =
          planner_manager_->global_data_.global_duration_;
      t = planner_manager_->global_data_.global_duration_;
    }

    // 勿把 20m+ 外的终点当作局部目标（易生成上千控制点 → L-BFGS 栈溢出 / SIGSEGV）
    const double local_dist = (local_target_pt_ - start_pt_).norm();
    if (local_dist > planning_horizen_ * 1.15)
    {
      double t_lo = planner_manager_->global_data_.last_progress_time_;
      double t_hi = planner_manager_->global_data_.global_duration_;
      double t_best = t_hi;
      for (int k = 0; k < 32; ++k)
      {
        const double tm = 0.5 * (t_lo + t_hi);
        const double dm = (planner_manager_->global_data_.getPosition(tm) - start_pt_).norm();
        if (dm < planning_horizen_)
          t_lo = tm;
        else
          t_hi = tm;
        t_best = tm;
      }
      local_target_pt_ = planner_manager_->global_data_.getPosition(t_best);
      planner_manager_->global_data_.last_progress_time_ = t_best;
      RCLCPP_WARN(
          node_->get_logger(),
          "getLocalTarget: clipped far target (was %.1fm) to horizon ~%.1fm",
          local_dist, planning_horizen_);
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      const double t_vel = std::min(
          t, planner_manager_->global_data_.global_duration_);
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t_vel);
    }
    if (planner_manager_->pp_.planar_mode_)
    {
      double ref_z = local_target_pt_(2);
      if (planner_manager_->global_data_.global_duration_ > 1e-3)
        ref_z = planner_manager_->global_data_.getPosition(
                    planner_manager_->global_data_.last_progress_time_)(2);
      planner_manager_->setPlanarReferenceZ(ref_z);
      local_target_pt_(2) = ref_z;
      local_target_vel_(2) = 0.0;
    }
  }

} // namespace ego_planner
