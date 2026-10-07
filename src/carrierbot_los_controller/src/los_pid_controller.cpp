#include "carrierbot_los_controller/los_pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

using nav2_util::declare_parameter_if_not_declared;

namespace carrierbot_los_controller
{

// ---------------------------------------------------------------------------
// Khác biệt API giữa Foxy và Humble chỉ nằm ở 2 hàm dưới đây
// ---------------------------------------------------------------------------
#ifdef CARRIERBOT_ROS_FOXY
void LosPidController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  doConfigure(parent, name, tf, costmap_ros);
}

geometry_msgs::msg::TwistStamped LosPidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  return doCompute(pose, velocity);
}
#else
void LosPidController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("LosPidController: không lock được lifecycle node");
  }
  doConfigure(node, name, tf, costmap_ros);
}

geometry_msgs::msg::TwistStamped LosPidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  return doCompute(pose, velocity);
}

void LosPidController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Nav2 dùng 0.0 (NO_SPEED_LIMIT) để báo bỏ giới hạn
  if (speed_limit <= 0.0) {
    core_.setSpeedLimitScale(1.0);
  } else if (percentage) {
    core_.setSpeedLimitScale(speed_limit / 100.0);
  } else {
    const double v_des = core_.speedParams().desired_linear_vel;
    core_.setSpeedLimitScale(v_des > 0.0 ? speed_limit / v_des : 1.0);
  }
}
#endif

// ---------------------------------------------------------------------------
void LosPidController::doConfigure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = node;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declareAndLoadParams(node);

  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_period_ = controller_frequency > 0.0 ? 1.0 / controller_frequency : 0.05;

  debug_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    plugin_name_ + "/los_debug", 10);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    plugin_name_ + "/los_lookahead_point", 10);

  // Cho phép chỉnh tham số khi đang chạy:
  //   ros2 param set /controller_server FollowPath.kp 2.0
  param_cb_ = node->add_on_set_parameters_callback(
    std::bind(&LosPidController::onParamChange, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Đã cấu hình LosPidController '%s'", plugin_name_.c_str());
}

void LosPidController::declareAndLoadParams(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  const std::string p = plugin_name_ + ".";
  const LosParams los_def;
  const PidGains pid_def;
  const SpeedParams sp_def;

  // LOS
  declare_parameter_if_not_declared(node, p + "lookahead_min", rclcpp::ParameterValue(los_def.lookahead_min));
  declare_parameter_if_not_declared(node, p + "lookahead_max", rclcpp::ParameterValue(los_def.lookahead_max));
  declare_parameter_if_not_declared(node, p + "lookahead_time", rclcpp::ParameterValue(los_def.lookahead_time));
  declare_parameter_if_not_declared(node, p + "los_integral_gain", rclcpp::ParameterValue(los_def.integral_gain));
  declare_parameter_if_not_declared(node, p + "tangent_window", rclcpp::ParameterValue(los_def.tangent_window));
  declare_parameter_if_not_declared(node, p + "search_window", rclcpp::ParameterValue(los_def.search_window));
  // PID góc hướng
  declare_parameter_if_not_declared(node, p + "kp", rclcpp::ParameterValue(pid_def.kp));
  declare_parameter_if_not_declared(node, p + "ki", rclcpp::ParameterValue(pid_def.ki));
  declare_parameter_if_not_declared(node, p + "kd", rclcpp::ParameterValue(pid_def.kd));
  declare_parameter_if_not_declared(node, p + "i_max", rclcpp::ParameterValue(pid_def.i_max));
  declare_parameter_if_not_declared(node, p + "d_filter", rclcpp::ParameterValue(pid_def.d_filter));
  // Vận tốc
  declare_parameter_if_not_declared(node, p + "desired_linear_vel", rclcpp::ParameterValue(sp_def.desired_linear_vel));
  declare_parameter_if_not_declared(node, p + "max_angular_vel", rclcpp::ParameterValue(sp_def.max_angular_vel));
  declare_parameter_if_not_declared(node, p + "max_linear_accel", rclcpp::ParameterValue(sp_def.max_linear_accel));
  declare_parameter_if_not_declared(node, p + "max_linear_decel", rclcpp::ParameterValue(sp_def.max_linear_decel));
  declare_parameter_if_not_declared(node, p + "max_angular_accel", rclcpp::ParameterValue(sp_def.max_angular_accel));
  declare_parameter_if_not_declared(node, p + "use_rotate_in_place", rclcpp::ParameterValue(sp_def.use_rotate_in_place));
  declare_parameter_if_not_declared(node, p + "rotate_in_place_angle", rclcpp::ParameterValue(sp_def.rotate_in_place_angle));
  declare_parameter_if_not_declared(node, p + "rotate_in_place_exit_angle", rclcpp::ParameterValue(sp_def.rotate_in_place_exit_angle));
  declare_parameter_if_not_declared(node, p + "min_approach_vel", rclcpp::ParameterValue(sp_def.min_approach_vel));
  declare_parameter_if_not_declared(node, p + "approach_dist", rclcpp::ParameterValue(sp_def.approach_dist));
  declare_parameter_if_not_declared(node, p + "transform_tolerance", rclcpp::ParameterValue(0.1));

  LosParams lp;
  node->get_parameter(p + "lookahead_min", lp.lookahead_min);
  node->get_parameter(p + "lookahead_max", lp.lookahead_max);
  node->get_parameter(p + "lookahead_time", lp.lookahead_time);
  node->get_parameter(p + "los_integral_gain", lp.integral_gain);
  node->get_parameter(p + "tangent_window", lp.tangent_window);
  node->get_parameter(p + "search_window", lp.search_window);

  SpeedParams sp;
  node->get_parameter(p + "desired_linear_vel", sp.desired_linear_vel);
  node->get_parameter(p + "max_angular_vel", sp.max_angular_vel);
  node->get_parameter(p + "max_linear_accel", sp.max_linear_accel);
  node->get_parameter(p + "max_linear_decel", sp.max_linear_decel);
  node->get_parameter(p + "max_angular_accel", sp.max_angular_accel);
  node->get_parameter(p + "use_rotate_in_place", sp.use_rotate_in_place);
  node->get_parameter(p + "rotate_in_place_angle", sp.rotate_in_place_angle);
  node->get_parameter(p + "rotate_in_place_exit_angle", sp.rotate_in_place_exit_angle);
  node->get_parameter(p + "min_approach_vel", sp.min_approach_vel);
  node->get_parameter(p + "approach_dist", sp.approach_dist);

  PidGains g;
  node->get_parameter(p + "kp", g.kp);
  node->get_parameter(p + "ki", g.ki);
  node->get_parameter(p + "kd", g.kd);
  node->get_parameter(p + "i_max", g.i_max);
  node->get_parameter(p + "d_filter", g.d_filter);
  g.out_max = sp.max_angular_vel;

  node->get_parameter(p + "transform_tolerance", transform_tolerance_);

  std::lock_guard<std::mutex> lock(mutex_);
  core_.los().setParams(lp);
  core_.pid().setGains(g);
  core_.setSpeedParams(sp);
}

rcl_interfaces::msg::SetParametersResult LosPidController::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(mutex_);
  LosParams lp = core_.los().params();
  PidGains g = core_.pid().gains();
  SpeedParams sp = core_.speedParams();
  const std::string prefix = plugin_name_ + ".";
  bool changed = false;

  for (const auto & param : params) {
    const std::string & full = param.get_name();
    if (full.rfind(prefix, 0) != 0) {
      continue;  // không phải tham số của plugin này
    }
    const std::string n = full.substr(prefix.size());
    changed = true;

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (n == "use_rotate_in_place") {sp.use_rotate_in_place = param.as_bool();}
      continue;
    }
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      continue;
    }
    const double v = param.as_double();
    if (n == "lookahead_min") {lp.lookahead_min = v;}
    else if (n == "lookahead_max") {lp.lookahead_max = v;}
    else if (n == "lookahead_time") {lp.lookahead_time = v;}
    else if (n == "los_integral_gain") {lp.integral_gain = v;}
    else if (n == "tangent_window") {lp.tangent_window = v;}
    else if (n == "search_window") {lp.search_window = v;}
    else if (n == "kp") {g.kp = v;}
    else if (n == "ki") {g.ki = v;}
    else if (n == "kd") {g.kd = v;}
    else if (n == "i_max") {g.i_max = v;}
    else if (n == "d_filter") {g.d_filter = v;}
    else if (n == "desired_linear_vel") {sp.desired_linear_vel = v;}
    else if (n == "max_angular_vel") {sp.max_angular_vel = v;}
    else if (n == "max_linear_accel") {sp.max_linear_accel = v;}
    else if (n == "max_linear_decel") {sp.max_linear_decel = v;}
    else if (n == "max_angular_accel") {sp.max_angular_accel = v;}
    else if (n == "rotate_in_place_angle") {sp.rotate_in_place_angle = v;}
    else if (n == "rotate_in_place_exit_angle") {sp.rotate_in_place_exit_angle = v;}
    else if (n == "min_approach_vel") {sp.min_approach_vel = v;}
    else if (n == "approach_dist") {sp.approach_dist = v;}
    else if (n == "transform_tolerance") {transform_tolerance_ = v;}
  }

  if (changed) {
    g.out_max = sp.max_angular_vel;
    core_.los().setParams(lp);
    core_.pid().setGains(g);
    core_.setSpeedParams(sp);
    RCLCPP_INFO(logger_, "LOS+PID: kp=%.3f ki=%.3f kd=%.3f, lookahead=[%.2f, %.2f], v=%.2f",
      g.kp, g.ki, g.kd, lp.lookahead_min, lp.lookahead_max, sp.desired_linear_vel);
  }
  return result;
}

void LosPidController::cleanup()
{
  RCLCPP_INFO(logger_, "Dọn dẹp LosPidController '%s'", plugin_name_.c_str());
  if (auto node = node_.lock()) {
    if (param_cb_) {
      node->remove_on_set_parameters_callback(param_cb_.get());
    }
  }
  param_cb_.reset();
  debug_pub_.reset();
  carrot_pub_.reset();
}

void LosPidController::activate()
{
  debug_pub_->on_activate();
  carrot_pub_->on_activate();
  std::lock_guard<std::mutex> lock(mutex_);
  core_.reset();
  has_last_time_ = false;
}

void LosPidController::deactivate()
{
  debug_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
}

void LosPidController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_ = path;

  std::vector<Point2> pts;
  pts.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    pts.push_back({ps.pose.position.x, ps.pose.position.y});
  }
  core_.setPath(pts);

  // BT replan liên tục (mỗi ~1 s) -> chỉ reset PID/ILOS khi ĐÍCH thay đổi,
  // không reset mỗi lần nhận đường mới để tránh giật lệnh.
  if (!path.poses.empty()) {
    const auto & g = path.poses.back();
    const bool goal_changed = !has_goal_ ||
      g.header.frame_id != last_goal_.header.frame_id ||
      std::hypot(g.pose.position.x - last_goal_.pose.position.x,
                 g.pose.position.y - last_goal_.pose.position.y) > 0.25;
    if (goal_changed) {
      core_.reset();
      has_last_time_ = false;
    }
    last_goal_ = g;
    has_goal_ = true;
  }
}

bool LosPidController::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id == frame) {
    out = in;
    return true;
  }
  try {
    geometry_msgs::msg::PoseStamped latest = in;
    latest.header.stamp = builtin_interfaces::msg::Time();  // dùng TF mới nhất
    tf_->transform(latest, out, frame, tf2::durationFromSec(transform_tolerance_));
    out.header.frame_id = frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Không biến đổi được tư thế sang '%s': %s", frame.c_str(), ex.what());
  }
  return false;
}

geometry_msgs::msg::TwistStamped LosPidController::doCompute(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (plan_.poses.size() < 2) {
    throw nav2_core::PlannerException("LosPidController: chưa có đường đi (cần >= 2 điểm)");
  }

  // Tư thế robot đến ở frame của local costmap (odom); đường đi ở frame map.
  geometry_msgs::msg::PoseStamped robot;
  const std::string path_frame = plan_.header.frame_id.empty() ?
    plan_.poses.front().header.frame_id : plan_.header.frame_id;
  if (!transformPose(path_frame, pose, robot)) {
    throw nav2_core::PlannerException("LosPidController: không biến đổi được tư thế robot");
  }

  // Bước thời gian thực tế giữa hai lần gọi
  const rclcpp::Time now = clock_->now();
  double dt = control_period_;
  if (has_last_time_) {
    dt = (now - last_time_).seconds();
    if (dt <= 0.0 || dt > 5.0 * control_period_) {
      // Bị gián đoạn lâu (dừng/khởi động lại) -> reset khâu nhớ của PID
      core_.pid().reset();
      dt = control_period_;
    }
  }
  last_time_ = now;
  has_last_time_ = true;

  const double yaw = tf2::getYaw(robot.pose.orientation);
  const ControlOutput u = core_.compute(
    robot.pose.position.x, robot.pose.position.y, yaw, velocity.linear.x, dt);

  if (!u.los.valid) {
    throw nav2_core::PlannerException("LosPidController: LOS không tính được hướng mong muốn");
  }

  // --- Topic debug để chỉnh hệ số ---
  if (debug_pub_ && debug_pub_->is_activated()) {
    std_msgs::msg::Float64MultiArray d;
    // [0] e_cross (m)  [1] psi_d  [2] psi  [3] e_psi (rad)
    // [4] v_cmd (m/s)  [5] w_cmd (rad/s)  [6] dist_to_goal (m)  [7] lookahead (m)
    // [8] rotating_in_place (0/1)  [9] dt (s)  [10] terminal: nhắm thẳng đích (0/1)
    d.data = {u.los.cross_track, u.los.psi_d, yaw, u.heading_error,
      u.v, u.w, u.los.dist_to_goal, u.los.lookahead,
      u.rotating_in_place ? 1.0 : 0.0, dt, u.los.terminal ? 1.0 : 0.0};
    debug_pub_->publish(d);
  }
  if (carrot_pub_ && carrot_pub_->is_activated()) {
    geometry_msgs::msg::PointStamped c;
    c.header.frame_id = path_frame;
    c.header.stamp = now;
    c.point.x = u.los.projection.x + u.los.lookahead * std::cos(u.los.path_angle);
    c.point.y = u.los.projection.y + u.los.lookahead * std::sin(u.los.path_angle);
    carrot_pub_->publish(c);
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = now;
  cmd.twist.linear.x = u.v;
  cmd.twist.angular.z = u.w;
  return cmd;
}

}  // namespace carrierbot_los_controller

PLUGINLIB_EXPORT_CLASS(carrierbot_los_controller::LosPidController, nav2_core::Controller)
