// Plugin controller Nav2: LOS + PID, thay cho Regulated Pure Pursuit.
// Biên dịch được trên ROS 2 Foxy (robot thật) và Humble (mô phỏng):
// CMakeLists.txt định nghĩa CARRIERBOT_ROS_FOXY khi ROS_DISTRO=foxy.

#ifndef CARRIERBOT_LOS_CONTROLLER__LOS_PID_CONTROLLER_HPP_
#define CARRIERBOT_LOS_CONTROLLER__LOS_PID_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "carrierbot_los_controller/los_pid_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"

namespace carrierbot_los_controller
{

class LosPidController : public nav2_core::Controller
{
public:
  LosPidController() = default;
  ~LosPidController() override = default;

#ifdef CARRIERBOT_ROS_FOXY
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;
#else
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
#endif

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

private:
  void doConfigure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros);

  geometry_msgs::msg::TwistStamped doCompute(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  void declareAndLoadParams(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in,
    geometry_msgs::msg::PoseStamped & out) const;

  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("LosPidController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::mutex mutex_;              // bảo vệ core_ khi đổi tham số lúc đang chạy
  LosPidCore core_;
  nav_msgs::msg::Path plan_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_goal_{false};
  double transform_tolerance_{0.1};
  double control_period_{0.05};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool has_last_time_{false};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
};

}  // namespace carrierbot_los_controller

#endif  // CARRIERBOT_LOS_CONTROLLER__LOS_PID_CONTROLLER_HPP_
