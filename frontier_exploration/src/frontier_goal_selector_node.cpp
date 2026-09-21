#include "frontier_exploration/frontier_goal_selector_node.hpp"

#include <tf2/exceptions.h>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

namespace frontier_exploration
{

FrontierGoalSelector::FrontierGoalSelector()
: Node("frontier_goal_selector"),
  goal_active_(false),
  exploration_complete_(false),
  logged_completion_(false)
{
  frontier_topic_ = declare_parameter<std::string>("frontier_topic", "/frontiers");
  status_topic_ = declare_parameter<std::string>("status_topic", "/exploration_status");
  action_server_name_ = declare_parameter<std::string>("action_server_name", "/navigate_to_pose");
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
  local_radius_ = declare_parameter<double>("local_radius", 3.0);
  min_decent_size_ = declare_parameter<int>("min_decent_size", 5);
  goal_reached_distance_ = declare_parameter<double>("goal_reached_distance", 0.5);
  check_distance_period_ = declare_parameter<double>("check_distance_period", 0.1);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  frontier_sub_ = create_subscription<frontier_exploration_msgs::msg::FrontierArray>(
    frontier_topic_, 10,
    std::bind(&FrontierGoalSelector::frontierArrayCallback, this, _1));

  // Latched on the publisher side (transient_local) so we pick up the last
  // known status immediately on startup.
  rclcpp::QoS status_qos(1);
  status_qos.reliable();
  status_qos.transient_local();
  status_sub_ = create_subscription<frontier_exploration_msgs::msg::ExplorationStatus>(
    status_topic_, status_qos,
    std::bind(&FrontierGoalSelector::statusCallback, this, _1));

  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_server_name_);

  check_distance_timer_ = create_wall_timer(
    std::chrono::duration<double>(check_distance_period_),
    std::bind(&FrontierGoalSelector::checkDistanceToGoal, this));

  RCLCPP_INFO(get_logger(), "frontier_goal_selector node started.");
}

void FrontierGoalSelector::statusCallback(
  const frontier_exploration_msgs::msg::ExplorationStatus::SharedPtr msg)
{
  exploration_complete_ = msg->exploration_complete;

  if (exploration_complete_ && !logged_completion_) {
    RCLCPP_INFO(
      get_logger(), "Exploration complete (%s). Halting goal selection.",
      msg->reason.c_str());
    logged_completion_ = true;

    if (goal_active_) {
      cancelCurrentGoal();
    }
  } else if (!exploration_complete_) {
    // Frontiers reappeared (e.g. map extended) - allow logging again if it
    // completes a second time later.
    logged_completion_ = false;
  }
}

void FrontierGoalSelector::frontierArrayCallback(
  const frontier_exploration_msgs::msg::FrontierArray::SharedPtr msg)
{
  latest_frontiers_.clear();
  latest_frontiers_.reserve(msg->frontiers.size());

  for (const auto & f : msg->frontiers) {
    FrontierCandidate c;
    c.x = f.position.x;
    c.y = f.position.y;
    c.size = f.size;
    latest_frontiers_.push_back(c);
  }

  if (!goal_active_ && !exploration_complete_) {
    processAndSendGoal();
  }
}

std::optional<std::pair<double, double>> FrontierGoalSelector::getRobotPose()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
    return std::nullopt;
  }
  return std::make_pair(tf.transform.translation.x, tf.transform.translation.y);
}

std::optional<FrontierCandidate> FrontierGoalSelector::selectBestFrontier(
  const std::vector<FrontierCandidate> & frontiers, double robot_x, double robot_y)
{
  if (frontiers.empty()) {
    return std::nullopt;
  }

  std::vector<FrontierCandidate> candidates;
  for (const auto & f : frontiers) {
    if (f.size >= min_decent_size_) {
      candidates.push_back(f);
    }
  }
  if (candidates.empty()) {
    RCLCPP_INFO(get_logger(), "All frontiers are below min_decent_size (%d).", min_decent_size_);
    return std::nullopt;
  }

  auto distance = [&](const FrontierCandidate & f) {
    return std::hypot(f.x - robot_x, f.y - robot_y);
  };

  std::vector<FrontierCandidate> local_frontiers;
  for (const auto & f : candidates) {
    if (distance(f) <= local_radius_) {
      local_frontiers.push_back(f);
    }
  }

  if (!local_frontiers.empty()) {
    auto min_it = std::min_element(local_frontiers.begin(), local_frontiers.end(),
      [&](const auto & a, const auto & b) { return distance(a) < distance(b); });
    return *min_it;
  } else {
    auto min_it = std::min_element(candidates.begin(), candidates.end(),
      [&](const auto & a, const auto & b) { return distance(a) < distance(b); });
    return *min_it;
  }
}

void FrontierGoalSelector::processAndSendGoal()
{
  if (latest_frontiers_.empty() || exploration_complete_) {
    return;
  }

  auto robot_pose = getRobotPose();
  if (!robot_pose) {
    return;
  }

  auto selected = selectBestFrontier(latest_frontiers_, robot_pose->first, robot_pose->second);
  if (!selected) {
    RCLCPP_INFO(get_logger(), "No suitable frontier found.");
    return;
  }

  current_target_ = *selected;

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = global_frame_;
  goal_msg.pose.header.stamp = get_clock()->now();
  goal_msg.pose.pose.position.x = selected->x;
  goal_msg.pose.pose.position.y = selected->y;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.w = 1.0;

  RCLCPP_INFO(get_logger(), "Sending goal to Nav2: (%.2f, %.2f) size=%d",
    selected->x, selected->y, selected->size);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
    current_target_.reset();
    return;
  }

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&FrontierGoalSelector::goalResultCallback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback =
    std::bind(&FrontierGoalSelector::goalResponseCallback, this, std::placeholders::_1);

  goal_active_ = true;
  goal_handle_ = nullptr;
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void FrontierGoalSelector::goalResponseCallback(const GoalHandleNavigate::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    goal_active_ = false;
    current_target_.reset();
  } else {
    goal_handle_ = goal_handle;
  }
}

void FrontierGoalSelector::goalResultCallback(const GoalHandleNavigate::WrappedResult & result)
{
  goal_active_ = false;
  goal_handle_ = nullptr;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal reached successfully.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(get_logger(), "Goal was aborted.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(), "Goal was canceled (close enough to frontier).");
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown result code.");
      break;
  }

  current_target_.reset();

  if (!exploration_complete_ && !latest_frontiers_.empty()) {
    processAndSendGoal();
  }
}

void FrontierGoalSelector::checkDistanceToGoal()
{
  if (!goal_active_ || !current_target_) {
    return;
  }

  auto robot_pose = getRobotPose();
  if (!robot_pose) {
    return;
  }

  double dist = std::hypot(
    robot_pose->first - current_target_->x,
    robot_pose->second - current_target_->y);

  if (dist <= goal_reached_distance_) {
    RCLCPP_INFO(get_logger(), "Close enough to frontier (%.2f m), canceling goal.", dist);
    cancelCurrentGoal();
  }
}

void FrontierGoalSelector::cancelCurrentGoal()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  } else {
    RCLCPP_WARN(get_logger(), "Cannot cancel goal: no goal handle.");
  }
}

}  // namespace frontier_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<frontier_exploration::FrontierGoalSelector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}