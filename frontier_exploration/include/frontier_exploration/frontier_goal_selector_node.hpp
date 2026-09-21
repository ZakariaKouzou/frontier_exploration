#ifndef FRONTIER_EXPLORATION__FRONTIER_GOAL_SELECTOR_NODE_HPP_
#define FRONTIER_EXPLORATION__FRONTIER_GOAL_SELECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <frontier_exploration_msgs/msg/frontier_array.hpp>
#include <frontier_exploration_msgs/msg/exploration_status.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <vector>
#include <string>

namespace frontier_exploration
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct FrontierCandidate
{
  double x;
  double y;
  int size;
};

class FrontierGoalSelector : public rclcpp::Node
{
public:
  FrontierGoalSelector();

private:
  // Callbacks
  void frontierArrayCallback(
    const frontier_exploration_msgs::msg::FrontierArray::SharedPtr msg);
  void statusCallback(
    const frontier_exploration_msgs::msg::ExplorationStatus::SharedPtr msg);
  void checkDistanceToGoal();
  void goalResponseCallback(const GoalHandleNavigate::SharedPtr & goal_handle);
  void goalResultCallback(const GoalHandleNavigate::WrappedResult & result);

  // Helpers
  std::optional<std::pair<double, double>> getRobotPose();
  std::optional<FrontierCandidate> selectBestFrontier(
    const std::vector<FrontierCandidate> & frontiers, double robot_x, double robot_y);
  void processAndSendGoal();
  void cancelCurrentGoal();

  // Parameters
  std::string frontier_topic_, status_topic_, action_server_name_;
  std::string global_frame_, robot_base_frame_;
  double local_radius_, goal_reached_distance_, check_distance_period_;
  int min_decent_size_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Pub / Sub / Action
  rclcpp::Subscription<frontier_exploration_msgs::msg::FrontierArray>::SharedPtr frontier_sub_;
  rclcpp::Subscription<frontier_exploration_msgs::msg::ExplorationStatus>::SharedPtr status_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr check_distance_timer_;

  // State
  bool goal_active_;
  bool exploration_complete_;
  bool logged_completion_;
  std::optional<FrontierCandidate> current_target_;
  GoalHandleNavigate::SharedPtr goal_handle_;
  std::vector<FrontierCandidate> latest_frontiers_;
};

}  // namespace frontier_exploration

#endif  // FRONTIER_EXPLORATION__FRONTIER_GOAL_SELECTOR_NODE_HPP_