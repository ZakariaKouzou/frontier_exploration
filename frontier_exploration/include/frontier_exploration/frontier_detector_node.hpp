#ifndef FRONTIER_EXPLORATION__FRONTIER_DETECTOR_NODE_HPP_
#define FRONTIER_EXPLORATION__FRONTIER_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <frontier_exploration_msgs/msg/frontier.hpp>
#include <frontier_exploration_msgs/msg/frontier_array.hpp>
#include <frontier_exploration_msgs/msg/exploration_status.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <optional>
#include <memory>
#include <string>

namespace frontier_exploration
{

struct FrontierInfo
{
  double x;
  double y;
  int size;
  double dist;
  double score;
};

class FrontierDetectorNode : public rclcpp::Node
{
public:
  FrontierDetectorNode();

private:
  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void detectAndPublish();

  // Helpers
  std::optional<std::pair<double, double>> getRobotPose();
  void publishMarkers(const std::vector<FrontierInfo> & frontier_infos, int selected_idx);
  void publishFrontierArray(const std::vector<FrontierInfo> & frontier_infos, int selected_idx);
  void publishStatus(bool complete, int num_remaining, const std::string & reason);

  // Parameters
  std::string map_topic_, marker_topic_, frontier_topic_, status_topic_;
  std::string global_frame_, robot_base_frame_;
  int free_threshold_, occupied_threshold_, min_frontier_size_;
  int wall_clearance_cells_;
  double distance_weight_, size_weight_;

  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  bool last_complete_state_;
  bool have_published_status_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Pub / Sub
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<frontier_exploration_msgs::msg::FrontierArray>::SharedPtr frontier_pub_;
  rclcpp::Publisher<frontier_exploration_msgs::msg::ExplorationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace frontier_exploration

#endif  // FRONTIER_EXPLORATION__FRONTIER_DETECTOR_NODE_HPP_