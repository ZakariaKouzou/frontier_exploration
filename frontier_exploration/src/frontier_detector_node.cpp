#include "frontier_exploration/frontier_detector_node.hpp"

#include <tf2/exceptions.h>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

namespace frontier_exploration
{

FrontierDetectorNode::FrontierDetectorNode()
: Node("frontier_detector_node"),
  last_complete_state_(false),
  have_published_status_(false)
{
  // ---------------- Parameters ----------------
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  marker_topic_ = declare_parameter<std::string>("marker_topic", "/frontier_markers");
  frontier_topic_ = declare_parameter<std::string>("frontier_topic", "/frontiers");
  status_topic_ = declare_parameter<std::string>("status_topic", "/exploration_status");
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");

  free_threshold_ = declare_parameter<int>("free_threshold", 50);
  occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
  min_frontier_size_ = declare_parameter<int>("min_frontier_size", 25);
  wall_clearance_cells_ = declare_parameter<int>("wall_clearance_cells", 1);

  distance_weight_ = declare_parameter<double>("distance_weight", 1.0);
  size_weight_ = declare_parameter<double>("size_weight", 1.0);

  double update_period_sec = declare_parameter<double>("update_period_sec", 1.0);

  // ---------------- TF ----------------
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---------------- Pub / Sub ----------------
  rclcpp::QoS map_qos(1);
  map_qos.reliable();
  map_qos.transient_local();

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&FrontierDetectorNode::mapCallback, this, _1));

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  frontier_pub_ = create_publisher<frontier_exploration_msgs::msg::FrontierArray>(
    frontier_topic_, 10);

  // Latched so a late-joining goal selector still gets the last known status.
  rclcpp::QoS status_qos(1);
  status_qos.reliable();
  status_qos.transient_local();
  status_pub_ = create_publisher<frontier_exploration_msgs::msg::ExplorationStatus>(
    status_topic_, status_qos);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(update_period_sec),
    std::bind(&FrontierDetectorNode::detectAndPublish, this));

  RCLCPP_INFO(get_logger(), "frontier_detector_node started.");
}

void FrontierDetectorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
}

std::optional<std::pair<double, double>> FrontierDetectorNode::getRobotPose()
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

void FrontierDetectorNode::detectAndPublish()
{
  if (!latest_map_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "No map received yet.");
    return;
  }

  auto robot_pose = getRobotPose();
  if (!robot_pose.has_value()) {
    return;
  }
  const double rx = robot_pose->first;
  const double ry = robot_pose->second;

  const auto & grid = *latest_map_;
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const double resolution = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const auto & data = grid.data;

  auto idx = [&](int x, int y) { return y * width + x; };
  auto in_bounds = [&](int x, int y) { return x >= 0 && x < width && y >= 0 && y < height; };
  auto cell_val = [&](int x, int y) { return data[idx(x, y)]; };
  auto is_free = [&](int x, int y) {
    int8_t v = cell_val(x, y);
    return v >= 0 && v <= free_threshold_;
  };
  auto is_unknown = [&](int x, int y) { return cell_val(x, y) == -1; };
  auto is_occupied = [&](int x, int y) { return cell_val(x, y) >= occupied_threshold_; };

  static const int dx8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  static const int dy8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

  auto has_occupied_neighbor = [&](int cx, int cy, int clearance) {
    for (int dy = -clearance; dy <= clearance; ++dy) {
      for (int dx = -clearance; dx <= clearance; ++dx) {
        int nx = cx + dx;
        int ny = cy + dy;
        if (!in_bounds(nx, ny)) continue;
        if (is_occupied(nx, ny)) return true;
      }
    }
    return false;
  };

  // ---------------- 1. Raw frontier cells ----------------
  std::unordered_set<int> frontier_cells;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (!is_free(x, y)) continue;
      if (has_occupied_neighbor(x, y, wall_clearance_cells_)) continue;

      for (int k = 0; k < 8; ++k) {
        int nx = x + dx8[k];
        int ny = y + dy8[k];
        if (in_bounds(nx, ny) && is_unknown(nx, ny)) {
          frontier_cells.insert(idx(x, y));
          break;
        }
      }
    }
  }

  if (frontier_cells.empty()) {
    RCLCPP_INFO(get_logger(), "No frontiers found - exploration may be complete.");
    publishMarkers({}, -1);
    publishFrontierArray({}, -1);
    publishStatus(true, 0, "no frontiers found");
    return;
  }

  // ---------------- 2. Cluster (BFS) ----------------
  std::unordered_set<int> visited;
  std::vector<std::vector<int>> clusters;

  for (int start : frontier_cells) {
    if (visited.count(start)) continue;

    std::vector<int> cluster;
    std::queue<int> q;
    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
      int cur = q.front();
      q.pop();
      cluster.push_back(cur);

      int cx = cur % width;
      int cy = cur / width;

      for (int k = 0; k < 8; ++k) {
        int nx = cx + dx8[k];
        int ny = cy + dy8[k];
        if (!in_bounds(nx, ny)) continue;
        int nidx = idx(nx, ny);
        if (frontier_cells.count(nidx) && !visited.count(nidx)) {
          visited.insert(nidx);
          q.push(nidx);
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= min_frontier_size_) {
      clusters.push_back(std::move(cluster));
    }
  }

  if (clusters.empty()) {
    RCLCPP_INFO(get_logger(), "Frontiers found but all below min_frontier_size.");
    publishMarkers({}, -1);
    publishFrontierArray({}, -1);
    publishStatus(true, 0, "all frontiers below min_frontier_size");
    return;
  }

  // ---------------- 3. Centroids ----------------
  std::vector<FrontierInfo> frontier_infos;
  frontier_infos.reserve(clusters.size());

  for (const auto & cluster : clusters) {
    double sum_x = 0.0, sum_y = 0.0;
    for (int c : cluster) {
      sum_x += (c % width);
      sum_y += (c / width);
    }
    double mean_x = sum_x / cluster.size();
    double mean_y = sum_y / cluster.size();

    FrontierInfo info;
    info.x = origin_x + (mean_x + 0.5) * resolution;
    info.y = origin_y + (mean_y + 0.5) * resolution;
    info.size = static_cast<int>(cluster.size());
    info.dist = 0.0;
    info.score = 0.0;
    frontier_infos.push_back(info);
  }

  // ---------------- 4. Score ----------------
  int max_size = 0;
  double max_dist = 0.0;
  for (auto & f : frontier_infos) {
    f.dist = std::hypot(f.x - rx, f.y - ry);
    max_size = std::max(max_size, f.size);
    max_dist = std::max(max_dist, f.dist);
  }
  if (max_dist <= 0.0) max_dist = 1.0;

  int best_idx = -1;
  double best_score = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < frontier_infos.size(); ++i) {
    auto & f = frontier_infos[i];
    double size_norm = static_cast<double>(f.size) / max_size;
    double dist_norm = f.dist / max_dist;
    f.score = (size_weight_ * size_norm) - (distance_weight_ * dist_norm);
    if (f.score > best_score) {
      best_score = f.score;
      best_idx = static_cast<int>(i);
    }
  }

  if (best_idx >= 0) {
    const auto & sel = frontier_infos[best_idx];
    RCLCPP_INFO(
      get_logger(),
      "Selected frontier: (%.2f, %.2f) size=%d dist=%.2f score=%.3f out of %zu frontiers",
      sel.x, sel.y, sel.size, sel.dist, sel.score, frontier_infos.size());
  }

  publishMarkers(frontier_infos, best_idx);
  publishFrontierArray(frontier_infos, best_idx);
  publishStatus(false, static_cast<int>(frontier_infos.size()), "exploring");
}

void FrontierDetectorNode::publishMarkers(
  const std::vector<FrontierInfo> & frontier_infos, int selected_idx)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = global_frame_;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  rclcpp::Time stamp = get_clock()->now();

  for (size_t i = 0; i < frontier_infos.size(); ++i) {
    const auto & f = frontier_infos[i];

    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = stamp;
    m.ns = "frontiers";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = f.x;
    m.pose.position.y = f.y;
    m.pose.position.z = 0.1;
    m.pose.orientation.w = 1.0;

    double blob_scale = std::min(0.15 + 0.01 * f.size, 0.6);
    m.scale.x = blob_scale;
    m.scale.y = blob_scale;
    m.scale.z = blob_scale;

    bool is_selected = (static_cast<int>(i) == selected_idx);
    if (is_selected) {
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
    } else {
      m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.8f;
    }

    marker_array.markers.push_back(m);
  }

  marker_pub_->publish(marker_array);
}

void FrontierDetectorNode::publishFrontierArray(
  const std::vector<FrontierInfo> & frontier_infos, int selected_idx)
{
  frontier_exploration_msgs::msg::FrontierArray msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = global_frame_;
  msg.selected_index = selected_idx;

  msg.frontiers.reserve(frontier_infos.size());
  for (const auto & f : frontier_infos) {
    frontier_exploration_msgs::msg::Frontier fm;
    fm.position.x = f.x;
    fm.position.y = f.y;
    fm.position.z = 0.0;
    fm.size = f.size;
    fm.distance = f.dist;
    fm.score = f.score;
    msg.frontiers.push_back(fm);
  }

  frontier_pub_->publish(msg);
}

void FrontierDetectorNode::publishStatus(
  bool complete, int num_remaining, const std::string & reason)
{
  // Only spam a log line on state transitions, but always publish (cheap, latched).
  if (!have_published_status_ || complete != last_complete_state_) {
    RCLCPP_INFO(
      get_logger(), "Exploration status: complete=%s reason='%s'",
      complete ? "true" : "false", reason.c_str());
  }
  last_complete_state_ = complete;
  have_published_status_ = true;

  frontier_exploration_msgs::msg::ExplorationStatus msg;
  msg.exploration_complete = complete;
  msg.num_frontiers_remaining = num_remaining;
  msg.reason = reason;
  status_pub_->publish(msg);
}

}  // namespace frontier_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<frontier_exploration::FrontierDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}