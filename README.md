# Frontier Exploration for Nav2

Autonomous frontier-based exploration for a ROS 2 / Nav2 robot. Detects unexplored frontiers from the occupancy grid, picks the best one, and sends it to Nav2 as a navigation goal — repeating until the map is fully explored.

Based on classic frontier-based exploration (Yamauchi, 1997), with BFS clustering of frontier cells (Topiwala et al., 2018).

## Packages

- **`frontier_exploration_msgs`** — custom messages (`Frontier`, `FrontierArray`, `ExplorationStatus`)
- **`frontier_exploration`** — the two nodes:
  - `frontier_detector_node` — scans the map, detects and scores frontiers
  - `frontier_goal_selector_node` — picks the best frontier and sends it to Nav2

## Requirements

- ROS 2 (tested on Jazzy)
- Nav2 (`nav2_bringup`)
- SLAM running and publishing `/map` + TF (e.g. `slam_toolbox`)

## Build

```bash
cd ~/your_ws
colcon build --packages-select frontier_exploration_msgs frontier_exploration
source install/setup.bash
```

## Run

With SLAM and Nav2 already running:

```bash
ros2 launch frontier_exploration frontier_exploration.launch.py use_sim_time:=true
```

## Topics

| Topic | Type | Description |
|---|---|---|
| `/frontiers` | `frontier_exploration_msgs/FrontierArray` | All detected frontiers + selected index |
| `/exploration_status` | `frontier_exploration_msgs/ExplorationStatus` | Whether exploration is complete |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | RViz visualization |

## Parameters

See `frontier_exploration/config/frontier_exploration_params.yaml`.