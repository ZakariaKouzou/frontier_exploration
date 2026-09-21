# Autonomous Frontier-Based Exploration

Autonomous exploration for a mobile robot running on ROS 2. The robot builds its own map of an unknown environment by repeatedly detecting the boundaries between known free space and unexplored space ("frontiers"), picking the most promising one, and navigating to it — with no human-provided waypoints.

The approach follows classic frontier-based exploration (Yamauchi, 1997), with frontier clustering done via BFS/flood-fill over the occupancy grid (Topiwala et al., 2018).

<img width="1247" height="825" alt="Screenshot 2026-09-21 205545" src="https://github.com/user-attachments/assets/1929b80a-0ecb-43e3-862d-dc856dbfd729" />

## DEMO 

https://github.com/user-attachments/assets/d182db0c-87ae-473a-9d77-e7d453870db9


## How it works

1. **`frontier_detector_node`** subscribes to the occupancy grid (`/map`), scans it for frontier cells (free cells adjacent to unknown space), clusters them, scores each cluster by size and distance from the robot, and publishes the result.
2. **`frontier_goal_selector_node`** subscribes to the detected frontiers, picks the best candidate (preferring nearby ones over far-away large ones), and sends it to Nav2's `navigate_to_pose` action server.
3. Once a goal is reached (or the robot gets close enough), the cycle repeats automatically until no frontiers remain, at which point exploration is marked complete.

## Packages

| Package | Purpose |
|---|---|
| `frontier_exploration_msgs` | Custom message definitions: `Frontier`, `FrontierArray`, `ExplorationStatus` |
| `frontier_exploration` | The two runtime nodes, launch file, and default parameters |

## Requirements

- ROS 2 (developed and tested on **Jazzy**)
- A working navigation stack (**Nav2** — `nav2_bringup`)
- A SLAM node publishing `/map` and the TF tree (e.g. **slam_toolbox**)
- `colcon` and a standard ROS 2 workspace

## Setup

### 1. Clone the repository

Clone this into the `src/` folder of an existing (or new) ROS 2 workspace:

```bash
cd ~/your_ws/src
git clone https://github.com/ZakariaKouzou/frontier_exploration.git
```

### 2. Install dependencies

From the root of your workspace:

```bash
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
colcon build --packages-select frontier_exploration_msgs frontier_exploration
```

### 4. Source the workspace

```bash
source install/setup.bash
```

Do this in every new terminal you use to run or inspect these nodes (or add it to your `~/.bashrc` if this is your primary workspace).

## Running

Make sure SLAM and Nav2 are already running and publishing `/map`, TF, and the `navigate_to_pose` action server. Then:

```bash
ros2 launch frontier_exploration frontier_exploration.launch.py use_sim_time:=true
```

Drop `use_sim_time:=true` if you're running on real hardware rather than simulation.

## Topics

| Topic | Type | Description |
|---|---|---|
| `/frontiers` | `frontier_exploration_msgs/FrontierArray` | All currently detected frontiers, plus the selected/best index |
| `/exploration_status` | `frontier_exploration_msgs/ExplorationStatus` | Whether exploration is complete, and why |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | Visualization for RViz — green marker is the selected frontier |

## Parameters

Full list and defaults live in `frontier_exploration/config/frontier_exploration_params.yaml`. Key ones to tune for your environment:

| Parameter | Node | Description |
|---|---|---|
| `min_frontier_size` | `frontier_detector_node` | Minimum cluster size (cells) to count as a valid frontier |
| `wall_clearance_cells` | `frontier_detector_node` | Rejects frontier cells too close to obstacles |
| `distance_weight` / `size_weight` | `frontier_detector_node` | Balances "closest" vs "largest" when scoring frontiers |
| `local_radius` | `frontier_goal_selector_node` | Prefer frontiers within this radius before considering farther ones |
| `min_decent_size` | `frontier_goal_selector_node` | Minimum frontier size the goal selector will actually drive to |

## References

- B. Yamauchi, *"A frontier-based approach for autonomous exploration,"* IEEE CIRA, 1997.
- Topiwala, Inani & Kathpal, *"Frontier Based Exploration for Autonomous Robot,"* arXiv:1806.03581, 2018.

## License

TODO: add a license (e.g. Apache-2.0, MIT).
