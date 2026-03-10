# Custom Navigation Stack for ROS2 Nav2

A **custom navigation system for mobile robots using ROS2 Nav2**, featuring:

* Custom **A* Global Planner**
* Custom **Pure Pursuit Local Controller**
* **Behavior Tree navigation pipeline**
* Costmap-based obstacle avoidance
* Path smoothing integration

The project demonstrates how to **extend Nav2 with custom plugins** and implement a complete navigation pipeline from planning to control.

---

# Features

• Custom **A*** grid-based global planner
• Custom **Pure Pursuit path tracking controller**
• Integration with **Nav2 Behavior Tree Navigator**
• Costmap-based obstacle avoidance
• Path smoothing using Nav2 smoother server
• Fully lifecycle-managed ROS2 nodes

---

# System Architecture

```
            +----------------------+
            |   Behavior Tree      |
            |   (BT Navigator)     |
            +----------+-----------+
                       |
                       v
            +----------------------+
            |   Global Planner     |
            |      A* Plugin       |
            +----------+-----------+
                       |
                       v
            +----------------------+
            |    Path Smoother     |
            |   Nav2 Smoother      |
            +----------+-----------+
                       |
                       v
            +----------------------+
            |   Local Controller   |
            |   Pure Pursuit       |
            +----------+-----------+
                       |
                       v
                   cmd_vel
```

---

# Package Structure

```
my_auto_nav_pkg
│
├── behavior_tree
│   └── simple.xml
│
├── config
│   ├── bt_nav.yaml
│   ├── controller_server.yaml
│   ├── costmap.yaml
│   ├── planner_server.yaml
│   └── smoother_server.yaml
│
├── include/my_auto_nav_pkg
│   ├── astar_planner.hpp
│   └── pure_pursuit.hpp
│
├── src
│   ├── astar_planner.cpp
│   └── pure_pursuit.cpp
│
├── launch
│   └── nav.launch.py
│
├── nav2_plugins.xml
├── CMakeLists.txt
└── package.xml
```

---

# Custom Global Planner (A*)

The planner implements **A* search over the Nav2 costmap grid**.

### Key Features

* Manhattan distance heuristic
* Costmap obstacle avoidance
* Priority queue based search
* Efficient visited-node lookup
* Path reconstruction using parent nodes
* Integration with Nav2 smoother

### Planner Flow

```
Start Pose
     |
     v
Convert to Grid
     |
     v
A* Search
     |
     v
Reconstruct Path
     |
     v
Smooth Path (Nav2 Smoother)
     |
     v
Return nav_msgs/Path
```

### Heuristic

```
h(n) = |x_goal - x| + |y_goal - y|
```

---

# Path Smoothing

After generating the path, the planner sends it to the **Nav2 smoother server**.

The smoother:

* removes sharp corners
* generates smoother trajectories
* improves controller stability

```
nav2_msgs/action/SmoothPath
```

---

# Custom Local Controller (Pure Pursuit)

The controller implements the **Pure Pursuit algorithm** for path tracking.

### Parameters

| Parameter         | Description              |
| ----------------- | ------------------------ |
| `la_dist`         | Lookahead distance       |
| `max_linear_vel`  | Maximum linear velocity  |
| `max_angular_vel` | Maximum angular velocity |

Example configuration:

```
la_dist: 0.5
max_linear_vel: 0.3
max_angular_vel: 1.0
```

---

# Pure Pursuit Algorithm

1. Transform global path into robot frame
2. Find lookahead point on the path
3. Compute curvature
4. Convert curvature into angular velocity
5. Publish velocity command

Curvature formula:

```
curvature = 2*y / (x^2 + y^2)
```

Velocity command:

```
angular_vel = curvature * max_angular_vel
linear_vel = max_linear_vel
```

---

# Behavior Tree

Navigation is controlled using a **Behavior Tree**.

Main execution pipeline:

```
ComputePathToPose
        ↓
FollowPath
```

Recovery behavior:

```
Wait
 ↓
Clear Costmap
 ↓
Replan
 ↓
Spin
```

This ensures robust navigation when the robot gets stuck.

---

# Costmaps

The navigation stack uses **global and local costmaps**.

### Global Costmap

* Static map layer
* Obstacle layer
* Inflation layer

### Local Costmap

* Rolling window
* Laser scan obstacle detection
* Real-time obstacle avoidance

---

# Running the System

## Build the workspace

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Launch Navigation

```
ros2 launch my_auto_nav_pkg nav.launch.py
```

This starts:

* Planner server
* Controller server
* Smoother server
* Behavior server
* BT Navigator
* Lifecycle manager

---

# Topics Used

| Topic      | Description        |
| ---------- | ------------------ |
| `/map`     | Occupancy grid map |
| `/scan`    | LiDAR data         |
| `/odom`    | Robot odometry     |
| `/cmd_vel` | Velocity commands  |

---

# Plugins

The package exports two Nav2 plugins.

### Global Planner

```
my_auto_nav_pkg/AstarPlanner
```

### Controller

```
my_auto_nav_pkg/PurePursuitController
```

These are registered in `nav2_plugins.xml`.

---

# Dependencies

Main dependencies:

```
rclcpp
nav2_core
nav2_costmap_2d
nav2_msgs
nav_msgs
geometry_msgs
pluginlib
tf2
```
---

# Author

**Pritam Paul**

Robotics | ROS2 | Autonomous Navigation | Embedded Systems
