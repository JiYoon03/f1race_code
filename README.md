# 🏎️ Final Race — Pure Pursuit + RRT* Hybrid Racer 🏎️

Our racing strategy used a pure pursuit backbone and switched to RRT* for obstacle avoidance. More details below.
# [video link](https://youtu.be/r6y5tmPvKGU)
# Team Members

- JiYoon Kang (JiYoon03)
- KunjieXu (KunjieXu)
- Neel Shejwalkar (nshejwalkar)
- Zwe Tun (ZweTun) 

# Race Strategy

Before the track was laid out, we brainstormed several different ideas for what our racing strategy could be. We were certain on pure pursuit on predefined waypoints as our main backbone due to how easily customizable it was. For obstacle avoidance, we explored artificial potential fields to dynamically move the waypoints around obstacles, as well as RRT*. We decided to go with the latter given that we could build off of our strong understanding of how it worked from class.

Initially, we were thinking about complex overtaking strategies, but decided that implementing them and ensuring that they worked robustly would be too difficult for very little gain. We anticipated that most races would be decided on one team either a) having faster pure pursuit or b) crashing less than the other, a suspicion that was verified on race day. In either case, decent obstacle avoidance would be sufficient to win.

We learned from race 2 not to spend too much time tuning algorithms in the simulator, and so the majority of our work started the day the track was laid out, tuning the car for the specific track.

### Pure pursuit
We realized during race 2 that the key to having a fast pure pursuit backbone was to shorten the iteration time of fixing waypoint tuning as much as possible. We designed an interactive waypoint editor (waypoint_editor.py) to easily overlay waypoints onto the SLAM map and visually change many different knobs that would have otherwise been extremely manual through the raw csv. The editor has an easy interface, allows for bulk editing of the location, associated speed, and lookahead for waypoints, and a switch to represent a subset of the waypoints as Catmull-Rom splines to easily edit curves. This editor is what allowed us to quickly validate our ideas for pure pursuit.

### RRT*
For obstacle avoidance, we built our algorithm off of the RRT lab assignment. We perform two checks to see if we should switch from pure pursuit -> RRT* - if either fail, the obstacle avoidance is triggered. If the goal waypoint (at the lookahead) is occupied, we switch to RRT* (planning onto a further waypoint, not the blocked goal). If the goal waypoint isn't blocked, but the path leading up to it is, we also switch to RRT*. Every occupancy grid check is done with an inflation_radius in mind so as to leave a bit of buffer room. When the path clears up, it takes several cycles to switch back to pure pursuit instead of switching back instantly, so as to avoid possible flickering between modes. Specific parameters and more details are listed at the end of this README and in the rrt_node.py file.

# Challenges
Prior to race day, we mostly hit problems with our pure pursuit algorithm being inaccurate on certain parts of the map, such as the straight close to the wall and the long curve around the first pillar. In all of these cases, improving the editor to be easier to use and changing the waypoints/curves to accomodate for potentially poor localization at those places was our solution.

The majority of challenges we ran into on race day were due to hardware faults and small mistakes. Our battery was very low charge for our first race, causing it to die mid race. For the second race, we forgot to localize properly at the beginning of the race, resulting in us starting much later than our opponent. For our third race, we made sure to pay more attention to our whole setup, ensuring our hardware and race setup was working properly before the race began.

# Results
We ended up losing both brackets in the first round. There wasn't one large singular thing we did wrong, but having more awareness of our hardware setup would have helped, as well as possibly creating more tooling for observability when our pure pursuit failed in real life, in a similar style as our waypoint editor. This would have helped us better diagnose failures for pure pursuit. Our focus on real life testing, quick iteration, and simple algorithms are lessons that we would carry on for next time. 

---

# Package Structure

```text
final-race/
├── particle_filter/                   # ROS2 package — localization (particle filter)
│   ├── package.xml
│   ├── setup.py / setup.cfg / CMakeLists.txt
│   ├── particle_filter/
│   │   └── particle_filter_node.py
│   ├── launch/localize_launch.py
│   └── config/localize.yaml
├── rrt_node/                          # ROS2 package — main racing node
│   ├── package.xml
│   ├── setup.py / setup.cfg / CMakeLists.txt
│   ├── rrt_node/
│   │   └── rrt_node.py                # RRT* + Pure Pursuit hybrid
│   └── resource/rrt_node
├── race3.csv                          # Default waypoint file (x, y, v)
└── waypoint_editor.py                 # Interactive waypoint editor (run directly with python)
```

---

# Prerequisites

## 1. Install Dependencies

```bash
# ROS 2 Humble
sudo apt install ros-humble-ackermann-msgs \
                 ros-humble-nav-msgs \
                 ros-humble-visualization-msgs

# Python
pip install numpy
```

## 2. Build the Workspace

```bash
cd ~/roboracer_ws
colcon build
source install/setup.bash
```

---

# Running the Stack

Run each command in a separate terminal.
Source the workspace in every new terminal before launching nodes.

## Terminal 1 — Simulator

```bash
source /opt/ros/humble/setup.bash
source ~/roboracer_ws/install/setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Open Foxglove at:

```text
http://localhost:8765
```

Import the layout from:

```text
f1tenth_gym_ros/config/foxglove/gym_bridge_foxglove.json
```

---

## Terminal 2 — Main Racing Node (RRT* Hybrid)

```bash
source /opt/ros/humble/setup.bash
source ~/roboracer_ws/install/setup.bash

ros2 run rrt_node rrt_node
```

To specify a waypoint CSV explicitly:

```bash
ros2 run rrt_node rrt_node \
  --ros-args -p waypoints_file:=/absolute/path/to/waypoints.csv
```

---

## Terminal 3 — Particle Filter (Real Car Only)

```bash
source /opt/ros/humble/setup.bash
source ~/roboracer_ws/install/setup.bash

ros2 launch particle_filter localize_launch.py
```

Required for real-car localization.
Not needed in simulation.

Also set:

```python
self.sim_mode = False
```

inside `rrt_node.py` before deploying on the real car.

---

# How Mode Switching Works

```text
No obstacle  →  Pure Pursuit
     ↓ obstacle detected
Obstacle     →  RRT* avoidance planning
     ↓ obstacle cleared + hysteresis wait
No obstacle  →  Back to Pure Pursuit
```

| Transition          | Condition                                                             |
| ------------------- | --------------------------------------------------------------------- |
| Pure Pursuit → RRT* | Goal waypoint blocked OR raceline not clear within `3.5 m`            |
| RRT* → Pure Pursuit | Obstacles cleared and `replan_hysteresis_cycles = 4` cooldown elapsed |

The hysteresis prevents rapid oscillation when obstacles sit near the detection boundary.

---

## Recording / Editing Waypoints

```bash
# Edit the default waypoint file
python waypoint_editor.py

# Create a new waypoint file
python waypoint_editor.py --csv new_track.csv

# Use a specific map
python waypoint_editor.py \
  --csv race2.csv \
  --map f1tenth_gym_ros/maps/my_map1.yaml
```

After saving, rebuild the workspace:

```bash
colcon build --packages-select pure_pursuit
source install/setup.bash
```

---

# Key Parameters

All parameters are defined at the top of `rrt_node.py`.

## Speed Parameters

| Parameter                | Default | Description                               |
| ------------------------ | ------- | ----------------------------------------- |
| `velocity_scale`         | `1.0`   | Global speed multiplier                   |
| `obstacle_speed`         | `3.0`   | Max speed during obstacle avoidance       |
| `min_speed`              | `0.6`   | Minimum allowed speed                     |
| `max_speed`              | `10.0`  | Hard speed cap                            |
| `default_waypoint_speed` | `2.0`   | Fallback speed when CSV has no `v` column |

---

## Pure Pursuit Parameters

| Parameter                 | Default | Description                                    |
| ------------------------- | ------- | ---------------------------------------------- |
| `pursuit_lookahead_clear` | `1.20`  | Lookahead distance on clear raceline           |
| `pursuit_lookahead_avoid` | `0.70`  | Lookahead distance during avoidance            |
| `max_steer`               | `0.40`  | Maximum steering angle (rad)                   |
| `curvature_speed_gain`    | `0.3`   | Speed reduction factor based on steering angle |

---

## RRT* Parameters

| Parameter            | Default | Description                                    |
| -------------------- | ------- | ---------------------------------------------- |
| `max_iterations`     | `300`   | Maximum RRT* iterations per planning cycle     |
| `max_expansion_dist` | `0.45`  | Maximum node expansion distance                |
| `goal_threshold`     | `0.30`  | Goal reach threshold                           |
| `search_radius`      | `0.9`   | RRT* rewiring radius                           |
| `early_term_iters`   | `60`    | Early stopping iterations after finding a path |

---

## Obstacle Detection Parameters

| Parameter                  | Default | Description                                      |
| -------------------------- | ------- | ------------------------------------------------ |
| `inflation_radius`         | `2`     | Obstacle inflation in occupancy grid cells       |
| `path_clear_check_dist`    | `3.5`   | Distance ahead to validate raceline              |
| `replan_hysteresis_cycles` | `4`     | Cooldown cycles before returning to Pure Pursuit |

---

