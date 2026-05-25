#!/usr/bin/env python3
"""
Race Controller
- Uses Pure Pursuit by default to follow waypoints (CSV with 3 columns: x, y, v)
- Switches to RRT* local obstacle avoidance only when an obstacle is detected ahead on the path
- Smoothly transitions back to Pure Pursuit once the obstacle is cleared
- Control commands are rate-limited for acceleration/deceleration and steering angle to prevent hard acceleration / spinning out
- Obstacle detection band: path-direction projection + consecutive beam clustering + multi-frame persistence + enter/exit hysteresis

Runs on real car by default (sim_mode = False)
"""

import csv
import math
import random
from enum import Enum
from pathlib import Path

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class Mode(Enum):
    PURE_PURSUIT = 1
    RRT_AVOIDANCE = 2


class RaceController(Node):
    def __init__(self):
        super().__init__('race_controller')

        # ====================================================================
        # Configuration parameters (modify as needed)
        # ====================================================================
        self.sim_mode = False                # Default: real car
        self.use_rrt_star = True             # True=RRT*, False=basic RRT

        # Waypoints file (leave empty to auto-find the latest CSV in common directories)
        # CSV format: 3 columns: x, y, v (speed in m/s); also supports 2 columns (no speed, uses default)
        self.waypoints_file = ""

        # ----- Main control loop -----
        self.control_frequency = 50.0        # Hz
        self.plan_frequency    = 10.0        # RRT planning frequency (used only in avoidance mode)
        self.wheelbase         = 0.33

        # ----- Speed / steering limits -----
        self.max_steer = 0.41                # rad
        self.max_speed = 6.0                 # m/s safety cap
        self.min_speed = 0.5

        # ----- Smoothing (prevent hard acceleration / spinning out) -----
        self.max_accel       = 4.0           # m/s^2 acceleration limit
        self.max_decel       = 7.0           # m/s^2 deceleration limit
        self.max_steer_rate  = 5.0           # rad/s steering angle rate limit
        self.steer_speed_softening = 0.40    # speed reduction factor at high steering angles

        # ----- Pure Pursuit -----
        self.pp_lookahead_base = 0.9         # base lookahead distance
        self.pp_lookahead_gain = 0.18        # lookahead increase per m/s
        self.pp_lookahead_min  = 0.7
        self.pp_lookahead_max  = 3.5

        # ----- Obstacle detection (robust) -----
        # An obstacle is detected if enough consecutive/clustered laser points exist
        # inside the "tube" along the current path direction
        self.obstacle_enter_dist          = 1.2   # detection distance to enter avoidance mode
        self.obstacle_exit_dist           = 2.2   # detection distance to exit avoidance mode (hysteresis: exit > enter)
        self.obstacle_tube_half_width     = 0.32  # tube half-width in PP mode
        self.obstacle_tube_half_width_rrt = 0.42  # tube half-width in avoidance mode (wider to ensure full clearance)
        self.obstacle_min_consecutive     = 4     # minimum N consecutive beams inside tube
        self.obstacle_min_total           = 6     # or total points inside tube >= N
        self.obstacle_persist_frames      = 3     # scan frames required to enter avoidance mode
        self.obstacle_clear_frames        = 4     # scan frames required to exit avoidance mode

        self.car_chassis_radius = 0.18

        # ----- RRT* -----
        self.max_expansion_dist = 0.40
        self.max_iterations     = 500
        self.rrt_lookahead      = 2.5
        self.rrt_max_lookahead  = 4.0
        self.goal_threshold     = 0.30
        self.goal_bias_prob     = 0.30
        self.search_radius      = 0.85       # RRT* neighbor search radius
        self.max_shortcut_dist  = 1.2

        # Local sampling window
        self.sample_x_min          = -0.4
        self.sample_x_max          = 4.5
        self.sample_y_max          = 2.0
        self.forward_sample_ratio  = 0.85

        # Speed strategy in avoidance mode
        self.rrt_speed_scale  = 0.55          # avoidance speed = waypoint speed * scale
        self.rrt_min_speed    = 0.7
        self.rrt_max_speed    = 2.8
        self.rrt_pursuit_lookahead = 0.7      # mini pure pursuit lookahead along RRT path

        # ----- Occupancy grid -----
        self.grid_resolution = 0.10
        self.grid_width      = 200
        self.grid_height     = 200
        self.inflation_radius = 1.5             # units: cells. Slightly larger than car body radius for safety

        # ----- Frames / topics -----
        if self.sim_mode:
            self.odom_topic = '/ego_racecar/odom'
            self.base_frame = 'ego_racecar/base_link'
        else:
            self.odom_topic = '/pf/pose/odom'
            self.base_frame = 'base_link'
        self.scan_topic  = '/scan'
        self.drive_topic = '/drive'
        self.map_frame   = 'map'

        # ====================================================================
        # State variables
        # ====================================================================
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.pose_ready = False
        self.scan_ready = False
        self.latest_scan = None

        self.mode = Mode.PURE_PURSUIT
        self.obstacle_detect_count = 0
        self.obstacle_clear_count  = 0

        self.last_cmd_speed = 0.0
        self.last_cmd_steer = 0.0

        self.waypoints = []                  # list of [x, y, v]
        self.last_closest_idx = 0
        self.default_wp_speed = 2.0          # used when CSV has only 2 columns

        self.occupancy_grid = [0] * (self.grid_width * self.grid_height)
        self.cached_path = []                # local-frame path [[x, y], ...]
        self.cached_path_target_speed = 0.0
        self.plan_fail_count = 0

        # Load waypoints
        self.load_waypoints()

        # ====================================================================
        # Publishers / subscribers
        # ====================================================================
        self.drive_pub      = self.create_publisher(AckermannDriveStamped, self.drive_topic, 1)
        self.tree_pub       = self.create_publisher(Marker, '/rrt_tree', 1)
        self.path_pub       = self.create_publisher(Marker, '/rrt_path', 1)
        self.waypoints_pub  = self.create_publisher(Marker, '/race_waypoints', 1)
        self.lookahead_pub  = self.create_publisher(Marker, '/race_lookahead', 1)
        self.goal_pub       = self.create_publisher(Marker, '/rrt_goal', 1)
        self.grid_pub       = self.create_publisher(OccupancyGrid, '/rrt_occupancy_grid', 1)

        self.create_subscription(Odometry,  self.odom_topic, self.pose_callback, 1)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 1)

        # ====================================================================
        # Timers
        # ====================================================================
        self.create_timer(1.0 / self.control_frequency, self.control_loop)
        self.create_timer(1.0 / self.plan_frequency,   self.plan_loop)
        self.create_timer(0.5, self.visualize_waypoints)

        self.get_logger().info(
            f'[race_controller] mode={"sim" if self.sim_mode else "real"}, '
            f'odom={self.odom_topic}, waypoints={len(self.waypoints)}, '
            f'use_rrt_star={self.use_rrt_star}'
        )

    # ========================================================================
    # ROS callbacks
    # ========================================================================
    def pose_callback(self, msg: Odometry):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.car_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose_ready = True

    def scan_callback(self, msg: LaserScan):
        """Update obstacle detection and mode state machine once per scan frame."""
        self.latest_scan = msg
        self.scan_ready = True
        if self.pose_ready and self.waypoints:
            self.update_mode()

    # ========================================================================
    # Waypoint loading
    # ========================================================================
    def resolve_waypoints_file(self):
        if self.waypoints_file:
            cand = Path(self.waypoints_file).expanduser()
            if cand.is_file():
                return cand
            self.get_logger().warn(f'Specified waypoint file does not exist: {cand}')

        search_dirs = []
        try:
            sd = Path(__file__).resolve().parent
            search_dirs.extend([sd, sd.parent, sd.parent / 'waypoints', sd.parent / 'logs'])
        except Exception:
            pass
        home = Path.home()
        search_dirs.extend([
            home / 'roboracer_ws' / 'src' / 'lab-6-motion-planning-team12' / 'waypoints',
            home / 'roboracer_ws' / 'src' / 'lab-6-motion-planning-team11' / 'waypoints',
            home / 'rcws' / 'logs',
            Path.cwd(),
        ])

        csv_files = []
        for d in search_dirs:
            if d.is_dir():
                csv_files.extend(d.glob('*.csv'))
        if not csv_files:
            return None
        return max(csv_files, key=lambda p: p.stat().st_mtime)

    def load_waypoints(self):
        wp_path = self.resolve_waypoints_file()
        if wp_path is None:
            self.get_logger().warn('No valid waypoint CSV found')
            return

        loaded = []
        skipped = 0
        try:
            with wp_path.open('r', encoding='utf-8') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row:
                        continue
                    first = str(row[0]).strip()
                    if not first or first.startswith('#'):
                        continue
                    if len(row) < 2:
                        skipped += 1
                        continue
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        v = float(row[2]) if len(row) >= 3 else self.default_wp_speed
                    except ValueError:
                        skipped += 1
                        continue
                    # Basic speed sanity check
                    v = max(self.min_speed, min(self.max_speed, v))
                    loaded.append([x, y, v])
            self.waypoints = loaded
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints from {wp_path} (skipped {skipped} rows)'
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to load waypoints: {e}')

    # ========================================================================
    # Obstacle detection (robust)
    # ========================================================================
    def detect_path_obstacle(self):
        """Detect whether an obstacle exists inside the 'path tube' ahead.
        - The tube is defined along the current lookahead direction (follows curves to avoid
          false positives from walls in turns)
        - Counts both: max consecutive beam run inside the tube + total point count;
          obstacle is flagged if either exceeds the threshold
        - update_mode() applies multi-frame persistence + hysteresis for final decision
        """
        if self.latest_scan is None:
            return False

        # Use PP lookahead direction as "forward"
        target = self.find_lookahead_waypoint_local()
        if target is None or math.hypot(target[0], target[1]) < 1e-3:
            dir_x, dir_y = 1.0, 0.0
        else:
            d = math.hypot(target[0], target[1])
            dir_x, dir_y = target[0] / d, target[1] / d

        # Hysteresis: different thresholds per mode
        if self.mode == Mode.PURE_PURSUIT:
            check_dist = self.obstacle_enter_dist
            tube_w     = self.obstacle_tube_half_width
        else:
            check_dist = self.obstacle_exit_dist
            tube_w     = self.obstacle_tube_half_width_rrt

        scan = self.latest_scan
        total = 0
        max_run = 0
        cur_run = 0

        angle = scan.angle_min
        for r in scan.ranges:
            in_zone = False
            if (math.isfinite(r)
                    and scan.range_min <= r <= scan.range_max
                    and r >= self.car_chassis_radius):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                along =  x * dir_x + y * dir_y
                lat   = -x * dir_y + y * dir_x
                if 0.25 <= along <= check_dist and abs(lat) <= tube_w:
                    in_zone = True

            if in_zone:
                total += 1
                cur_run += 1
                if cur_run > max_run:
                    max_run = cur_run
            else:
                cur_run = 0

            angle += scan.angle_increment

        return (max_run >= self.obstacle_min_consecutive
                or total >= self.obstacle_min_total)

    def update_mode(self):
        """State machine: multi-frame persistence + enter/exit hysteresis to prevent jitter."""
        obstacle_now = self.detect_path_obstacle()

        if self.mode == Mode.PURE_PURSUIT:
            if obstacle_now:
                self.obstacle_detect_count += 1
                self.obstacle_clear_count = 0
            else:
                self.obstacle_detect_count = 0

            if self.obstacle_detect_count >= self.obstacle_persist_frames:
                self.mode = Mode.RRT_AVOIDANCE
                self.obstacle_detect_count = 0
                self.cached_path = []
                self.plan_fail_count = 0
                self.get_logger().info('Obstacle detected -> switching to RRT* avoidance')
        else:  # RRT_AVOIDANCE
            if not obstacle_now:
                self.obstacle_clear_count += 1
                self.obstacle_detect_count = 0
            else:
                self.obstacle_clear_count = 0

            if self.obstacle_clear_count >= self.obstacle_clear_frames:
                self.mode = Mode.PURE_PURSUIT
                self.obstacle_clear_count = 0
                self.cached_path = []
                self.get_logger().info('✅ Obstacle cleared -> returning to Pure Pursuit')

    # ========================================================================
    # Pure Pursuit
    # ========================================================================
    def find_closest_waypoint_idx(self):
        if not self.waypoints:
            return -1
        n = len(self.waypoints)
        # Local window for speed
        window = 30
        if 0 <= self.last_closest_idx < n:
            start = self.last_closest_idx - 5
            indices = [(start + k) % n for k in range(window)]
        else:
            indices = range(n)

        best_idx = self.last_closest_idx
        best_d2 = float('inf')
        for i in indices:
            wp = self.waypoints[i]
            d2 = (wp[0] - self.car_x) ** 2 + (wp[1] - self.car_y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i

        # Fall back to global search if local window fails
        if best_d2 > 9.0:
            for i in range(n):
                wp = self.waypoints[i]
                d2 = (wp[0] - self.car_x) ** 2 + (wp[1] - self.car_y) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_idx = i

        self.last_closest_idx = best_idx
        return best_idx

    def waypoint_to_local(self, wp):
        dx = wp[0] - self.car_x
        dy = wp[1] - self.car_y
        cy = math.cos(self.car_yaw)
        sy = math.sin(self.car_yaw)
        return cy * dx + sy * dy, -sy * dx + cy * dy

    def get_lookahead_distance(self):
        L = self.pp_lookahead_base + self.pp_lookahead_gain * abs(self.last_cmd_speed)
        return max(self.pp_lookahead_min, min(self.pp_lookahead_max, L))

    def find_lookahead_waypoint_local(self):
        """Returns [lx, ly, target_speed] or None."""
        if not self.waypoints:
            return None
        n = len(self.waypoints)
        closest = self.find_closest_waypoint_idx()
        if closest < 0:
            return None

        L = self.get_lookahead_distance()
        for k in range(n):
            i = (closest + k) % n
            wp = self.waypoints[i]
            d = math.hypot(wp[0] - self.car_x, wp[1] - self.car_y)
            if d >= L:
                lx, ly = self.waypoint_to_local(wp)
                if lx > -0.3:
                    return [lx, ly, wp[2]]
        # All waypoints are closer than lookahead -> use closest
        wp = self.waypoints[closest]
        lx, ly = self.waypoint_to_local(wp)
        return [lx, ly, wp[2]]

    def pure_pursuit_step(self):
        target = self.find_lookahead_waypoint_local()
        if target is None:
            return 0.0, 0.0
        lx, ly, v = target
        self.visualize_lookahead(lx, ly)

        L2 = lx * lx + ly * ly
        if L2 < 1e-6:
            return v, 0.0
        steer = math.atan2(2.0 * self.wheelbase * ly, L2)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # Speed reduction in turns: larger steer angle -> lower speed
        scale = 1.0 - self.steer_speed_softening * abs(steer) / max(self.max_steer, 1e-6)
        target_speed = max(self.min_speed, v * scale)
        target_speed = min(target_speed, self.max_speed)
        return target_speed, steer

    # ========================================================================
    # RRT* path tracking (mini pure pursuit)
    # ========================================================================
    def rrt_track_step(self):
        """Track cached_path. If no valid path is available, coast at safe speed."""
        if not self.cached_path or len(self.cached_path) < 2:
            # Path not yet generated or planning failed: smoothly slow down and center steering
            slow_speed = max(self.rrt_min_speed, self.last_cmd_speed * 0.6)
            slow_steer = self.last_cmd_steer * 0.5
            return slow_speed, slow_steer

        # Advance rrt_pursuit_lookahead distance along the path to get target point
        target_x, target_y = self.cached_path[-1]
        traveled = 0.0
        for i in range(1, len(self.cached_path)):
            p0 = self.cached_path[i - 1]
            p1 = self.cached_path[i]
            seg = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if traveled + seg >= self.rrt_pursuit_lookahead:
                t = (self.rrt_pursuit_lookahead - traveled) / max(seg, 1e-6)
                target_x = p0[0] + t * (p1[0] - p0[0])
                target_y = p0[1] + t * (p1[1] - p0[1])
                break
            traveled += seg

        L2 = target_x * target_x + target_y * target_y
        steer = 0.0
        if L2 > 1e-6:
            steer = math.atan2(2.0 * self.wheelbase * target_y, L2)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        base = self.cached_path_target_speed * self.rrt_speed_scale
        base = max(self.rrt_min_speed, min(self.rrt_max_speed, base))
        scale = 1.0 - 0.55 * abs(steer) / max(self.max_steer, 1e-6)
        target_speed = max(self.rrt_min_speed, base * scale)
        return target_speed, steer

    # ========================================================================
    # Smoothing (prevent hard acceleration / spinning out)
    # ========================================================================
    def smooth_speed(self, target):
        dt = 1.0 / self.control_frequency
        diff = target - self.last_cmd_speed
        if diff > 0:
            diff = min(diff, self.max_accel * dt)
        else:
            diff = max(diff, -self.max_decel * dt)
        return self.last_cmd_speed + diff

    def smooth_steer(self, target):
        dt = 1.0 / self.control_frequency
        diff = target - self.last_cmd_steer
        max_step = self.max_steer_rate * dt
        if diff > max_step:
            diff = max_step
        elif diff < -max_step:
            diff = -max_step
        return self.last_cmd_steer + diff

    # ========================================================================
    # Main control loop (50 Hz)
    # ========================================================================
    def control_loop(self):
        if not (self.pose_ready and self.scan_ready):
            return

        if self.mode == Mode.PURE_PURSUIT:
            target_speed, target_steer = self.pure_pursuit_step()
        else:
            target_speed, target_steer = self.rrt_track_step()

        cmd_speed = self.smooth_speed(target_speed)
        cmd_steer = self.smooth_steer(target_steer)

        self.publish_drive(cmd_speed, cmd_steer)
        self.last_cmd_speed = cmd_speed
        self.last_cmd_steer = cmd_steer

    # ========================================================================
    # RRT* planning loop (active only in avoidance mode)
    # ========================================================================
    def plan_loop(self):
        if self.mode != Mode.RRT_AVOIDANCE:
            return
        if not (self.pose_ready and self.scan_ready):
            return

        self.build_occupancy_grid()
        self.publish_occupancy_grid()

        goal = self.find_rrt_goal()
        if goal is None:
            return
        gx, gy, gv = goal

        path = self.run_rrt(gx, gy)
        if path:
            simplified = self.shortcut_path(path)
            self.cached_path = [[n['x'], n['y']] for n in simplified]
            self.cached_path_target_speed = gv
            self.plan_fail_count = 0
            self.visualize_path(self.cached_path)
            self.visualize_goal(gx, gy)
        else:
            self.plan_fail_count += 1
            # Clear path after repeated failures so rrt_track_step uses the safe-slow branch
            if self.plan_fail_count >= 3:
                self.cached_path = []

    def find_rrt_goal(self):
        """Select a waypoint that is not in an occupied cell and is at a suitable distance."""
        if not self.waypoints:
            return None
        n = len(self.waypoints)
        closest = self.find_closest_waypoint_idx()
        if closest < 0:
            return None

        for L in (self.rrt_lookahead, self.rrt_max_lookahead):
            for k in range(n):
                i = (closest + k) % n
                wp = self.waypoints[i]
                d = math.hypot(wp[0] - self.car_x, wp[1] - self.car_y)
                if d >= L:
                    lx, ly = self.waypoint_to_local(wp)
                    if lx > 0.0:
                        valid, gxi, gyi = self.world_to_grid(lx, ly)
                        if valid and not self.grid_occupied(gxi, gyi):
                            return [lx, ly, wp[2]]
        # Fallback: pick a somewhat farther waypoint and let RRT try again
        i = (closest + max(1, n // 8)) % n
        wp = self.waypoints[i]
        lx, ly = self.waypoint_to_local(wp)
        return [lx, ly, wp[2]]

    # ========================================================================
    # Occupancy grid
    # ========================================================================
    def build_occupancy_grid(self):
        self.occupancy_grid = [0] * (self.grid_width * self.grid_height)
        if self.latest_scan is None:
            return
        scan = self.latest_scan
        angle = scan.angle_min
        for r in scan.ranges:
            if (math.isfinite(r)
                    and scan.range_min <= r <= scan.range_max
                    and r >= self.car_chassis_radius):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                valid, gx, gy = self.world_to_grid(x, y)
                if valid:
                    self.inflate_cell(gx, gy)
            angle += scan.angle_increment

    def world_to_grid(self, wx, wy):
        half_w = self.grid_width // 2
        half_h = self.grid_height // 2
        gx = int(round(wx / self.grid_resolution)) + half_w
        gy = int(round(wy / self.grid_resolution)) + half_h
        return self.grid_in_bounds(gx, gy), gx, gy

    def grid_in_bounds(self, gx, gy):
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

    def grid_occupied(self, gx, gy):
        return self.occupancy_grid[gy * self.grid_width + gx] != 0

    def inflate_cell(self, gx, gy):
        for dx in range(-self.inflation_radius, self.inflation_radius + 1):
            for dy in range(-self.inflation_radius, self.inflation_radius + 1):
                nx, ny = gx + dx, gy + dy
                if self.grid_in_bounds(nx, ny):
                    self.occupancy_grid[ny * self.grid_width + nx] = 1

    # ========================================================================
    # RRT* core algorithm
    # ========================================================================
    def run_rrt(self, goal_x, goal_y):
        tree = [self.make_node(0.0, 0.0, parent=-1, cost=0.0, is_root=True)]
        for _ in range(self.max_iterations):
            sampled = self.sample(goal_x, goal_y)
            nearest_idx = self.nearest(tree, sampled)
            new_node = self.steer(tree[nearest_idx], sampled)
            if self.check_collision(tree[nearest_idx], new_node):
                continue
            if self.use_rrt_star:
                new_idx = self.add_rrt_star_node(tree, new_node, nearest_idx)
            else:
                new_node['parent'] = nearest_idx
                new_node['cost'] = (tree[nearest_idx]['cost']
                                    + self.line_cost(tree[nearest_idx], new_node))
                tree[nearest_idx]['children'].append(len(tree))
                tree.append(new_node)
                new_idx = len(tree) - 1

            if self.is_goal(tree[new_idx], goal_x, goal_y):
                self.visualize_tree(tree)
                return self.find_path(tree, new_idx)

        self.visualize_tree(tree)
        return []

    def make_node(self, x, y, parent=-1, cost=0.0, is_root=False):
        return {
            'x': float(x), 'y': float(y),
            'parent': int(parent), 'cost': float(cost),
            'is_root': bool(is_root), 'children': [],
        }

    def sample(self, goal_x, goal_y):
        if random.random() < self.goal_bias_prob:
            return [goal_x, goal_y]
        forward_x_max = max(self.sample_x_max, goal_x + 1.0)
        if random.random() < self.forward_sample_ratio:
            x = random.uniform(0.0, forward_x_max)
            y = random.uniform(-self.sample_y_max, self.sample_y_max)
        else:
            lat = max(self.sample_y_max, abs(goal_y) + 0.5)
            x = random.uniform(self.sample_x_min, forward_x_max)
            y = random.uniform(-lat, lat)
        return [x, y]

    def nearest(self, tree, p):
        best = 0
        best_d = float('inf')
        for i, n in enumerate(tree):
            d = (n['x'] - p[0]) ** 2 + (n['y'] - p[1]) ** 2
            if d < best_d:
                best_d = d
                best = i
        return best

    def steer(self, nearest_node, p):
        dx = p[0] - nearest_node['x']
        dy = p[1] - nearest_node['y']
        d = math.hypot(dx, dy)
        if d <= self.max_expansion_dist:
            return self.make_node(p[0], p[1])
        s = self.max_expansion_dist / max(d, 1e-9)
        return self.make_node(nearest_node['x'] + dx * s, nearest_node['y'] + dy * s)

    def add_rrt_star_node(self, tree, new_node, nearest_idx):
        neighbors = self.near(tree, new_node)
        best_p = nearest_idx
        best_c = tree[nearest_idx]['cost'] + self.line_cost(tree[nearest_idx], new_node)
        for idx in neighbors:
            c = tree[idx]['cost'] + self.line_cost(tree[idx], new_node)
            if c + 1e-9 < best_c and not self.check_collision(tree[idx], new_node):
                best_c = c
                best_p = idx

        new_node['parent'] = best_p
        new_node['cost']   = best_c
        tree[best_p]['children'].append(len(tree))
        tree.append(new_node)
        ni = len(tree) - 1

        for idx in neighbors:
            if idx == best_p or idx == 0:
                continue
            c = tree[ni]['cost'] + self.line_cost(tree[ni], tree[idx])
            if c + 1e-9 < tree[idx]['cost'] and not self.check_collision(tree[ni], tree[idx]):
                old_p = tree[idx]['parent']
                if old_p >= 0 and idx in tree[old_p]['children']:
                    tree[old_p]['children'].remove(idx)
                tree[idx]['parent'] = ni
                tree[ni]['children'].append(idx)
                tree[idx]['cost'] = c
                self.propagate_costs(tree, idx)
        return ni

    def propagate_costs(self, tree, parent_idx):
        # Iterative to avoid deep recursion on large trees
        stack = [parent_idx]
        while stack:
            pi = stack.pop()
            p = tree[pi]
            for ci in p['children']:
                tree[ci]['cost'] = p['cost'] + self.line_cost(p, tree[ci])
                stack.append(ci)

    def near(self, tree, node):
        out = []
        r = self.search_radius
        for i, n in enumerate(tree):
            if math.hypot(n['x'] - node['x'], n['y'] - node['y']) <= r:
                out.append(i)
        return out

    def check_collision(self, n1, n2):
        v1, x0, y0 = self.world_to_grid(n1['x'], n1['y'])
        v2, x1, y1 = self.world_to_grid(n2['x'], n2['y'])
        if not v1 or not v2:
            return True
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        cx, cy = x0, y0
        while True:
            if not self.grid_in_bounds(cx, cy) or self.grid_occupied(cx, cy):
                return True
            if cx == x1 and cy == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; cx += sx
            if e2 < dx:
                err += dx; cy += sy
        return False

    def is_goal(self, node, gx, gy):
        return math.hypot(node['x'] - gx, node['y'] - gy) < self.goal_threshold

    def find_path(self, tree, idx):
        path = []
        while idx >= 0:
            path.append(tree[idx])
            if tree[idx]['is_root']:
                break
            idx = tree[idx]['parent']
        return path[::-1]

    def line_cost(self, n1, n2):
        return math.hypot(n2['x'] - n1['x'], n2['y'] - n1['y'])

    def shortcut_path(self, path):
        if len(path) <= 2:
            return path
        result = [path[0]]
        i = 0
        while i < len(path) - 1:
            best = i + 1
            for j in range(len(path) - 1, i + 1, -1):
                if self.line_cost(path[i], path[j]) > self.max_shortcut_dist:
                    continue
                if not self.check_collision(path[i], path[j]):
                    best = j
                    break
            result.append(path[best])
            i = best
        return result

    # ========================================================================
    # Publishing
    # ========================================================================
    def publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

    def publish_occupancy_grid(self):
        m = OccupancyGrid()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = float(self.grid_resolution)
        m.info.width  = self.grid_width
        m.info.height = self.grid_height
        m.info.origin.position.x = -(self.grid_width  / 2.0) * self.grid_resolution
        m.info.origin.position.y = -(self.grid_height / 2.0) * self.grid_resolution
        m.info.origin.orientation.w = 1.0
        m.data = [100 if v else 0 for v in self.occupancy_grid]
        self.grid_pub.publish(m)

    # ========================================================================
    # Visualization
    # ========================================================================
    def visualize_waypoints(self):
        if not self.waypoints:
            return
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'waypoints'; m.id = 0
        m.type = Marker.POINTS; m.action = Marker.ADD
        m.scale.x = 0.10; m.scale.y = 0.10
        m.color.r = 1.0; m.color.g = 1.0; m.color.a = 1.0
        for wp in self.waypoints:
            m.points.append(Point(x=wp[0], y=wp[1], z=0.0))
        self.waypoints_pub.publish(m)

    def visualize_lookahead(self, x, y):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'lookahead'; m.id = 1
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y
        m.scale.x = 0.20; m.scale.y = 0.20; m.scale.z = 0.20
        if self.mode == Mode.PURE_PURSUIT:
            m.color.g = 1.0
        else:
            m.color.r = 1.0
        m.color.a = 1.0
        self.lookahead_pub.publish(m)

    def visualize_path(self, path):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'rrt_path'; m.id = 2
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.06
        m.color.r = 1.0; m.color.b = 0.5; m.color.a = 1.0
        for p in path:
            m.points.append(Point(x=p[0], y=p[1], z=0.0))
        self.path_pub.publish(m)

    def visualize_tree(self, tree):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'rrt_tree'; m.id = 3
        m.type = Marker.LINE_LIST; m.action = Marker.ADD
        m.scale.x = 0.015
        m.color.g = 0.7; m.color.b = 0.2; m.color.a = 0.5
        for i in range(1, len(tree)):
            p = tree[i]['parent']
            if p < 0:
                continue
            m.points.append(Point(x=tree[p]['x'], y=tree[p]['y'], z=0.0))
            m.points.append(Point(x=tree[i]['x'], y=tree[i]['y'], z=0.0))
        self.tree_pub.publish(m)

    def visualize_goal(self, x, y):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'rrt_goal'; m.id = 4
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y
        m.scale.x = 0.30; m.scale.y = 0.30; m.scale.z = 0.30
        m.color.r = 1.0; m.color.g = 0.4; m.color.a = 1.0
        self.goal_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = RaceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a zero-velocity command before exiting to ensure safe stop
        try:
            stop = AckermannDriveStamped()
            stop.header.stamp = node.get_clock().now().to_msg()
            stop.drive.speed = 0.0
            stop.drive.steering_angle = 0.0
            node.drive_pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()