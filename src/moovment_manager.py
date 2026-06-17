from __future__ import annotations

import math
import time
from collections.abc import Iterable

try:
    from network.roslibpy_connector import RoslibpyConnector
except ImportError:
    from .network.roslibpy_connector import RoslibpyConnector


class MoovmentManager:
    def __init__(
        self,
        connector: RoslibpyConnector,
        cmd_vel_topic: str = "/robot1/cmd_vel",
        linear_speed: float = 0.25,
        lateral_speed: float | None = None,
        angular_speed: float = 1.0,
    ):
        self.connector = connector
        self.cmd_vel_topic = cmd_vel_topic
        self.linear_speed = linear_speed
        self.lateral_speed = linear_speed if lateral_speed is None else lateral_speed
        self.angular_speed = angular_speed

        # --- CONTINUOUS CONTROL PARAMETERS ---
        self.Kp_yaw = 0.08                  # Gain for orientation (angle) correction
        self.Kp_lat = 0.15                  # Gain for position (distance to wall) correction
        
        self.current_lidar_angle = 0.0      # Instantaneous wall angle from line-tracking
        self.target_lidar_angle = 0.0       # Ideal wall angle locked when A/D is pressed
        
        self.current_wall_distance = 0.0    # Instantaneous minimum distance to the wall
        self.target_wall_distance = 0.0     # Ideal distance locked when A/D is pressed
        
        self.is_correcting = False          # Tracking flag for the closed-loop activation state
        self._last_print_time = 0.0         # Console print throttle timer

        # Telemetry variables for the Matplotlib live plot
        self.raw_ranges: list[float] = []   
        self.chosen_point_idx: int | None = None  

    def connect(self, host: str | None = None, port: int | None = None):
        self.connector.connect(host, port)

    def start_lidar_subscription(self, scan_topic: str = "/scan"):
        try:
            self.connector.listen(topic=scan_topic, callback=self._handle_lidar_data)
            print("[LiDAR] Subscription active. Continuous tracking and GUI plotting enabled.")
        except Exception as e:
            print(f"[LiDAR] ERROR: {e}")

    def _handle_lidar_data(self, message: dict):
        """Processes LiDAR data continuously to update the GUI plot and track spatial metrics."""
        ranges = message.get("ranges", [])
        if not ranges:
            return

        self.raw_ranges = ranges
        num_points = len(ranges)
        angle_increment = (2 * math.pi) / num_points

        idx_center_left = int(num_points * 0.25)
        window = int(num_points * 0.06)  

        X_pts = []
        Y_pts = []
        valid_indices = []
        min_distance = float('inf')

        for i in range(idx_center_left - window, idx_center_left + window):
            if 0 <= i < num_points:
                r = ranges[i]
                if r is not None and 0.1 < r < 6.0:
                    if r < min_distance:
                        min_distance = r
                    
                    theta = i * angle_increment
                    X_pts.append(r * math.cos(theta))
                    Y_pts.append(r * math.sin(theta))
                    valid_indices.append(i)

        if min_distance != float('inf'):
            self.current_wall_distance = min_distance

        n = len(X_pts)
        if n > 5:
            sum_x = sum(X_pts)
            sum_y = sum(Y_pts)
            sum_xx = sum(x*x for x in X_pts)
            sum_xy = sum(x*y for x, y in zip(X_pts, Y_pts))

            denominatore = (n * sum_xx - sum_x * sum_x)
            if abs(denominatore) > 1e-5:
                m = (n * sum_xy - sum_x * sum_y) / denominatore
                self.current_lidar_angle = math.degrees(math.atan(m))
                self.chosen_point_idx = valid_indices[len(valid_indices) // 2]

    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        payload = {
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }
        self.connector.send(self.cmd_vel_topic, payload, "geometry_msgs/msg/Twist")

    def stop(self):
        if self.is_correcting:
            print("[CONTROL] Lateral closed-loop deactivated.")
        self.is_correcting = False
        self.publish_twist()

    def apply_pressed_keys(self, pressed_keys: Iterable[str]):
        keys = {key.lower() for key in pressed_keys}
        
        forward = ("w" in keys) - ("s" in keys)
        sideways = ("a" in keys) - ("d" in keys)   
        rotation = ("q" in keys) - ("e" in keys)   

        if forward == 0 and sideways == 0 and rotation == 0:
            self.stop()
            return

        vx_input = forward * self.linear_speed
        vy_input = sideways * self.lateral_speed
        yaw_output = rotation * self.angular_speed

        # --- CONTINUOUS CLOSED-LOOP CONTROL ONLY ON PURE STRAFING (A / D) ---
        if forward == 0 and rotation == 0 and abs(vy_input) > 0.01:
            # First key strike: Lock the baseline physical coordinates from the active background tracking
            if not self.is_correcting:
                self.target_lidar_angle = self.current_lidar_angle
                self.target_wall_distance = self.current_wall_distance
                self.is_correcting = True
                print(f"\n[CLOSED-LOOP ACTIVATED] Target Distance: {self.target_wall_distance:.2f}m | Target Angle: {self.target_lidar_angle:.2f}°")

            # 1. Compute Angular Deviation Error
            angular_error = self.target_lidar_angle - self.current_lidar_angle
            while angular_error > 180:  angular_error -= 360
            while angular_error < -180: angular_error += 360
            
            # Apply Angular Correction deadzone threshold (> 10 degrees)
            if abs(angular_error) > 10.0:
                yaw_output = angular_error * self.Kp_yaw
            else:
                yaw_output = 0.0

            # 2. Compute Distance Translation Error (Drifting adjustment)
            distance_error = self.target_wall_distance - self.current_wall_distance
            
            # Apply correction to linear_y vector if the translation drift exceeds 2 centimeters
            if abs(distance_error) > 0.02:
                vy_input += (distance_error * self.Kp_lat)

            # Throttle telemetry logs to the console every 200ms
            now = time.time()
            if now - self._last_print_time > 0.2:
                print(f"[TRACKING] Dist Error: {distance_error:.3f}m -> Vy: {vy_input:.3f} | Angle Error: {angular_error:.1f}° -> Yaw: {yaw_output:.3f}")
                self._last_print_time = now

        else:
            # If driving straight (W/S) or manuals rotations (Q/E) occur, disable control interference entirely
            self.is_correcting = False

        # Publish execution array to ROS 2
        self.publish_twist(linear_x=vx_input, linear_y=vy_input, angular_z=yaw_output)