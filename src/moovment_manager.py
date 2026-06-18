from __future__ import annotations

import math
import time
import numpy as np
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

        # --- PARAMETRI DI CONTROLLO CONTINUO (PID / PROPORZIONALE) ---
        self.Kp_yaw = 0.08                  
        self.Kp_lat = 0.15                  
        
        self.current_lidar_angle = 0.0      
        self.target_lidar_angle = 0.0       
        
        self.current_wall_distance = 0.0    
        self.target_wall_distance = 0.0     
        
        self.is_correcting = False          
        self._last_print_time = 0.0         

        # Variabili di telemetria LiDAR
        self.raw_ranges: list[float] = []   
        self.chosen_point_idx: int | None = None  

        # --- VARIABILI SLAM INTERNO E ARREDO GRIGLIA ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # Espresso in radianti

        self.map_size_meters = 20.0   # Area totale coperta (20 metri x 20 metri)
        self.resolution = 0.1         # Dimensione di una cella della griglia (10 cm)
        self.grid_size = int(self.map_size_meters / self.resolution)
        
        # Matrice di occupazione locale vuota (0 = libero, valori alti = ostacolo)
        self.custom_map = np.zeros((self.grid_size, self.grid_size))

    def connect(self, host: str | None = None, port: int | None = None):
        self.connector.connect(host, port)

    def start_subscriptions(self, scan_topic: str = "/scan", odom_topic: str = "/car2/odom"):
        """Si iscrive simultaneamente al LiDAR e al flusso di Odometria/Slam_toolbox."""
        try:
            # Sottoscrizione al LiDAR Scan
            self.connector.listen(topic=scan_topic, callback=self._handle_lidar_data, message_type="sensor_msgs/msg/LaserScan")
            # Sottoscrizione all'Odometria per tracciare la posa dello SLAM
            self.connector.listen(topic=odom_topic, callback=self._handle_odom_data, message_type="nav_msgs/msg/Odometry")
            print(f"[ROS Bridge] Ascolto attivo su: {scan_topic} e {odom_topic}")
        except Exception as e:
            print(f"[ROS Bridge] ERRORE durante le sottoscrizioni: {e}")

    def _handle_odom_data(self, message: dict):
        """Estrae la posizione cartesiana e l'angolo di orientamento (Yaw) dal robot."""
        pose = message.get("pose", {}).get("pose", {})
        if not pose: return

        # Coordinate Cartesiane del Robot
        position = pose.get("position", {})
        self.robot_x = position.get("x", 0.0)
        self.robot_y = position.get("y", 0.0)

        # Trasformazione Quaternione -> Angolo Eulero (Yaw in radianti)
        ori = pose.get("orientation", {})
        qx, qy, qz, qw = ori.get("x", 0.0), ori.get("y", 0.0), ori.get("z", 0.0), ori.get("w", 1.0)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _handle_lidar_data(self, message: dict):
        """Elabora i raggi LiDAR per il controllo e disegna la griglia cartesiana locale."""
        ranges = message.get("ranges", [])
        if not ranges: return

        self.raw_ranges = ranges
        num_points = len(ranges)
        angle_min = message.get("angle_min", 0.0)
        angle_increment = message.get("angle_increment", (2 * math.pi) / num_points)

        # --- PARTE A: CALCOLO DISTANZA PARETE (REGRESSIONE LINEARE ESISTENTE) ---
        idx_center_left = int(num_points * 0.25)
        window = int(num_points * 0.06)  
        X_pts, Y_pts, valid_indices = [], [], []
        min_distance = float('inf')

        for i in range(idx_center_left - window, idx_center_left + window):
            if 0 <= i < num_points:
                r = ranges[i]
                if r is not None and 0.1 < r < 6.0:
                    if r < min_distance: min_distance = r
                    theta = angle_min + (i * angle_increment)
                    X_pts.append(r * math.cos(theta))
                    Y_pts.append(r * math.sin(theta))
                    valid_indices.append(i)

        if min_distance != float('inf'):
            self.current_wall_distance = min_distance

        if len(X_pts) > 5:
            sum_x, sum_y = sum(X_pts), sum(Y_pts)
            sum_xx = sum(x*x for x in X_pts)
            sum_xy = sum(x*y for x, y in zip(X_pts, Y_pts))
            denominatore = (len(X_pts) * sum_xx - sum_x * sum_x)
            if abs(denominatore) > 1e-5:
                m = (len(X_pts) * sum_xy - sum_x * sum_y) / denominatore
                self.current_lidar_angle = math.degrees(math.atan(m))
                self.chosen_point_idx = valid_indices[len(valid_indices) // 2]

        # --- PARTE B: COSTRUZIONE GENERATIVA DELLA MAPPA LOCALE (MAPPING) ---
        for i, r in enumerate(ranges):
            if r is None or r < 0.1 or r > 8.0 or math.isinf(r) or math.isnan(r):
                continue
            
            # Angolo assoluto del raggio laser corrente
            angle = angle_min + (i * angle_increment)
            
            # Coordinate cartesiane dell'ostacolo RELATIVE al frame robot
            x_local = r * math.cos(angle)
            y_local = r * math.sin(angle)
            
            # Rototraslazione nel sistema di riferimento GLOBALE (Mappa del mondo)
            x_global = self.robot_x + (x_local * math.cos(self.robot_yaw) - y_local * math.sin(self.robot_yaw))
            y_global = self.robot_y + (x_local * math.sin(self.robot_yaw) + y_local * math.cos(self.robot_yaw))
            
            # Trasformazione da coordinate metriche reali ad indici della Matrice numpy
            col_idx = int((x_global + self.map_size_meters / 2) / self.resolution)
            row_idx = int((y_global + self.map_size_meters / 2) / self.resolution)
            
            # Se il punto laser è interno ai confini della matrice, marchiamo l'ostacolo
            if 0 <= col_idx < self.grid_size and 0 <= row_idx < self.grid_size:
                if self.custom_map[row_idx, col_idx] < 100:
                    self.custom_map[row_idx, col_idx] += 15  # Incremento ad accumulo (pulisce il rumore)

    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        payload = {
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }
        self.connector.send(self.cmd_vel_topic, payload, "geometry_msgs/msg/Twist")

    def stop(self):
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

        if forward == 0 and rotation == 0 and abs(vy_input) > 0.01:
            if not self.is_correcting:
                self.target_lidar_angle = self.current_lidar_angle
                self.target_wall_distance = self.current_wall_distance
                self.is_correcting = True

            angular_error = self.target_lidar_angle - self.current_lidar_angle
            while angular_error > 180:  angular_error -= 360
            while angular_error < -180: angular_error += 360
            
            if abs(angular_error) > 10.0:
                yaw_output = angular_error * self.Kp_yaw
            else:
                yaw_output = 0.0

            distance_error = self.target_wall_distance - self.current_wall_distance
            if abs(distance_error) > 0.02:
                vy_input += (distance_error * self.Kp_lat)

            now = time.time()
            if now - self._last_print_time > 0.2:
                self._last_print_time = now
        else:
            self.is_correcting = False

        self.publish_twist(linear_x=vx_input, linear_y=vy_input, angular_z=yaw_output)