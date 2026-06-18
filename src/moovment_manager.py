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

        # --- PARAMETRI DI CONTROLLO CONTINUO (PID / PROPORZIONALE) ---
        self.Kp_yaw = 0.08                  # Guadagno per correggere l'orientamento (angolo)
        self.Kp_lat = 0.15                  # Guadagno per correggere la posizione (distanza dal muro)
        
        self.current_lidar_angle = 0.0      # Angolo istantaneo calcolato in background dal LiDAR
        self.target_lidar_angle = 0.0       # Angolo ideale iniziale fissato al click di A/D
        
        self.current_wall_distance = 0.0    # Distanza reale istantanea calcolata dal LiDAR
        self.target_wall_distance = 0.0     # Distanza ideale iniziale fissata al click di A/D
        
        self.is_correcting = False          # Flag di attivazione dell'anello chiuso (Solo per A/D)
        self._last_print_time = 0.0         # Timer per limitare i log in console

        # Variabili di telemetria usate da main.py per disegnare il grafico radar
        self.raw_ranges: list[float] = []   
        self.chosen_point_idx: int | None = None  

    def connect(self, host: str | None = None, port: int | None = None):
        self.connector.connect(host, port)

    def start_lidar_subscription(self, scan_topic: str = "/scan"):
        """Si iscrive al topic del LiDAR per aggiornare costantemente il grafico in background."""
        try:
            self.connector.listen(topic=scan_topic, callback=self._handle_lidar_data)
            print("[LiDAR] Sottoscrizione attiva. Grafico e tracking di sfondo abilitati.")
        except Exception as e:
            print(f"[LiDAR] ERRORE durante la sottoscrizione: {e}")

    def _handle_lidar_data(self, message: dict):
        """Elabora SEMPRE i dati del LiDAR per aggiornare la GUI, evitando salti di riferimento."""
        ranges = message.get("ranges", [])
        if not ranges:
            return

        self.raw_ranges = ranges
        num_points = len(ranges)
        angle_increment = (2 * math.pi) / num_points

        # Finestra geometrica centrata sui 90 gradi (sinistra del robot)
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
                    # Estrae la distanza minima reale dalla parete laterale
                    if r < min_distance:
                        min_distance = r
                    
                    theta = i * angle_increment
                    # Trasformazione in coordinate cartesiane locali per la regressione della linea
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
                # Aggiorna l'angolo attuale stimato della parete
                self.current_lidar_angle = math.degrees(math.atan(m))
                # Seta l'indice centrale della linea per disegnare stabilmente la X rossa nel radar
                self.chosen_point_idx = valid_indices[len(valid_indices) // 2]

    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        payload = {
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }
        self.connector.send(self.cmd_vel_topic, payload, "geometry_msgs/msg/Twist")

    def stop(self):
        if self.is_correcting:
            print("[CONTROL] Movimento trasversale terminato. Disattivazione anello chiuso.")
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

        # --- ANELLO CHIUSO ATTIVO SOLO SU MOVIMENTI TRASVERSALI PURI (A / D) ---
        if forward == 0 and rotation == 0 and abs(vy_input) > 0.01:
            # Al primissimo frame in cui premi il tasto, congela i riferimenti esatti letti dallo sfondo
            if not self.is_correcting:
                self.target_lidar_angle = self.current_lidar_angle
                self.target_wall_distance = self.current_wall_distance
                self.is_correcting = True
                print(f"\n[TARGET AGGANCIATO] Distanza: {self.target_wall_distance:.2f}m | Angolo: {self.target_lidar_angle:.2f}°")

            # 1. Correzione dell'Inclinazione (Yaw) - Soglia > 10 gradi
            angular_error = self.target_lidar_angle - self.current_lidar_angle
            while angular_error > 180:  angular_error -= 360
            while angular_error < -180: angular_error += 360
            
            if abs(angular_error) > 10.0:
                yaw_output = angular_error * self.Kp_yaw
            else:
                yaw_output = 0.0

            # 2. Correzione dello scostamento di traiettoria (Posizione laterale Vy)
            # Se scivolando trasversalmente il robot si allontana o avvicina di oltre 2 centimetri:
            distance_error = self.target_wall_distance - self.current_wall_distance
            if abs(distance_error) > 0.02:
                vy_input += (distance_error * self.Kp_lat)

            # Stampa di debug temporizzata ogni 200ms
            now = time.time()
            if now - self._last_print_time > 0.2:
                print(f"[CORREZIONE] Err Dist: {distance_error:.3f}m -> Vy: {vy_input:.3f} | Err Angolo: {angular_error:.1f}° -> Yaw: {yaw_output:.3f}")
                self._last_print_time = now

        else:
            # Se guidi dritto (W/S) o ruoti manualmente (Q/E), disattiva qualsiasi interferenza sui motori
            self.is_correcting = False

        # Invia il vettore cinematico finale a ROS 2
        self.publish_twist(linear_x=vx_input, linear_y=vy_input, angular_z=yaw_output)