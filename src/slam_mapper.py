from __future__ import annotations

import copy
import math
import threading
from typing import Any


class SimpleLidarSlamMapper:
    """
    Mapping locale leggero senza ROS2 e senza /map.

    Usa:
    - LaserScan ricevuto via roslibpy;
    - posa stimata tramite dead-reckoning dai comandi inviati al robot;
    - occupancy grid con log-odds.

    Non fa loop-closure e non corregge automaticamente la deriva.
    Però permette di generare una mappa locale usando solo i punti del LIDAR.
    """

    def __init__(
        self,
        map_size_m: float = 14.0,
        resolution_m: float = 0.06,
        max_lidar_range_m: float = 8.0,
    ):
        self.map_size_m = float(map_size_m)
        self.resolution_m = float(resolution_m)
        self.max_lidar_range_m = float(max_lidar_range_m)

        self.map_size_cells = int(round(self.map_size_m / self.resolution_m))
        self.origin_cell = self.map_size_cells // 2

        self._lock = threading.Lock()

        self.grid: list[list[float]] = []
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        self.path: list[tuple[float, float, float]] = []
        self.scan_updates = 0

        self.reset()

    def reset(self):
        with self._lock:
            self.grid = [
                [0.0 for _ in range(self.map_size_cells)]
                for _ in range(self.map_size_cells)
            ]

            self.pose_x = 0.0
            self.pose_y = 0.0
            self.pose_theta = 0.0

            self.path = [(0.0, 0.0, 0.0)]
            self.scan_updates = 0

    def update_pose(
        self,
        velocity_x: float,
        velocity_y: float,
        angular_z: float,
        dt: float,
    ):
        """
        Integra la posa stimata.

        Convenzione:
        - velocity_x: avanti/indietro nel frame robot;
        - velocity_y: laterale nel frame robot;
        - angular_z: rotazione yaw;
        - theta: orientamento nel mondo.
        """
        if dt <= 0:
            return

        with self._lock:
            theta = self.pose_theta

            world_vx = math.cos(theta) * velocity_x - math.sin(theta) * velocity_y
            world_vy = math.sin(theta) * velocity_x + math.cos(theta) * velocity_y

            self.pose_x += world_vx * dt
            self.pose_y += world_vy * dt
            self.pose_theta = self._normalize_angle(theta + angular_z * dt)

            if not self.path:
                self.path.append((self.pose_x, self.pose_y, self.pose_theta))
            else:
                last_x, last_y, _ = self.path[-1]
                dist = math.hypot(self.pose_x - last_x, self.pose_y - last_y)

                if dist > 0.03 or abs(angular_z * dt) > 0.03:
                    self.path.append((self.pose_x, self.pose_y, self.pose_theta))

                    if len(self.path) > 2500:
                        self.path = self.path[-2500:]

    def add_lidar_scan(self, scan: dict[str, Any]):
        ranges = scan.get("ranges") or []

        if not ranges:
            return

        try:
            angle_min = float(scan.get("angle_min", 0.0))
            angle_increment = float(scan.get("angle_increment", 0.0))
            range_min = float(scan.get("range_min", 0.0) or 0.0)
            range_max = float(scan.get("range_max", self.max_lidar_range_m) or self.max_lidar_range_m)
        except (TypeError, ValueError):
            return

        usable_range_max = min(range_max, self.max_lidar_range_m)

        # Limita il numero di raggi integrati per non appesantire Tkinter.
        step = max(1, len(ranges) // 360)

        with self._lock:
            robot_x = self.pose_x
            robot_y = self.pose_y
            robot_theta = self.pose_theta

            robot_cell = self._world_to_grid(robot_x, robot_y)

            if robot_cell is None:
                return

            rgx, rgy = robot_cell

            for index in range(0, len(ranges), step):
                raw_range = ranges[index]

                if not self._is_valid_range(raw_range, range_min, usable_range_max):
                    continue

                distance = min(float(raw_range), usable_range_max)

                # ROS LaserScan: angolo 0 tipicamente sull'asse x del robot.
                local_angle = angle_min + index * angle_increment
                world_angle = robot_theta + local_angle

                end_x = robot_x + math.cos(world_angle) * distance
                end_y = robot_y + math.sin(world_angle) * distance

                end_cell = self._world_to_grid(end_x, end_y)

                if end_cell is None:
                    continue

                egx, egy = end_cell

                cells = self._bresenham(rgx, rgy, egx, egy)

                if not cells:
                    continue

                # Celle libere lungo il raggio.
                for gx, gy in cells[:-1]:
                    self._add_log_odds(gx, gy, -0.25)

                # Endpoint occupato.
                self._add_log_odds(egx, egy, 0.85)

            self.scan_updates += 1

    def get_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "grid": copy.deepcopy(self.grid),
                "pose": (self.pose_x, self.pose_y, self.pose_theta),
                "path": list(self.path),
                "resolution_m": self.resolution_m,
                "origin_cell": self.origin_cell,
                "map_size_cells": self.map_size_cells,
                "scan_updates": self.scan_updates,
            }

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int(round(self.origin_cell + x / self.resolution_m))
        gy = int(round(self.origin_cell + y / self.resolution_m))

        if gx < 0 or gx >= self.map_size_cells:
            return None

        if gy < 0 or gy >= self.map_size_cells:
            return None

        return gx, gy

    def _add_log_odds(self, gx: int, gy: int, delta: float):
        if gx < 0 or gx >= self.map_size_cells:
            return

        if gy < 0 or gy >= self.map_size_cells:
            return

        value = self.grid[gy][gx] + delta

        if value > 4.0:
            value = 4.0
        elif value < -4.0:
            value = -4.0

        self.grid[gy][gx] = value

    @staticmethod
    def _bresenham(
        x0: int,
        y0: int,
        x1: int,
        y1: int,
    ) -> list[tuple[int, int]]:
        points: list[tuple[int, int]] = []

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx + dy
        x = x0
        y = y0

        while True:
            points.append((x, y))

            if x == x1 and y == y1:
                break

            e2 = 2 * err

            if e2 >= dy:
                err += dy
                x += sx

            if e2 <= dx:
                err += dx
                y += sy

        return points

    @staticmethod
    def _is_valid_range(
        value: Any,
        range_min: float,
        range_max: float,
    ) -> bool:
        try:
            distance = float(value)
        except (TypeError, ValueError):
            return False

        if not math.isfinite(distance):
            return False

        if distance <= 0:
            return False

        if range_min > 0 and distance < range_min:
            return False

        if range_max > 0 and distance > range_max:
            return False

        return True

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle