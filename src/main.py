from __future__ import annotations

import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import tkinter as tk
from tkinter import messagebox

from moovment_manager import MoovmentManager
from slam_mapper import SimpleLidarSlamMapper

try:
    from network.roslibpy_connector import RoslibpyConnector
except ImportError:
    from roslibpy_connector import RoslibpyConnector


class MainRemote:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("SmartRobot Remote")
        self.root.geometry("1180x700")
        self.root.minsize(980, 620)
        self.root.configure(bg="#111827")

        self.connector = RoslibpyConnector()
        self.movement_manager = MoovmentManager(self.connector)
        self.pressed_keys: set[str] = set()
        self.connected = False

        self.mapper = SimpleLidarSlamMapper(
            map_size_m=14.0,
            resolution_m=0.06,
            max_lidar_range_m=8.0,
        )

        self._lidar_lock = threading.Lock()
        self._latest_scan: dict[str, Any] | None = None
        self._scan_count = 0
        self._last_scan_drawn = -1
        self._last_scan_mapped = -1

        self._lidar_topic: str | None = None
        self._lidar_message_type = "sensor_msgs/msg/LaserScan"

        self._last_tick_time = time.monotonic()

        self.recent_path = Path("recent_connections.json")
        default_host, default_port = self._load_most_recent()

        self.host_var = tk.StringVar(value=default_host or "127.0.0.1")
        self.port_var = tk.StringVar(value=str(default_port or 9090))
        self.robot_var = tk.StringVar(value="car2")
        self.lidar_topic_var = tk.StringVar(value="/scan")

        self.status_var = tk.StringVar(value="Non connesso")
        self.lidar_status_var = tk.StringVar(value="LIDAR non connesso")
        self.slam_status_var = tk.StringVar(value="SLAM locale in attesa dati")

        self._build_ui()

        self.robot_var.trace_add("write", self._on_robot_name_changed)

        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.root.after(40, self._tick)
        self.root.after(100, self._draw_lidar)
        self.root.after(250, self._draw_map)

    def _build_ui(self):
        title = tk.Label(
            self.root,
            text="SmartRobot Remote",
            bg="#111827",
            fg="#f9fafb",
            font=("Helvetica", 22, "bold"),
        )
        title.pack(pady=(16, 4))

        subtitle = tk.Label(
            self.root,
            text="Controllo OmniBlue: WASD traslazione + Q/E rotazione | LIDAR + mappa locale senza /map",
            bg="#111827",
            fg="#9ca3af",
            font=("Helvetica", 11),
        )
        subtitle.pack(pady=(0, 12))

        outer = tk.Frame(self.root, bg="#111827")
        outer.pack(fill="both", expand=True, padx=18, pady=10)

        left = tk.Frame(outer, bg="#111827", width=330)
        left.pack(side="left", fill="y", padx=(0, 16))

        form = tk.Frame(left, bg="#111827")
        form.pack(pady=6, anchor="n")

        tk.Label(
            form,
            text="ROS bridge host",
            bg="#111827",
            fg="#e5e7eb",
        ).grid(row=0, column=0, sticky="w", padx=6, pady=4)

        tk.Entry(
            form,
            textvariable=self.host_var,
            width=22,
        ).grid(row=0, column=1, padx=6, pady=4)

        tk.Label(
            form,
            text="Port",
            bg="#111827",
            fg="#e5e7eb",
        ).grid(row=1, column=0, sticky="w", padx=6, pady=4)

        tk.Entry(
            form,
            textvariable=self.port_var,
            width=22,
        ).grid(row=1, column=1, padx=6, pady=4)

        tk.Label(
            form,
            text="Robot",
            bg="#111827",
            fg="#e5e7eb",
        ).grid(row=2, column=0, sticky="w", padx=6, pady=4)

        tk.Entry(
            form,
            textvariable=self.robot_var,
            width=22,
        ).grid(row=2, column=1, padx=6, pady=4)

        tk.Label(
            form,
            text="Topic LIDAR",
            bg="#111827",
            fg="#e5e7eb",
        ).grid(row=3, column=0, sticky="w", padx=6, pady=4)

        tk.Entry(
            form,
            textvariable=self.lidar_topic_var,
            width=22,
        ).grid(row=3, column=1, padx=6, pady=4)

        actions = tk.Frame(left, bg="#111827")
        actions.pack(pady=14)

        tk.Button(
            actions,
            text="Connetti",
            command=self.connect,
            width=12,
        ).grid(row=0, column=0, padx=5, pady=4)

        tk.Button(
            actions,
            text="Stop",
            command=self.stop,
            width=12,
        ).grid(row=0, column=1, padx=5, pady=4)

        tk.Button(
            actions,
            text="Reset mappa",
            command=self.reset_map,
            width=12,
        ).grid(row=1, column=0, padx=5, pady=4)

        tk.Button(
            actions,
            text="Chiudi",
            command=self.close,
            width=12,
        ).grid(row=1, column=1, padx=5, pady=4)

        help_text = tk.Label(
            left,
            text="W/S avanti-indietro, A/D laterale, Q/E rotazione. "
                 "La mappa viene costruita dai punti LIDAR e dalla posa stimata dai comandi.",
            bg="#111827",
            fg="#d1d5db",
            wraplength=300,
            justify="center",
        )
        help_text.pack(padx=12, pady=(0, 10))

        tk.Label(
            left,
            textvariable=self.status_var,
            bg="#111827",
            fg="#34d399",
            font=("Helvetica", 11, "bold"),
            wraplength=310,
            justify="center",
        ).pack(pady=(0, 8))

        tk.Label(
            left,
            textvariable=self.lidar_status_var,
            bg="#111827",
            fg="#93c5fd",
            font=("Helvetica", 10, "bold"),
            wraplength=310,
            justify="center",
        ).pack(pady=(0, 8))

        tk.Label(
            left,
            textvariable=self.slam_status_var,
            bg="#111827",
            fg="#fbbf24",
            font=("Helvetica", 10, "bold"),
            wraplength=310,
            justify="center",
        ).pack(pady=(0, 8))

        graphs = tk.Frame(outer, bg="#111827")
        graphs.pack(side="right", fill="both", expand=True)

        lidar_panel = tk.Frame(graphs, bg="#111827")
        lidar_panel.pack(side="left", fill="both", expand=True, padx=(0, 8))

        tk.Label(
            lidar_panel,
            text="Vista LIDAR live",
            bg="#111827",
            fg="#f9fafb",
            font=("Helvetica", 14, "bold"),
        ).pack(pady=(0, 8))

        self.lidar_canvas = tk.Canvas(
            lidar_panel,
            width=390,
            height=470,
            bg="#020617",
            highlightthickness=1,
            highlightbackground="#334155",
        )
        self.lidar_canvas.pack(fill="both", expand=True)

        map_panel = tk.Frame(graphs, bg="#111827")
        map_panel.pack(side="right", fill="both", expand=True, padx=(8, 0))

        tk.Label(
            map_panel,
            text="Mappa locale generata dai punti",
            bg="#111827",
            fg="#f9fafb",
            font=("Helvetica", 14, "bold"),
        ).pack(pady=(0, 8))

        self.map_canvas = tk.Canvas(
            map_panel,
            width=390,
            height=470,
            bg="#020617",
            highlightthickness=1,
            highlightbackground="#334155",
        )
        self.map_canvas.pack(fill="both", expand=True)

        self._draw_empty_lidar("In attesa scan")
        self._draw_empty_map("In attesa mappa")
        self.root.focus_force()

    def connect(self):
        try:
            host = self.host_var.get().strip()
            port = int(self.port_var.get().strip())
            robot_name = self.robot_var.get().strip().lower()

            self.movement_manager.cmd_vel_topic = f"/{robot_name}/cmd_vel"
            self.movement_manager.connect(host, port)

            self.connected = True
            self.status_var.set(f"Connesso a {host}:{port} ({robot_name})")

            self._subscribe_lidar()

            try:
                self._save_connection(host, port)
            except (OSError, json.JSONDecodeError, ValueError):
                pass

            self.root.focus_force()

        except (ValueError, ConnectionError, RuntimeError, OSError) as exc:
            self.connected = False
            self.status_var.set("Connessione fallita")
            self.lidar_status_var.set("LIDAR non connesso")
            messagebox.showerror("Connessione ROS", str(exc))

    def _subscribe_lidar(self):
        topic = self.lidar_topic_var.get().strip()

        if not topic:
            topic = f"/{self.robot_var.get().strip().lower()}/scan"
            self.lidar_topic_var.set(topic)

        if self._lidar_topic and self._lidar_topic != topic:
            self.connector.unsubscribe(self._lidar_topic, self._lidar_message_type)

        self._lidar_topic = topic

        with self._lidar_lock:
            self._latest_scan = None
            self._scan_count = 0
            self._last_scan_drawn = -1
            self._last_scan_mapped = -1

        self.connector.subscribe(
            topic=topic,
            message_type=self._lidar_message_type,
            callback=self._on_lidar_scan,
        )

        self.lidar_status_var.set(f"Sottoscritto a {topic}")

    def _on_lidar_scan(self, message: dict[str, Any]):
        with self._lidar_lock:
            self._latest_scan = message
            self._scan_count += 1

    def stop(self):
        self.pressed_keys.clear()

        if self.connected:
            self.movement_manager.stop()

    def reset_map(self):
        self.mapper.reset()

        with self._lidar_lock:
            self._last_scan_mapped = -1

        self.slam_status_var.set("Mappa resettata")
        self._draw_empty_map("Mappa resettata")

    def close(self):
        try:
            self.stop()
        finally:
            self.connector.close()
            self.root.destroy()

    def _load_most_recent(self) -> tuple[str | None, int | None]:
        try:
            if not self.recent_path.exists():
                return None, None

            with self.recent_path.open("r", encoding="utf-8") as f:
                data = json.load(f)

            if not data:
                return None, None

            latest = max(data, key=lambda e: e.get("date", ""))
            return latest.get("host"), int(latest.get("port", 9090))

        except (OSError, json.JSONDecodeError, ValueError, TypeError):
            return None, None

    def _save_connection(self, host: str, port: int):
        entry = {
            "host": host,
            "port": int(port),
            "date": datetime.utcnow().isoformat(),
        }

        data = []

        if self.recent_path.exists():
            try:
                with self.recent_path.open("r", encoding="utf-8") as f:
                    data = json.load(f)
            except (OSError, json.JSONDecodeError, ValueError, TypeError):
                data = []

        data.append(entry)
        data = data[-20:]

        with self.recent_path.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def _on_key_press(self, event):
        key = event.keysym.lower()

        if key in {"w", "a", "s", "d", "q", "e"}:
            if key not in self.pressed_keys:
                print(f"Key pressed: {key}")

                if self.connected:
                    try:
                        self.connector.send(
                            topic="/remote_debug",
                            payload=f"Pressed key: {key.upper()}",
                            message_type="std_msgs/msg/String",
                        )
                    except (ConnectionError, RuntimeError, OSError) as exc:
                        print(f"Debug pub failed: {exc}")

            self.pressed_keys.add(key)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        self.pressed_keys.discard(key)

    def _on_robot_name_changed(self, *_):
        robot_name = self.robot_var.get().strip().lower()

        if robot_name:
            self.lidar_topic_var.set(f"/{robot_name}/scan")

    def _tick(self):
        now = time.monotonic()
        dt = max(0.0, min(now - self._last_tick_time, 0.2))
        self._last_tick_time = now

        if self.connected:
            try:
                self.movement_manager.apply_pressed_keys(self.pressed_keys)

                vx, vy, wz = self.movement_manager.velocity_from_keys(self.pressed_keys)
                self.mapper.update_pose(vx, vy, wz, dt)

            except (ConnectionError, RuntimeError, OSError) as exc:
                self.connected = False
                self.status_var.set(f"Errore ROS: {exc}")
                self.pressed_keys.clear()

        self.root.after(40, self._tick)

    def _draw_empty_lidar(self, text: str):
        self.lidar_canvas.delete("all")

        width = max(self.lidar_canvas.winfo_width(), 390)
        height = max(self.lidar_canvas.winfo_height(), 470)

        cx = width / 2
        cy = height / 2
        radius = min(width, height) * 0.42

        self.lidar_canvas.create_oval(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            outline="#1e293b",
        )

        self.lidar_canvas.create_line(
            cx,
            cy - radius,
            cx,
            cy + radius,
            fill="#1e293b",
        )

        self.lidar_canvas.create_line(
            cx - radius,
            cy,
            cx + radius,
            cy,
            fill="#1e293b",
        )

        self.lidar_canvas.create_oval(
            cx - 5,
            cy - 5,
            cx + 5,
            cy + 5,
            fill="#f8fafc",
            outline="",
        )

        self.lidar_canvas.create_text(
            cx,
            cy + radius + 18,
            text=text,
            fill="#94a3b8",
        )

    def _draw_empty_map(self, text: str):
        self.map_canvas.delete("all")

        width = max(self.map_canvas.winfo_width(), 390)
        height = max(self.map_canvas.winfo_height(), 470)

        cx = width / 2
        cy = height / 2

        self.map_canvas.create_line(
            cx,
            0,
            cx,
            height,
            fill="#1e293b",
        )

        self.map_canvas.create_line(
            0,
            cy,
            width,
            cy,
            fill="#1e293b",
        )

        self.map_canvas.create_oval(
            cx - 5,
            cy - 5,
            cx + 5,
            cy + 5,
            fill="#f8fafc",
            outline="",
        )

        self.map_canvas.create_text(
            cx,
            cy + 26,
            text=text,
            fill="#94a3b8",
        )

    def _draw_lidar(self):
        try:
            with self._lidar_lock:
                scan = dict(self._latest_scan) if self._latest_scan else None
                scan_count = self._scan_count

            if scan is None:
                if self._last_scan_drawn != scan_count:
                    self._draw_empty_lidar("In attesa scan")
                    self._last_scan_drawn = scan_count
            else:
                self._render_scan(scan)
                self._last_scan_drawn = scan_count

                if self._last_scan_mapped != scan_count:
                    self.mapper.add_lidar_scan(scan)
                    self._last_scan_mapped = scan_count

        finally:
            self.root.after(100, self._draw_lidar)

    def _render_scan(self, scan: dict[str, Any]):
        ranges = scan.get("ranges") or []

        if not ranges:
            self._draw_empty_lidar("Scan vuoto")
            return

        angle_min = float(scan.get("angle_min", 0.0))
        angle_increment = float(scan.get("angle_increment", 0.0))
        range_min = float(scan.get("range_min", 0.0) or 0.0)
        range_max = float(scan.get("range_max", 0.0) or 0.0)

        finite_ranges = [
            float(r)
            for r in ranges
            if self._is_valid_range(r, range_min, range_max)
        ]

        if not finite_ranges:
            self._draw_empty_lidar("Nessun punto valido")
            return

        max_visible_range = range_max if range_max > 0 else max(finite_ranges)
        max_visible_range = max(max_visible_range, 0.5)

        self.lidar_canvas.delete("all")

        width = max(self.lidar_canvas.winfo_width(), 390)
        height = max(self.lidar_canvas.winfo_height(), 470)

        cx = width / 2
        cy = height / 2
        radius = min(width, height) * 0.43
        scale = radius / max_visible_range

        for factor in (0.25, 0.5, 0.75, 1.0):
            r = radius * factor

            self.lidar_canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                outline="#1e293b",
            )

            self.lidar_canvas.create_text(
                cx + 4,
                cy - r,
                text=f"{max_visible_range * factor:.1f} m",
                fill="#475569",
                anchor="nw",
                font=("Helvetica", 8),
            )

        self.lidar_canvas.create_line(
            cx,
            cy - radius,
            cx,
            cy + radius,
            fill="#1e293b",
        )

        self.lidar_canvas.create_line(
            cx - radius,
            cy,
            cx + radius,
            cy,
            fill="#1e293b",
        )

        points_drawn = 0
        step = max(1, len(ranges) // 720)

        for index in range(0, len(ranges), step):
            raw_range = ranges[index]

            if not self._is_valid_range(raw_range, range_min, range_max):
                continue

            distance = float(raw_range)
            angle = angle_min + index * angle_increment

            # Convenzione grafica:
            # angolo 0 davanti al robot, disegnato verso l'alto.
            x = cx + math.sin(angle) * distance * scale
            y = cy - math.cos(angle) * distance * scale

            self.lidar_canvas.create_oval(
                x - 1.7,
                y - 1.7,
                x + 1.7,
                y + 1.7,
                fill="#38bdf8",
                outline="",
            )

            points_drawn += 1

        self.lidar_canvas.create_oval(
            cx - 7,
            cy - 7,
            cx + 7,
            cy + 7,
            fill="#f8fafc",
            outline="",
        )

        self.lidar_canvas.create_line(
            cx,
            cy,
            cx,
            cy - 26,
            fill="#f8fafc",
            arrow=tk.LAST,
            width=2,
        )

        self.lidar_canvas.create_text(
            12,
            14,
            text=f"{points_drawn} punti | range max {max_visible_range:.2f} m",
            fill="#94a3b8",
            anchor="nw",
            font=("Helvetica", 9),
        )

    def _draw_map(self):
        try:
            self._render_map()
        finally:
            self.root.after(250, self._draw_map)

    def _render_map(self):
        snapshot = self.mapper.get_snapshot()

        grid = snapshot["grid"]
        pose = snapshot["pose"]
        path = snapshot["path"]
        resolution = snapshot["resolution_m"]
        origin = snapshot["origin_cell"]
        map_size_cells = snapshot["map_size_cells"]
        scan_updates = snapshot["scan_updates"]

        if scan_updates <= 0:
            self._draw_empty_map("In attesa punti LIDAR")
            return

        self.map_canvas.delete("all")

        width = max(self.map_canvas.winfo_width(), 390)
        height = max(self.map_canvas.winfo_height(), 470)

        canvas_size = min(width, height) * 0.92
        left = (width - canvas_size) / 2
        top = (height - canvas_size) / 2
        cell_px = canvas_size / map_size_cells

        def grid_to_canvas(gx: int, gy: int) -> tuple[float, float]:
            px = left + gx * cell_px
            py = top + (map_size_cells - gy) * cell_px
            return px, py

        self.map_canvas.create_rectangle(
            left,
            top,
            left + canvas_size,
            top + canvas_size,
            outline="#334155",
        )

        # Griglia metrica ogni 1 metro.
        cells_per_meter = int(round(1.0 / resolution))

        if cells_per_meter > 0:
            for c in range(0, map_size_cells, cells_per_meter):
                x = left + c * cell_px
                y = top + c * cell_px

                self.map_canvas.create_line(
                    x,
                    top,
                    x,
                    top + canvas_size,
                    fill="#0f172a",
                )

                self.map_canvas.create_line(
                    left,
                    y,
                    left + canvas_size,
                    y,
                    fill="#0f172a",
                )

        # Celle occupate.
        threshold = 1.1
        drawn_cells = 0

        for gy, row in enumerate(grid):
            for gx, value in enumerate(row):
                if value < threshold:
                    continue

                x1, y1 = grid_to_canvas(gx, gy + 1)
                x2, y2 = grid_to_canvas(gx + 1, gy)

                self.map_canvas.create_rectangle(
                    x1,
                    y1,
                    x2,
                    y2,
                    fill="#e5e7eb",
                    outline="",
                )

                drawn_cells += 1

        # Path del robot.
        if len(path) > 1:
            points = []

            for wx, wy, _theta in path:
                gx = int(origin + wx / resolution)
                gy = int(origin + wy / resolution)
                px, py = grid_to_canvas(gx, gy)
                points.extend([px, py])

            if len(points) >= 4:
                self.map_canvas.create_line(
                    *points,
                    fill="#fbbf24",
                    width=2,
                )

        # Robot.
        x, y, theta = pose
        rgx = int(origin + x / resolution)
        rgy = int(origin + y / resolution)
        rcx, rcy = grid_to_canvas(rgx, rgy)

        self.map_canvas.create_oval(
            rcx - 6,
            rcy - 6,
            rcx + 6,
            rcy + 6,
            fill="#22c55e",
            outline="",
        )

        heading_len = 22
        hx = rcx + math.cos(theta) * heading_len
        hy = rcy - math.sin(theta) * heading_len

        self.map_canvas.create_line(
            rcx,
            rcy,
            hx,
            hy,
            fill="#22c55e",
            arrow=tk.LAST,
            width=2,
        )

        self.map_canvas.create_text(
            12,
            14,
            text=f"pose x={x:.2f} y={y:.2f} θ={math.degrees(theta):.0f}° | celle occ={drawn_cells}",
            fill="#94a3b8",
            anchor="nw",
            font=("Helvetica", 9),
        )

        self.slam_status_var.set(
            f"SLAM locale attivo | scan integrati: {scan_updates}"
        )

    @staticmethod
    def _is_valid_range(value: Any, range_min: float, range_max: float) -> bool:
        try:
            distance = float(value)
        except (TypeError, ValueError):
            return False

        if not math.isfinite(distance):
            return False

        if range_min > 0 and distance < range_min:
            return False

        if range_max > 0 and distance > range_max:
            return False

        return distance > 0


def main():
    app = MainRemote()
    app.root.mainloop()


if __name__ == "__main__":
    main()