from __future__ import annotations

import json
import math
import threading
from datetime import datetime
from pathlib import Path
from typing import Any

import tkinter as tk
from tkinter import messagebox

from moovment_manager import MoovmentManager
try:
    from network.roslibpy_connector import RoslibpyConnector
except ImportError:
    from roslibpy_connector import RoslibpyConnector


class MainRemote:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("SmartRobot Remote")
        self.root.geometry("900x620")
        self.root.minsize(760, 560)
        self.root.configure(bg="#111827")

        self.connector = RoslibpyConnector()
        self.movement_manager = MoovmentManager(self.connector)
        self.pressed_keys: set[str] = set()
        self.connected = False

        # Stato LIDAR: aggiornato dal callback roslibpy e disegnato nel thread Tkinter.
        self._lidar_lock = threading.Lock()
        self._latest_scan: dict[str, Any] | None = None
        self._scan_count = 0
        self._last_scan_count_drawn = -1
        self._lidar_topic: str | None = None
        self._lidar_message_type = "sensor_msgs/msg/LaserScan"

        self.recent_path = Path("recent_connections.json")
        default_host, default_port = self._load_most_recent()
        self.host_var = tk.StringVar(value=default_host or "127.0.0.1")
        self.port_var = tk.StringVar(value=str(default_port or 9090))

        # Campo dinamico per decidere il nome del robot basandoci sulla tua ROS topic list
        self.robot_var = tk.StringVar(value="car2")
        self.lidar_topic_var = tk.StringVar(value="/scan")
        self.status_var = tk.StringVar(value="Non connesso")
        self.lidar_status_var = tk.StringVar(value="LIDAR non connesso")

        self._build_ui()
        self.robot_var.trace_add("write", self._on_robot_name_changed)
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(40, self._tick)
        self.root.after(120, self._draw_lidar)

    def _build_ui(self):
        title = tk.Label(
            self.root,
            text="SmartRobot Remote",
            bg="#111827",
            fg="#f9fafb",
            font=("Helvetica", 22, "bold"),
        )
        title.pack(pady=(18, 6))

        subtitle = tk.Label(
            self.root,
            text="Controllo OmniBlue flotta: WASD (Traslazione) + Q/E (Rotazione) + visualizzazione LIDAR",
            bg="#111827",
            fg="#9ca3af",
            font=("Helvetica", 11),
        )
        subtitle.pack(pady=(0, 12))

        content = tk.Frame(self.root, bg="#111827")
        content.pack(fill="both", expand=True, padx=18, pady=8)

        left = tk.Frame(content, bg="#111827")
        left.pack(side="left", fill="y", padx=(0, 18))

        form = tk.Frame(left, bg="#111827")
        form.pack(pady=6, anchor="n")

        tk.Label(form, text="ROS bridge host", bg="#111827", fg="#e5e7eb").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        tk.Entry(form, textvariable=self.host_var, width=22).grid(row=0, column=1, padx=6, pady=4)

        tk.Label(form, text="Port", bg="#111827", fg="#e5e7eb").grid(row=1, column=0, sticky="w", padx=6, pady=4)
        tk.Entry(form, textvariable=self.port_var, width=22).grid(row=1, column=1, padx=6, pady=4)

        tk.Label(form, text="Seleziona Robot (es. car2)", bg="#111827", fg="#e5e7eb").grid(row=2, column=0, sticky="w", padx=6, pady=4)
        tk.Entry(form, textvariable=self.robot_var, width=22).grid(row=2, column=1, padx=6, pady=4)

        tk.Label(form, text="Topic LIDAR", bg="#111827", fg="#e5e7eb").grid(row=3, column=0, sticky="w", padx=6, pady=4)
        tk.Entry(form, textvariable=self.lidar_topic_var, width=22).grid(row=3, column=1, padx=6, pady=4)

        actions = tk.Frame(left, bg="#111827")
        actions.pack(pady=14)

        tk.Button(actions, text="Connetti", command=self.connect, width=12).grid(row=0, column=0, padx=5, pady=4)
        tk.Button(actions, text="Stop", command=self.stop, width=12).grid(row=0, column=1, padx=5, pady=4)
        tk.Button(actions, text="Chiudi", command=self.close, width=12).grid(row=1, column=0, columnspan=2, padx=5, pady=4)

        help_text = tk.Label(
            left,
            text="W/S avanti-indietro, A/D laterale. Q/E rotazione. Tieni attiva la finestra per guidare.",
            bg="#111827",
            fg="#d1d5db",
            wraplength=300,
            justify="center",
        )
        help_text.pack(padx=18, pady=(0, 10))

        status = tk.Label(
            left,
            textvariable=self.status_var,
            bg="#111827",
            fg="#34d399",
            font=("Helvetica", 11, "bold"),
            wraplength=320,
            justify="center",
        )
        status.pack(pady=(0, 8))

        lidar_status = tk.Label(
            left,
            textvariable=self.lidar_status_var,
            bg="#111827",
            fg="#93c5fd",
            font=("Helvetica", 10, "bold"),
            wraplength=320,
            justify="center",
        )
        lidar_status.pack(pady=(0, 16))

        right = tk.Frame(content, bg="#111827")
        right.pack(side="right", fill="both", expand=True)

        tk.Label(
            right,
            text="Vista LIDAR",
            bg="#111827",
            fg="#f9fafb",
            font=("Helvetica", 14, "bold"),
        ).pack(pady=(0, 8))

        self.lidar_canvas = tk.Canvas(
            right,
            width=430,
            height=430,
            bg="#020617",
            highlightthickness=1,
            highlightbackground="#334155",
        )
        self.lidar_canvas.pack(fill="both", expand=True)
        self._draw_empty_lidar("In attesa scan")

        self.root.focus_force()

    def connect(self):
        try:
            host = self.host_var.get().strip()
            port = int(self.port_var.get().strip())
            robot_name = self.robot_var.get().strip().lower()

            # Genera i topic corretti dinamicamente prendendo il valore inserito nel form.
            self.movement_manager.cmd_vel_topic = f"/{robot_name}/cmd_vel"
            self.movement_manager.connect(host, port)
            self.connected = True
            self.status_var.set(f"Connesso a {host}:{port} ({robot_name})")

            self._subscribe_lidar()
            self.root.focus_force()
            try:
                self._save_connection(host, port)
            except (OSError, json.JSONDecodeError, ValueError):
                pass
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
            self._last_scan_count_drawn = -1

        self.connector.subscribe(topic, self._lidar_message_type, self._on_lidar_scan)
        self.lidar_status_var.set(f"Sottoscritto a {topic}")

    def _on_lidar_scan(self, message: dict[str, Any]):
        with self._lidar_lock:
            self._latest_scan = message
            self._scan_count += 1

    def stop(self):
        self.pressed_keys.clear()
        if self.connected:
            self.movement_manager.stop()

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
        entry = {"host": host, "port": int(port), "date": datetime.utcnow().isoformat()}
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
        if self.connected:
            try:
                self.movement_manager.apply_pressed_keys(self.pressed_keys)
            except (ConnectionError, RuntimeError, OSError) as exc:
                self.connected = False
                self.status_var.set(f"Errore ROS: {exc}")
                self.pressed_keys.clear()
        self.root.after(40, self._tick)

    def _draw_empty_lidar(self, text: str):
        self.lidar_canvas.delete("all")
        width = max(self.lidar_canvas.winfo_width(), 430)
        height = max(self.lidar_canvas.winfo_height(), 430)
        cx, cy = width / 2, height / 2
        radius = min(width, height) * 0.42
        self.lidar_canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#1e293b")
        self.lidar_canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#1e293b")
        self.lidar_canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#1e293b")
        self.lidar_canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#f8fafc", outline="")
        self.lidar_canvas.create_text(cx, cy + radius + 18, text=text, fill="#94a3b8")

    def _draw_lidar(self):
        try:
            with self._lidar_lock:
                scan = dict(self._latest_scan) if self._latest_scan else None
                scan_count = self._scan_count

            if scan is None:
                if self._last_scan_count_drawn != scan_count:
                    self._draw_empty_lidar("In attesa scan")
                    self._last_scan_count_drawn = scan_count
            else:
                self._render_scan(scan)
                self._last_scan_count_drawn = scan_count
        finally:
            self.root.after(120, self._draw_lidar)

    def _render_scan(self, scan: dict[str, Any]):
        ranges = scan.get("ranges") or []
        if not ranges:
            self._draw_empty_lidar("Scan vuoto")
            return

        angle_min = float(scan.get("angle_min", 0.0))
        angle_increment = float(scan.get("angle_increment", 0.0))
        range_min = float(scan.get("range_min", 0.0) or 0.0)
        range_max = float(scan.get("range_max", 0.0) or 0.0)

        finite_ranges = [float(r) for r in ranges if self._is_valid_range(r, range_min, range_max)]
        if not finite_ranges:
            self._draw_empty_lidar("Nessun punto valido")
            return

        # Usa range_max se disponibile, altrimenti scala sui dati ricevuti.
        max_visible_range = range_max if range_max > 0 else max(finite_ranges)
        max_visible_range = max(max_visible_range, 0.5)

        self.lidar_canvas.delete("all")
        width = max(self.lidar_canvas.winfo_width(), 430)
        height = max(self.lidar_canvas.winfo_height(), 430)
        cx, cy = width / 2, height / 2
        radius = min(width, height) * 0.43
        scale = radius / max_visible_range

        # Griglia metrica
        for factor in (0.25, 0.5, 0.75, 1.0):
            r = radius * factor
            self.lidar_canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#1e293b")
            label = f"{max_visible_range * factor:.1f} m"
            self.lidar_canvas.create_text(cx + 4, cy - r, text=label, fill="#475569", anchor="nw", font=("Helvetica", 8))
        self.lidar_canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#1e293b")
        self.lidar_canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#1e293b")

        points_drawn = 0
        step = max(1, len(ranges) // 720)  # limita il numero di ovali se lo scan è molto denso
        for index in range(0, len(ranges), step):
            raw_range = ranges[index]
            if not self._is_valid_range(raw_range, range_min, range_max):
                continue
            distance = float(raw_range)
            angle = angle_min + index * angle_increment

            # Coordinate: davanti al robot verso l'alto nella canvas.
            x = cx + math.sin(angle) * distance * scale
            y = cy - math.cos(angle) * distance * scale
            self.lidar_canvas.create_oval(x - 1.7, y - 1.7, x + 1.7, y + 1.7, fill="#38bdf8", outline="")
            points_drawn += 1

        # Robot e direzione frontale
        self.lidar_canvas.create_oval(cx - 7, cy - 7, cx + 7, cy + 7, fill="#f8fafc", outline="")
        self.lidar_canvas.create_line(cx, cy, cx, cy - 26, fill="#f8fafc", arrow=tk.LAST, width=2)
        self.lidar_canvas.create_text(
            12,
            14,
            text=f"{points_drawn} punti | range max {max_visible_range:.2f} m",
            fill="#94a3b8",
            anchor="nw",
            font=("Helvetica", 9),
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