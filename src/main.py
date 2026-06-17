from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
import os
import math

import tkinter as tk
from tkinter import messagebox

# Importazioni per Matplotlib integrate in Tkinter
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from moovment_manager import MoovmentManager
from network.roslibpy_connector import RoslibpyConnector


class MainRemote:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("SmartRobot Remote")
        self.root.geometry("900x450" if os.getenv('SHOW_LIDAR', 'false').lower() in ('true', '1') else "520x360")
        self.root.configure(bg="#111827")

        self.connector = RoslibpyConnector()
        self.movement_manager = MoovmentManager(self.connector)
        self.pressed_keys: set[str] = set()
        self.connected = False

        self.recent_path = Path("recent_connections.json")
        default_host, default_port = self._load_most_recent()
        self.host_var = tk.StringVar(value=default_host or "127.0.0.1")
        self.port_var = tk.StringVar(value=str(default_port or 9090))
        
        self.robot_var = tk.StringVar(value=os.getenv('ROBOT_NAME', 'car2'))
        self.status_var = tk.StringVar(value="Non connesso")

        # --- SETUP GRAPHICS COMPONENT ---
        self.show_lidar_enabled = os.getenv('SHOW_LIDAR', 'true').lower() in ('true', '1')
        self.canvas = None
        self.ax = None

        self._build_ui()
        self.root.bind_all("<KeyPress> ", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(40, self._tick)

    def _build_ui(self):
        # Layout principale sdoppiato a pannelli se abilitato il radar grafico
        main_container = tk.Frame(self.root, bg="#111827")
        main_container.pack(fill="both", expand=True)

        left_panel = tk.Frame(main_container, bg="#111827")
        left_panel.pack(side="left", padx=20, fill="y")

        title = tk.Label(left_panel, text="SmartRobot Remote", bg="#111827", fg="#f9fafb", font=("Helvetica", 18, "bold"))
        title.pack(pady=(15, 4))

        form = tk.Frame(left_panel, bg="#111827")
        form.pack(pady=6)

        tk.Label(form, text="ROS bridge host", bg="#111827", fg="#e5e7eb").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        tk.Entry(form, textvariable=self.host_var, width=16).grid(row=0, column=1, padx=4, pady=2)

        tk.Label(form, text="Port", bg="#111827", fg="#e5e7eb").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        tk.Entry(form, textvariable=self.port_var, width=16).grid(row=1, column=1, padx=4, pady=2)

        tk.Label(form, text="Robot name", bg="#111827", fg="#e5e7eb").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        tk.Entry(form, textvariable=self.robot_var, width=16).grid(row=2, column=1, padx=4, pady=2)

        actions = tk.Frame(left_panel, bg="#111827")
        actions.pack(pady=10)

        tk.Button(actions, text="Connetti", command=self.connect, width=10).grid(row=0, column=0, padx=4)
        tk.Button(actions, text="Stop", command=self.stop, width=10).grid(row=0, column=1, padx=4)
        tk.Button(actions, text="Chiudi", command=self.close, width=10).grid(row=0, column=2, padx=4)

        status = tk.Label(left_panel, textvariable=self.status_var, bg="#111827", fg="#34d399", font=("Helvetica", 11, "bold"))
        status.pack(pady=10)

        # --- SEZIONE MATPLOTLIB (POLAR RADAR) ---
        if self.show_lidar_enabled:
            right_panel = tk.Frame(main_container, bg="#1f2937")
            right_panel.pack(side="right", fill="both", expand=True, padx=10, pady=10)

            fig = Figure(figsize=(4, 4), facecolor='#1f2937')
            self.ax = fig.add_subplot(111, polar=True)
            self.ax.set_facecolor('#111827')
            self.ax.tick_params(colors='#9ca3af', labelsize=8)
            self.ax.grid(True, color='#374151')

            self.canvas = FigureCanvasTkAgg(fig, master=right_panel)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.root.focus_force()

    def connect(self):
        try:
            host = self.host_var.get().strip()
            port = int(self.port_var.get().strip())
            robot_name = self.robot_var.get().strip().lower()
            
            self.movement_manager.cmd_vel_topic = f"/{robot_name}/cmd_vel"
            self.movement_manager.connect(host, port)
            self.movement_manager.start_lidar_subscription(scan_topic="/scan")
            
            self.connected = True
            self.status_var.set("Connesso + LiDAR attivo")
            self.root.focus_force()
            self._save_connection(host, port)
        except Exception as exc:
            self.connected = False
            self.status_var.set("Connessione fallita")
            messagebox.showerror("ROS Error", str(exc))

    def _update_lidar_plot(self):
        """Aggiorna il disegno del radar in coordinate polari."""
        if not self.show_lidar_enabled or self.ax is None:
            return

        ranges = self.movement_manager.raw_ranges
        if not ranges:
            return

        self.ax.clear()
        self.ax.grid(True, color='#374151')

        # Calcola gli angoli associati a ciascun raggio laser
        num_points = len(ranges)
        angles = [i * (2 * math.pi / num_points) for i in range(num_points)]

        # Disegna la nuvola totale di punti LiDAR (punti verdi)
        self.ax.scatter(angles, ranges, s=2, color='#34d399', alpha=0.7, label='LiDAR Scan')

        # Disegna il robot al centro (cerchio blu)
        self.ax.scatter([0], [0], s=120, color='#3b82f6', zorder=5, label='Robot Center')

        # Evidenzia il punto scelto dall'algoritmo (X rossa sul muro)
        chosen_idx = self.movement_manager.chosen_point_idx
        if chosen_idx is not None and chosen_idx < len(ranges):
            r_val = ranges[chosen_idx]
            theta_val = angles[chosen_idx]
            if r_val is not None and r_val != float('inf'):
                self.ax.scatter([theta_val], [r_val], s=150, color='#ef4444', marker='X', zorder=10, label='Target Wall')

        self.ax.set_rmax(max([r for r in ranges if r is not None and r < 10] or [4.0]))
        self.canvas.draw()

    def stop(self):
        self.pressed_keys.clear()
        if self.connected:
            self.movement_manager.stop()

    def close(self):
        self.stop()
        self.connector.close()
        self.root.destroy()

    def _load_most_recent(self) -> tuple[str | None, int | None]:
        try:
            if not self.recent_path.exists(): return None, None
            with self.recent_path.open("r", encoding="utf-8") as f: data = json.load(f)
            if not data: return None, None
            latest = max(data, key=lambda e: e.get("date", ""))
            return latest.get("host"), int(latest.get("port", 9090))
        except Exception: return None, None

    def _save_connection(self, host: str, port: int):
        entry = {"host": host, "port": int(port), "date": datetime.utcnow().isoformat()}
        data = []
        if self.recent_path.exists():
            try:
                with self.recent_path.open("r", encoding="utf-8") as f: data = json.load(f)
            except Exception: data = []
        data.append(entry)
        data = data[-20:]
        with self.recent_path.open("w", encoding="utf-8") as f: json.dump(data, f, indent=2)

    def _on_key_press(self, event):
        key = event.keysym.lower()
        print(f'Key pressed: {key}')
        if key in {"w", "a", "s", "d", "q", "e"}:
            self.pressed_keys.add(key)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        self.pressed_keys.discard(key)

    def _tick(self):
        if self.connected:
            try:
                self.movement_manager.apply_pressed_keys(self.pressed_keys)
                # Forza il refresh del grafico ad ogni ciclo di calcolo
                self._update_lidar_plot()
            except Exception as exc:
                self.connected = False
                self.status_var.set(f"ROS Error: {exc}")
                self.pressed_keys.clear()
        self.root.after(40, self._tick)

def main():
    app = MainRemote()
    app.root.mainloop()

if __name__ == "__main__":
    main()