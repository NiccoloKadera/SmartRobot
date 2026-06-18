from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import tkinter as tk
from tkinter import messagebox

from moovment_manager import MoovmentManager
from network.roslibpy_connector import RoslibpyConnector


class MainRemote:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("SmartRobot Remote")
        self.root.geometry("520x360")
        self.root.configure(bg="#111827")

        self.connector = RoslibpyConnector()
        self.movement_manager = MoovmentManager(self.connector)
        self.pressed_keys: set[str] = set()
        self.connected = False

        self.recent_path = Path("recent_connections.json")
        default_host, default_port = self._load_most_recent()
        self.host_var = tk.StringVar(value=default_host or "127.0.0.1")
        self.port_var = tk.StringVar(value=str(default_port or 9090))
        
        # Campo dinamico per decidere il nome del robot basandoci sulla tua ROS topic list
        self.robot_var = tk.StringVar(value="car2")
        self.status_var = tk.StringVar(value="Non connesso")

        self._build_ui()
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(40, self._tick)

    def _build_ui(self):
        title = tk.Label(
            self.root,
            text="SmartRobot Remote",
            bg="#111827",
            fg="#f9fafb",
            font=("Helvetica", 20, "bold"),
        )
        title.pack(pady=(18, 6))

        subtitle = tk.Label(
            self.root,
            text="Controllo OmniBlue flotta: WASD (Traslazione) + Q/E (Rotazione)",
            bg="#111827",
            fg="#9ca3af",
            font=("Helvetica", 11),
        )
        subtitle.pack(pady=(0, 14))

        form = tk.Frame(self.root, bg="#111827")
        form.pack(pady=6)

        tk.Label(form, text="ROS bridge host", bg="#111827", fg="#e5e7eb").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        tk.Entry(form, textvariable=self.host_var, width=20).grid(row=0, column=1, padx=6, pady=4)

        tk.Label(form, text="Port", bg="#111827", fg="#e5e7eb").grid(row=1, column=0, sticky="w", padx=6, pady=4)
        tk.Entry(form, textvariable=self.port_var, width=20).grid(row=1, column=1, padx=6, pady=4)

        tk.Label(form, text="Seleziona Robot (es. car2)", bg="#111827", fg="#e5e7eb").grid(row=2, column=0, sticky="w", padx=6, pady=4)
        tk.Entry(form, textvariable=self.robot_var, width=20).grid(row=2, column=1, padx=6, pady=4)

        actions = tk.Frame(self.root, bg="#111827")
        actions.pack(pady=14)

        tk.Button(actions, text="Connetti", command=self.connect, width=12).grid(row=0, column=0, padx=6)
        tk.Button(actions, text="Stop", command=self.stop, width=12).grid(row=0, column=1, padx=6)
        tk.Button(actions, text="Chiudi", command=self.close, width=12).grid(row=0, column=2, padx=6)

        help_text = tk.Label(
            self.root,
            text="W/S avanti-indietro, A/D laterale. Q/E rotazione. Tieni attiva la finestra per guidare.",
            bg="#111827",
            fg="#d1d5db",
            wraplength=460,
            justify="center",
        )
        help_text.pack(padx=18, pady=(0, 10))

        status = tk.Label(
            self.root,
            textvariable=self.status_var,
            bg="#111827",
            fg="#34d399",
            font=("Helvetica", 11, "bold"),
        )
        status.pack(pady=(0, 16))

        self.root.focus_force()

    def connect(self):
        try:
            host = self.host_var.get().strip()
            port = int(self.port_var.get().strip())
            robot_name = self.robot_var.get().strip().lower()
            
            # Genera il topic corretto dinamicamente prendendo il valore inserito nel form
            self.movement_manager.cmd_vel_topic = f"/{robot_name}/cmd_vel"
            self.movement_manager.connect(host, port)
            self.connected = True
            self.status_var.set(f"Connesso a {host}:{port} ({robot_name})")
            self.root.focus_force()
            try:
                self._save_connection(host, port)
            except (OSError, json.JSONDecodeError, ValueError):
                pass
        except (ValueError, ConnectionError, RuntimeError, OSError) as exc:
            self.connected = False
            self.status_var.set("Connessione fallita")
            messagebox.showerror("Connessione ROS", str(exc))

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
                            message_type="std_msgs/msg/String"
                        )
                    except (ConnectionError, RuntimeError, OSError) as exc:
                        print(f"Debug pub failed: {exc}")
            self.pressed_keys.add(key)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        self.pressed_keys.discard(key)

    def _tick(self):
        if self.connected:
            try:
                self.movement_manager.apply_pressed_keys(self.pressed_keys)
            except (ConnectionError, RuntimeError, OSError) as exc:
                self.connected = False
                self.status_var.set(f"Errore ROS: {exc}")
                self.pressed_keys.clear()
        self.root.after(40, self._tick)


def main():
    app = MainRemote()
    app.root.mainloop()


if __name__ == "__main__":
    main()