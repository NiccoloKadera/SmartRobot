#!/usr/bin/env python3
"""
OmniBlue Fleet Control Panel
============================
One window for the whole fleet. Grows the old robot-dashboard:

  * live availability dot   (green = SSH reachable, re-checked every few s)
  * Connect                 (opens a terminal SSH'd into the robot)
  * LiDAR                   (SSHes in and launches the driver — correct baud per robot)
  * Drive                   (selects the robot; W/A/S/D + Q/E control it from this window)
  * ▶ Add to RViz           (appends this robot's scan to ONE shared .rviz, relaunches RViz)

WASD is holonomic because OmniBlue is an omni/mecanum base:
  W/S = forward/back   A/D = strafe left/right   Q/E = rotate   (release = stop)

------------------------------------------------------------------
 IMPORTANT — topics & frames (read once)
------------------------------------------------------------------
Each robot below has cmd_vel_topic / scan_topic / scan_frame fields.
They default to the CURRENT single-robot reality: plain /cmd_vel, /scan,
frame "laser". That means the panel works today on whichever robot is
connected, one at a time.

To show TWO+ robots in one RViz at once (or drive them independently),
the robots must publish on DISTINCT topics/frames — i.e. namespacing,
which is a small change on the Pi side, not here. When you're ready for
that, just flip these fields to e.g. /robot3/scan + frame robot3/laser
and add the matching namespace on the robot. Until then, two robots
added to RViz would stack on the same origin.
------------------------------------------------------------------
"""

import os
import sys
import socket
import shutil
import subprocess
import threading

# --- DDS env must be set BEFORE rclpy is imported so the participant picks it up ---
LAUNCH_DIR = os.path.expanduser("~/Desktop/Omniblue Launch")
DDS_PROFILE = os.path.join(LAUNCH_DIR, "fastdds.xml")
os.environ.setdefault("ROS_DOMAIN_ID", "0")
if os.path.isfile(DDS_PROFILE):
    os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", DDS_PROFILE)

import tkinter as tk
from tkinter import messagebox

# rclpy is optional: the panel still does everything except in-window WASD
# if ROS isn't sourced. (Launch from a Foxy-sourced shell to enable Drive.)
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    RCLPY_OK = True
except Exception as _e:
    RCLPY_OK = False
    RCLPY_ERR = str(_e)

# ============================================================
#  FLEET CONFIG  — edit here if the fleet changes
# ============================================================
ROBOTS = [
    {"name": "robot1", "host": "robot1.local", "user": "ubuntu",
     "lidar_launch": "rplidar_a2m12_launch.py",
     "cmd_vel_topic": "/robot1/cmd_vel", "scan_topic": "/scan", "scan_frame": "laser",
     "color": "255; 80; 80"},     # red
    {"name": "robot2", "host": "robot2.local", "user": "ubuntu",
     "lidar_launch": "rplidar_a2m8_launch.py",
     "cmd_vel_topic": "/robot2/cmd_vel", "scan_topic": "/scan", "scan_frame": "laser",
     "color": "80; 200; 120"},    # green
    {"name": "robot3", "host": "robot3.local", "user": "ubuntu",
     "lidar_launch": "rplidar_a2m12_launch.py",
     "cmd_vel_topic": "/robot3/cmd_vel", "scan_topic": "/scan", "scan_frame": "laser",
     "color": "90; 150; 255"},    # blue
]

SSH_PORT, POLL_MS, TIMEOUT = 22, 3000, 2.0          # 2.0s: avoids the robot2 false-red
PI_HUMBLE = "source /opt/ros/humble/setup.bash"
PI_DDS = ("export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/omniblue_config/fastdds.xml "
          "&& export ROS_DOMAIN_ID=0")
PC_FOXY = "source /opt/ros/foxy/setup.bash"
# PC-side workspace + Gazebo, lifted verbatim from the working robot2 launcher (v3).
# NB: the PC workspace is ~/omniblue_ws, which is NOT the Pis' ~/ros2_ws.
PC_WS = "source $HOME/omniblue_ws/install/setup.bash"
GAZEBO_LAUNCH = "ros2 launch omniblue_gazebo one_omniblue.launch.py"
# Gazebo's spawn reads /robot_description, which v3 publishes via a STANDALONE
# robot_state_publisher started BEFORE Gazebo (the RSP was pulled out of the
# Gazebo launch). Launching Gazebo without this => empty world / no model.
URDF_XACRO = "$HOME/omniblue_ws/src/omniblue_description/urdf/omniblue.urdf.xacro"

COMBINED_RVIZ = os.path.join(LAUNCH_DIR, "omniblue_combined.rviz")
LIN_SPEED, ANG_SPEED, PUB_HZ = 0.25, 1.0, 10        # m/s, rad/s, publish rate

# ============================================================
#  Reachability
# ============================================================
def is_up(host):
    try:
        with socket.create_connection((host, SSH_PORT), timeout=TIMEOUT):
            return True
    except OSError:
        return False

# ============================================================
#  Terminal launcher (auto-detect emulator)
# ============================================================
def find_terminal():
    if shutil.which("gnome-terminal"):
        return lambda title, c: ["gnome-terminal", "--title", title, "--", "bash", "-c", c]
    if shutil.which("konsole"):
        return lambda title, c: ["konsole", "-p", f"tabtitle={title}", "-e", "bash", "-c", c]
    if shutil.which("xfce4-terminal"):
        return lambda title, c: ["xfce4-terminal", "-T", title, "-x", "bash", "-c", c]
    if shutil.which("xterm"):
        return lambda title, c: ["xterm", "-T", title, "-e", "bash", "-c", c]
    return None

TERM = find_terminal()

def open_terminal(title, cmd):
    if TERM:
        subprocess.Popen(TERM(title, cmd + "; exec bash"))

# ============================================================
#  Combined RViz config writer
# ============================================================
def laserscan_block(r):
    return f"""    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds: {{Max Value: 10, Min Value: -10, Value: true}}
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: {r['color']}
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: {r['name']} scan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.04
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: {r['scan_topic']}
      Use Fixed Frame: true
      Use rainbow: false
      Value: true
"""

def write_combined_rviz(robots):
    """robots = list of robot dicts currently added. Fixed frame = first one's frame."""
    fixed_frame = robots[0]["scan_frame"] if robots else "laser"
    scans = "".join(laserscan_block(r) for r in robots)
    cfg = f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Color: 160; 160; 160
      Enabled: true
      Line Style: {{Line Width: 0.03, Value: Lines}}
      Name: Grid
      Plane: XY
      Plane Cell Count: 20
      Value: true
{scans}  Global Options:
    Background Color: 30; 30; 30
    Fixed Frame: {fixed_frame}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 12
      Focal Point: {{X: 0, Y: 0, Z: 0}}
      Name: Current View
      Pitch: 1.4
      Yaw: 0.78
    Saved: ~
"""
    os.makedirs(LAUNCH_DIR, exist_ok=True)
    with open(COMBINED_RVIZ, "w") as f:
        f.write(cfg)

# ============================================================
#  The app
# ============================================================
class Panel:
    def __init__(self):
        self.rclpy_node = None
        self.publishers = {}
        self.active = None              # robot dict currently driven
        self.pressed = set()            # keys held (debounced)
        self._release_jobs = {}         # pending key-release timers (X11 autorepeat fix)
        self.added = []                 # robots currently in the combined RViz
        self.rviz_proc = None
        self.lidar_procs = {}           # name -> Popen

        if RCLPY_OK:
            try:
                rclpy.init(args=None)
                self.rclpy_node = Node("omniblue_panel_teleop")
            except Exception:
                self.rclpy_node = None

        self._build_ui()
        self._poll()
        self._pub_loop()

    # ---------- UI ----------
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("OmniBlue Fleet")
        self.root.configure(bg="#1e1e1e")
        self.root.geometry("900x400")

        self.rows = {}
        for r in ROBOTS:
            f = tk.Frame(self.root, bg="#1e1e1e")
            f.pack(fill="x", padx=14, pady=6)

            dot = tk.Label(f, text="\u25cf", fg="#888", bg="#1e1e1e", font=("", 16))
            dot.pack(side="left")
            tk.Label(f, text=r["name"], fg="#eee", bg="#1e1e1e",
                     font=("", 12, "bold"), width=7, anchor="w").pack(side="left", padx=6)

            def mk(txt, cmd, r=r):
                b = tk.Button(f, text=txt, command=lambda r=r: cmd(r),
                              relief="flat", bg="#333", fg="#eee",
                              activebackground="#444", padx=6, state="disabled")
                b.pack(side="left", padx=3)
                return b

            b_conn = mk("Connect", self.connect)
            b_lidar = mk("LiDAR", self.toggle_lidar)
            b_drive = mk("Drive", self.set_active)
            b_gazebo = mk("Gazebo", self.launch_gazebo)
            b_rviz = tk.Button(f, text="\u25b6 RViz", command=lambda r=r: self.toggle_rviz(r),
                               relief="flat", bg="#333", fg="#eee",
                               activebackground="#444", padx=6, state="disabled")
            b_rviz.pack(side="left", padx=3)
            # Reboot — packed far right, reddish to flag that it's destructive.
            b_reboot = tk.Button(f, text="Reboot", command=lambda r=r: self.reboot(r),
                                 relief="flat", bg="#5a2a2a", fg="#eee",
                                 activebackground="#7a3a3a", padx=6, state="disabled")
            b_reboot.pack(side="right", padx=3)
            self.rows[r["name"]] = dict(dot=dot, conn=b_conn, lidar=b_lidar,
                                        drive=b_drive, gazebo=b_gazebo,
                                        rviz=b_rviz, reboot=b_reboot)

        self.status = tk.Label(self.root, text="", fg="#9ad", bg="#1e1e1e",
                               font=("", 11), anchor="w", justify="left")
        self.status.pack(fill="x", padx=16, pady=(8, 2))
        self._set_status()

        # ---------- global control bar (not per-robot) ----------
        bar = tk.Frame(self.root, bg="#1e1e1e")
        bar.pack(fill="x", padx=14, pady=(4, 10))
        tk.Button(bar, text="Restart Panel", command=self.restart_panel,
                  relief="flat", bg="#333", fg="#eee",
                  activebackground="#444", padx=10).pack(side="right", padx=3)

        # keyboard focus + bindings for WASD
        self.root.bind("<KeyPress>", self._on_press)
        self.root.bind("<KeyRelease>", self._on_release)
        self.root.focus_set()

    def _set_status(self, extra=""):
        if self.active:
            base = f"Driving {self.active['name']}  —  W/S fwd · A/D strafe · Q/E turn  (click window to focus)"
        elif not RCLPY_OK:
            base = "WASD disabled: ROS not sourced. Launch from a `source /opt/ros/foxy/setup.bash` shell."
        else:
            base = "Click 'Drive' on a robot to control it with the keyboard."
        self.status.config(text=base + extra)

    # ---------- actions ----------
    def connect(self, r):
        open_terminal(r["name"], f'ssh {r["user"]}@{r["host"]}')

    def toggle_lidar(self, r):
        cmd = (f"ssh -t {r['user']}@{r['host']} "
               f"'{PI_DDS} && {PI_HUMBLE} && ros2 launch rplidar_ros {r['lidar_launch']}'")
        open_terminal(f"{r['name']} LiDAR", cmd)

    def reboot(self, r):
        if not messagebox.askyesno(
                f"Reboot {r['name']}",
                f"Reboot {r['name']} ({r['host']})?\n\n"
                "This drops its LiDAR driver, and on robot3 it tears down the drive "
                "stack: can0 comes back DOWN and omniblue_node must be relaunched "
                "after boot (handoff \u00a711)."):
            return
        # ssh -t allocates a TTY so sudo can prompt for a password in the terminal
        # if the ubuntu user lacks NOPASSWD; with NOPASSWD it reboots immediately.
        cmd = f"ssh -t {r['user']}@{r['host']} 'sudo reboot'"
        open_terminal(f"{r['name']} reboot", cmd)

    def launch_gazebo(self, r):
        # The v3 launcher's order is what makes the model appear: a standalone
        # robot_state_publisher publishes /robot_description (the xacro URDF)
        # FIRST, then Gazebo comes up and spawns from that topic. We do both in
        # one terminal: RSP in the background, short settle, then Gazebo in front.
        rsp = (f'ros2 run robot_state_publisher robot_state_publisher '
               f'--ros-args -p robot_description:="$(xacro {URDF_XACRO} '
               f'robot_name:={r["name"]})"')
        cmd = (f'{PC_FOXY} && {PC_WS} && export ROS_DOMAIN_ID=0 && '
               f'export FASTRTPS_DEFAULT_PROFILES_FILE="{DDS_PROFILE}" && '
               f'{rsp} & '
               f'sleep 2 && {GAZEBO_LAUNCH}')
        open_terminal(f"{r['name']} Gazebo", cmd)

    def restart_panel(self):
        if not messagebox.askyesno("Restart panel",
                                   "Restart the control panel process?"):
            return
        # Tidy our own children/handles, then re-exec this same script in place.
        # Detached terminals (Connect/LiDAR/Gazebo) and the robots are unaffected.
        self._kill_rviz()
        if RCLPY_OK and self.rclpy_node:
            try:
                self.rclpy_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        self.root.destroy()
        os.execv(sys.executable, [sys.executable] + sys.argv)

    def set_active(self, r):
        self.active = r
        if RCLPY_OK and self.rclpy_node:
            # Recreate the cmd_vel publisher FRESH on every Drive press. In this
            # fleet a DataWriter created before the robot's node (the subscriber)
            # is up won't match it once the node appears, so an early/stale
            # publisher stays dead. Tearing it down and making a new one here
            # means the recovery is just "press Drive again after the node is
            # up" — no more nuking everything and starting from scratch.
            topic = r["cmd_vel_topic"]
            old = self.publishers.pop(topic, None)
            if old is not None:
                try:
                    self.rclpy_node.destroy_publisher(old)
                except Exception:
                    pass
            self.publishers[topic] = self.rclpy_node.create_publisher(Twist, topic, 10)
        for name, w in self.rows.items():
            w["drive"].config(bg="#2a6" if name == r["name"] else "#333")
        self.root.focus_set()
        self._set_status()
        # Let discovery settle, then say whether the robot's node is actually
        # listening — so "did Drive connect?" stops being guesswork.
        self.root.after(800, self._check_link)

    def _check_link(self):
        if not (self.active and RCLPY_OK and self.rclpy_node):
            return
        pub = self.publishers.get(self.active["cmd_vel_topic"])
        if pub is None:
            return
        if pub.get_subscription_count() > 0:
            self._set_status(f"   \u2014  linked to {self.active['name']} \u2713")
        else:
            self._set_status(
                f"   \u2014  \u26a0 nothing subscribed to "
                f"{self.active['cmd_vel_topic']} — is omniblue_node running on "
                f"{self.active['name']}? (start it, then click Drive again)")

    def toggle_rviz(self, r):
        if r in self.added:
            self.added.remove(r)
            self.rows[r["name"]]["rviz"].config(bg="#333")
        else:
            self.added.append(r)
            self.rows[r["name"]]["rviz"].config(bg="#458")
        if not self.added:
            return self._kill_rviz()
        write_combined_rviz(self.added)
        self._launch_rviz()

    def _kill_rviz(self):
        if self.rviz_proc and self.rviz_proc.poll() is None:
            self.rviz_proc.terminate()
        self.rviz_proc = None

    def _launch_rviz(self):
        self._kill_rviz()
        cmd = (f"{PC_FOXY} && export ROS_DOMAIN_ID=0 && "
               f'export FASTRTPS_DEFAULT_PROFILES_FILE="{DDS_PROFILE}" && '
               f'rviz2 -d "{COMBINED_RVIZ}"')
        self.rviz_proc = subprocess.Popen(["bash", "-c", cmd])

    # ---------- WASD (debounced for X11 key autorepeat) ----------
    def _on_press(self, e):
        k = e.keysym.lower()
        if k in ("w", "a", "s", "d", "q", "e"):
            self.pressed.add(k)
            job = self._release_jobs.pop(k, None)
            if job:
                self.root.after_cancel(job)

    def _on_release(self, e):
        k = e.keysym.lower()
        if k in self.pressed:
            # schedule removal; a real hold re-fires KeyPress and cancels this
            self._release_jobs[k] = self.root.after(60, lambda k=k: self.pressed.discard(k))

    def _pub_loop(self):
        if self.active and RCLPY_OK and self.rclpy_node:
            t = Twist()
            t.linear.x = LIN_SPEED * (("w" in self.pressed) - ("s" in self.pressed))
            t.linear.y = LIN_SPEED * (("a" in self.pressed) - ("d" in self.pressed))
            t.angular.z = ANG_SPEED * (("q" in self.pressed) - ("e" in self.pressed))
            pub = self.publishers.get(self.active["cmd_vel_topic"])
            if pub:
                pub.publish(t)
        self.root.after(int(1000 / PUB_HZ), self._pub_loop)

    # ---------- polling ----------
    def _poll(self):
        def worker():
            res = {r["name"]: is_up(r["host"]) for r in ROBOTS}
            self.root.after(0, lambda: self._update(res))
        threading.Thread(target=worker, daemon=True).start()
        self.root.after(POLL_MS, self._poll)

    def _update(self, res):
        for name, up in res.items():
            w = self.rows[name]
            w["dot"].config(fg="#3ddc84" if up else "#e05555")
            st = "normal" if up else "disabled"
            for key in ("conn", "lidar", "drive", "gazebo", "rviz", "reboot"):
                # don't disable the active drive button mid-drive if it blips
                w[key].config(state=st)

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self._kill_rviz()
            if RCLPY_OK and self.rclpy_node:
                self.rclpy_node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    Panel().run()