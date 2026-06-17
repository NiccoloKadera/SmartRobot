from __future__ import annotations

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

    def connect(self, host: str | None = None, port: int | None = None):
        self.connector.connect(host, port)

    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        payload = {
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }
        self.connector.send(self.cmd_vel_topic, payload, "geometry_msgs/msg/Twist")

    def stop(self):
        self.publish_twist()

    def apply_pressed_keys(self, pressed_keys: Iterable[str]):
        keys = {key.lower() for key in pressed_keys}
        
        forward = ("w" in keys) - ("s" in keys)
        sideways = ("a" in keys) - ("d" in keys)   # A = sinistra (+), D = destra (-)
        rotation = ("q" in keys) - ("e" in keys)   # Q = rotazione sx (+), E = rotazione dx (-)

        if forward == 0 and sideways == 0 and rotation == 0:
            self.stop()
            return

        self.publish_twist(
            linear_x=forward * self.linear_speed,
            linear_y=sideways * self.lateral_speed,
            angular_z=rotation * self.angular_speed,
        )