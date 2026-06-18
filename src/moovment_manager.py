from __future__ import annotations

from collections.abc import Iterable

try:
    from network.roslibpy_connector import RoslibpyConnector
except ImportError:
    try:
        from roslibpy_connector import RoslibpyConnector
    except ImportError:
        from .network.roslibpy_connector import RoslibpyConnector


class MoovmentManager:
    def __init__(
        self,
        connector: RoslibpyConnector,
        cmd_vel_topic: str = "/robot1/cmd_vel",
        linear_speed: float = 0.2,
        lateral_speed: float | None = None,
        angular_speed: float = 0.25,
        max_linear_speed: float = 0.1,
    ):
        self.connector = connector
        self.cmd_vel_topic = cmd_vel_topic

        self.max_linear_speed = abs(max_linear_speed)

        self.linear_speed = self._clamp(
            linear_speed,
            self.max_linear_speed,
        )

        self.lateral_speed = (
            self.linear_speed
            if lateral_speed is None
            else self._clamp(lateral_speed, self.max_linear_speed)
        )

        self.angular_speed = angular_speed

        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_angular_z = 0.0

    def _clamp(
        self,
        value: float,
        limit: float,
    ) -> float:
        return max(-limit, min(value, limit))

    def connect(
        self,
        host: str | None = None,
        port: int | None = None,
    ):
        self.connector.connect(host, port)

    def publish_twist(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        angular_z: float = 0.0,
    ):
        linear_x = self._clamp(
            linear_x,
            self.max_linear_speed,
        )

        linear_y = self._clamp(
            linear_y,
            self.max_linear_speed,
        )

        self.last_linear_x = linear_x
        self.last_linear_y = linear_y
        self.last_angular_z = angular_z

        payload = {
            "linear": {
                "x": linear_x,
                "y": linear_y,
                "z": 0.0,
            },
            "angular": {
                "x": 0.0,
                "y": 0.0,
                "z": angular_z,
            },
        }

        self.connector.send(
            self.cmd_vel_topic,
            payload,
            "geometry_msgs/msg/Twist",
        )

    def stop(self):
        self.publish_twist()

    def velocity_from_keys(
        self,
        pressed_keys: Iterable[str],
    ) -> tuple[float, float, float]:
        keys = {key.lower() for key in pressed_keys}

        forward = ("w" in keys) - ("s" in keys)
        sideways = ("a" in keys) - ("d" in keys)
        rotation = ("q" in keys) - ("e" in keys)

        linear_x = forward * self.linear_speed
        linear_y = sideways * self.lateral_speed
        angular_z = rotation * self.angular_speed

        linear_x = self._clamp(
            linear_x,
            self.max_linear_speed,
        )

        linear_y = self._clamp(
            linear_y,
            self.max_linear_speed,
        )

        return linear_x, linear_y, angular_z

    def apply_pressed_keys(
        self,
        pressed_keys: Iterable[str],
    ):
        linear_x, linear_y, angular_z = self.velocity_from_keys(pressed_keys)

        if linear_x == 0 and linear_y == 0 and angular_z == 0:
            self.stop()
            return

        self.publish_twist(
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
        )