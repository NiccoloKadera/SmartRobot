from __future__ import annotations

import json
import platform
from pathlib import Path

# Importiamo la libreria ufficiale con il suo nome reale
import roslibpy

try:
    from network.connector import Connector
except ImportError:
    from connector import Connector


class RoslibpyConnector(Connector):
    __connection_conf_path = Path("mqtt_connection_conf.json")

    def __init__(self):
        self.client: roslibpy.Ros | None = None
        self._topics: dict[tuple[str, str], roslibpy.Topic] = {}
        self.host: str | None = None
        self.port: int | None = None
        super().__init__()

    def connect(self, host: str | None = None, port: int | None = None):
        if host is None or port is None:
            conf = self.get_json_connection_config()
            host = conf.get("ip", host)
            port = conf.get("port", port)

        if host is None or port is None:
            host, port = self.get_input_connection_config()

        self.host = host
        self.port = int(port)
        self.client = roslibpy.Ros(host=self.host, port=self.port)
        self.client.run()

        if not self.client.is_connected:
            raise ConnectionError(f"Impossible to connect to ROS bridge at {self.host}:{self.port}")

    def get_input_connection_config(self):
        broker_address = input("Enter ROS bridge IP address: ")
        port = int(input("Enter ROS bridge port: "))
        return broker_address, port

    def get_json_connection_config(self):
        if not self.__connection_conf_path.exists():
            return {}
        with self.__connection_conf_path.open("r", encoding="utf-8") as f:
            return json.load(f)

    def _require_client(self):
        if self.client is None or not self.client.is_connected:
            raise RuntimeError("ROS bridge is not connected")

    def _get_topic(self, topic: str, message_type: str):
        self._require_client()
        key = (topic, message_type)
        ros_topic = self._topics.get(key)
        if ros_topic is None:
            ros_topic = roslibpy.Topic(self.client, topic, message_type)
            ros_topic.advertise()
            self._topics[key] = ros_topic
        return ros_topic

    def send(self, topic: str, payload, message_type: str = "std_msgs/String"):
        ros_topic = self._get_topic(topic, message_type)
        if message_type in ("std_msgs/String", "std_msgs/msg/String"):
            message = roslibpy.Message({"data": str(payload)})
        elif isinstance(payload, dict):
            message = roslibpy.Message(payload)
        else:
            raise TypeError("Payload must be a dict for non-string ROS messages")
        ros_topic.publish(message)

    def show_connection(self):
        if self.client is None:
            print("ROS bridge not connected")
            return
        print(f"Connected to ROS bridge at {self.host}:{self.port}")

    def send_test(self):
        machine_name = platform.node()
        self.send("/test/topic", f"Hello from {machine_name}!")

    def close(self):
        for ros_topic in self._topics.values():
            try:
                ros_topic.unadvertise()
            except (AttributeError, RuntimeError, OSError):
                pass
        self._topics.clear()

        if self.client is not None:
            try:
                self.client.terminate()
            except (AttributeError, RuntimeError, OSError):
                pass
            self.client = None


MQTTConnector = RoslibpyConnector


if __name__ == "__main__":
    connector = RoslibpyConnector()
    connector.connect()
    connector.show_connection()
    connector.send_test()