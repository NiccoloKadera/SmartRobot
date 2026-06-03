from connector import Connector
import json
import platform
try: import paho.mqtt.client as mqtt
except ImportError as e: raise Exception(f"paho-mqtt import error: {e}")

class MQTTConnector(Connector):

    __connection_conf_path = "mqtt_connection_conf.json"

    def __init__(self):
        self.client = mqtt.Client()
        super().__init__()

    def connect(self, broker_address: str | None = None, port: int | None = None):
        if broker_address is None or port is None:
            conf = self.get_json_connection_config()
            broker_address = conf["ip"]
            port = conf["port"]
        
        if broker_address is None or port is None:
            broker_address, port = self.get_input_connection_config()

        self.client.connect(broker_address, port=port)

    def get_input_connection_config(self):
        broker_address = input("Enter MQTT broker IP address: ")
        port = int(input("Enter MQTT broker port: "))
        return broker_address, port

    def get_json_connection_config(self):
        import json
        with open(self.__connection_conf_path, "r") as f:
            conf = json.load(f)
        return conf
    
    def send(self, topic: str, payload: str):
        self.client.publish(topic, payload)
 
    def send_json(self, topic: str, payload: dict):
        payload_str = json.dumps(payload)
        self.send(topic, payload_str)

    def show_connection(self):
        print(f"Connected to MQTT broker at {self.client._host}:{self.client._port}")

    def send_test(self):
        machine_name = platform.node()
        self.send("test/topic", f"Hello from {machine_name}!")

        
if __name__ == "__main__":
    connector = MQTTConnector()
    connector.connect()
    connector.show_connection()
    connector.send_test()