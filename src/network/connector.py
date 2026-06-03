from abc import ABC, abstractmethod


class Connector(ABC):

    @abstractmethod
    def connect(self, host: str | None = None, port: int | None = None):
        pass

    @abstractmethod
    def show_connection(self):
        pass

    @abstractmethod
    def send(self, topic: str, payload, message_type: str = "std_msgs/String"):
        pass

    @abstractmethod
    def send_test(self):
        pass

    def close(self):
        pass