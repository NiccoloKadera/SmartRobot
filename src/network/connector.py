from abc import ABC, abstractmethod


class Connector(ABC):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def show_connection(self):
        pass

    @abstractmethod
    def send(self):
        pass

    @abstractmethod
    def send_test(self):
        pass