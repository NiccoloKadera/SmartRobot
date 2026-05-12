from abc import ABC, abstractmethod

class Connector(ABC):

    @abstractmethod
    def connect(self):
        pass

    
    