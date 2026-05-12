from abc import ABC, abstractmethod
# L'import di Keyboard ora funziona perché main.py ha già configurato il path
from controller import Keyboard

class RobotController(ABC):
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

    @abstractmethod
    def update(self):
        """Metodo da chiamare in ogni loop della simulazione"""
        pass

    @abstractmethod
    def stop(self):
        """Ferma tutti i motori"""
        pass