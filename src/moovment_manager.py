import roslibpy
import time
from robot import Robot

class MoovmentManager:
    def __init__(self, robot: Robot):
        self.robot = robot        

    def moove_forward(self, speed):
        