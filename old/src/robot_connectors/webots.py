from controller import Robot

class WebotsConnector:
    """
    Questa classe si occupa solo di inizializzare e fornire l'istanza del Robot.
    Non contiene logica di controllo o loop.
    """
    def __init__(self):
        self.robot = Robot()
        print(f"--- Robot '{self.robot.getName()}' connesso ---")

    def get_robot(self):
        return self.robot