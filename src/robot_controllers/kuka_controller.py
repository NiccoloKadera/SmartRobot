from .base_controller import RobotController

class KukaManualController(RobotController):
    def __init__(self, robot):
        super().__init__(robot)
        # Nomi dei motori dello YouBot (standard PROTO)
        self.motor_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
        self.motors = []
        
        for name in self.motor_names:
            motor = self.robot.getDevice(name)
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            self.motors.append(motor)
        
        self.speed = 5.0 # Velocità massima rad/s

    def update(self):
        key = self.keyboard.getKey()
        
        # Inizializza vettori velocità [front_left, front_right, back_left, back_right]
        v = [0.0, 0.0, 0.0, 0.0]

        if key == ord('W'): # Avanti
            v = [1, 1, 1, 1]
        elif key == ord('S'): # Indietro
            v = [-1, -1, -1, -1]
        elif key == ord('A'): # Laterale Sinistra
            v = [-1, 1, 1, -1]
        elif key == ord('D'): # Laterale Destra
            v = [1, -1, -1, 1]
        elif key == ord('Q'): # Rotazione Sinistra
            v = [-1, 1, -1, 1]
        elif key == ord('E'): # Rotazione Destra
            v = [1, -1, 1, -1]
        else:
            self.stop()
            return

        # Applica velocità moltiplicata per il gain
        for i in range(4):
            self.motors[i].setVelocity(v[i] * self.speed)

    def stop(self):
        for motor in self.motors:
            motor.setVelocity(0.0)