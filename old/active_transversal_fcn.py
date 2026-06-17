import math

class MecanumController:
    def __init__(self):
        self.Kp = 0.05 
        self.target_angle = 0.0
        self.is_correcting = False

    def get_lidar_heading(self, lidar_points):
        # Aquí va tu lógica para extraer el ángulo actual del LiDAR (en grados)
        return self.extract_dominant_angle(lidar_points)

    def extract_dominant_angle(self, points):
        return 0.0 

    def process_movement(self, key, lidar_points):
        vx_input, vy_input, yaw_input = 0.0, 0.0, 0.0
        
        # 1. MAPEO DE TECLAS (Q = Antihorario [+], E = Horario [-])
        if key == 'w':   vx_input = 1.0   # Forward
        elif key == 's': vx_input = -1.0  # Backward
        elif key == 'a': vy_input = -1.0  # Strafe Left
        elif key == 'd': vy_input = 1.0   # Strafe Right
        elif key == 'q': yaw_input = 0.5   # Turn CCW (Positive)
        elif key == 'e': yaw_input = -0.5  # Turn CW (Negative)

        # 2. Obtener lectura del LiDAR
        current_angle = self.get_lidar_heading(lidar_points)

        # 3. CONTROL DE RUMBO INTERNO
        if abs(yaw_input) > 0.05:
            # Si pulsas Q o E, el robot gira libremente y actualiza el objetivo
            self.target_angle = current_angle
            self.is_correcting = False
            corrected_yaw = yaw_input
        else:
            # Si te mueves con WASD sin pulsar Q/E, el LiDAR corrige el desvío
            if (abs(vx_input) > 0.05 or abs(vy_input) > 0.05):
                if not self.is_correcting:
                    self.target_angle = current_angle
                    self.is_correcting = True
                
                angular_error = self.target_angle - current_angle
                
                # Normalizar el error (-180 a 180)
                while angular_error > 180:  angular_error -= 360
                while angular_error < -180: angular_error += 360
                
                # El LiDAR calcula cuánta fuerza aplicar para volver al rumbo
                corrected_yaw = angular_error * self.Kp
            else:
                self.is_correcting = False
                corrected_yaw = 0.0

        # 4. MEZCLA DE VECTORES (Configuración X estándar)
        # Nota cómo 'corrected_yaw' suma a la izquierda y resta a la derecha.
        # Si corrected_yaw es positivo (Q), el lado izquierdo va atrás y el derecho adelante -> Giro Antihorario.
        fl = vx_input + vy_input + corrected_yaw
        fr = vx_input - vy_input - corrected_yaw
        rl = vx_input - vy_input + corrected_yaw
        rr = vx_input + vy_input - corrected_yaw

        # 5. NORMALIZACIÓN
        max_power = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
        
        return fl/max_power, fr/max_power, rl/max_power, rr/max_power