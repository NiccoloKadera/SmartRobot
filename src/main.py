import sys
import os
import argparse

# 1. Configurazione Ambiente Webots (ESEGUITA PRIMA DI TUTTO)
# Questo blocco DEVE essere eseguito prima di qualsiasi import che dipende da Webots.
try:
    os.environ['WEBOTS_HOME'] = '/Applications/Webots.app'
    webots_python_path = '/Applications/Webots.app/Contents/lib/controller/python'
    if webots_python_path not in sys.path:
        sys.path.append(webots_python_path)
    # Questo import ora ha successo perché il path è stato aggiunto
    from controller import Robot
except ImportError:
    sys.exit("Errore: Libreria 'controller' di Webots non trovata. Controlla il percorso in 'main.py'.")


# 2. Import relativi al progetto (ora possono usare le librerie Webots)
from .robot_connectors.webots import WebotsConnector
from .robot_controllers.kuka_controller import KukaManualController

def main():
    parser = argparse.ArgumentParser(description="SmartRobot Controller - Kuka Edition")
    # ... resto del codice invariato ...
    parser.add_argument('--mode', type=str, default='simulation', choices=['simulation', 'live'],
                        help='Scegli "simulation" per Webots.')
    args = parser.parse_args()

    if args.mode == 'simulation':
        print("--- Avvio in modalità SIMULATION (Webots) ---")
        try:
            # Inizializza la connessione a Webots
            webots_conn = WebotsConnector()
            
            # Crea il controller manuale passando l'istanza del robot
            # Il controller KukaManualController gestirà i tasti WASD
            robot_controller = KukaManualController(webots_conn.robot)
            
            print("Controllo pronto. Clicca sulla finestra Webots e usa WASD/QE.")
            
            # Loop principale: esegue il controller finché la simulazione è attiva
            while webots_conn.robot.step(robot_controller.timestep) != -1:
                robot_controller.update()
                
        except Exception as e:
            print(f"Errore durante l'esecuzione: {e}")
    else:
        print("Modalità LIVE (ROS) non ancora configurata.")

if __name__ == "__main__":
    main()