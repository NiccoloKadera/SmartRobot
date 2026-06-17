import sys
from pathlib import Path
import time
import os

# Risolve il percorso se lanciato da sottocartelle
progetto_root = str(Path(__file__).resolve().parent.parent)
if progetto_root not in sys.path:
    sys.path.insert(0, progetto_root)

from roslibpy_connector import RoslibpyConnector

def elabora_dati_lidar(message):
    """Questa funzione viene chiamata automaticamente ogni volta che arriva una scansione."""
    # DEBUG: Vediamo se arriva qualcosa e che chiavi ha il dizionario
    print(f"[{time.strftime('%H:%M:%S')}] Messaggio ricevuto! Chiavi disponibili: {list(message.keys())}")
    
    # Estraiamo la lista delle distanze
    ranges = message.get("ranges", [])
    
    if not ranges:
        print("Attenzione: La lista 'ranges' è vuota o assente nel messaggio.")
        return

    try:
        # Calcoliamo gli indici geometrici nell'array a 360°
        idx_sinistra = int(len(ranges) * 0.25)  # 90 gradi
        idx_destra = int(len(ranges) * 0.75)    # 270 gradi
        
        # Gestiamo il caso in cui i valori arrivino come stringhe o None (inf)
        val_sx = ranges[idx_sinistra]
        val_dx = ranges[idx_destra]
        
        dist_sinistra = float(val_sx) if val_sx is not None else float('inf')
        dist_destra = float(val_dx) if val_dx is not None else float('inf')

        # Stampa le distanze laterali in tempo reale
        print(f"-> Muro Sinistra: {dist_sinistra:.3f} m | Muro Destra: {dist_destra:.3f} m (Totale punti LiDAR: {len(ranges)})")
        print("-" * 50)
        
    except Exception as e:
        print(f"Errore durante l'elaborazione dei dati indicizzati: {e}")
def main():
    connector = RoslibpyConnector()
    
    # Inserisci l'IP reale del tuo Raspberry Pi
    connector.connect(host=os.getenv('ROBOT_IP', '192.168.0.12'), port=9090) 
    
    # IMPORTANTE: dal file del pannello flotta si vede che il topic standard è "/scan"
    topic_lidar = "/scan" 
    
    # Avviamo l'ascolto
    connector.listen(topic=topic_lidar, callback=elabora_dati_lidar)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nDisconnessione...")
        connector.close()

if __name__ == "__main__":
    main()