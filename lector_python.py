import subprocess, sys
subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
import serial
import csv
import time
from pathlib import Path
from datetime import datetime

# CAMBIAR POR EL PUERTO DE TU ARDUINO
PUERTO = "COM5"
BAUDIOS = 115200
DURACION = 10  # segundos

CARPETA = Path("datos_imu")
CARPETA.mkdir(exist_ok=True)

CLASES = {
    "1": "roll_positivo",
    "2": "roll_negativo",
    "3": "pitch_positivo",
    "4": "pitch_negativo",
    "5": "yaw_positivo",
    "6": "yaw_negativo",
    "7": "reposo"
}

COLUMNAS = [
    "timestamp", "accX", "accY", "accZ",
    "gyrX", "gyrY", "gyrZ"
]


def grabar(arduino, clase):
    print(f"Preparando: {clase}")
    for i in range(3, 0, -1):
        print(i)
        time.sleep(1)

    arduino.reset_input_buffer()
    arduino.readline()  # Descarta la primera trama por si quedó cortada al vaciar el buffer

    print("¡GRABANDO!")
    muestras = []
    inicio = time.monotonic()
    tiempo_base = None

    while time.monotonic() - inicio < DURACION:
        linea = arduino.readline().decode(
            "utf-8", errors="ignore"
        ).strip()

        valores = linea.split(",")

        if len(valores) != 7:
            continue

        try:
            tiempo = int(valores[0])
            sensores = [float(v) for v in valores[1:]]
        except ValueError:
            continue

        if tiempo_base is None:
            tiempo_base = tiempo

        muestras.append([tiempo - tiempo_base] + sensores)

    if not muestras:
        print("No se recibieron muestras.")
        return

    fecha = datetime.now().strftime("%Y%m%d_%H%M%S")
    archivo = CARPETA / f"{clase}_{fecha}.csv"

    with open(archivo, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNAS)
        writer.writerows(muestras)

    print(f"Guardado: {archivo}")
    print(f"Muestras: {len(muestras)}")


def main():
    with serial.Serial(PUERTO, BAUDIOS, timeout=0.2) as arduino:
        time.sleep(2)

        while True:
            print("\n--- CAPTURA DE MOVIMIENTOS ---")
            for numero, clase in CLASES.items():
                print(f"{numero}. {clase}")
            print("0. Salir")

            opcion = input("Selecciona movimiento: ").strip()

            if opcion == "0":
                break

            if opcion not in CLASES:
                print("Opción incorrecta.")
                continue

            grabar(arduino, CLASES[opcion])


if __name__ == "__main__":
    main()