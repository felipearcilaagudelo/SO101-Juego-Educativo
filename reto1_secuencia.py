"""
Reto 1 — Secuencia: trayectoria y agarre básico (un solo brazo).

Mueve un cubo desde 'pick' hasta 'place', pasando por 'home' entre medio
para levantar la pieza antes de trasladarla (evita arrastrarla sobre la
mesa). El primer salto a 'home' es directo (punto de partida arbitrario);
los tramos largos home<->pick y home<->place siguen trayectorias grabadas
con varios puntos intermedios en vez de un solo salto largo, mucho más
confiable en este brazo.

Requiere haber capturado antes:
    python capturar_posiciones.py --port /dev/ttyACM0 --id A
        (posiciones 'home', 'pick', 'place', 'gripper_abierto', 'gripper_cerrado')
    python capturar_trayectoria.py --port /dev/ttyACM0 --id A --nombre home_pick
    python capturar_trayectoria.py --port /dev/ttyACM0 --id A --nombre home_place

Uso:
    python reto1_secuencia.py --port /dev/ttyACM0 --id A
"""

from __future__ import annotations

import argparse

from robot_api import ParoDeEmergencia, Robot


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Puerto serial, ej. /dev/ttyACM0")
    ap.add_argument("--id", required=True, help="Id de calibración, ej. A")
    ap.add_argument("--archivo", default="posiciones.json")
    ap.add_argument("--archivo_trayectorias", default="trayectorias.json")
    args = ap.parse_args()

    with Robot(
        port=args.port,
        id=args.id,
        archivo_posiciones=args.archivo,
        archivo_trayectorias=args.archivo_trayectorias,
    ) as robot:
        print("Reto 1: mover un cubo de 'pick' a 'place'\n")

        robot.ir_a_posicion("home")
        robot.abrir_gripper()

        robot.seguir_trayectoria("home_pick")            # home -> pick, paso a paso
        robot.cerrar_gripper()

        robot.seguir_trayectoria("home_pick", invertida=True)   # pick -> home
        robot.seguir_trayectoria("home_place")            # home -> place, paso a paso
        robot.abrir_gripper()

        robot.seguir_trayectoria("home_place", invertida=True)  # place -> home

        print("\nReto 1 completado.")


if __name__ == "__main__":
    try:
        main()
    except ParoDeEmergencia:
        pass  # el propio Robot ya imprimió el mensaje de E-Stop
    except KeyError as e:
        print(f"\n[ERROR] {e}")
