"""
mi_secuencia.py
Plantilla de ejemplo: cómo combinar los "pasos" que ya enseñaste con
ensenar_y_jugar.py dentro de tu propio programa, usando for y while como
en cualquier código de Python normal. Este archivo es un punto de partida
para modificar — no un script fijo.

Antes de correr esto, revisa qué nombres de pasos tienes guardados: este
script los imprime al conectar. Cámbialos en el código de abajo por los
tuyos (ej. "paso1", "recoger", "girar", como los hayas llamado).

Uso:
    python mi_secuencia.py --port /dev/ttyACM0 --id A
"""

from __future__ import annotations

import argparse

from robot_api import ParoDeEmergencia, Robot


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Puerto serial, ej. /dev/ttyACM0")
    ap.add_argument("--id", required=True, help="Id de calibración, ej. A")
    ap.add_argument("--archivo", default="trayectorias.json")
    args = ap.parse_args()

    with Robot(port=args.port, id=args.id, archivo_trayectorias=args.archivo) as robot:
        pasos = robot.listar_pasos()
        print(f"Pasos disponibles: {', '.join(pasos) or '(ninguno — enseña alguno primero con ensenar_y_jugar.py)'}\n")

        # ---------- Ejemplo con FOR: repite un paso varias veces ----------
        # Cambia "paso1" por el nombre real de uno de tus pasos.
        for vuelta in range(3):
            print(f"Vuelta {vuelta + 1}/3")
            robot.seguir_trayectoria("paso1")

        # ---------- Ejemplo combinando dos pasos en secuencia ----------
        robot.seguir_trayectoria("paso1")
        robot.seguir_trayectoria("paso2")
        robot.seguir_trayectoria("paso1", invertida=True)  # el mismo paso1, pero al revés

        # ---------- Ejemplo con WHILE: repite mientras el usuario diga que sí ----------
        seguir = True
        while seguir:
            robot.seguir_trayectoria("paso1")
            robot.seguir_trayectoria("paso2", invertida=True)
            seguir = input("¿Otra vuelta? [S/n]: ").strip().lower() in ("", "s", "si", "sí")

        print("\nSecuencia terminada.")


if __name__ == "__main__":
    try:
        main()
    except ParoDeEmergencia:
        pass  # el propio Robot ya imprimió el mensaje de E-Stop
    except KeyError as e:
        print(f"\n[ERROR] {e}")
