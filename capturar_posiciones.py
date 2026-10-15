"""
capturar_posiciones.py
Modo "enseñar": desactiva el torque para que muevas el brazo con la mano,
y guarda cada postura con un nombre.

Antes de guardarla, la VERIFICA: le pide al brazo (con torque activado,
usando la misma función mover_suave() que usará el juego) que se sostenga
solo exactamente en esa postura. Si alguna articulación fue empujada a
mano más allá de lo que puede comandarse a sí misma (típico cerca de los
límites del rango calibrado), te avisa ahí mismo y guarda el valor que el
brazo sí puede sostener — así el Reto 1 nunca se topa con un objetivo
inalcanzable a mitad de partida.

Uso:
    python capturar_posiciones.py --port /dev/ttyACM0 --id A

Con el brazo sin torque, muévelo con la mano hasta la postura que quieras,
escribe un nombre y presiona ENTER para guardarla. Escribe 'fin' para
terminar. El archivo se reescribe (agregando o reemplazando nombres) cada
vez que corres el script, así que puedes volver a capturar solo una
posición sin perder las demás.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from robot_api import ParoDeEmergencia, Robot


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Puerto serial, ej. /dev/ttyACM0")
    ap.add_argument("--id", required=True, help="Id de calibración, ej. A")
    ap.add_argument("--archivo", default="posiciones.json")
    args = ap.parse_args()

    ruta = Path(args.archivo)
    posiciones = json.loads(ruta.read_text()) if ruta.exists() else {}
    if posiciones:
        print(f"Ya existen {len(posiciones)} posiciones en {ruta}: {', '.join(posiciones)}")

    with Robot(port=args.port, id=args.id, archivo_posiciones=args.archivo) as robot:
        print("\nEscribe un nombre y ENTER para guardar la postura actual (ej: 'home', 'pick', 'place').")
        print("Escribe 'fin' para terminar y guardar el archivo.\n")

        while True:
            robot.desactivar_torque()
            print("(torque desactivado: mueve el brazo con la mano)")
            nombre = input("Nombre de la posición (o 'fin'): ").strip()
            if nombre.lower() == "fin":
                break
            if not nombre:
                continue

            objetivo = robot.leer_posicion_actual()

            robot.activar_torque()
            try:
                robot.mover_suave(objetivo)
            except ParoDeEmergencia:
                raise
            except Exception as e:
                print(f"  [AVISO] {e}")
                print("  Se guarda el valor que el brazo sí logró sostener, no el que pediste a mano.")

            final = robot.leer_posicion_actual()
            posiciones[nombre] = {k: round(v, 2) for k, v in final.items()}
            print(f"  guardado '{nombre}': {posiciones[nombre]}\n")

        robot.desactivar_torque()

    ruta.write_text(json.dumps(posiciones, indent=2, ensure_ascii=False))
    print(f"[OK] {len(posiciones)} posiciones guardadas en {ruta.resolve()}")


if __name__ == "__main__":
    main()
