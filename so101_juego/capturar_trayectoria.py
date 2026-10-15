"""
capturar_trayectoria.py
Graba una trayectoria: una sucesión de posturas intermedias entre un punto
de partida y uno de llegada, en vez de confiar en un solo salto grande
entre dos posiciones lejanas (eso fue lo que causó los saltos y bloqueos
grandes que vimos con posiciones sueltas tipo 'home' <-> 'pick').

Mueve el brazo LENTAMENTE a mano por el camino que quieres que siga,
marcando puntos con ENTER cada tanto — mientras más cerca uno del otro,
más corto y confiable es cada tramo automático después.

Al terminar de marcar puntos, reproduce automáticamente la trayectoria
para verificar que el brazo puede seguirla solo. Si algún tramo falla, te
dice cuál y NO la guarda — agrega un punto intermedio justo ahí y vuelve
a intentarlo.

Uso:
    python capturar_trayectoria.py --port /dev/ttyACM0 --id A --nombre home_pick

Una sola trayectoria sirve para ir y volver (reto1_secuencia.py la recorre
al revés con seguir_trayectoria(..., invertida=True) para el camino de
vuelta), así que solo necesitas grabar el camino en un sentido.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from robot_api import ParoDeEmergencia, Robot


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", required=True)
    ap.add_argument("--nombre", required=True, help="Nombre de la trayectoria, ej. home_pick")
    ap.add_argument("--archivo", default="trayectorias.json")
    args = ap.parse_args()

    ruta = Path(args.archivo)
    trayectorias = json.loads(ruta.read_text()) if ruta.exists() else {}
    if args.nombre in trayectorias:
        print(
            f"Ya existe una trayectoria '{args.nombre}' con "
            f"{len(trayectorias[args.nombre])} puntos; se va a reemplazar."
        )

    with Robot(port=args.port, id=args.id, archivo_trayectorias=args.archivo) as robot:
        robot.desactivar_torque()
        print("\n(torque desactivado: mueve el brazo con la mano LENTAMENTE por el camino deseado)")
        print("Presiona ENTER para marcar cada punto intermedio. Escribe 'fin' para terminar.\n")

        puntos: list[dict[str, float]] = []
        while True:
            linea = input(f"Punto {len(puntos) + 1} (ENTER para marcar, o 'fin'): ").strip()
            if linea.lower() == "fin":
                break
            punto = robot.leer_posicion_actual()
            puntos.append(punto)
            print(f"  punto {len(puntos)} marcado")

        if len(puntos) < 2:
            print("\n[AVISO] Se necesitan al menos 2 puntos para una trayectoria. No se guardó nada.")
            return

        print(f"\nVerificando que el brazo pueda seguir solo estos {len(puntos)} puntos...")
        robot.activar_torque()
        try:
            robot.seguir_trayectoria_puntos(puntos)
        except ParoDeEmergencia:
            raise
        except TimeoutError as e:
            print(f"\n[ERROR] La trayectoria falló en la verificación: {e}")
            print("No se guardó. Vuelve a grabarla agregando un punto intermedio donde falló.")
            return

        print("[OK] Verificación exitosa: el brazo siguió la trayectoria solo.")

    trayectorias[args.nombre] = puntos
    ruta.write_text(json.dumps(trayectorias, indent=2, ensure_ascii=False))
    print(f"\n[OK] Trayectoria '{args.nombre}' guardada con {len(puntos)} puntos en {ruta.resolve()}")


if __name__ == "__main__":
    main()
