"""
ensenar_con_leader.py
Modo "enseñar" alternativo a capturar_posiciones.py: en vez de apagar el
torque del follower y moverlo con la mano, teleoperas el follower moviendo
el brazo LÍDER (igual que en lerobot-teleoperate). El follower se mantiene
siempre bajo control de torque —lo mueve el comando del líder, nunca se
fuerza a mano— así que cualquier postura que guardes es, por definición,
una que el follower ya puede sostener por sí solo cuando se la vuelvas a
pedir. Evita el problema de "enseñé una posición que el brazo no puede
replicar exacto" que sí puede pasar moviendo el follower a mano.

El archivo que genera (posiciones.json) tiene el mismo formato que produce
capturar_posiciones.py, así que reto1_secuencia.py funciona igual sin
importar con cuál de los dos scripts se enseñaron las posiciones.

Requiere: calibración existente tanto del líder como del follower, con el
mismo id de pareja (ej. Leader A + Follower A) — la misma que ya usa tu
README para teleoperación.

Uso:
    python ensenar_con_leader.py \
        --puerto_follower /dev/ttyACM1 --id_follower A \
        --puerto_leader   /dev/ttyACM0 --id_leader   A

Mueve el líder para posicionar el follower. Cuando esté donde quieres,
presiona ENTER: el lazo de teleoperación se pausa, te pregunta el nombre
de la postura, la guarda, y sigue teleoperando para la siguiente. Escribe
'fin' como nombre para terminar.
"""

from __future__ import annotations

import argparse
import json
import select
import sys
import time
from pathlib import Path

try:
    from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
except ModuleNotFoundError:
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

HZ_TELEOP = 50  # frecuencia del lazo de teleoperación (igual de fluido que lerobot-teleoperate)


def _enter_presionado() -> bool:
    """True si hay una línea esperando en stdin, sin bloquear el lazo de teleoperación."""
    return bool(select.select([sys.stdin], [], [], 0)[0])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto_follower", required=True)
    ap.add_argument("--id_follower", required=True)
    ap.add_argument("--puerto_leader", required=True)
    ap.add_argument("--id_leader", required=True)
    ap.add_argument("--archivo", default="posiciones.json")
    args = ap.parse_args()

    ruta = Path(args.archivo)
    posiciones = json.loads(ruta.read_text()) if ruta.exists() else {}
    if posiciones:
        print(f"Ya existen {len(posiciones)} posiciones en {ruta}: {', '.join(posiciones)}")

    follower = SO101Follower(SO101FollowerConfig(port=args.puerto_follower, id=args.id_follower))
    leader = SO101Leader(SO101LeaderConfig(port=args.puerto_leader, id=args.id_leader))

    follower.connect(calibrate=False)
    leader.connect(calibrate=False)
    print("[OK] Follower y líder conectados.\n")
    print("Mueve el brazo LÍDER para posicionar el brazo FOLLOWER.")
    print("Cuando el follower esté donde quieres, presiona ENTER para guardar esa postura.")
    print("Escribe 'fin' cuando te pregunte el nombre para terminar.\n")

    try:
        while True:
            action = leader.get_action()
            follower.send_action(action)

            if _enter_presionado():
                sys.stdin.readline()  # consume el ENTER que disparó la captura
                nombre = input("Nombre de la posición (o 'fin'): ").strip()
                if nombre.lower() == "fin":
                    break
                if nombre:
                    obs = follower.get_observation()
                    posiciones[nombre] = {
                        k.replace(".pos", ""): round(v, 2)
                        for k, v in obs.items() if k.endswith(".pos")
                    }
                    print(f"  guardado '{nombre}': {posiciones[nombre]}")
                print("Sigue moviendo el líder para la siguiente postura (o ENTER de nuevo)...\n")

            time.sleep(1.0 / HZ_TELEOP)
    except KeyboardInterrupt:
        print("\n[E-STOP] Detenido por el usuario (Ctrl+C).")
    finally:
        follower.disconnect()
        leader.disconnect()
        ruta.write_text(json.dumps(posiciones, indent=2, ensure_ascii=False))
        print(f"\n[OK] {len(posiciones)} posiciones guardadas en {ruta.resolve()}")


if __name__ == "__main__":
    main()
