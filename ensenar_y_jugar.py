"""
ensenar_y_jugar.py
Enseña PASOS moviendo el brazo LÍDER —el follower lo sigue en vivo— y los
guarda con nombre para reusarlos después en tus propios programas con for
y while (ver mi_secuencia.py). En vez de marcar puntos a mano uno por uno,
grabas por tiempo: ENTER para empezar, mueves el líder de forma natural y
continua, ENTER para terminar. Mientras tanto el programa guarda la
posición automáticamente cada medio segundo — así el camino grabado sigue
el gesto real del líder, con puntos siempre cercanos entre sí.

Puedes grabar VARIOS pasos seguidos en una sola sesión (sin reconectar el
brazo cada vez) — al terminar cada paso te pregunta si quieres grabar otro.

Uso:
    python ensenar_y_jugar.py \
        --puerto_follower /dev/ttyACM0 --id_follower A \
        --puerto_leader   /dev/ttyACM1 --id_leader   A

Flujo (se repite por cada paso que quieras grabar):
  1. Mueve el brazo LÍDER con la mano — el follower lo sigue en vivo.
  2. ENTER cuando quieras EMPEZAR a grabar el paso.
  3. Mueve el líder por el camino que quieres enseñar — el programa
     guarda un punto automáticamente cada medio segundo.
  4. ENTER de nuevo para TERMINAR de grabar ese paso.
  5. Pregunta si quieres reproducirlo ahora mismo, solo con el follower.
  6. Pregunta si quieres guardarlo con un nombre (ej. 'paso1') para
     reusarlo después con robot.seguir_trayectoria('paso1').
  7. Pregunta si quieres grabar otro paso más, o terminar la sesión.
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

from robot_api import ParoDeEmergencia, Robot

HZ_TELEOP = 50
INTERVALO_MUESTREO_S = 0.5  # cada cuánto se guarda un punto automáticamente mientras grabas


def _enter_presionado() -> bool:
    """True si hay una línea esperando en stdin, sin bloquear el lazo de teleoperación."""
    return bool(select.select([sys.stdin], [], [], 0)[0])


def _si(pregunta: str) -> bool:
    return input(pregunta).strip().lower() in ("", "s", "si", "sí")


def _grabar_un_paso(robot: Robot, leader) -> list[dict[str, float]]:
    """
    Corre el lazo de teleoperación (líder -> follower en vivo) y graba un
    punto automáticamente cada INTERVALO_MUESTREO_S mientras 'grabando'
    está activo. Devuelve la lista de puntos capturados para ESTE paso
    (vacía si se terminó sin grabar nada).
    """
    while _enter_presionado():  # descarta teclas que hayan quedado pendientes del buffer
        sys.stdin.readline()
    print("Mueve el brazo LÍDER para acomodarlo (el follower lo sigue en vivo).")
    print("Presiona ENTER cuando quieras EMPEZAR a grabar este paso.\n")

    puntos: list[dict[str, float]] = []
    grabando = False
    ultimo_muestreo = 0.0

    while True:
        accion = leader.get_action()
        robot.enviar_accion_cruda(accion)
        ahora = time.monotonic()

        if grabando and ahora - ultimo_muestreo >= INTERVALO_MUESTREO_S:
            puntos.append(robot.leer_posicion_actual())
            ultimo_muestreo = ahora
            print(f"  punto {len(puntos)} grabado")

        if _enter_presionado():
            sys.stdin.readline()  # consume el ENTER
            if not grabando:
                grabando = True
                ultimo_muestreo = ahora
                print(">>> GRABANDO — mueve el líder. ENTER de nuevo para terminar este paso.\n")
            else:
                print(f"\n>>> Paso terminado: {len(puntos)} puntos capturados.")
                break

        time.sleep(1.0 / HZ_TELEOP)

    return puntos


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto_follower", required=True)
    ap.add_argument("--id_follower", required=True)
    ap.add_argument("--puerto_leader", required=True)
    ap.add_argument("--id_leader", required=True)
    ap.add_argument("--archivo", default="trayectorias.json")
    args = ap.parse_args()

    ruta = Path(args.archivo)
    trayectorias = json.loads(ruta.read_text()) if ruta.exists() else {}
    if trayectorias:
        print(f"Ya existen {len(trayectorias)} pasos guardados: {', '.join(trayectorias)}\n")

    with Robot(
        port=args.puerto_follower, id=args.id_follower, archivo_trayectorias=args.archivo
    ) as robot:
        leader = SO101Leader(SO101LeaderConfig(port=args.puerto_leader, id=args.id_leader))
        leader.connect(calibrate=False)
        print("[OK] Líder conectado. Vas a poder grabar varios pasos seguidos en esta sesión.\n")

        try:
            while True:
                puntos = _grabar_un_paso(robot, leader)

                if not puntos:
                    print("No se grabó ningún punto en este paso — no hay nada que reproducir ni guardar.")
                else:
                    if _si(f"\n¿Reproducir este paso de {len(puntos)} puntos ahora? [S/n]: "):
                        print(
                            "Reproduciendo (solo el follower, ya sin el líder), interpolado y "
                            "directo, con el ritmo con el que se grabó..."
                        )
                        robot.seguir_trayectoria_directo(puntos, pausa_s=INTERVALO_MUESTREO_S)
                        print("Listo, paso reproducido completo.")

                    nombre = input(
                        "\n¿Guardar este paso con un nombre para reusarlo después? "
                        "(ENTER para no guardar): "
                    ).strip()
                    if nombre:
                        trayectorias[nombre] = puntos
                        ruta.write_text(json.dumps(trayectorias, indent=2, ensure_ascii=False))
                        print(
                            f"[OK] Guardado como '{nombre}' en {ruta.resolve()}. "
                            f"Pasos guardados hasta ahora: {', '.join(trayectorias)}"
                        )

                if not _si("\n¿Grabar otro paso? [S/n]: "):
                    break
        finally:
            leader.disconnect()

    print("\nSesión de enseñanza terminada.")


if __name__ == "__main__":
    try:
        main()
    except ParoDeEmergencia:
        pass  # el propio Robot ya imprimió el mensaje de E-Stop
