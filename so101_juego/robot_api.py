"""
robot_api.py
Capa de seguridad y movimiento para UN brazo SO-101 (LeRobot).

Qué resuelve, comparado con llamar send_action() directo:
  - Movimiento interpolado: nunca salta de golpe a la posición objetivo,
    se mueve en pasos pequeños a una velocidad controlada.
  - Ctrl+C funciona como parada de emergencia real: detiene el movimiento
    en el punto en que va, no a medio ciclo de escritura al bus.
  - Al salir (incluso por error), el brazo siempre queda desconectado
    de forma ordenada (try/finally dentro del context manager).
  - Posiciones "enseñadas a mano" (capturadas con capturar_posiciones.py)
    se cargan desde un JSON y se referencian por nombre, sin cinemática
    inversa: robot.ir_a_posicion("pick").

Requiere que el brazo ya esté calibrado (A.json o B.json existente) y que
conozcas su puerto serial (lerobot-find-port) y su id de calibración.

Uso:
    from robot_api import Robot

    with Robot(port="/dev/ttyACM0", id="A") as robot:
        robot.ir_a_posicion("home")
        robot.ir_a_posicion("pick")
        robot.cerrar_gripper()
        robot.ir_a_posicion("place")
        robot.abrir_gripper()
"""

from __future__ import annotations

import json
import signal
import time
from pathlib import Path
from typing import Callable

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# ---------------- Configuración de seguridad ----------------
# Estos valores son conservadores a propósito. Súbelos solo después de
# probar en vacío (sin piezas ni obstáculos) y confirmar que el brazo
# se comporta bien.
PASO_MAXIMO_POR_CICLO = 4.0    # % del rango calibrado, por ciclo de control
HZ_CONTROL = 20                # ciclos de control por segundo
VELOCIDAD_POR_DEFECTO = 0.6    # fracción (0-1) de PASO_MAXIMO_POR_CICLO
# Con esto el brazo se mueve ~3x más rápido que antes (sigue siendo
# interpolado en pasos pequeños, no un salto brusco). Si lo sientes muy
# rápido o brusco, baja VELOCIDAD_POR_DEFECTO; si lo sientes lento, sube
# PASO_MAXIMO_POR_CICLO. Pruébalo siempre primero en vacío, sin piezas.
TOLERANCIA_LLEGADA = 3.0       # % de error aceptado para considerar "llegó"
                                # (los STS3215 muestran de forma normal 2-3% de
                                #  error de estado estable bajo carga del propio
                                #  brazo, sobre todo con el hombro sosteniendo peso;
                                #  no es una falla, es holgura de torque esperable)
TIMEOUT_MINIMO_S = 20.0        # piso de tiempo — la velocidad real bajo carga
MARGEN_TIMEOUT = 4.0           # (gravedad, fricción) es más lenta que el cálculo
                                # teórico, sobre todo con el hombro levantando peso.
                                # El límite real contra atascos no es este timeout
                                # total sino SIN_PROGRESO_TIMEOUT_S de abajo, que no
                                # depende de cuánto tiempo le demos en total.
SIN_PROGRESO_TIMEOUT_S = 4.0   # si no mejora en este tiempo, corta ya (esto sí
                                # detecta un atasco real, sin esperar el timeout total)
PROGRESO_MINIMO = 0.1          # % de mejora mínima para contar como "avanzó"
ERROR_ACEPTABLE_SIN_CONVERGER = 8.0
# Si al agotarse el tiempo el error ya bajó de aquí, se acepta con un aviso
# en vez de abortar el programa entero: para una tarea física (mover un
# cubo) unos pocos puntos de diferencia no cambian el resultado. Por
# encima de este umbral sí se considera un problema real (posición
# enseñada en un límite mecánico, o calibración desalineada).

GRIPPER_ABIERTO = 90.0
GRIPPER_CERRADO = -20.0        # ajusta tras probar con una pieza real

NOMBRES_ARTICULACIONES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]


class ParoDeEmergencia(Exception):
    """Se lanza cuando el usuario pide detener el programa (Ctrl+C)."""


class Robot:
    def __init__(
        self,
        port: str,
        id: str,
        archivo_posiciones: str | Path = "posiciones.json",
        archivo_trayectorias: str | Path = "trayectorias.json",
        capturar_sigint: bool = True,
    ):
        self.port = port
        self.id = id
        self.archivo_posiciones = Path(archivo_posiciones)
        self.archivo_trayectorias = Path(archivo_trayectorias)
        self._robot: SO101Follower | None = None
        self._detener = False
        self._posiciones_guardadas = self._cargar_posiciones()
        self._trayectorias = self._cargar_trayectorias()
        self._manejador_anterior = None
        # capturar_sigint=False: no registra un manejador de Ctrl+C propio.
        # Úsalo cuando Robot vive dentro de un proceso que ya maneja sus
        # propias señales (ej. un servidor web con uvicorn) — ahí registrar
        # otro manejador de SIGINT puede interferir con el apagado normal
        # del servidor. En ese caso la única forma de detener un movimiento
        # es el parámetro 'debe_detener' de seguir_trayectoria_directo().
        self._capturar_sigint = capturar_sigint

    # ---------- Ciclo de vida ----------
    def __enter__(self) -> "Robot":
        self._robot = SO101Follower(SO101FollowerConfig(port=self.port, id=self.id))
        self._robot.connect(calibrate=False)
        # Fuerza torque activado en las 6 articulaciones al conectar. Si una
        # quedó con el torque apagado por una falla anterior (ej. un Overload
        # de sesiones pasadas) y nadie lo volvió a activar explícitamente,
        # esa articulación deja de responder a comandos de posición sin dar
        # ningún error — se ve exactamente como "no llega nunca", que es lo
        # que estábamos viendo. Esto lo evita desde el arranque.
        try:
            self._robot.bus.enable_torque()
        except Exception as e:
            print(f"[AVISO] No se pudo forzar torque activado al conectar: {e}")
        # Ctrl+C se captura como E-Stop en vez de dejar que Python interrumpa
        # a medio envío de comando al bus serial.
        if self._capturar_sigint:
            self._manejador_anterior = signal.signal(signal.SIGINT, self._on_sigint)
        print(f"[OK] Conectado a '{self.id}' en {self.port}")
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        if self._manejador_anterior is not None:
            signal.signal(signal.SIGINT, self._manejador_anterior)
        if self._robot is not None:
            try:
                self._robot.disconnect()
                print("[OK] Brazo desconectado de forma segura.")
            except Exception as e:
                print(f"[AVISO] Error al desconectar: {e}")
        if exc_type is ParoDeEmergencia:
            print("[E-STOP] Programa detenido por el usuario (Ctrl+C).")
            return True  # no propagar el traceback: fue una parada intencional
        return False

    def _on_sigint(self, signum, frame):
        self._detener = True
        raise ParoDeEmergencia()

    # ---------- Lectura ----------
    def leer_posicion_actual(self) -> dict[str, float]:
        obs = self._robot.get_observation()
        return {k.replace(".pos", ""): v for k, v in obs.items() if k.endswith(".pos")}

    # ---------- Movimiento ----------
    def mover_suave(self, objetivo: dict[str, float], velocidad: float = VELOCIDAD_POR_DEFECTO) -> None:
        """
        Interpola desde la posición actual hasta 'objetivo' en pasos pequeños.
        'objetivo' puede traer solo algunas articulaciones: las que faltan
        no se mueven. Lanza ParoDeEmergencia si el usuario presiona Ctrl+C.

        El timeout escala con la distancia a recorrer, para no cortar de golpe
        un movimiento largo. Pero si el movimiento deja de progresar antes de
        eso (típico cuando una articulación fue enseñada justo en su límite
        mecánico y nunca logra asentarse ahí dentro de la tolerancia), se
        aborta mucho antes y te dice exactamente qué articulación quedó
        atascada, en vez de esperar el timeout completo a ciegas.
        """
        velocidad = max(0.05, min(1.0, velocidad))
        paso_maximo = PASO_MAXIMO_POR_CICLO * velocidad
        velocidad_efectiva = paso_maximo * HZ_CONTROL  # % por segundo

        actual = self.leer_posicion_actual()
        destino = dict(actual)
        destino.update(objetivo)

        distancia_inicial = max(abs(destino[j] - actual[j]) for j in destino)
        timeout = max(TIMEOUT_MINIMO_S, (distancia_inicial / velocidad_efectiva) * MARGEN_TIMEOUT)

        inicio = time.monotonic()
        mejor_distancia = distancia_inicial
        ultimo_progreso = inicio
        ultimo_print = inicio

        while True:
            if self._detener:
                raise ParoDeEmergencia()

            actual = self.leer_posicion_actual()
            diffs = {j: destino[j] - actual[j] for j in destino}
            distancia_max = max(abs(d) for d in diffs.values())

            if distancia_max < TOLERANCIA_LLEGADA:
                break

            ahora = time.monotonic()
            if distancia_max < mejor_distancia - PROGRESO_MINIMO:
                mejor_distancia = distancia_max
                ultimo_progreso = ahora

            if ahora - ultimo_print > 1.0:
                print(f"    ...acercándose, falta {distancia_max:.1f} ({self._diffs_pendientes(diffs)})")
                ultimo_print = ahora

            timeout_agotado = ahora - inicio > timeout
            sin_progreso = ahora - ultimo_progreso > SIN_PROGRESO_TIMEOUT_S
            if timeout_agotado or sin_progreso:
                if distancia_max <= ERROR_ACEPTABLE_SIN_CONVERGER:
                    # No llegó exacto, pero quedó razonablemente cerca (esto es
                    # normal en estos servos bajo la carga del propio brazo).
                    # Para una tarea física como mover un cubo, esto no importa:
                    # seguimos en vez de abortar todo el programa por unos
                    # pocos puntos de diferencia.
                    print(
                        f"  [aviso] no llegó exacto (quedó a {distancia_max:.1f} del objetivo: "
                        f"{self._diffs_pendientes(diffs)}) — suficientemente cerca, continúo."
                    )
                    break
                motivo = "no convergió a tiempo" if timeout_agotado else "dejó de progresar"
                raise TimeoutError(
                    f"El movimiento {motivo}, y quedó demasiado lejos del objetivo "
                    f"({distancia_max:.1f}). Articulaciones que aún no llegan: "
                    f"{self._diffs_pendientes(diffs)}. Probablemente esa posición fue "
                    "enseñada justo en un límite mecánico, o la calibración cambió "
                    "desde que se enseñó — vuelve a enseñarla."
                )

            accion = {}
            for j, d in diffs.items():
                paso = max(-paso_maximo, min(paso_maximo, d))
                accion[f"{j}.pos"] = actual[j] + paso

            self._robot.send_action(accion)
            time.sleep(1.0 / HZ_CONTROL)

    @staticmethod
    def _diffs_pendientes(diffs: dict[str, float]) -> str:
        pendientes = {j: d for j, d in diffs.items() if abs(d) >= TOLERANCIA_LLEGADA}
        return ", ".join(f"{j}: falta {d:+.1f}" for j, d in pendientes.items()) or "(ninguna)"

    # ---------- Torque (para enseñar posiciones a mano) ----------
    def desactivar_torque(self) -> None:
        self._robot.bus.disable_torque()

    def activar_torque(self) -> None:
        self._robot.bus.enable_torque()

    # ---------- Paso directo (para teleoperación en vivo con el líder) ----------
    def enviar_accion_cruda(self, accion: dict[str, float]) -> None:
        """
        Manda la acción directo al follower, SIN interpolar. Úsalo solo en
        teleoperación en vivo (la acción viene de leader.get_action()),
        donde la velocidad la controla la persona moviendo el líder — no
        uses esto para reproducir posiciones guardadas, para eso está
        mover_suave()/seguir_trayectoria().
        """
        self._robot.send_action(accion)

    def ir_a_home(self, velocidad: float = VELOCIDAD_POR_DEFECTO) -> None:
        self.mover_suave({j: 0.0 for j in NOMBRES_ARTICULACIONES if j != "gripper"}, velocidad)

    def _valor_gripper(self, nombre_postura: str, valor_por_defecto: float) -> float:
        """
        Usa el valor de gripper enseñado a mano bajo 'nombre_postura'
        (ej. 'gripper_cerrado', capturado sosteniendo el cubo real) si
        existe en posiciones.json; si no, cae al valor por defecto.
        """
        postura = self._posiciones_guardadas.get(nombre_postura)
        if postura and "gripper" in postura:
            return postura["gripper"]
        return valor_por_defecto

    def abrir_gripper(self) -> None:
        objetivo = self._valor_gripper("gripper_abierto", GRIPPER_ABIERTO)
        try:
            self.mover_suave({"gripper": objetivo}, velocidad=0.6)
        except ParoDeEmergencia:
            raise
        except Exception as e:
            print(f"  [gripper] aviso de hardware al abrir ({e}); continuando.")

    def cerrar_gripper(self) -> None:
        """
        A diferencia de las articulaciones del brazo, que siempre deben
        llegar exactas a su objetivo, cerrar el gripper sobre una pieza
        real SE SUPONE que topa con algo antes de llegar al valor "cerrado
        del todo" — así es como se sostiene un cubo sin aplastarlo. Por eso
        aquí un estancamiento o una sobrecarga del servo no se trata como
        error fatal: se interpreta como "ya sujetó algo" (o llegó a su tope
        mecánico si no había nada entre las pinzas) y el programa continúa.
        """
        objetivo = self._valor_gripper("gripper_cerrado", GRIPPER_CERRADO)
        try:
            self.mover_suave({"gripper": objetivo}, velocidad=0.6)
        except ParoDeEmergencia:
            raise
        except Exception as e:
            print(f"  [gripper] dejó de avanzar al cerrar ({e}) — asumiendo que sujetó algo.")

    # ---------- Posiciones guardadas (enseñadas a mano) ----------
    def _cargar_posiciones(self) -> dict[str, dict[str, float]]:
        if self.archivo_posiciones.exists():
            return json.loads(self.archivo_posiciones.read_text())
        return {}

    def ir_a_posicion(self, nombre: str, velocidad: float = VELOCIDAD_POR_DEFECTO) -> None:
        if nombre not in self._posiciones_guardadas:
            disponibles = ", ".join(self._posiciones_guardadas) or "(ninguna aún)"
            raise KeyError(
                f"No existe la posición '{nombre}'. Posiciones disponibles: {disponibles}. "
                f"Grábala primero con capturar_posiciones.py"
            )
        self.mover_suave(self._posiciones_guardadas[nombre], velocidad=velocidad)

    # ---------- Trayectorias (varios puntos intermedios, no un solo salto) ----------
    def _cargar_trayectorias(self) -> dict[str, list[dict[str, float]]]:
        if self.archivo_trayectorias.exists():
            return json.loads(self.archivo_trayectorias.read_text())
        return {}

    def seguir_trayectoria_puntos(
        self,
        puntos: list[dict[str, float]],
        velocidad: float = VELOCIDAD_POR_DEFECTO,
        detener_en_error: bool = True,
    ) -> None:
        """
        Sigue una lista de posturas intermedias, una tras otra, en vez de
        saltar directo del inicio al final. Cada tramo es corto, así que
        es mucho menos probable que un tramo se estanque que con un solo
        salto largo entre dos posturas lejanas.

        detener_en_error=True (por defecto): si un tramo no puede llegar,
        aborta toda la trayectoria — úsalo cuando cada punto importa de
        verdad (ej. un 'pick' real sobre una pieza).
        detener_en_error=False: un tramo atascado se salta con un aviso y
        sigue con el siguiente punto — mejor para una demostración larga
        enseñada a mano, donde perder un punto intermedio no arruina el
        resultado, y donde un atasco ocasional del servo/bus no debería
        tumbar toda la secuencia.
        """
        for i, punto in enumerate(puntos, start=1):
            try:
                self.mover_suave(punto, velocidad=velocidad)
            except ParoDeEmergencia:
                raise
            except TimeoutError as e:
                if detener_en_error:
                    raise TimeoutError(f"tramo {i}/{len(puntos)}: {e}") from e
                print(f"  [aviso] tramo {i}/{len(puntos)} atascado, se salta: {e}")

    def listar_pasos(self) -> list[str]:
        """
        Nombres de todos los "pasos" (trayectorias) guardados, en el orden
        en que están en el archivo. Útil para que un programa de un
        estudiante los liste o los recorra con un for, sin tener que abrir
        el JSON a mano:

            for nombre in robot.listar_pasos():
                robot.seguir_trayectoria(nombre)
        """
        return list(self._trayectorias)

    def seguir_trayectoria(
        self,
        nombre: str,
        velocidad: float = VELOCIDAD_POR_DEFECTO,
        invertida: bool = False,
        detener_en_error: bool = True,
        directo: bool = True,
        pausa_s: float = 0.3,
        debe_detener: Callable[[], bool] | None = None,
    ) -> bool:
        """
        Reproduce un paso guardado por su nombre — pensado para que un
        programa lo llame dentro de un for/while, combinando varios pasos
        en la secuencia que quiera ("paso1", "paso2", "paso1" otra vez...).

        'invertida=True' recorre la misma trayectoria guardada al revés —
        así una sola grabación (ej. 'paso1') sirve para ir Y volver, sin
        tener que enseñar el camino dos veces.

        directo=True (por defecto): reproduce con seguir_trayectoria_directo
        —comando directo interpolado, sin esperar convergencia— que es lo
        que confirmamos que funciona de forma confiable con pasos grabados
        por teleoperación (ensenar_y_jugar.py). 'pausa_s' controla el ritmo
        (más chico = más rápido). 'debe_detener' se pasa tal cual a
        seguir_trayectoria_directo (ver ahí) y el valor de retorno indica si
        terminó completo (True) o se interrumpió (False); con directo=False
        se ignora y siempre devuelve True.

        directo=False: usa el modo anterior (mover_suave, punto a punto,
        con tolerancia y timeout) — más lento y es el que mostró el bug de
        articulaciones atascadas, pero valida que el brazo sí haya llegado
        a cada punto; consérvalo solo si necesitas esa validación y ya
        confirmaste que no se atasca con tu trayectoria en particular.
        """
        if nombre not in self._trayectorias:
            disponibles = ", ".join(self._trayectorias) or "(ninguna aún)"
            raise KeyError(
                f"No existe el paso '{nombre}'. Pasos disponibles: {disponibles}. "
                f"Grábalo primero con ensenar_y_jugar.py"
            )
        puntos = self._trayectorias[nombre]
        if invertida:
            puntos = list(reversed(puntos))
        if directo:
            return self.seguir_trayectoria_directo(puntos, pausa_s=pausa_s, debe_detener=debe_detener)
        self.seguir_trayectoria_puntos(puntos, velocidad=velocidad, detener_en_error=detener_en_error)
        return True

    def seguir_trayectoria_directo(
        self,
        puntos: list[dict[str, float]],
        pausa_s: float = 0.5,
        hz_subpasos: float = HZ_CONTROL,
        debe_detener: Callable[[], bool] | None = None,
    ) -> bool:
        """
        Reproduce los puntos mandando comandos DIRECTOS al follower —sin el
        limitador de pasos de mover_suave() ni su espera de convergencia por
        tolerancia— igual que en teleoperación en vivo con el líder
        (enviar_accion_cruda). Esto evita el atasco que mostraba mover_suave
        en algunas articulaciones durante la reproducción.

        Entre un punto grabado y el siguiente, en vez de un solo salto
        brusco seguido de una pausa quieta, interpola linealmente en
        varios sub-pasos (a 'hz_subpasos' comandos por segundo) repartidos
        en 'pausa_s' segundos — el mismo intervalo con el que se grabó ese
        tramo. Así el movimiento se ve continuo en vez de a saltos, sin
        volver a depender de leer la posición real ni esperar que llegue
        (eso es justo lo que fallaba en mover_suave): es interpolación
        "ciega", igual de simple y confiable que un comando directo.

        'debe_detener': función opcional sin argumentos que devuelve True
        cuando hay que interrumpir el movimiento cuanto antes. Se revisa en
        cada sub-paso (cada ~1/hz_subpasos segundos), así que corta casi de
        inmediato, incluso a mitad de un tramo — pensado para un botón
        "Detener" en una interfaz externa (ej. una página web), donde
        Ctrl+C no está disponible. Devuelve False si se interrumpió así, o
        True si terminó la secuencia completa.

        Ctrl+C (ParoDeEmergencia) también la interrumpe si Robot se creó
        con capturar_sigint=True (el valor por defecto). No valida llegada
        ni lanza TimeoutError por atascos.
        """
        if not puntos:
            return True
        anterior = self.leer_posicion_actual()
        for punto in puntos:
            subpasos = max(1, round(pausa_s * hz_subpasos))
            pausa_subpaso = pausa_s / subpasos
            for i in range(1, subpasos + 1):
                if self._detener:
                    raise ParoDeEmergencia()
                if debe_detener is not None and debe_detener():
                    return False
                t = i / subpasos
                accion = {
                    f"{j}.pos": anterior.get(j, v) + (v - anterior.get(j, v)) * t
                    for j, v in punto.items()
                }
                self._robot.send_action(accion)
                time.sleep(pausa_subpaso)
            anterior = dict(punto)
        return True
