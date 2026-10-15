"""
app.py
Servidor web local para armar y correr secuencias con los pasos ya
enseñados (ver ensenar_y_jugar.py), desde una página en el navegador en
vez de escribir código a mano. Pensado para un salón: lo corres en el
computador conectado al brazo, y los estudiantes abren la página desde
sus propios computadores en la misma red.

Ojo: enseñar pasos nuevos (mover el líder) se sigue haciendo con
ensenar_y_jugar.py en la terminal, como hasta ahora — cierra este
servidor antes de correr ensenar_y_jugar.py (o viceversa), porque los dos
necesitan tener el puerto serial del brazo para ellos solos; no pueden
estar los dos conectados al mismo tiempo.

Instalar dependencias (una sola vez, con el entorno de lerobot activado):
    pip install fastapi "uvicorn[standard]"

Configura PUERTO_FOLLOWER / ID_FOLLOWER abajo con los de tu brazo, y
corre:
    uvicorn app:app --host 0.0.0.0 --port 8000

Después abre en el navegador:
    http://localhost:8000                (en el mismo computador)
    http://<ip-de-este-computador>:8000   (desde otro computador de la red)
"""

from __future__ import annotations

import threading
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel

from robot_api import Robot

# ---------------- Configuración: edita esto según tu brazo ----------------
PUERTO_FOLLOWER = "/dev/ttyACM0"
ID_FOLLOWER = "A"
ARCHIVO_TRAYECTORIAS = "trayectorias.json"
# ---------------------------------------------------------------------------

robot: Robot | None = None

# Estado compartido de la ejecución en curso, leído por /estado y escrito
# por el hilo que corre el programa. No hace falta un lock para estos
# campos simples (strings, bools, listas reasignadas enteras) porque el
# GIL de Python ya evita una lectura a medio escribir.
estado: dict[str, Any] = {"corriendo": False, "detener_pedido": False, "log": []}
lock_ejecucion = threading.Lock()


@asynccontextmanager
async def lifespan(app: FastAPI):
    global robot
    # capturar_sigint=False: este proceso lo apaga uvicorn (Ctrl+C en la
    # terminal del servidor), no Robot — así no compiten por el manejador
    # de la señal. La forma de detener un movimiento desde la web es el
    # botón "Detener" (endpoint /detener), no Ctrl+C.
    r = Robot(
        port=PUERTO_FOLLOWER,
        id=ID_FOLLOWER,
        archivo_trayectorias=ARCHIVO_TRAYECTORIAS,
        capturar_sigint=False,
    )
    robot = r.__enter__()
    try:
        yield
    finally:
        r.__exit__(None, None, None)
        robot = None


app = FastAPI(lifespan=lifespan)


# ---------------- Ejecutar un "programa" (árbol de bloques) ----------------
# Un programa es una lista de bloques. Cada bloque es uno de:
#   {"tipo": "paso", "nombre": "paso1", "invertida": false}
#   {"tipo": "repetir", "veces": 3, "cuerpo": [ ...bloques... ]}
#   {"tipo": "mientras", "cuerpo": [ ...bloques... ]}   # hasta presionar Detener


def _debe_detener() -> bool:
    return bool(estado["detener_pedido"])


def _ejecutar_bloques(bloques: list[dict[str, Any]]) -> bool:
    """Devuelve False en cuanto haya que interrumpir, True si terminó todo."""
    for bloque in bloques:
        if estado["detener_pedido"]:
            return False
        tipo = bloque.get("tipo")

        if tipo == "paso":
            nombre = bloque.get("nombre", "")
            invertida = bool(bloque.get("invertida", False))
            estado["log"].append(f"→ {nombre}" + (" (invertido)" if invertida else ""))
            try:
                completo = robot.seguir_trayectoria(
                    nombre, invertida=invertida, debe_detener=_debe_detener
                )
            except KeyError as e:
                estado["log"].append(f"[ERROR] {e}")
                return False
            if not completo:
                return False

        elif tipo == "repetir":
            veces = max(1, int(bloque.get("veces", 1)))
            cuerpo = bloque.get("cuerpo", [])
            for i in range(veces):
                if estado["detener_pedido"]:
                    return False
                estado["log"].append(f"repetir: vuelta {i + 1}/{veces}")
                if not _ejecutar_bloques(cuerpo):
                    return False

        elif tipo == "mientras":
            cuerpo = bloque.get("cuerpo", [])
            vuelta = 1
            while not estado["detener_pedido"]:
                estado["log"].append(f"mientras: vuelta {vuelta}")
                if not _ejecutar_bloques(cuerpo):
                    return False
                vuelta += 1

        else:
            estado["log"].append(f"[AVISO] tipo de bloque desconocido: {tipo!r}, se ignora")

    return True


def _correr_programa(programa: list[dict[str, Any]]) -> None:
    try:
        estado["log"].append("Iniciando programa...")
        completo = _ejecutar_bloques(programa)
        estado["log"].append("Programa terminado." if completo else "Programa detenido.")
    except Exception as e:  # nunca dejar el hilo morir en silencio
        estado["log"].append(f"[ERROR inesperado] {e}")
    finally:
        estado["corriendo"] = False
        estado["detener_pedido"] = False


# ---------------- Endpoints ----------------
class ProgramaIn(BaseModel):
    programa: list[dict[str, Any]]


@app.get("/")
def index() -> FileResponse:
    return FileResponse("index.html")


@app.get("/pasos")
def get_pasos() -> dict[str, list[str]]:
    return {"pasos": robot.listar_pasos()}


@app.get("/info")
def get_info() -> dict[str, str]:
    return {"puerto": PUERTO_FOLLOWER, "id": ID_FOLLOWER}


@app.get("/estado")
def get_estado() -> dict[str, Any]:
    return {"corriendo": estado["corriendo"], "log": estado["log"][-100:]}


@app.post("/ejecutar")
def ejecutar(cuerpo: ProgramaIn) -> dict[str, bool]:
    with lock_ejecucion:
        if estado["corriendo"]:
            raise HTTPException(409, "Ya hay un programa corriendo — espera a que termine o detenlo.")
        estado["corriendo"] = True
        estado["detener_pedido"] = False
        estado["log"] = []
        hilo = threading.Thread(target=_correr_programa, args=(cuerpo.programa,), daemon=True)
        hilo.start()
    return {"ok": True}


@app.post("/detener")
def detener() -> dict[str, bool]:
    estado["detener_pedido"] = True
    return {"ok": True}
