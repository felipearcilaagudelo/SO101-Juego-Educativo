# SO101 — Juego de pasos con teleoperación

Extensión del proyecto [LeRobot-SO101-DualArm](https://github.com/AMBIENTE-INTEGRADO-DE-APRENDIZAJE/LeRobot-SO101-DualArm):
en vez de programar cinemática o usar imitation learning, **enseñas movimientos
moviendo el brazo líder con la mano**, los guardas con nombre, y los combinas
en secuencias (`for`/`while`) desde código Python o desde una página web con
botones — pensado para estudiantes de primeros semestres de ingeniería.

Un solo brazo (follower), control físico directo, sin simuladores ni redes
neuronales.

## Cómo funciona

1. **Enseñar un paso**: mueves el brazo **líder** con la mano; el **follower**
   lo sigue en vivo. El programa graba la postura automáticamente cada medio
   segundo mientras grabas, y guardas la secuencia con un nombre
   (`paso1`, `recoger`, `girar`, ...).
2. **Reproducir un paso**: se manda cada punto grabado directo al follower,
   interpolado en sub-pasos para que el movimiento se vea fluido — sin
   depender de que el brazo "llegue exacto" a cada punto, que es lo que
   fallaba con el enfoque inicial de interpolación por tolerancia.
3. **Combinar pasos**: en código con `for`/`while` normales de Python, o
   desde la página web armando bloques de "Paso", "Repetir N veces" y
   "Mientras no se detenga" — que muestra al lado el código Python
   equivalente.

## Archivos

| Archivo | Para qué sirve |
|---|---|
| `robot_api.py` | Capa de seguridad y movimiento (capa base: todo lo demás la usa) |
| `ensenar_y_jugar.py` | Enseña pasos por teleoperación con el líder (varios por sesión) |
| `mi_secuencia.py` | Plantilla de ejemplo para combinar pasos con `for`/`while` en código |
| `app.py` + `index.html` | Servidor web local + página para armar y correr secuencias con botones |
| `reto1_secuencia.py` | Reto fijo de pick-and-place (flujo más antiguo, con posiciones nombradas) |
| `capturar_posiciones.py` | Enseña posiciones sueltas a mano, con el torque apagado (flujo de `reto1_secuencia.py`) |
| `capturar_trayectoria.py` | Enseña trayectorias de varios puntos a mano (flujo de `reto1_secuencia.py`) |
| `ensenar_con_leader.py` | Enseña posiciones sueltas por teleoperación (versión anterior de `ensenar_y_jugar.py`) |

## Uso rápido

```bash
# 1. Enseñar pasos (terminal)
python ensenar_y_jugar.py \
    --puerto_follower /dev/ttyACM0 --id_follower A \
    --puerto_leader   /dev/ttyACM1 --id_leader   A

# 2. Probar una secuencia por código
python mi_secuencia.py --port /dev/ttyACM0 --id A

# 3. Página web para armar secuencias con botones
pip install fastapi "uvicorn[standard]"
# edita PUERTO_FOLLOWER / ID_FOLLOWER al inicio de app.py
uvicorn app:app --host 0.0.0.0 --port 8000
# abre http://localhost:8000
```

No corras `ensenar_y_jugar.py` y `app.py` al mismo tiempo — ambos necesitan
el puerto serial del brazo para ellos solos.

## Requiere

- Brazo(s) SO-101 calibrado(s) (ver repo base para el proceso de calibración).
- Entorno con [LeRobot](https://github.com/huggingface/lerobot) instalado.
- Python 3.10+.

## Seguridad

- El torque se fuerza a activado al conectar, para evitar que una
  articulación quede "muerta" por una falla anterior.
- Ctrl+C corta el movimiento como parada de emergencia en los scripts de
  terminal; desde la página web, el botón "Detener" revisa varias veces por
  segundo y corta casi de inmediato — pero ninguno de los dos reemplaza el
  interruptor físico. Supervisa siempre el brazo mientras está energizado.
