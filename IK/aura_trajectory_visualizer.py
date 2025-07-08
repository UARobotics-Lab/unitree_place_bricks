#Visualizacion del movimiento de la trayectoria de IK
"""
aura_trajectory_visualizer.py

- Interpola desde qr hasta q_LM en pasos suaves.
- Visualiza la trayectoria en Swift 3D.
"""

import numpy as np
import os
import asyncio

from roboticstoolbox import Robot
from roboticstoolbox.robot.ETS import ETS
from spatialmath import SE3
from swift import Swift

# --- URDF ---
urdf_path = os.path.abspath("g1_dual_arm_copy.urdf")
print(f"Usando URDF: {urdf_path}")

robot = Robot.URDF(urdf_path)
robot.qr = [0.0] * robot.n
print(f"DOF: {robot.n}")

# --- ETS ---
ets = robot.ets(end="left_rubber_hand")
ets.jindices = np.asarray(ets.jindices, dtype=int)

print(f"ETS: {ets}")
print(f"qr: {robot.qr}")

# --- Solución IK (se carga del resultado que se obtiene con aura_test.py) ---

# --- Cargar q_LM calculado ---
q_goal = np.load("q_LM_result.npy")
print(" q_goal cargado:", q_goal)


# --- Trajectory ---
steps = 50
q_traj = np.linspace(robot.qr, q_goal, steps)

print(f"Generando {steps} pasos de interpolación...")

# --- Asegura asyncio OK en Windows ---
""" import asyncio

try:
    loop = asyncio.get_running_loop()
except RuntimeError:
    loop = None

if loop and loop.is_running():
    print(" Event loop YA está corriendo")
else:
    print(" Abriendo nuevo event loop")
    asyncio.set_event_loop(asyncio.new_event_loop())

# --- Lanza Swift ---
env = Swift()
env.launch(reload=True)

env.add(robot)

# --- Mostrar trayectoria ---
for i, q in enumerate(q_traj):
    robot.q = q
    env.step()
    print(f"Paso {i+1}/{steps} q={q}")

print(" Trayectoria visualizada.")

input("Presiona ENTER para cerrar Swift...")
env.close()
print("Fin.")
 """