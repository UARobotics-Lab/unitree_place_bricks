"""
aura_test.py

Este script:
- Carga tu robot Aura (brazo) desde URDF.
- Verifica la cadena ETS generada.
- Prueba la cinemática directa (FK).
- Prueba la IK de la Toolbox interna.
- Prueba TU solver NR personalizado (desde aura_ik_methods.py).

Asegúrate de:
- Tener aura_ik_methods.py en la misma carpeta.
- Haber activado tu entorno conda: conda activate dktutorial
"""

# --- Librerías base ---
import roboticstoolbox as rtb # Asegúrate de tener instalada la versión correcta

import numpy as np
from roboticstoolbox.robot.ETS import ETS

# --- Importa tu clase ---
from IK import NR  # O LM, QP según quieras probar

#Monkey patch para reescribir el método eval de ETS
def robust_solve(self, Tep, q0=None):
    #Se copia la funcion original de robotic toolbox para ajustarla a nuestro robot Aura
    

    #Encuentra el index del joint mas grande
    max_jindex: int = int(np.max(self.jindices))  # Encuentra el índice máximo de jindices

        #Reserva q0_method
    q0_method = np.zeros((self.slimit, max_jindex + 1))
    
    jindices= np.asarray(self.jindices).astype(int)  # Aseguramos que jindices sea de tipo entero
    print("DEBUG >>> jindices:", jindices, type(jindices))


    if q0 is None:
        q0_method[:, jindices]= self._random_q(self, self.slimit)

    elif not isinstance(q0, np.ndarray):
        q0 = np.array(q0)

    if q0 is not None and q0.ndim == 1:

        q0_method[:, jindices]= self._random_q(self, self.slimit)
        q0_method[0, jindices]= q0

    if q0 is not None and q0.ndim == 2:
        q0_method[:, jindices]= self._random_q(self, self.slimit)
        q0_method[:q0.shape[0], self.jindices]= q0

    return q0_method

ETS.solve = robust_solve   
print(" Método eval de ETS modificado para usar jindices como enteros.")

from roboticstoolbox import Robot
from spatialmath import SE3
# ---  Carga el URDF ---
import os
#from roboticstoolbox import Robot

#  Ruta al URDF del robot Aura
urdf_path = os.path.abspath("g1_dual_arm_copy.urdf")
print("📁 Usando URDF:", urdf_path)

robot = Robot.URDF(urdf_path)

robot.qr= [0.0]*robot.n

print("Configuracion  (qr) definida:", robot.qr)


print(" Robot Aura cargado.")
print(f"Grados de libertad: {robot.n}")

# ---  Ver ETS ---
ets = robot.ets(end="right_rubber_hand")
print("ETS generado:")
print(ets)

#Forzamos jindices a tipo entero para evitar problemas con el solver
ets.jindices = np.array(ets.jindices, dtype=int)

print(" jindices:", ets.jindices)

# --- FK simple con configuración por defecto ---
Te_fk = ets.eval(robot.qr)
print(" FK de configuración por defecto (qr):")
print(Te_fk)

# --- IK usando toolbox integrada ---
T_goal = SE3(0.5, 0.2, 0.3)  #  Pose deseada 
print(f"Pose objetivo:\n{T_goal}")

# ---  IK usando NR ---
# Genera semilla de búsqueda

solver = NR(pinv=True, ilimit=30, slimit=50)
q0 = np.tile(robot.qr, (solver.slimit, 1))  

print(f"\n🔍 Probando TU solver: {solver.name}")


# Toolbox solver interno (prueba base)
#q_toolbox, success, _ = robot.ikine_LM(T_goal)
q_custom, success, _, _, _,_, _ = solver.solve(ets, T_goal.A, q0)
print(" Mi ETS.solve:", ETS.solve)
print(" ets.solve:", ets.solve)

""" print(" IK toolbox:")
print(f"q: {q_toolbox}")
print(f"FK del resultado toolbox: {robot.fkine(q_toolbox)}") """



q_custom, success, its, searches, E, jl_valid, t = solver.solve(ets, T_goal.A, q0)

print("✅ IK con TU NR:")
print(f"q: {q_custom}")
print(f"FK del resultado NR: {ets.eval(q_custom)}")
print(f"Converge: {success} | Iteraciones: {its} | Busquedas: {searches}")
print(f"Error E: {E} | Dentro de límites: {jl_valid} | Tiempo total: {t:.4f}s")

print("\n🎉 Prueba finalizada.")
