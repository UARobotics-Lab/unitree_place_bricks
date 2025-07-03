"""
aura_test.py

Este script:
- Carga Aura (brazo) desde URDF.
- Verifica la cadena ETS generada.
- Prueba la cinemática directa (FK).
- Prueba la IK de la Toolbox interna.
- Prueba solver NR personalizado (desde aura_ik_methods.py).

Es necesario:
- Tener IK.py en la misma carpeta.
- Haber activado entorno conda: conda activate dktutorial
"""

# --- Librerías base ---
import roboticstoolbox as rtb # Asegúrate de tener instalada la versión correcta

import numpy as np
from roboticstoolbox.robot.ETS import ETS

# --- Importa tu clase ---
from IK import NR, LM_Chan  # metodos de IK personalizados

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
print(" Usando URDF:", urdf_path)

robot = Robot.URDF(urdf_path)

robot.qr= [0.0]*robot.n

print("Configuracion  (qr) definida:", robot.qr)


print(" Robot Aura cargado.")
print(f"Grados de libertad: {robot.n}")

# ---  Ver ETS ---
ets = robot.ets(end="right_rubber_hand")
ets.jindices = np.array(ets.jindices, dtype=int)#Forzamos jindices a tipo entero para evitar problemas con el solver
print(f"ETS: {ets}")
print(" jindices: {ets.jindices}")

# --- FK simple con configuración por defecto ---
Te_fk = ets.eval(robot.qr)
print(" FK de configuración por defecto (qr):")
print(Te_fk)

# --- IK usando toolbox integrada ---
T_goal = SE3(0.5, 0.2, 0.3)  #  Pose deseada 
print(f"Pose objetivo:\n{T_goal}")

# ---  Solver de IK usando NR ---
# Genera inicio de búsqueda
np.random.seed(42)  # Para reproducibilidad
# Crea un punto de partida aleatorio para el solver
solver_NR = NR(pinv=True, ilimit=30, slimit=30)
q0=np.random.uniform(-1, 1, (solver_NR.slimit, robot.n))  # 30 intentos de inicio aleatorio

#q0 = np.tile(robot.qr, (solver.slimit, 1))  

print(f"\n Probando solver NR: {solver_NR.name}")

q_NR, success_NR, its_NR, searches_NR, E_NR, jl_valid_NR, t_NR = solver_NR.solve(ets, T_goal.A, q0)
print(" IK con NR:")
print(f"q: {q_NR}")
print(f"FK del resultado NR: {ets.eval(q_NR)}")
print(f"Converge: {success_NR} | Iteraciones: {its_NR} | Busquedas: {searches_NR}")
print(f"Error E: {E_NR} | Dentro de límites: {jl_valid_NR} | Tiempo total: {t_NR:.4f}s")

# ---  Solver de IK usando LM ---
solver_LM = LM_Chan(ilimit=30, slimit=30)
print(f"\n Probando solver LM: {solver_LM.name}")
# Crea un punto de partida aleatorio para el solver

q_LM, success_LM, its_LM, searches_LM, E_LM, jl_valid_LM, t_LM = solver_LM.solve(ets, T_goal.A, q0)
print(" IK con LM:")
print(f"q: {q_LM}")
print(f"FK del resultado LM: {ets.eval(q_LM)}")
print(f"Converge: {success_LM} | Iteraciones: {its_LM} | Busquedas: {searches_LM}")
print(f"Error E: {E_LM} | Dentro de límites: {jl_valid_LM} | Tiempo total: {t_LM:.4f}s")

# Toolbox solver interno (prueba base)
#q_toolbox, success, _ = robot.ikine_LM(T_goal)
#q_custom, success, _, _, _,_, _ = solver.solve(ets, T_goal.A, q0)
#print(" Mi ETS.solve:", ETS.solve)
#print(" ets.solve:", ets.solve)

""" print(" IK toolbox:")
print(f"q: {q_toolbox}")
print(f"FK del resultado toolbox: {robot.fkine(q_toolbox)}") """



#q_custom, success, its, searches, E, jl_valid, t = solver.solve(ets, T_goal.A, q0)

#print("✅ IK con TU NR:")
""" print(f"q: {q_custom}")
print(f"FK del resultado NR: {ets.eval(q_custom)}")
print(f"Converge: {success} | Iteraciones: {its} | Busquedas: {searches}")
print(f"Error E: {E} | Dentro de límites: {jl_valid} | Tiempo total: {t:.4f}s")
 """
print("\n🎉 Prueba finalizada.")

from swift import Swift

# Inicia Swift para visualización
env= Swift() #Crea un entorno de visualización
env.launch(reload=True)  # Lanza la ventana de visualización

# Carga el robot en Swift
env.add(robot, "aura_robot")  # Añade el robot Aura al entorno

# Muestra la configuración inicial del robot
robot.q = robot.qr  # Configuración inicial
env.step()  # Actualiza la visualización

input("Presiona Enter para mostrar solucion con NR...")
# Muestra la solución de IK con NR
robot.q = q_NR[0]  # Usa la primera solución de NR
env.step()  # Actualiza la visualización
input("Presiona Enter para mostrar solucion con LM...")
# Muestra la solución de IK con LM  
robot.q = q_LM[0]  # Usa la primera solución de LM
env.step()  # Actualiza la visualización
print("Visualización completa. Cierra la ventana de Swift para finalizar.")