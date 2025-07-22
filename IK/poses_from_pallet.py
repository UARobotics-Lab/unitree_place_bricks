from pallet import Pallet
#from roboticstoolbox import Robot
from spatialmath import SE3, SO3
from IK import NR, LM_Chan  # métodos de IK personalizadosi 
import numpy as np

#Cargar el URDF del robot Aura
import roboticstoolbox as rtb 
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox import Robot


import os

#  Ruta al URDF del robot Aura
urdf_path = os.path.abspath("g1_dual_arm_copy.urdf") #URDF modificado para Aura
robot = Robot.URDF(urdf_path)
robot.qr= [0.0]*robot.n
print(" Robot Aura cargado.")

print(robot.fkine(robot.qr))  # FK en la pose neutral

# ---  Ver ETS ---
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
#ETS: Elementary Transformation Sequence-> Cadena cinematica usando transformaciones elementales

ets = robot.ets(end="left_rubber_hand")
ets.jindices = np.array(ets.jindices, dtype=int)#Forzamos jindices a tipo entero para evitar problemas con el solver

# --- Crear pallet ---
brick_size = (0.1, 0.2, 0.06)  # Tamaño del ladrillo (ancho, largo, alto)
z = 0.15 # Altura del pallet
# Inclinacion en eje y
orientacion = SO3.Ry(np.deg2rad(45))  # Rotación de 45 grados alrededor del eje Y
pose  = SE3(0.2, 0, 0) * SE3(orientacion)  # Posición base del pallet con orientación
pallet_pose = SE3(0.2, 0, z)  # Posición base del pallet respecto a coordenadas globales
pallet = Pallet(rows=3, cols=4, brick_size=brick_size, base_pose=pallet_pose)

# --- Solver de IK ---

solver = LM_Chan(ilimit=40, slimit=30, tol_min=1e-6, tol_max=1e-3)
# --- Vector de busqueda inicial aletorio ---
np.random.seed(42)  
#q0 =  np.random.uniform(-1, 1, (solver.slimit, robot.n))  # 30 intentos de inicio aleatorio
q0 = np.tile(robot.qr,(solver.slimit, 1)) + np.random.uniform(-0.3, 0.3, (solver.slimit, robot.n)) 

# --- Guardar pasos de IK ---
q_steps = []

for row in range(pallet.rows):
    for col in range(pallet.cols):
        # Obtener pose del ladrillo en la cuadrícula
        T_goal = pallet.get_pose(row, col)
        print(f"Pose objetivo para ladrillo en fila {row}, columna {col}:\n{T_goal}")

        # Resolver IK
        q, success, its, searches, E, valid, t_total, q_steps_local, = solver.solve(ets, T_goal.A, q0=q0)
        
        if success:
            print(f"Solución encontrada para ladrillo en fila {row}, columna {col}: {q}")
            q_steps.extend(q_steps_local)
        
        else:
            print(f"No se encontró solución para ladrillo en fila {row}, columna {col}")

# --- Convertir a np.array y guardar pasos ---
q_steps = np.array(q_steps)
np.savetxt("q_steps_pallet_LM.csv", q_steps, delimiter=",")
print("Pasos de IK guardados en 'q_steps_pallet_LM.csv'.")