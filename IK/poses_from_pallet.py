from pallet import Pallet

from rich.console import Console
from rich.table import Table

#from roboticstoolbox import Robot
from spatialmath import SE3, SO3
from spatialmath.base import trprint 
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
#print(" Método eval de ETS modificado para usar jindices como enteros.")
#ETS: Elementary Transformation Sequence-> Cadena cinematica usando transformaciones elementales

ets = robot.ets(end="left_rubber_hand")
ets.jindices = np.array(ets.jindices, dtype=int)#Forzamos jindices a tipo entero para evitar problemas con el solver

# --- Crear pallet ---
brick_size = (0.1, 0.2, 0.06)  # Tamaño del ladrillo (ancho, largo, alto)
z = 0.15 # Altura del pallet
# Inclinacion en eje y
orientacion = SO3.Ry(np.deg2rad(45))  # Rotación de 45 grados alrededor del eje Y
#pallet_pose = SE3(0.2, 0, z) 
pallet_pose = SE3(0.2, 0, z) * SE3(orientacion)  # Posición base del pallet respecto a coordenadas globales
pallet = Pallet(rows=3, cols=4, brick_size=brick_size, base_pose=pallet_pose)

# --- Solver de IK ---

solver = LM_Chan(ilimit=40, slimit=30, tol_min=1e-6, tol_max=1e-3)
# --- Vector de busqueda inicial aletorio ---
np.random.seed(42)  
#q0 =  np.random.uniform(-1, 1, (solver.slimit, robot.n))  # 30 intentos de inicio aleatorio
q0 = np.tile(robot.qr,(solver.slimit, 1)) + np.random.uniform(-0.3, 0.3, (solver.slimit, robot.n)) 

#Creación de tabla para mostrar resultados
console = Console()
table = Table(title="Resultados de IK", show_lines=True)

table.add_column("Fila", justify="left", style="cyan")
table.add_column("Columna", justify="left", style="cyan")
table.add_column("Estado", justify="left", style="green")
table.add_column("Iteraciones", justify="left", style="yellow")
table.add_column("Error", justify="left", style="magenta")
table.add_column("q solución", justify="left", style="blue")

# --- Guardar pasos de IK ---
q_steps = []

#print("\n{:<8} {:<8} {:<10} {:<12} {:<10}".format("Fila", "Columna", "Estado", "Iteraciones", "Error"))
#print("=" * 65)

for row in range(pallet.rows):
    for col in range(pallet.cols):
        # Obtener pose del ladrillo en la cuadrícula
        T_goal = pallet.get_pose(row, col)
        #print(f"Pose objetivo para ladrillo en fila {row}, columna {col}:\n{T_goal}")

        # Resolver IK
        q, success, its, searches, E, valid, t_total, q_steps_local, = solver.solve(ets, T_goal.A, q0=q0)
        
        estado = "ÉXITO" if success else "FALLO"
        iteraciones = str(its) if success else "-"
        error = f"{E:.2e}" if success else "0"
        q_str = f"[green]{np.round(q, 4).tolist()}[/green]" if success else "[red]―[/red]"
        style = "bold green" if success else "bold red"

        table.add_row(
            str(row),
            str(col),
            estado,
            iteraciones,
            error,
            q_str,
            style=style
           
        )
        
        # Detalles adicionales por cada ladrillo (opcional)
        console.print(f"\n[bold]Pose objetivo (SE3) para ({row}, {col}):[/bold]")
        console.print(np.array_str(T_goal.A, precision=4, suppress_small=True))

        #Encabezado de la tabla
        #print("{:<8} {:<8} {:<10} {:<12} {:.2e}".format(row, col, estado, its if success else "-", E if success else 0))

        # Mostrar pose SE3 formateada
        #print("┌──────────── Pose objetivo (SE3) ────────────┐")
        #trprint(T_goal.A, fmt="{:+.4f}", label="│ ", file=None)
        #print(np.array_str(T_goal.A, precision=4, suppress_small=True))
        #print("└────────────────────────────────────────────┘")
        
        if success:
            q_rounded = np.round(q, 4).tolist()  # Redondear a 4 decimales
            #print(f"Solución encontrada para ladrillo en fila {row}, columna {col}: {q}")
            #print(f"q solucion: {q_rounded.tolist()}\n")
            console.print(f"[green]q solución para ladrillo ({row}, {col}):[/green] {q_rounded}")
            q_steps.extend(q_steps_local)
        
        else:
            #print(f"No se encontró solución para ladrillo en fila {row}, columna {col}")
            #print("  ↳ No se pudo alcanzar la pose objetivo.\n")
            console.print("[red]↳ No se pudo alcanzar la pose objetivo.[/red]")

        #print("-" * 65)
        console.rule("")  # Línea horizontal de separación

# Mostrar tabla de resultados
console.print(table)


# --- Convertir a np.array y guardar pasos ---
q_steps = np.array(q_steps)
np.savetxt("q_steps_pallet_LM.csv", q_steps, delimiter=",")
print("Pasos de IK guardados en 'q_steps_pallet_LM.csv'.")