from pallet import Pallet , Brick

from rich.console import Console
from rich.table import Table

#from roboticstoolbox import Robot
from spatialmath import SE3, SO3
from spatialmath.base import trprint 
from IK import NR, LM_Chan  # métodos de IK personalizadosi 
import numpy as np

from mover_ladrillo import MoverLadrillo

#Cargar el URDF del robot Aura
import roboticstoolbox as rtb 
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox import Robot


import os
import json


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

ets = robot.ets(end="left_rubber_hand")
ets.jindices = np.array(ets.jindices, dtype=int)#Forzamos jindices a tipo entero para evitar problemas con el solver

# --- Definición de ladrillos y pallets ---
#Definición de ladrillo
brick = Brick( width=0.1, length=0.2, height=0.06)

# Crear pallet 
z = 0.10 # Altura del pallet
orientacion = SO3.Rz(np.deg2rad(0)) 

""" 
pallet_pose = SE3(0.2, 0, z)  
pallet_pose = SE3(0.1, 0.15, z) * SE3(orientacion)  # Posición base del pallet respecto a coordenadas globales
pallet = Pallet(rows=2, cols=1, layers=3, brick_size=brick_size, base_pose=pallet_pose)
 """ 
#Pallet 1 y pallet 2 para mover ladrillos
pallet1_pose = SE3(0.1, 0.15, z) * SE3(orientacion)
pallet2_pose = SE3(0.1, 0.15, z) * SE3(orientacion)

pallet1 = Pallet(rows=1, cols=3, layers=3, brick= brick, base_pose=pallet1_pose)
pallet2 = Pallet(rows=1, cols=3, layers=3, brick= brick, base_pose=pallet2_pose)


# --- Solver de IK ---

solver = LM_Chan(ilimit=40, slimit=30, tol_min=1e-6, tol_max=1e-3)
np.random.seed(42) 
#solver = LM_Chan(ilimit=40, slimit=30)
#q0 =  np.random.uniform(-1, 1, (solver.slimit, robot.n))  # 30 intentos de inicio aleatorio
q0 = np.tile(robot.qr,(solver.slimit, 1)) + np.random.uniform(-0.3, 0.3, (solver.slimit, robot.n)) 


# --- Ejecutar mover_ladrillo ---

# rutina = mover_ladrillo(
#     ets=ets,
#     solver=solver,
#     robot=robot,
#     pallet1=pallet1,
#     pallet2=pallet2,
#     pos_origen=(0, 0, 0),
#     pos_destino=(1, 0, 0)
# )

# np.savetxt("rutina_ladrillo.csv", rutina, delimiter=",")

mover = MoverLadrillo(robot=robot, solver=solver, ets=ets, pallet1=pallet1, pallet2=pallet2)

rutina = mover.mover(pos_origen=(0, 0, 0), pos_destino=(0, 0, 0), cintura_giro_rad=np.deg2rad(90))

cintura_por_pallet = {
    "Pallet 1": 0.0,
    "Pallet 2": -1.57
}

with open("rutina_ladrillo.json", "w") as f:
    json.dump(rutina, f, indent=4)

print("Rutina de movimiento guardada en 'rutina_ladrillo.json'.")





#Creación de tabla para mostrar resultados
console = Console()
table = Table(title="Resultados de IK", show_lines=True)

table.add_column("Pallet", justify="left", style="bold blue")
table.add_column("Fila", justify="left", style="cyan")
table.add_column("Columna", justify="left", style="cyan")
table.add_column("Layer", justify="left", style="cyan")
table.add_column("Cintura (rad)", justify="left", style="magenta")
table.add_column("Estado", justify="left", style="green")
table.add_column("Iteraciones", justify="left", style="yellow")
table.add_column("Error", justify="left", style="magenta")
table.add_column("q solución", justify="left", style="blue")

# --- Guardar pasos de IK ---
q_steps = []

# --- Diccionario para iterar sobre ambos pallets ---
pallets = {
    "Pallet 1": pallet1,
    "Pallet 2": pallet2
}

for nombre, pallet in pallets.items():
    console.rule(f"[bold blue] Revisión de {nombre} [/bold blue]")
    cintura_yaw = cintura_por_pallet[nombre]

    for row in range(pallet.rows):
        for col in range(pallet.cols):
            for layer in range(pallet.layers):
                # Obtener pose del ladrillo en la cuadrícula
                T_goal = pallet.get_pose(row, col, layer)
                #print(f"Pose objetivo para ladrillo en fila {row}, columna {col}:\n{T_goal}")

                # Resolver IK
                #q, success, its, searches, E, valid, t_total, q_steps_local, = solver.solve(ets, T_goal.A, q0=q0)
                q, success, its, searches, E, valid, t_total, = solver.solve(ets, T_goal.A, q0=q0)
                
                estado = "ÉXITO" if success else "FALLO"
                iteraciones = str(its) if success else "-"
                error = f"{E:.2e}" if success else "0"
                q_str = f"[green]{np.round(q, 4).tolist()}[/green]" if success else "[red]―[/red]"
                style = "bold green" if success else "bold red"
                
                for paso in rutina:
                    if "brazo" in paso:
                        q_rutina = np.array(paso["brazo"])
                        if np.allclose(q[:len(q_rutina)], q_rutina, atol=1e-3):
                            cintura_yaw = paso["cintura"].get("12", 0.0)
                            break

                table.add_row(
                    nombre,
                    str(row),
                    str(col),
                    str(layer),
                    f"{cintura_yaw:.2f}",
                    estado,
                    iteraciones,
                    error,
                    q_str,
                    style=style
                
                )
                
                # Detalles adicionales por cada ladrillo (opcional)
                console.print(f"\n[bold]Pose objetivo (SE3) para ({row}, {col}, {layer}):[/bold]")
                console.print(np.array_str(T_goal.A, precision=4, suppress_small=True))
                
                if success:
                    q_rounded = np.round(q, 4).tolist()  # Redondear a 4 decimales
                    #print(f"Solución encontrada para ladrillo en fila {row}, columna {col}: {q}")
                    #print(f"q solucion: {q_rounded.tolist()}\n")
                    console.print(f"[green]q solución para ladrillo ({row}, {col}):[/green] {q_rounded}")
                    q_steps.append(q.copy())  # Guardar el paso actual de q
                
                else:
                    #print(f"No se encontró solución para ladrillo en fila {row}, columna {col}")
                    #print("  ↳ No se pudo alcanzar la pose objetivo.\n")
                    console.print("[red]↳ No se pudo alcanzar la pose objetivo.[/red]")
                
                console.rule("")  # Línea horizontal de separación

# Mostrar tabla de resultados
console.print(table)


# --- Convertir a np.array y guardar pasos ---
q_steps = np.array(q_steps)
np.savetxt("q_steps_pallet_LM.csv", q_steps, delimiter=",")
print("Pasos de IK guardados en 'q_steps_pallet_LM.csv'.")