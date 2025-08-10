from spatialmath import SE3
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Dict, Literal, Optional


brick_size = (0.1, 0.2, 0.06) # Tamaño del ladrillo (ancho, largo, alto)
rows, cols = 3, 4  # Número de filas y columnas en el pallet

GraspName = Literal["top_center", "long_edge_mid", "short_edge_mid", "custom"]

@dataclass
class Brick:

    #Dimensiones en metros : ancho (x), largo (y), alto (z)
    width: float = brick_size[0]
    length: float = brick_size[1]
    height: float = brick_size[2]
    mass = 0.5  # Masa del ladrillo en kg (ajustar según sea necesario)
    #Por defecto: centro superior del ladrillo
    grasp_points: Dict[GraspName, SE3] = None

    def __post_init__(self):
        """
        Inicializa los puntos de agarre del ladrillo.
        """
        if self.grasp_points is None:
                    self.grasp_points = {
                "top_center": SE3(0, 0, +self.height / 2.0),
                "long_edge_mid": SE3(0, +self.length / 2.0, 0),
                "short_edge_mid": SE3(+self.width / 2.0, 0, 0),
                "custom": SE3()  # Puede ser modificado según necesidad
            }
    def grasp_pose_local(self, name: GraspName = "top_center") -> SE3:
        """
        Obtiene la pose de agarre local del ladrillo.

        :param name: Nombre del punto de agarre.
        :return: Pose del punto de agarre en coordenadas locales.
        """
        return self.grasp_points.get(name, self.grasp_points["top_center"])


    def __init__(self, position: SE3(), size: tuple = brick_size):
        """
        Inicializa un ladrillo con una posición y tamaño específicos.

        :param position: Posición del ladrillo en coordenadas globales.
        :param size: Tamaño del ladrillo (ancho, largo, alto).
        """
        self.position = position
        self.size = size  # Tamaño del ladrillo (ancho, largo, alto)
        self.width, self.length, self.height = size
        self.pose = SE3(self.position)
        self.pose = self.pose * SE3.Rx(np.pi / 2)
    
    def grip(self):
        """
        Simula el agarre del ladrillo.
        """
        print(f"Agarra el ladrillo en la posición: {self.position}")


    def release(self):
        """
        Simula la liberación del ladrillo.
        """
        print(f"Libera el ladrillo en la posición: {self.position}")

        return self.position
        

    

class Pallet:
    def __init__(self, rows, cols, layers, brick: Brick, base_pose: SE3()):

        """        Inicializa un pallet con una cuadrícula de ladrillos.

        :param rows: Número de filas de ladrillos.
        :param cols: Número de columnas de ladrillos.
        :param brick: Instancia de Brick (dimensiones, grasp points).
        :param base_pose: Pose base del pallet en coordenadas globales.        

        """

        self.rows = rows
        self.cols = cols
        self.layers = layers
        self.brick = brick
        self.base_pose = base_pose

        #Separación entre ladrillos
        self.pitch_x = self.brick.width  # Separación en x (ancho)
        self.pitch_y = self.brick.length  # Separación en y (largo)
        self.pitch_z = self.brick.height  # Separación en z (alto)

        self._grid_center = self._create_grid_centers()

        #self.grid = self._create_grid()

    def _create_grid_centers(self):
        """
        Crea una cuadrícula de posiciones centrales para los ladrillos en el pallet.

        """

        centers = []
        for i in range(self.rows):
            for j in range(self.cols):
                for k in range(self.layers):
                    #Posicion del centro del ladrillo k, i, j
                    x = j * self.pitch_x 
                    y = i * self.pitch_y 
                    z = k * self.pitch_z
                    local_center = SE3(x, y, z)
                    global_center = self.base_pose * local_center
                    centers.append(global_center)
        return centers
    
    def index_of(self, row: int, col: int, layer: int) -> int:
        if (
            0 <= row < self.rows and
            0 <= col < self.cols and
            0 <= layer < self.layers
        ):
            return layer*(self.rows*self.cols) + row*self.cols + col
        raise IndexError("Índice fuera de rango definido en el pallet.")

    def get_center_pose(self, row: int, col: int, layer: int) -> SE3:
        return self._grid_centers[self.index_of(row, col, layer)]

    def _create_grid(self):

        """
        Crea una cuadrícula de posiciones para los ladrillos en el pallet.

        """

        poses = []
        for i in range(self.rows):
            for j in range(self.cols):
                for k in range(self.layers):
                    x = j * self.brick_width #columnas en x
                    y = i * self.brick_length # filas en y
                    z = k * self.brick_height # todos los ladrillos están en el mismo plano

                    #Pose local (respecto al pallet)

                    local_pose = SE3(x, y, z)

                    # Pose global (respecto al mundo)
                    global_pose = self.base_pose * local_pose

                    poses.append(global_pose)

        return poses
    
    def get_pose(self, row, col, layer, grasp: GraspName = "top_center", approach_rot: Optional[SO3] = None, approach_offset: float = 0.0) -> SE3:
        """
        Devuelve la pose objetivo del **punto de agarre** del ladrillo en mundo.
        - grasp: cuál punto de agarre usar (definido en Brick)
        - approach_rot: orientación adicional del efector al aproximar (SO3), ej. rotar la pinza
        - approach_offset: desplazar en -Z local del efector (acercamiento vertical) en metros
        """
        center_world = self.get_center_pose(row, col, layer)
        
        #Pose del grasp en el frame del ladrillo
        T_grasp_local = self.brick.grasp_pose_local(grasp)

        # Orientación del efector: por defecto, misma que el pallet (vertical Z+),
        # Se puede sobreescribir con approach_rot
        R_approach = approach_rot if approach_rot is not None else SO3()

        # Armamos: (mundo→centro) * (centro→grasp) * (rot extra de la herramienta)
        T_grasp_world = center_world * T_grasp_local * SE3(R_approach)

        #Para llegar desde arriba, se agrega un offset en -z del efector
        if approach_offset != 0.0:
            T_grasp_world = T_grasp_world * SE3(0, 0, -approach_offset)

        return T_grasp_world
        
 
    