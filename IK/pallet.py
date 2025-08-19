from spatialmath import SE3 , SO3
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Dict, Literal, Optional, Union 


brick_size = (0.1, 0.2, 0.06) # Tamaño del ladrillo (ancho, largo, alto)
rows, cols = 3, 4  # Número de filas y columnas en el pallet

GraspName = Literal["top_center", "long_edge_mid", "short_edge_mid", "custom"]
#GraspName = str

@dataclass
class Brick:

    def __init__(
        self,
        width: float,
        length: float,
        height: float,
        grasp_points: Optional[Dict[GraspName, SE3]] = None,
        default_grasp: GraspName = "top_center",
    ):
        """Define dimensiones del ladrillo y puntos de agarre locales."""
        self.width = width
        self.length = length
        self.height = height

        # Si no nos pasan puntos de agarre, creamos unos por defecto
        if grasp_points is None:
            grasp_points = {
                "top_center":   SE3(0, 0,  +self.height / 2.0),
                "long_edge_mid":  SE3(0,  +self.length / 2.0, 0),
                "short_edge_mid": SE3(+self.width  / 2.0, 0, 0),
            }
        elif not isinstance(grasp_points, dict):
            raise TypeError("grasp_points debe ser un dict o None")

        self.grasp_points = grasp_points
        self.default_grasp = default_grasp if default_grasp in self.grasp_points else "top_center"
      
   
    def grasp_pose_local(self, name: Optional[GraspName] = None) -> SE3:
        """Devuelve la pose local del punto de agarre (por nombre)."""
        if name is None:
            name = self.default_grasp
        return self.grasp_points.get(name, self.grasp_points[self.default_grasp])

class Pallet:
    def __init__(
        self,
        rows,
        cols,
        layers,
        brick: Union[Brick, Tuple[float, float, float]],
        base_pose: SE3 = SE3(),
        gap: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        layout: str = "ordenado",
    ):

        """Inicializa un pallet con una cuadrícula de ladrillos.

        :param rows: Número de filas de ladrillos.
        :param cols: Número de columnas de ladrillos.
        :param brick: Instancia de Brick (dimensiones, grasp points).
        :param base_pose: Pose base del pallet en coordenadas globales.
        :param gap: Holgura (separación extra) entre ladrillos en (x, y, z).
        :param layout: "ordenado" para filas alineadas o "inclinado" para filas intercaladas.
        """

        if not isinstance(brick, Brick):
            # permitir (width, length, height)
            w, l, h = brick
            brick = Brick(w, l, h)

        self.rows = rows
        self.cols = cols
        self.layers = layers
        self.brick = brick
        self.base_pose = base_pose
        self.gap = gap
        self.layout = layout.lower()
        if self.layout not in {"ordenado", "inclinado"}:
            raise ValueError("layout debe ser 'ordenado' o 'inclinado'")

        # Separación entre ladrillos (dimensión del ladrillo más la holgura)
        self.pitch_x = self.brick.width + self.gap[0]   # Separación en x (ancho)
        self.pitch_y = self.brick.length + self.gap[1]  # Separación en y (largo)
        self.pitch_z = self.brick.height + self.gap[2]  # Separación en z (alto)

        # Precalcular las poses del centro de cada ladrillo
        self._grid_center = self._create_grid_centers()

        #self.grid = self._create_grid()

    def _row_offset(self, row: int) -> float:
        """Desplazamiento en X para una fila dada según el *layout*.

        En modo ``inclinado`` las filas impares se desplazan medio ladrillo en +X.
        """

        if self.layout == "inclinado" and (row % 2 == 1):
            return self.brick.width / 2.0
        return 0.0

    def _create_grid_centers(self):
        """Crea una cuadrícula de posiciones centrales para los ladrillos."""

        centers = []
        for i in range(self.rows):
            offset_x = self._row_offset(i)
            for j in range(self.cols):
                for k in range(self.layers):
                    # Posición del centro del ladrillo k, i, j
                    x = j * self.pitch_x + offset_x
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
        return self._grid_center[self.index_of(row, col, layer)]

    def is_slot_valid(self, row: int, col: int, layer: int) -> bool:
        """Verifica si la posición solicitada está dentro del pallet.

        Esta verificación incluye el desplazamiento aplicado en modo ``inclinado``.
        """

        if not (0 <= row < self.rows and 0 <= col < self.cols and 0 <= layer < self.layers):
            return False

        offset_x = self._row_offset(row)
        x_center = col * self.pitch_x + offset_x
        y_center = row * self.pitch_y

        # Bordes del ladrillo respecto al origen del pallet
        left = x_center - self.brick.width / 2.0
        right = x_center + self.brick.width / 2.0
        front = y_center - self.brick.length / 2.0
        back = y_center + self.brick.length / 2.0

        x_min = -self.brick.width / 2
        x_max = (self.cols - 1) * self.pitch_x + self.brick.width / 2
        y_min = -self.brick.length / 2
        y_max = (self.rows - 1) * self.pitch_y + self.brick.length / 2

        return (x_min <= left <= x_max) and (x_min <= right <= x_max) and \
               (y_min <= front <= y_max) and (y_min <= back <= y_max)

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
    
    def get_pose(self, row, col, layer, grasp: Optional[str] = None) -> SE3:
        """Devuelve la pose global del centro (o del ``grasp``) de un ladrillo."""

        if not self.is_slot_valid(row, col, layer):
            raise IndexError("Índice fuera de rango definido en el pallet.")

        index = self.index_of(row, col, layer)
        slot_pose = self._grid_center[index]

        # offset local del agarre
        T_grasp_local = self.brick.grasp_pose_local(grasp)
        return slot_pose * T_grasp_local
        
    