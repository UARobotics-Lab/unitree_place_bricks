from spatialmath import SE3
import numpy as np


brick_size = (0.1, 0.2, 0.06) # Tamaño del ladrillo (ancho, largo, alto)
rows, cols = 3, 4  # Número de filas y columnas en el pallet

class Pallet:
    def __init__(self, rows, cols, brick_size, base_pose: SE3()):
        self.rows = rows
        self.cols = cols
        self.brick_size = brick_size
        self.base_pose = base_pose

        self.grid = self._create_grid()

    def _create_grid(self):

        """
        Crea una cuadrícula de posiciones para los ladrillos en el pallet.

        """

        poses = []
        for i in range(self.rows):
            for j in range(self.cols):
                x = j * self.brick_size[0]
                y = i * self.brick_size[1]
                z = 0 # todos los ladrillos están en el mismo plano

                #Pose local (respecto al pallet)

                local_pose = SE3(x, y, z)

                # Pose global (respecto al mundo)
                global_pose = self.base_pose * local_pose

                poses.append(global_pose)

        return poses
    
    def get_pose(self, row, col):
        """
        Obtiene la pose de un ladrillo específico en la cuadrícula.

        :param row: Fila del ladrillo.
        :param col: Columna del ladrillo.
        :return: Pose del ladrillo en el mundo.
        """
                
        index = row * self.cols + col

        if 0 <= index < len(self.grid):
            return self.grid[index]
        else:
            raise IndexError("Índice fuera de rango.")
        
        return self.grid[index]
    