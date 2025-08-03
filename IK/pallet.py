from spatialmath import SE3
import numpy as np


brick_size = (0.1, 0.2, 0.06) # Tamaño del ladrillo (ancho, largo, alto)
rows, cols = 3, 4  # Número de filas y columnas en el pallet

class brick:
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
    def __init__(self, rows, cols, layers, brick_size, base_pose: SE3()):

        """        Inicializa un pallet con una cuadrícula de ladrillos.

        :param rows: Número de filas de ladrillos.
        :param cols: Número de columnas de ladrillos.
        :param brick_size: Tamaño de cada ladrillo (ancho, largo, alto).
        :param base_pose: Pose base del pallet en coordenadas globales.        

        """

        self.rows = rows
        self.cols = cols
        self.layers = layers
        self.brick_width, self.brick_length, self.brick_height   = brick_size
        self.base_pose = base_pose

        self.grid = self._create_grid()

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
    
    def get_pose(self, row, col, layer):
        """
        Obtiene la pose de un ladrillo específico en la cuadrícula.

        :param row: Fila del ladrillo.
        :param col: Columna del ladrillo.
        :return: Pose del ladrillo en el mundo.
        """
                
        
        if (
            0 <= row < self.rows and 
            0 <= col < self.cols and
            0 <= layer < self.layers
        ):
            index = layer*(self.rows * self.cols) + row*self.cols + col
            return self.grid[index]
        
        else:
            raise IndexError("Índice fuera de rango definido en el pallet.")
        
        return self.grid[index]
    