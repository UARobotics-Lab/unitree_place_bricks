from spatialmath import SE3
import numpy as np


class MoverLadrillo:
    """Rutinas para trasladar ladrillos entre dos pallets."""

    # Presets de mano (ABRIR y CERRAR)
    # LEFT_HAND_OPEN  = {0: 0.452213, 1: 0.228086, 2: 0.416099, 3: 0.700468, 4: 0.476052, 5: 0.805903, 6: 0.365497}
    # LEFT_HAND_CLOSE = {0: 0.452482, 1: 0.399409, 2: 0.0984169, 3: 0.66075, 4: 0.373582, 5: 0.744951, 6: 0.30257}
    LEFT_HAND_OPEN = {0: 0.496, 1: 0.2206, 2: 0.5171, 3: 0.7987, 4: 0.56881, 5: 0.7866, 6: 0.602}
    LEFT_HAND_CLOSE = {0: 0.496, 1: 0.21736, 2: 0.6289, 3: 0.7919, 4: 0.443957, 5: 0.787762, 6: 0.412952}

    def __init__(self, ets, solver, robot, pallet1, pallet2, altura_intermedia=0.02, tiempo_mov=3.0):
        self.robot = robot
        self.solver = solver
        self.ets = ets
        self.pallet1 = pallet1
        self.pallet2 = pallet2
        self.altura_intermedia = altura_intermedia
        self.tiempo_mov = tiempo_mov

    def resolver_pose(self, pose, cintura_rad=0.0, mano_izq=None, mano_der=None):
        np.random.seed(42)  # Valor fijo para reproducibilidad
        q0 = np.tile(self.robot.qr, (self.solver.slimit, 1))
        q, success, *_ = self.solver.solve(self.ets, pose.A, q0=q0)
        if not success:
            print(" IK falló para pose:\n", np.array_str(pose.A, precision=4, suppress_small=True))
            return None

        print(f"✅ IK exitosa. Pose objetivo:\n{np.array_str(pose.A, precision=4, suppress_small=True)}")
        print(f"➡️  q resuelto: {np.round(q, 4).tolist()} con cintura: {round(cintura_rad, 4)}\n")

        paso = {
            "tiempo": self.tiempo_mov,
            "brazo": np.round(q, 4).tolist(),
            "cintura": {

                "12": round(cintura_rad, 4),  # WaistYaw
                "13": 0.0,  # WaistRoll
                "14": 0.0   # WaistPitch
            }

        }
        if mano_izq is not None:
            paso["mano_izq"] = {int(k): float(v) for k, v in mano_izq.items()}
        if mano_der is not None:
            paso["mano_der"] = {int(k): float(v) for k, v in mano_der.items()}
        return paso

    def mover(self, posiciones_origen, posiciones_destino, cintura_giro_rad=-1.57):
        """Genera una rutina para trasladar varios ladrillos de un pallet a otro."""

        # Permitir que se pase una sola tupla para compatibilidad hacia atrás
        if isinstance(posiciones_origen, tuple):
            posiciones_origen = [posiciones_origen]
        if isinstance(posiciones_destino, tuple):
            posiciones_destino = [posiciones_destino]

        if len(posiciones_origen) != len(posiciones_destino):
            raise ValueError("La cantidad de posiciones de origen y destino debe coincidir")

        rutina = []

        for n, (pos_origen, pos_destino) in enumerate(zip(posiciones_origen, posiciones_destino)):
            # Validar posiciones solicitadas antes de obtener las poses
            # if not self.pallet1.is_slot_valid(*pos_origen):
            #     raise ValueError("Posición de origen fuera del rango del pallet 1")
            # if not self.pallet2.is_slot_valid(*pos_destino):
            #     raise ValueError("Posición de destino fuera del rango del pallet 2")

            # Obtener poses de cada par
            pose_origen = self.pallet1.get_pose(*pos_origen)
            pose_destino = self.pallet2.get_pose(*pos_destino)

            T_arriba_origen = pose_origen * SE3(0, 0, self.altura_intermedia)
            T_arriba_destino = pose_destino * SE3(0, 0, self.altura_intermedia)

            # Secuencia de movimientos para un ladrillo
            pasos = [
                (T_arriba_origen, 1.57),  # Ir encima del ladrillo
                (pose_origen, 1.57),       # Bajar a tomar el ladrillo
                (T_arriba_origen, 1.57),   # Subir el ladrillo
                (T_arriba_origen, 0.0),    # Giro de cintura para trasladar
                (T_arriba_destino, 0.0),   # Posición elevada sobre destino
                (pose_destino, 0.0),       # Bajar para dejar el ladrillo
                (T_arriba_destino, 0.0),   # Subir sin el ladrillo
            ]

            for idx, (pose, cintura_rad) in enumerate(pasos, start=1):
                if idx == 1:
                    paso = self.resolver_pose(pose, cintura_rad, mano_izq=self.LEFT_HAND_OPEN)
                elif idx == 2:
                    paso = self.resolver_pose(pose, cintura_rad, mano_izq=self.LEFT_HAND_CLOSE)
                elif idx == 6:
                    paso = self.resolver_pose(pose, cintura_rad, mano_izq=self.LEFT_HAND_OPEN)
                else:
                    paso = self.resolver_pose(pose, cintura_rad)

                if paso:
                    rutina.append(paso)
                else:
                    print(f"Fallo en el paso {idx} de la rutina.")

            # Rotar de vuelta hacia el pallet de origen si hay más ladrillos
            if n < len(posiciones_origen) - 1:
                paso_rot = self.resolver_pose(T_arriba_destino, 1.57)
                if paso_rot:
                    rutina.append(paso_rot)

        # --- Paso final antes del release ---
        paso_final_brazo = [
            0.27512645721435547,
            0.20702576637268066,
            -0.015451669692993164,
            0.978867769241333,
            0.07278227806091309,
            0.03146076202392578,
            0.004648685455322266,
        ]
        paso_final = {
            "tiempo": 3.0,
            "brazo": paso_final_brazo,
            "cintura": {
                "12": 0.0,
                "13": 0.0,
                "14": 0.0,
            },
            "mano_izq": self.LEFT_HAND_OPEN,
        }
        rutina.append(paso_final)
        return rutina
    

