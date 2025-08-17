# # from spatialmath import SE3
# # import numpy as np

# # #Codigo para mover un ladrillo de un pallet a otro

# # from spatialmath import SE3
# # import numpy as np

# # def mover_ladrillo(ets, solver, robot, pallet1, pallet2,
# #                    pos_origen, pos_destino,
# #                    joint_cintura=22, angulo_giro=1.57):
# #     """
# #     Genera una rutina para mover un ladrillo desde una posición de pallet1 hacia otra en pallet2.
    
# #     :param ets: ETS del brazo a utilizar
# #     :param solver: Solucionador IK
# #     :param robot: Robot Aura
# #     :param pallet1: Pallet origen
# #     :param pallet2: Pallet destino
# #     :param pos_origen: (fila, columna, capa)
# #     :param pos_destino: (fila, columna, capa)
# #     :param joint_cintura: índice del motor de la cintura
# #     :param angulo_giro: ángulo en radianes para girar cintura (por defecto 90°)
    
# #     :return: Lista de posiciones articulares completas (con brazo y cintura)
# #     """

# #     q_rutina = []

# #     # --- 1. Obtener pose de recogida ---
# #     fila_o, col_o, layer_o = pos_origen
# #     T_pick = pallet1.get_pose(fila_o, col_o, layer_o)

# #     # Resolver IK para recogida
# #     q_pick, success_pick, *_ = solver.solve(ets, T_pick.A, q0=np.tile(robot.qr, (solver.slimit, 1)))
# #     if not success_pick:
# #         raise ValueError("No se pudo encontrar solución IK para recoger el ladrillo.")

# #     # Pose completa con brazo + cintura (sin giro)
# #     q_pick_full = robot.qr.copy()
# #     for idx_q, idx_joint in enumerate(ets.jindices):
# #         q_pick_full[int(idx_joint)] = q_pick[idx_q]
# #     q_pick_full[joint_cintura] = 0.0
# #     q_rutina.append(q_pick_full.copy())

# #     # --- 2. Elevar brazo para evitar colisión (opcional) ---
# #     lift = T_pick * SE3(0, 0, 0.1)  # subir 10 cm
# #     q_lift, success_lift, *_ = solver.solve(ets, lift.A, q0=np.tile(robot.qr, (solver.slimit, 1)))
# #     if success_lift:
# #         q_lift_full = robot.qr.copy()
# #         for idx_q, idx_joint in enumerate(ets.jindices):
# #             q_lift_full[int(idx_joint)] = q_lift[idx_q]
# #         q_lift_full[joint_cintura] = 0.0
# #         q_rutina.append(q_lift_full.copy())

# #     # --- 3. Giro cintura ---
# #     q_giro = q_lift_full.copy()
# #     q_giro[joint_cintura] = angulo_giro
# #     q_rutina.append(q_giro.copy())

# #     # --- 4. Pose de destino ---
# #     fila_d, col_d, layer_d = pos_destino
# #     T_place = pallet2.get_pose(fila_d, col_d, layer_d)

# #     # Resolver IK para colocar
# #     q_place, success_place, *_ = solver.solve(ets, T_place.A, q0=np.tile(robot.qr, (solver.slimit, 1)))
# #     if not success_place:
# #         raise ValueError("No se pudo encontrar solución IK para colocar el ladrillo.")

# #     q_place_full = robot.qr.copy()
# #     for idx_q, idx_joint in enumerate(ets.jindices):
# #         q_place_full[int(idx_joint)] = q_place[idx_q]
# #     q_place_full[joint_cintura] = angulo_giro
# #     q_rutina.append(q_place_full.copy())

# #     return q_rutina

# import numpy as np
# from spatialmath import SE3
# from IK import LM_Chan



# # === Clase para generar rutina de movimiento entre pallets ===
# def mover_ladrillo(ets, solver: LM_Chan, robot, pallet1, pallet2,
#                    pos_origen: tuple, pos_destino: tuple,
#                    altura_intermedia: float = 0.05,
#                    tiempo_mov: float = 3.0,
#                    cintura_giro_rad: float = 1.57):
#     """
#     Genera una rutina de movimiento para transportar un ladrillo desde una posición en pallet1
#     hasta otra posición en pallet2. Incluye movimiento intermedio y rotación de la cintura.

#     Retorna: lista de pasos (dicts), cada uno con:
#     {
#         'tiempo': duración del paso,
#         'brazo': lista de ángulos articulares (q),
#         'cintura': dict con valores para WaistYaw, WaistPitch, WaistRoll
#     }
#     """
#     rutina = []

#     def resolver_pose(pose, cintura_rad=0.0):
#         q0 = np.tile(robot.qr, (solver.slimit, 1)) + np.random.uniform(-0.3, 0.3, (solver.slimit, robot.n))
#         q, success, its, searches, E, valid, t_total = solver.solve(ets, pose.A, q0=q0)
#         if not success:
#             raise ValueError("No se pudo resolver IK para pose objetivo")

#         paso = {
#             "tiempo": tiempo_mov,
#             "brazo": np.round(q, 4).tolist(),
#             "cintura": {
#                 12: round(cintura_rad, 4),  # WaistYaw
#                 13: 0.0,  # WaistRoll
#                 14: 0.0   # WaistPitch
#             }

#         }
#         return paso

#     # --- Obtener poses ---
#     pose_origen = pallet1.get_pose(*pos_origen)
#     pose_destino = pallet2.get_pose(*pos_destino)

#     # --- Calcular altura intermedia ---
#     T_arriba_origen = pose_origen * SE3(0, 0, altura_intermedia)
#     T_arriba_destino = pose_destino * SE3(0, 0, altura_intermedia)

#     # === Secuencia de movimientos ===

#     # 1. Ir encima del ladrillo
#     rutina.append(resolver_pose(T_arriba_origen, cintura_rad=0.0))

#     # 2. Bajar a tomar el ladrillo
#     rutina.append(resolver_pose(pose_origen, cintura_rad=0.0))

#     # 3. Subir el ladrillo
#     rutina.append(resolver_pose(T_arriba_origen, cintura_rad=0.0))

#     # 4. Giro de cintura para trasladar
#     rutina.append(resolver_pose(T_arriba_origen, cintura_rad=cintura_giro_rad))

#     # 5. Posición elevada sobre destino
#     rutina.append(resolver_pose(T_arriba_destino, cintura_rad=cintura_giro_rad))

#     # 6. Bajar para dejar el ladrillo
#     rutina.append(resolver_pose(pose_destino, cintura_rad=cintura_giro_rad))

#     # 7. Subir sin el ladrillo
#     rutina.append(resolver_pose(T_arriba_destino, cintura_rad=cintura_giro_rad))

#     # 8. Volver a orientación inicial de cintura
#     rutina.append(resolver_pose(T_arriba_destino, cintura_rad=0.0))

#     return rutina

from spatialmath import SE3
import numpy as np

class MoverLadrillo:
   
    # Presets de mano (ABRIR y CERRAR)
    # LEFT_HAND_OPEN  = {0: 0.452213, 1: 0.228086, 2: 0.416099, 3: 0.700468, 4: 0.476052, 5: 0.805903, 6: 0.365497}
    # LEFT_HAND_CLOSE = {0: 0.452482, 1: 0.399409, 2: 0.0984169, 3: 0.66075, 4: 0.373582, 5: 0.744951, 6: 0.30257}
    LEFT_HAND_OPEN  = {0: 0.496, 1: 0.2206, 2: 0.5171, 3: 0.7987, 4: 0.56881, 5: 0.7866, 6: 0.602}
    LEFT_HAND_CLOSE = {0: 0.496, 1: 0.21736, 2: 0.6289, 3: 0.7919, 4: 0.443957 , 5: 0.787762 , 6: 0.412952}

 
    def __init__(self, ets, solver, robot, pallet1, pallet2,
                 altura_intermedia=0.02, tiempo_mov=3.0):
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

        #q0 = np.tile(self.robot.qr, (self.solver.slimit, 1)) + np.random.uniform(-0.3, 0.3, (self.solver.slimit, self.robot.n))
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
                12: round(cintura_rad, 4),  # WaistYaw
                13: 0.0,  # WaistRoll
                14: 0.0   # WaistPitch
            }
        }
        if mano_izq is not None:
            paso["mano_izq"] = {int(k): float(v) for k, v in mano_izq.items()}
        if mano_der is not None:
            paso["mano_der"] = {int(k): float(v) for k, v in mano_der.items()}
        return paso

    def mover(self, pos_origen: tuple, pos_destino: tuple, cintura_giro_rad=-1.57):
        """
        Genera una rutina para mover un ladrillo desde pallet1 a pallet2.
        
        :param pos_origen: (fila, columna, capa) en pallet1
        :param pos_destino: (fila, columna, capa) en pallet2
        :param cintura_giro_rad: ángulo de giro de la cintura en radianes
        :return: lista de pasos de movimiento
        """
        rutina = []

        # Validar posiciones solicitadas antes de obtener las poses
        if not self.pallet1.is_slot_valid(*pos_origen):
            raise ValueError("Posición de origen fuera del rango del pallet 1")
        if not self.pallet2.is_slot_valid(*pos_destino):
            raise ValueError("Posición de destino fuera del rango del pallet 2")

        # Obtener poses
        pose_origen = self.pallet1.get_pose(*pos_origen)
        print("\n[INFO] Pose de origen (SE3):")
        print(np.array_str(pose_origen.A, precision=4, suppress_small=True))

        pose_destino = self.pallet2.get_pose(*pos_destino)
        print("\n[INFO] Pose de destino (SE3):")
        print(np.array_str(pose_destino.A, precision=4, suppress_small=True))


        # Calcular altura intermedia
        T_arriba_origen = pose_origen * SE3(0, 0, self.altura_intermedia)
        T_arriba_destino = pose_destino * SE3(0, 0, self.altura_intermedia)

        print("\n[INFO] Pose elevada sobre origen (T_arriba_origen):")
        print(np.array_str(T_arriba_origen.A, precision=4, suppress_small=True))

        print("\n[INFO] Pose elevada sobre destino (T_arriba_destino):")
        print(np.array_str(T_arriba_destino.A, precision=4, suppress_small=True))


        # Secuencia de movimientos
        rutina.append(self.resolver_pose(T_arriba_origen, cintura_rad=1.57, mano_izq=self.LEFT_HAND_OPEN, mano_der=None))  # Ir encima del ladrillo

        pasos = [ 
            (T_arriba_origen, 1.57),  # Ir encima del ladrillo
            (pose_origen, 1.57),       # Bajar a tomar el ladrillo
            (T_arriba_origen, 1.57),   # Subir el ladrillo
            (T_arriba_origen, 0.0),  # Giro de cintura para trasladar
            (T_arriba_destino, 0.0),  # Posición elevada sobre destino
            (pose_destino, 0.0),       # Bajar para dejar el ladrillo
            (T_arriba_destino, 0.0),   # Subir sin el ladrillo
            (T_arriba_destino, 0.0)    # Volver a orientación
        ]

        # for idx, (pose, cintura_rad) in enumerate(pasos):
        #     paso = self.resolver_pose(pose, cintura_rad)
        #     if paso :
        #         rutina.append(paso)
        #     else:
        #         print(f"Fallo en el paso {idx + 1} de la rutina.")
        for idx, (pose, cintura_rad) in enumerate(pasos, start=1):
        # Inyecta mano solo donde corresponde
            if idx == 2:
                # En el contacto de origen: cerrar para agarrar
                paso = self.resolver_pose(pose, cintura_rad, mano_izq=self.LEFT_HAND_CLOSE)
            elif idx == 6:
                # En el contacto de destino: abrir para soltar
                paso = self.resolver_pose(pose, cintura_rad, mano_izq=self.LEFT_HAND_OPEN)
            else:
                paso = self.resolver_pose(pose, cintura_rad)

            if paso:
                rutina.append(paso)
            else:
                print(f"Fallo en el paso {idx} de la rutina.")

# --- Paso final antes del release ---

        # --- Paso final antes del release ---
        paso_final_brazo = [
            0.27512645721435547,
            0.20702576637268066,
            -0.015451669692993164,
            0.978867769241333,
            0.07278227806091309,
            0.03146076202392578,
            0.004648685455322266
        ]
        paso_final = {
            "tiempo": 3.0,
            "brazo": paso_final_brazo,
            "cintura": {
                "12": 0.0,  # WaistYaw
                "13": 0.0,  # WaistRoll
                "14": 0.0,  # WaistPitch
            },
            "mano_izq": self.LEFT_HAND_OPEN,
        }
        rutina.append(paso_final)
        return rutina
    