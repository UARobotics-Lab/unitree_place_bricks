import sys
import time
import math
import numpy as np
import csv
import pandas as pd
import json
import os
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# sequence = ["release_arm_sdk.txt",
#             "initial_position_arm.txt",
#             "R_START_POINT_HAND_OPEN.txt",
#             "R_START_POINT_HAND_CLOSE.txt",
#             "R_N0_RIGHT_IMG.txt",
#             "R_N0_RIGHT_REAL.txt",
#             "R_N0_CENTER_IMG.txt",
#             "R_N0_CENTER_REAL.txt",
#             "R_N0_LEFT_IMG.txt",
#             "R_N0_LEFT_REAL.txt",
#             "R_N1_LEFT_IMG.txt",
#             "R_N1_LEFT_REAL.txt",
#             "R_N1_RIGHT_IMG.txt",
#             "R_N1_RIGHT_REAL.txt",
#             "R_N2_CENTER_IMG.txt",
#             "R_N2_CENTER_REAL.txt",
#             "initial_position_arm.txt",
#             "release_arm_sdk.txt"
#             ]

# sequence = ["R_JOIN_INITIAL_POSITION.json",
#             "R_JOIN_RESTORE_POSITION.json"
#             ]

sequence = ["R_PICK_PLACE_N2_TOP_TO_N0_LEFT.json"]

sequence = ["R_PICK_PLACE_N1_RIGHT_TO_N0_CENTER.json"]

sequence = ["R_PICK_PLACE_N1_LEFT_TO_N0_RIGHT.json"]

# sequence = ["R_PICK_PLACE_N0_RIGH_TO_N1_RIGHT .json"]

sequence = ["R_PICK_PLACE_N0_LEFT_TO_N1_LEFT.json"]

sequence = ["R_PICK_PLACE_N0_LEFT_TO_N2_TOP.json"]

sequence = ["R_00_PICK_PLACE_N2_TOP_TO_N0_RIGHT.json",
            "R_01_PICK_PLACE_N1_RIGHT_TO_N0_CENTER.json",
            "R_02_PICK_PLACE_N1_LEFT_TO_N0_LEFT.json",
            "R_03_PICK_PLACE_N0_RIGHT_TO_N1_RIGHT .json",
            "R_04_PICK_PLACE_N0_CENTER_TO_N1_LEFT.json",
            "R_05_PICK_PLACE_N0_LEFT_TO_N2_TOP.json"]

sequence = ["R_RW_00_PICK_PLACE_N2_TOP_TO_N0_RIGHT.json",
            "R_RW_01_PICK_PLACE_N1_RIGHT_TO_N0_CENTER.json"]
            
sequence = ["R_TJOIN_RW_00_PICK_PLACE_N2_TOP_TO_N0_RIGHT.json",
            "R_SAME_PLACE_01_PICK_PLACE_N2_TOP_TO_N0_RIGHT.json"]

# sequence = ["R_SAME_PLACE_00_PICK_PLACE_NS_TO_N0_LEFT.json",
#             "R_SAME_PLACE_01_PICK_PLACE_NS_TO_N0_CENTER.json"
#             ]

sequence = ["R_SAME_PLACE_00_PICK_PLACE_NS_TO_N0_LEFT.json",
            "R_SAME_PLACE_01_PICK_PLACE_NS_TO_N0_CENTER.json",
            "R_SAME_PLACE_02_PICK_PLACE_NS_TO_N1_RIGHT.json",
            "00_PLANTILLA.json"
            ]

# sequence = ["00_PLANTILLA.json"]

sequence = ["00_LAST_BRICK.json"]

# sequence = ["00_LAST_BRICK_WORK_WITH_DETAILS.json"]

release = False
ruta="./move_steps/"

# === Índices de articulaciones ===
class G1JointIndex:
    #Brazos 
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
    kNotUsedJoint = 29
    #Cintura
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14



# === Control de la mano ===
class HandSequence:
    def __init__(self):
        self.publisher_left = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        self.publisher_left.Init()
        self.publisher_right = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        self.publisher_right.Init()

        self.msg_left = unitree_hg_msg_dds__HandCmd_()
        self.msg_right = unitree_hg_msg_dds__HandCmd_()

        self.num_motors = 7
        self.kp = 1.5
        self.kd = 0.2

        self._init_msg(self.msg_left)
        self._init_msg(self.msg_right)

    def _init_msg(self, msg):
        for i in range(self.num_motors):
            mode = (i & 0x0F) | ((0x01 & 0x07) << 4)
            msg.motor_cmd[i].mode = mode
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].tau = 0.0
            msg.motor_cmd[i].kp = self.kp
            msg.motor_cmd[i].kd = self.kd

    def send_left(self, posiciones: dict):
        msg = self.msg_left
        for i in range(self.num_motors):
            msg.motor_cmd[i].q = posiciones.get(i, 0.0)
            mode = (i & 0x0F) | ((0x01 & 0x07) << 4)
            msg.motor_cmd[i].mode = mode
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].tau = 0.0
            msg.motor_cmd[i].kp = self.kp
            msg.motor_cmd[i].kd = self.kd
        self.publisher_left.Write(msg)

    def send_right(self, posiciones: dict):
        msg = self.msg_right
        for i in range(self.num_motors):
            msg.motor_cmd[i].q = posiciones.get(i, 0.0)
            mode = (i & 0x0F) | ((0x01 & 0x07) << 4)
            msg.motor_cmd[i].mode = mode
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].tau = 0.0
            msg.motor_cmd[i].kp = self.kp
            msg.motor_cmd[i].kd = self.kd
        self.publisher_right.Write(msg)

    def freeze_and_release(self):
        for i in range(self.num_motors):
            self.msg_left.motor_cmd[i].q = 0.0
            self.msg_left.motor_cmd[i].kp = 0.0
            self.msg_left.motor_cmd[i].kd = 0.0

            self.msg_right.motor_cmd[i].q = 0.0
            self.msg_right.motor_cmd[i].kp = 0.0
            self.msg_right.motor_cmd[i].kd = 0.0

        self.publisher_left.Write(self.msg_left)
        self.publisher_right.Write(self.msg_right)


# === Control del brazo ===
class ArmSequence:
    def __init__(self):
        self.control_dt = 0.02
        self.kp = 60.0
        self.kd = 1.5
        self.crc = CRC()
        self.t = 0.0
        self.T = 3.0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update = False
        self.target_pos = {}
        self.q_init_override = None

        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw, G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll, G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw, 
            G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch, #Cintura
            #Brazo derecho
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw, G1JointIndex.RightElbow,
            # G1JointIndex.RightWristRoll, G1JointIndex.RightWristPitch,
            # G1JointIndex.RightWristYaw,
        ]

    def Init(self):
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.thread = RecurrentThread(interval=self.control_dt, target=self.LowCmdWrite, name="arm_control")
        while not self.first_update:
            time.sleep(0.1)
        self.thread.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        if not self.first_update:
            self.first_update = True

    def interpolate_position(self, q_init, q_target):

        """
        Perfil polinómico de 5to grado para movimiento suave.
        """
        #Normalizacion del tiempo
        
        if self.t >= self.T: #self.t es el tiempo actual, self.T es el tiempo total del movimiento
            s = 1.0
        else:
            s = self.t / self.T #s es el tiempo normalizado entre 0 y 1

        #Calculo de potencias de s
        s3 = s ** 3
        s4 = s3 * s
        s5 = s4 * s

        s_quintic = 10 * s3 - 15 * s4 + 6 * s5

        return q_init + (q_target - q_init) * s_quintic

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

        for joint in self.arm_joints:
            q_init = self.q_init_override.get(joint, self.low_state.motor_state[joint].q) if self.q_init_override else self.low_state.motor_state[joint].q
            q_target = self.target_pos.get(joint, q_init)
            pos = self.interpolate_position(q_init, q_target)
            self.low_cmd.motor_cmd[joint].q = pos
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)
        self.t += self.control_dt

    def move_to(self, updates: dict, duration=1.5, q_init_override=None):
        # Actualiza o restablece la posición objetivo para el siguiente paso
        self.target_pos = updates.copy() if updates else {}
        self.T = duration
        self.t = 0.0
        self.q_init_override = q_init_override
        while self.t < self.T:
            time.sleep(self.control_dt)
        # Al finalizar el movimiento se limpia cualquier override inicial
        self.q_init_override = None

    def freeze_and_release_a(self):
        for joint in self.arm_joints:
            self.low_cmd.motor_cmd[joint].q = self.low_state.motor_state[joint].q
            self.low_cmd.motor_cmd[joint].kp = 0.0
            self.low_cmd.motor_cmd[joint].kd = 0.0

        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)


# === Bloque principal ===
def main():
    ChannelFactoryInitialize(0, sys.argv[1])  # Init DDS

    seq = ArmSequence()
    hand_seq = HandSequence()
    seq.Init()
    seq.Start()



    #ruta_archivo_txt = ruta


    arm_joints = [
        G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
        G1JointIndex.LeftShoulderYaw, G1JointIndex.LeftElbow,
        G1JointIndex.LeftWristRoll, G1JointIndex.LeftWristPitch,
        G1JointIndex.LeftWristYaw,
        G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch,
        #Brazo derecho
        G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
        G1JointIndex.RightShoulderYaw, G1JointIndex.RightElbow,
        # G1JointIndex.RightWristRoll, G1JointIndex.RightWristPitch,
        # G1JointIndex.RightWristYaw,
    ]


    res = ''

    for file in sequence:
        print(f"{file}")
        try:
            with open(ruta + file, "r") as f:
                pasos = json.load(f)
                print(pasos)
        except:
            print("error")
            sys.exit()

        articulaciones_activas = [0, 1, 2, 3, 4, 5, 6] #Posicion del joint dentro de la lista arm_joints

        q_anterior = None

        res = input("Presiona Enter para continuar, X para salir: ")
        if res.lower() == 'x':
            print("Cancelado por el usuario. Moviendo a postura segura de release...")
            break
                
        for i, paso in enumerate(pasos):
            print(f"{i} *** {file} --- {paso}")
            q_brazo = paso["brazo"]
            brazo_r = paso["brazo_r"]
            cintura = paso["cintura"]
            
            mano_izq = paso.get("mano_izq")  # dict opcional {0..6: q}
            mano_der = paso.get("mano_der")  # dict opcional {0..6: q}
            dur = float(paso.get("tiempo", 1.25))

            posiciones_brazo = {}

            # Articulaciones del brazo izquierdo
            for j, joint_idx in enumerate([
                G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
                G1JointIndex.LeftShoulderYaw, G1JointIndex.LeftElbow,
                G1JointIndex.LeftWristRoll, G1JointIndex.LeftWristPitch,
                G1JointIndex.LeftWristYaw
                ]):
                posiciones_brazo[joint_idx] = q_brazo[j]

            # Agregar cintura (Yaw, Roll, Pitch)
            posiciones_brazo[G1JointIndex.WaistYaw] = cintura.get("12", 0.0)
            posiciones_brazo[G1JointIndex.WaistRoll] = cintura.get("13", 0.0)
            posiciones_brazo[G1JointIndex.WaistPitch] = cintura.get("14", 0.0)

            # Right arm
            posiciones_brazo[G1JointIndex.RightShoulderPitch] = brazo_r.get("22", 0.0)
            posiciones_brazo[G1JointIndex.RightShoulderRoll] = brazo_r.get("23", 0.0)
            posiciones_brazo[G1JointIndex.RightShoulderYaw] = brazo_r.get("24", 0.0)
            posiciones_brazo[G1JointIndex.RightElbow] = brazo_r.get("25", 0.0)
            # posiciones_brazo[G1JointIndex.RightWristRoll] = brazo_r.get("26", 0.0)
            # posiciones_brazo[G1JointIndex.RightWristPitch] = brazo_r.get("27", 0.0)
            # posiciones_brazo[G1JointIndex.RightWristYaw] = brazo_r.get("28", 0.0)

            print(f"Brazo: {q_brazo}")
            print(f"Cintura: {cintura}")

            if mano_izq is not None: print(f"Mano izq: {mano_izq}")
            if mano_der is not None: print(f"Mano der: {mano_der}")

            # if res.lower()=='x':
            #     break

            # res = input("Presiona Enter para continuar, X para salir: ")
            # if res.lower() == 'x':
            #     print("Cancelado por el usuario. Moviendo a postura segura de release...")
            #     break

            seq.move_to(posiciones_brazo, duration=paso["tiempo"], q_init_override=q_anterior)

            if isinstance(mano_izq, dict) and len(mano_izq) > 0:
                hand_seq.send_left({int(k): float(v) for k, v in mano_izq.items()})
            if isinstance(mano_der, dict) and len(mano_der) > 0:
                hand_seq.send_right({int(k): float(v) for k, v in mano_der.items()})
            q_anterior = posiciones_brazo

            # Cancelación limpia: mover a postura segura (release) y terminar

    time.sleep(0.5)
    if release == True:
        seq.freeze_and_release_a()

    hand_seq.freeze_and_release()


if __name__ == "__main__":
    main()   
