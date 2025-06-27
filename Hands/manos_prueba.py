#RUTA DE LA RUTINA
ruta="BrazoMano.txt"

import sys
import time
import math
import json

import numpy as np
from enum import IntEnum
import time
#import os
import sys
import threading
from multiprocessing import Process, shared_memory, Array, Lock

# Importando las clases necesarias para el manejo de mensajes y canales
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_                               # idl
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

# for gripper
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_ 
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
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

class HandSequence:
    def __init__(self):
        self.publisher = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        self.publisher.Init()
        self.publisher_right = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        self.publisher_right.Init()


        #Inicializar mensajes de comando
        self.msg_left = unitree_hg_msg_dds__HandCmd_()
        self.msg_right = unitree_hg_msg_dds__HandCmd_()

        self.num_motors = 7  # Número de motores en cada mano 
        self.kp= 1.5
        self.kd = 0.2

        self._init_msg(self.msg_left)
        self._init_msg(self.msg_right)

    def _init_msg(self, msg):
        for i in range(self.num_motors):
            mode = (i & 0x0F) | ((0x01 & 0x07) << 4) #id + status
            msg.motor_cmd[i].mode = mode
            #msg.motor_cmd[i].q = 0.0
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].tau = 0.0
            msg.motor_cmd[i].kp = self.kp
            msg.motor_cmd[i].kd = self.kd

    def send(self, posiciones: dict, mano: str ):
        
        """Posicionnes: dict con claves int (índices de motores) y valores float (posiciones deseadas)(indices 0-6) y valores en radianes.
            Mano: str, "left" o "right" para especificar la mano a controlar.
        """
        if mano == "left":
            msg = self.msg_left
            publisher= self.publisher

        elif mano == "right":
            msg = self.msg_right
            publisher = self.publisher_right

        else:
            raise ValueError("Mano debe ser 'left' o 'right'")

        for i in range(self.num_motors):
            if i in posiciones:
                msg.motor_cmd[i].q = posiciones[i]
            else:
                msg.motor_cmd[i].q = 0.0  # Valor por defecto si no se especifica
        
        publisher.Write(msg)

        # Calcular CRC y enviar el mensaje
        # msg.crc = CRC().Crc(msg)
        if mano == "left":
            self.publisher.Write(msg)
        else:
            self.publisher_right.Write(msg)

    def freeze_and_release(self):
        for i in range(self.num_motors):
            self.msg_right.motor_cmd[i].q = 0
            self.msg_right.motor_cmd[i].dq = 0.0
            self.msg_right.motor_cmd[i].tau = 0.0
            self.msg_right.motor_cmd[i].kp = 0.0
            self.msg_right.motor_cmd[i].kd = 0.0

        for i in range(self.num_motors):
            self.msg_left.motor_cmd[i].q = 0
            self.msg_left.motor_cmd[i].dq = 0.0
            self.msg_left.motor_cmd[i].tau = 0.0
            self.msg_left.motor_cmd[i].kp = 0.0
            self.msg_left.motor_cmd[i].kd = 0.0

        self.publisher.Write(self.msg_right) 
        self.publisher.Write(self.msg_left)    

        
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
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw, G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll, G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
            G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch
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
        ratio = (1 - math.cos(math.pi * (self.t / self.T))) / 2 if self.t < self.T else 1.0
        return q_init + (q_target - q_init) * ratio

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

    def move_to(self, updates: dict, duration=1.25, q_init_override=None):
        self.target_pos.update(updates)
        self.T = duration
        self.t = 0.0
        self.q_init_override = q_init_override
        while self.t < self.T:
            time.sleep(self.control_dt)

    def freeze_and_release_a(self):
       for joint in self.arm_joints:
           self.low_cmd.motor_cmd[joint].q = self.low_state.motor_state[joint].q
           self.low_cmd.motor_cmd[joint].dq = 0.0
           self.low_cmd.motor_cmd[joint].tau = 0.0
           self.low_cmd.motor_cmd[joint].kp = 0.0
           self.low_cmd.motor_cmd[joint].kd = 0.0
       self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0
       self.low_cmd.crc = self.crc.Crc(self.low_cmd)
       self.publisher.Write(self.low_cmd)


def main():
    if len(sys.argv) < 2: #Si no hay argumentos, salir
        sys.exit()
    ruta_archivo_txt = ruta # RUTA DE LA RUTINA, definido en la parte superior del script

    #Apertura del archivo JSON
    try:
        with open(ruta_archivo_txt, 'r') as f:
            data = json.load(f)
    except:
        sys.exit()

    #Obtener los pasos del archivo JSON
    #Se espera que el JSON tenga una estructura como {"pasos": [{"posiciones":
    pasos = data.get("pasos", [])

    ChannelFactoryInitialize(0, sys.argv[1]) #Inicializar el canal de comunicación DDS
    
    seq = ArmSequence() #Control de brazo y cintura
    seq.Init()
    seq.Start()

    hand_seq = HandSequence() #Control de manos

    hand_seq.freeze_and_release() #Congelar el brazo antes de iniciar
    seq.freeze_and_release_a() #Congelar el brazo antes de iniciar

    q_anterior = None #Posición anterior del brazo, para interpolación

    for paso in pasos:
        #Obtener las posiciones y duración del paso actual
        posiciones = paso.get("posiciones", {})
        duracion = paso.get("duracion", 1.25) #Si no se especifica duración, usar 1.25 segundos

        #Dividir las posiciones: brazo/cintura (int), manos (str)
        posiciones_brazo = {int(k): v for k, v in posiciones.items() if isinstance(k, int) or k.isdigit()}
        posiciones_mano_izq = {int(k.split('_')[-1]): v for k, v in posiciones.items() if isinstance(k, str) and k.startswith("mano_izq") }
        posiciones_mano_der = {int(k.split('_')[-1]): v for k, v in posiciones.items() if isinstance(k, str) and k.startswith("mano_der")}

        #Mover brazo y cintura
        if posiciones_brazo:
            #Mover brazo y cintura
            seq.move_to(posiciones_brazo, duration=duracion, q_init_override=q_anterior)
            q_anterior = posiciones_brazo

        #Mover manos
        if posiciones_mano_izq:
            #Mover mano izquierda
            posiciones_mano = {i: posiciones_mano_izq.get(f"mano_izquierda_{i}", 0.0) for i in range(7)}
            hand_seq.send(posiciones_mano, mano="left")

        if posiciones_mano_der:
            #Mover mano derecha
            posiciones_mano = {i: posiciones_mano_der.get(f"mano_derecha_{i}", 0.0) for i in range(7)}
            hand_seq.send(posiciones_mano, mano="right")

        time.sleep(duracion)  # Esperar la duración del paso

    seq.freeze_and_release_a()  # Congelar el brazo al final
    hand_seq.freeze_and_release()  # Congelar el brazo al final

if __name__ == "__main__":
    main()
