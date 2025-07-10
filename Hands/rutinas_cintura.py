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

    def send_left(self, posiciones: dict ):
        msg= self.msg_left
        
        """Posicionnes: dict con claves int (índices de motores) y valores float (posiciones deseadas)(indices 0-6) y valores en radianes.
            Mano: str, "left" o "right" para especificar la mano a controlar.
        """
        """ if mano == "left":
            msg = self.msg_left
            publisher= self.publisher

        elif mano == "right":
            msg = self.msg_right
            publisher = self.publisher_right

        else:
            raise ValueError("Mano debe ser 'left' o 'right'")
 """
        for i in range(self.num_motors):
            msg.motor_cmd[i].q=posiciones.get(i,0.0)
            mode = (i & 0x0F) | ((0x01 & 0x07) << 4) #id + status
            msg.motor_cmd[i].mode = mode
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].tau = 0.0
            msg.motor_cmd[i].kp = self.kp
            msg.motor_cmd[i].kd = self.kd 
        
        self.publisher.Write(self.msg_left)

    def send_right(self, posiciones: dict ):
        msg= self.msg_right
            

        for i in range(self.num_motors):
                    msg.motor_cmd[i].q=posiciones.get(i,0.0)
                    mode = (i & 0x0F) | ((0x01 & 0x07) << 4) #id + status
                    msg.motor_cmd[i].mode = mode
                    msg.motor_cmd[i].dq = 0.0
                    msg.motor_cmd[i].tau = 0.0
                    msg.motor_cmd[i].kp = self.kp
                    msg.motor_cmd[i].kd = self.kd 
                
        self.publisher_right.Write(self.msg_right)
""" 
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

        self.publisher_right.Write(self.msg_right) 
        self.publisher.Write(self.msg_left)    

 """        
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

"""     def freeze_and_release_a(self):
       for joint in self.arm_joints:
           self.low_cmd.motor_cmd[joint].q = self.low_state.motor_state[joint].q
           self.low_cmd.motor_cmd[joint].dq = 0.0
           self.low_cmd.motor_cmd[joint].tau = 0.0
           self.low_cmd.motor_cmd[joint].kp = 0.0
           self.low_cmd.motor_cmd[joint].kd = 0.0
       self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0
       self.low_cmd.crc = self.crc.Crc(self.low_cmd)
       self.publisher.Write(self.low_cmd)
 """

#Funciones de ejecución de la rutina
def centro_derecha():
    """
    Función para mover el brazo y la mano derecha al centro.
    Se inicia con la posicion inicial (L2+B), y de allí se mueve a la posición deseada
    (en este caso, a la derecha del robot).
    """
    print("Moviendo brazo y mano derecha al centro... (posición inicial L2+B)")
    pos_aura = {
        G1JointIndex.LeftShoulderPitch: 0.0,
        G1JointIndex.LeftShoulderRoll: 0.0,
        G1JointIndex.LeftShoulderYaw: 0.0,
        G1JointIndex.LeftElbow: 0.0,
        G1JointIndex.LeftWristRoll: 0.0,
        G1JointIndex.LeftWristPitch: 0.0,
        G1JointIndex.LeftWristYaw: 0.0,
        G1JointIndex.WaistYaw: 0.0,
        G1JointIndex.WaistRoll: 0.0,
        G1JointIndex.WaistPitch: 0.0,
        G1JointIndex.RightShoulderPitch: 0.0,
        G1JointIndex.RightShoulderRoll: 0.0,
        G1JointIndex.RightShoulderYaw: 0.0,
        G1JointIndex.RightElbow: 0.0,
        G1JointIndex.RightWristRoll: 0.0,
        G1JointIndex.RightWristPitch: 0.0,
        G1JointIndex.RightWristYaw: 0.0,
        G1JointIndex.RightHipPitch: 0.0,
        G1JointIndex.WaistYaw: -1.54, 
        G1JointIndex.WaistRoll: 0.0, 
        G1JointIndex.WaistPitch: 0.0,

    }

    print("Torso orientado a la derecha:", pos_aura)

    return pos_aura

def derecha_centro():
    """
    Función para mover el brazo y la mano derecha al centro.
    Se inicia con la posicion inicial (en este caso, la ultima que se obtuvo en la funcion de derecha-centro),
      y de allí se mueve a la posición deseada (en este caso, al centro del robot).
    """
    print("Moviendo brazo y mano derecha al centro... ")
    pos_aura = {
        G1JointIndex.LeftShoulderPitch: 0.0,
        G1JointIndex.LeftShoulderRoll: 0.0,
        G1JointIndex.LeftShoulderYaw: 0.0,
        G1JointIndex.LeftElbow: 0.0,
        G1JointIndex.LeftWristRoll: 0.0,
        G1JointIndex.LeftWristPitch: 0.0,
        G1JointIndex.LeftWristYaw: 0.0,
        G1JointIndex.WaistYaw: 0.0,
        G1JointIndex.WaistRoll: 0.0,
        G1JointIndex.WaistPitch: 0.0,
        G1JointIndex.RightShoulderPitch: 0.0,
        G1JointIndex.RightShoulderRoll: 0.0,
        G1JointIndex.RightShoulderYaw: 0.0,
        G1JointIndex.RightElbow: 0.0,
        G1JointIndex.RightWristRoll: 0.0,
        G1JointIndex.RightWristPitch: 0.0,
        G1JointIndex.RightWristYaw: 0.0,
        G1JointIndex.RightHipPitch: 0.0,
        G1JointIndex.WaistYaw: 0.0, 
        G1JointIndex.WaistRoll: 0.0, 
        G1JointIndex.WaistPitch: 0.0,

    }
    print("Posición central:", pos_aura)
       
    return pos_aura

def centro_izquierda():
    """
    Función para mover el brazo y la mano izquierda al centro.
    Se inicia con la posicion inicial (L2+B), y de allí se mueve a la posición deseada
    (en este caso, a la izquierda del robot).
    """
    print("Moviendo brazo y mano izquierda al centro... (posición inicial L2+B)")
    pos_aura = {
        G1JointIndex.LeftShoulderPitch: 0.0,
        G1JointIndex.LeftShoulderRoll: 0.0,
        G1JointIndex.LeftShoulderYaw: 0.0,
        G1JointIndex.LeftElbow: 0.0,
        G1JointIndex.LeftWristRoll: 0.0,
        G1JointIndex.LeftWristPitch: 0.0,
        G1JointIndex.LeftWristYaw: 0.0,
        G1JointIndex.WaistYaw: 0.0,
        G1JointIndex.WaistRoll: 0.0,
        G1JointIndex.WaistPitch: 0.0,
        G1JointIndex.RightShoulderPitch: 0.0,
        G1JointIndex.RightShoulderRoll: 0.0,
        G1JointIndex.RightShoulderYaw: 0.0,
        G1JointIndex.RightElbow: 0.0,
        G1JointIndex.RightWristRoll: 0.0,
        G1JointIndex.RightWristPitch: 0.0,
        G1JointIndex.RightWristYaw: 0.0,
        G1JointIndex.RightHipPitch: 0.0,
        G1JointIndex.WaistYaw: 1.54, 
        G1JointIndex.WaistRoll: 0.0, 
        G1JointIndex.WaistPitch: 0.0,
 
    
    }

    print("Torso orientado a la izquierda:", pos_aura)

    return pos_aura

def izquierda_centro():
    """ Función para mover el brazo y la mano izquierda al centro.
    Se inicia con la posicion inicial (en este caso, la ultima que se obtuvo en la funcion de izquierda-centro), 
    y de allí se mueve a la posición deseada (en este caso, al centro del robot).
    """
    print("Moviendo torso de Aura desde la izquierda al centro... ")
    pos_aura = {
        G1JointIndex.LeftShoulderPitch: 0.0,
        G1JointIndex.LeftShoulderRoll: 0.0,
        G1JointIndex.LeftShoulderYaw: 0.0,
        G1JointIndex.LeftElbow: 0.0,
        G1JointIndex.LeftWristRoll: 0.0,
        G1JointIndex.LeftWristPitch: 0.0,
        G1JointIndex.LeftWristYaw: 0.0,
        G1JointIndex.WaistYaw: 0.0,
        G1JointIndex.WaistRoll: 0.0,
        G1JointIndex.WaistPitch: 0.0,
        G1JointIndex.RightShoulderPitch: 0.0,
        G1JointIndex.RightShoulderRoll: 0.0,
        G1JointIndex.RightShoulderYaw: 0.0,
        G1JointIndex.RightElbow: 0.0,
        G1JointIndex.RightWristRoll: 0.0,
        G1JointIndex.RightWristPitch: 0.0,
        G1JointIndex.RightWristYaw: 0.0,
        G1JointIndex.RightHipPitch: 0.0,
        G1JointIndex.WaistYaw: 0.0, 
        G1JointIndex.WaistRoll: 0.0, 
        G1JointIndex.WaistPitch: 0.0,

    
    }
    print("Posición central:", pos_aura)
    
    return pos_aura


def main():

    if len(sys.argv) < 2: #Si no hay argumentos, salir
        sys.exit()
       

    ChannelFactoryInitialize(0, sys.argv[1]) #Inicializar el canal de comunicación DDS
    
    seq = ArmSequence() #Control de brazo y cintura
    seq.Init()
    seq.Start()

    #hand_seq = HandSequence() #Control de manos

    print("\nRutina de cintura por pasos.")
    print(" 1: Centro → Derecha")
    print(" 2: Derecha → Centro")
    print(" 3: Centro → Izquierda")
    print(" 4: Izquierda → Centro")
    print(" 5: Salir")

    modo = input("Ingrese el modo de ejecución (1-4): ")

    if modo == "1":
        pos_aura = centro_derecha()
    elif modo == "2":
        pos_aura = derecha_centro()
    elif modo == "3":
        pos_aura = centro_izquierda()
    elif modo == "4":
        pos_aura = izquierda_centro()
    elif modo == "5":
        sys.exit()
    else:
        print("Modo no válido. Saliendo...")

    seq.move_to(pos_aura, duration=2.0)  # Mover a la posición deseada
    

if __name__ == "__main__":
    main()
