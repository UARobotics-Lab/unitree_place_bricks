import sys
import time
import math
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


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
        self.target_pos.update(updates)
        self.T = duration
        self.t = 0.0
        self.q_init_override = q_init_override
        while self.t < self.T:
            time.sleep(self.control_dt)

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
    seq.Init()
    seq.Start()

    hand_seq = HandSequence()

    # === Carga pasos calculados ===
    
    q_steps = np.load("q_steps_LM.npy")  # Cargar pasos de LM desde un archivo .npy

    print(f"Ejecutando {len(q_steps)} pasos con control manual...")

    T_total = 3.0  # Segundos total
    T_step = T_total / len(q_steps)

    arm_joints = [
        G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
        G1JointIndex.LeftShoulderYaw, G1JointIndex.LeftElbow,
        G1JointIndex.LeftWristRoll, G1JointIndex.LeftWristPitch,
        G1JointIndex.LeftWristYaw,
    ]

    q_anterior = None

    for i, q in enumerate(q_steps):
        posiciones_brazo = {joint_idx: q[j] for j, joint_idx in enumerate(arm_joints)}

        print(f"\n Siguiente paso {i+1}/{len(q_steps)}:")
        print(f"Posiciones (radianes): {posiciones_brazo}")


        res=input("Presiona Enter para continuar, X para salir: ")
        if res.lower() == 'x':
            print("Proceso cancelado.")
            break

        seq.move_to(posiciones_brazo, duration=T_step, q_init_override=q_anterior)
        q_anterior = posiciones_brazo

    seq.freeze_and_release_a()
    hand_seq.freeze_and_release()


if __name__ == "__main__":
    main()
