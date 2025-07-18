import sys
import time
import math
import json
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

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

class G1JointIndex:
    #Cadera
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

""" 
    LeftHandThumb0=30
    LeftHandThumb1=31
    LeftHandThumb2=32
    
    LeftHandMiddle0=33
    LeftHandMiddle1=34
    
    LeftHandIndex0=35
    LeftHandIndex1=36

    RightHandThumb0=37
    RightHandThumb1=38
    RightHandThumb2=39
    
    RightHandMiddle0=40
    RightHandMiddle1=41
    
    RightHandIndex0=42
    RightHandIndex1=43 """
""

BRAZO_IZQ = [15, 16, 17, 18, 19, 20, 21]
BRAZO_DER = [22, 23, 24, 25, 26, 27, 28]


CINTURA = [12, 13, 14]
BRAZOS_Y_CINTURA = BRAZO_IZQ + BRAZO_DER + CINTURA

MIRROR_MAP = {
    15: (22, 1), 16: (23, -1), 17: (24, -1), 18: (25, 1),
    19: (26, -1), 20: (27, 1), 21: (28, -1)
}

id_a_nombre = {v: k for k, v in G1JointIndex.__dict__.items() if not k.startswith('__') and not callable(v)}

#Control de manos y dedos del robot Unitree Dex3-1
class Dex3_1_Reader: 
    def __init__(self):
        self.left_hand_state = None
        self.right_hand_state = None
        self.first_update = False
        self.left_ready = False
        self.right_ready = False
    def init(self): 
        #Crear los suscriptores para los estados de las posiciones de las manos actuales 
       
        # LeftHandState_subscriber
        self.LeftHandState_subscriber = ChannelSubscriber("rt/dex3/left/state", HandState_)
        self.LeftHandState_subscriber.Init(self.left_hand_state_callback, 10)
        
        # RightHandState_subscriber
        self.RightHandState_subscriber = ChannelSubscriber("rt/dex3/right/state", HandState_)
        self.RightHandState_subscriber.Init(self.right_hand_state_callback, 10)

    #Funciones de callback para los estados de las manos    
    def left_hand_state_callback(self, msg: HandState_):
        self.left_hand_state = msg
        self.left_ready = True
        if self.right_ready:
            self.first_update = True

    def right_hand_state_callback(self, msg: HandState_):
        self.right_hand_state = msg
        self.right_ready = True
        if self.left_ready:
            self.first_update = True

    #Loop de actualización de los estados de las manos
    def update_loop(self):
        import time
        while not (self.left_ready and self.right_ready):
            time.sleep(0.01)
            print("[Dex3_1_Reader] Lectura de estado de las manos incializada...")
        while True:
            time.sleep(0.05) #20 Hz

    #Obtener posiciones de las manos
    # Esta función devuelve un diccionario con las posiciones de las manos
    def get_hand_positions(self):
        if not (self.left_ready and self.right_ready):
            return None
        
        left_q = [m.q for m in self.left_hand_state.motor_state]
        right_q = [m.q for m in self.right_hand_state.motor_state]

        return {
            "left": left_q,
            "right": right_q
        }
    
#Controlador de brazos y cintura del robot Unitree G1
class ArmStateReader:
    def __init__(self):
        self.low_state = None
        self.first_update = False

    def init(self):
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.lowstate_callback, 10)

    def lowstate_callback(self, msg: LowState_):
        self.low_state = msg
        self.first_update = True

    def get_joint_positions(self, joint_list):
        if self.low_state is None:
            return {}
        return {j: self.low_state.motor_state[j].q for j in joint_list}

def vista_previa_parcial(junta, pos, paso_idx):
    print(f"\nPaso {paso_idx + 1} ({junta}) capturado.")
    for motor_id in sorted(pos):
        nombre_mostrar = id_a_nombre.get(motor_id, f"Joint {motor_id}")
        valor_rad = pos[motor_id]
        valor_deg = math.degrees(valor_rad)
        print(f"  {nombre_mostrar:18}: {valor_rad:7.4f} rad ({valor_deg:6.2f} deg)")
    print()

def solicitar_duracion():
    while True:
        try:
            dur = float(input("Ingrese la duración deseada para este paso (en segundos): "))
            if dur <= 0:
                print("La duración debe ser mayor a cero.")
                continue
            return dur
        except ValueError:
            print("Entrada inválida. Ingrese un número válido.")

def guardar_archivo(pasos):
    nombre_rutina = input("Ingrese el nombre de la rutina: ").strip()
    if not nombre_rutina:
        nombre_rutina = "rutina_sin_nombre"

    data = {
        "nombre_rutina": nombre_rutina,
        "fecha_creacion": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "numero_pasos": len(pasos),
        "pasos": pasos
    }

    filename = nombre_rutina + ".txt"
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"\nArchivo guardado exitosamente como '{filename}'.")

def eliminar_ultimo_paso(pasos):
    if not pasos:
        print("No hay pasos para eliminar.")
        return

    try:
        for i, paso in enumerate(pasos):
            print(f"{i+1}: {paso['nombre']} (Duración: {paso['duracion']} s)")

        idx = input("Ingrese el número del paso a eliminar (Enter para eliminar el último): ").strip()

        if idx == '':
            eliminado = pasos.pop()
            print(f"Último paso eliminado: {eliminado['nombre']}")
        else:
            idx = int(idx) - 1
            if not (0 <= idx < len(pasos)):
                print("Índice fuera de rango.")
                return
            eliminado = pasos.pop(idx)
            print(f"Paso {idx + 1} eliminado: {eliminado['nombre']}")
    except ValueError:
        print("Entrada inválida. Debe ingresar un número válido.")

def grabar_modo_1(reader, pasos, contador):
    print(f"\nCapturando paso {contador} en modo 1 (brazos → cintura)...")
    
    pos_brazos = reader.get_joint_positions(BRAZO_IZQ + BRAZO_DER)
    vista_previa_parcial("brazos", pos_brazos, contador - 1)

    incluir_cintura = input("¿Capturar cintura también? [s/n]: ").strip().lower()
    if incluir_cintura == 's':
        pos_cintura = reader.get_joint_positions(CINTURA)
        vista_previa_parcial("cintura", pos_cintura, contador - 1)
    else:
        pos_cintura = {j: 0.0 for j in CINTURA}

    dur = solicitar_duracion()

    paso = {
        "nombre": f"Paso {contador}",
        "posiciones": {**pos_brazos, **pos_cintura},
        "duracion": dur
    }
    pasos.append(paso)
    print(f"Duración asignada: {dur} segundos")
    return contador + 1


def grabar_modo_2(reader, pasos, contador):
    pos_izq = reader.get_joint_positions(BRAZO_IZQ)
    paso = {
        "nombre": f"Paso {contador}",
        "posiciones": pos_izq,
        "duracion": 0
    }
    pasos.append(paso)
    vista_previa_parcial("brazo izquierdo", pos_izq, contador - 1)

    input(f"Captura brazo derecho para paso {contador}. Enter para continuar...")
    pos_der = reader.get_joint_positions(BRAZO_DER)
    pasos[-1]["posiciones"].update(pos_der)
    vista_previa_parcial("brazo derecho", pos_der, contador - 1)

    grabar_cintura = input("¿Capturar cintura para este paso? [s/n]: ").strip().lower()
    if grabar_cintura == 's':
        pos_cintura = reader.get_joint_positions(CINTURA)
        pasos[-1]["posiciones"].update(pos_cintura)
        vista_previa_parcial("cintura", pos_cintura, contador - 1)
    else:
        for j in CINTURA:
            pasos[-1]["posiciones"][j] = 0.0

    pasos[-1]["duracion"] = solicitar_duracion()
    print(f"Duración asignada: {pasos[-1]['duracion']} segundos")
    return contador + 1

def grabar_modo_3(reader, pasos, contador):
    pos_izq = reader.get_joint_positions(BRAZO_IZQ)
    paso = {k: v for k, v in pos_izq.items()}
    for izq_id, (der_id, signo) in MIRROR_MAP.items():
        if izq_id in paso:
            paso[der_id] = paso[izq_id] * signo
    paso_obj = {
        "nombre": f"Paso {contador}",
        "posiciones": paso,
        "duracion": 0
    }
    pasos.append(paso_obj)
    vista_previa_parcial("brazo izquierdo + espejo derecho", paso, contador - 1)

    grabar_cintura = input("¿Capturar cintura para este paso? [s/n]: ").strip().lower()
    if grabar_cintura == 's':
        pos_cintura = reader.get_joint_positions(CINTURA)
        pasos[-1]["posiciones"].update(pos_cintura)
        vista_previa_parcial("cintura", pos_cintura, contador - 1)
    else:
        for j in CINTURA:
            pasos[-1]["posiciones"][j] = 0.0

    pasos[-1]["duracion"] = solicitar_duracion()
    print(f"Duración asignada: {pasos[-1]['duracion']} segundos")
    return contador + 1

#Manos
def grabar_modo_4(reader, pasos, contador, manos):
    print(f"\nCapturando paso {contador} en modo 4 (manos → brazos → cintura)...")

    estado_manos = manos.get_hand_positions()
    if estado_manos is None:
        print("Error: No se pudieron obtener las posiciones de las manos.")
        return contador
    
    # pos_izq = {f"mano_izq_{i}": val for i, val in enumerate(estado_manos["left"])}
    
    # paso = {
    #     "nombre": f"Paso {contador}",
    #     "posiciones": pos_izq,
    #     "duracion": 0
    # }
    
    # pasos.append(paso)

    # vista_previa_parcial("Mano izquierda", pos_izq, contador - 1)
    
    input(f"Captura brazo derecho para paso {contador}. Enter para continuar...")
    
    pos_der = {f"mano_der_{i}": val for i, val in enumerate(estado_manos["right"])}
    paso = {
        "nombre": f"Paso {contador}",
        "posiciones": pos_der,
        "duracion": 0
    }
    
    #pasos.append(paso)

    pasos[-1]["posiciones"].update(pos_der)
    vista_previa_parcial("Mano derecha", pos_der, contador - 1)

    grabar_cintura = input("¿Capturar cintura para este paso? [s/n]: ").strip().lower()
    if grabar_cintura == 's':
        pos_cintura = reader.get_joint_positions(CINTURA)
        pasos[-1]["posiciones"].update(pos_cintura)
        vista_previa_parcial("cintura", pos_cintura, contador - 1)
    else:
        for j in CINTURA:
            pasos[-1]["posiciones"][j] = 0.0

    pasos[-1]["duracion"] = solicitar_duracion()
    print(f"Duración asignada: {pasos[-1]['duracion']} segundos")
    return contador + 1

#NUEVO
# def grabar_modo_4(reader, pasos, contador,manos):
#     pos_izq = reader.get_joint_positions(MANO_IZQ)
#     paso = {
#         "nombre": f"Paso {contador}",   
#         "posiciones": pos_izq,
#         "duracion": 0
#     }
#     pasos.append(paso)
#     vista_previa_parcial("MANO izquierda", pos_izq, contador - 1)

#     input(f"Captura brazo derecho para paso {contador}. Enter para continuar...")
#     pos_der = reader.get_joint_positions(MANO_DER)
#     pasos[-1]["posiciones"].update(pos_der)
#     vista_previa_parcial("Mano derecho", pos_der, contador - 1)

#     grabar_cintura = input("¿Capturar cintura para este paso? [s/n]: ").strip().lower()
#     if grabar_cintura == 's':
#         pos_cintura = reader.get_joint_positions(CINTURA)
#         pasos[-1]["posiciones"].update(pos_cintura)
#         vista_previa_parcial("cintura", pos_cintura, contador - 1)
#     else:
#         for j in CINTURA:
#             pasos[-1]["posiciones"][j] = 0.0

#     pasos[-1]["duracion"] = solicitar_duracion()
#     print(f"Duración asignada: {pasos[-1]['duracion']} segundos")
#     return contador + 1

# def grabar_modo_4(reader, pasos, contador):
    # print("\n--- Modo 4: Captura de mano y brazo ---")
    
    # Primero capturamos la mano
    # print("\nCapturando posición de la MANO izquierda...")
    # pos_izq = reader.get_joint_positions(MANO_IZQ)
    # vista_previa_parcial("mano izquierda", pos_izq, contador - 1)
    
    # Preguntar si se desea capturar el brazo también
    # capturar_brazo = input("¿Desea capturar también la posición del BRAZO izquierdo? [s/n]: ").strip().lower()
    # pos_izq = {}

    # if capturar_brazo == 's':
    #     pos_izq = reader.get_joint_positions(BRAZO_IZQ)
    #     vista_previa_parcial("brazo izquierdo", pos_izq, contador - 1)

    # else:
    #     pos_izq = {j: pasos[-1]['posiciones'].get(j, 0.0) for j in BRAZO_IZQ}
    #     print("\nUsando posiciones anteriores de brazo izquierda:")
    #     vista_previa_parcial("brazo izquierda (anteriores)", pos_izq, contador - 1)
    
    # # Mano derecha
    # input("\n Capturar MANO derecha. Enter para continuar...")
    # pos_der = reader.get_joint_positions(MANO_DER)
    # vista_previa_parcial("mano derecha", pos_der, contador - 1)
    
    #  # Preguntar si se desea capturar el brazo también
    # capturar_brazo = input("¿Desea capturar también la posición del BRAZO derecho [s/n]: ").strip().lower()
    # pos_der = {}
    # if capturar_brazo == 's':
    #     input("Preparado para capturar BRAZO derecho. Enter para continuar...")
    #     pos_der = reader.get_joint_positions(BRAZO_DER)
    #     vista_previa_parcial("brazo derecho", pos_der, contador - 1)
    # else:
    #     pos_der = {j: pasos[-1]['posiciones'].get(j, 0.0) for j in BRAZO_DER}
    #     print("\nUsando posiciones anteriores de brazo derecho:")
    #     vista_previa_parcial("brazo derecha (anteriores)", pos_der, contador - 1)

    # # Combinar todas las posiciones
    # #posiciones_completas = {**pos_izq, **pos_izq, **pos_der, **pos_der}
    # posiciones_completas = {**pos_der, **pos_der}
    
    # # Preguntar por la cintura
    # grabar_cintura = input("¿Capturar cintura para este paso? [s/n]: ").strip().lower()
    # if grabar_cintura == 's':
    #     pos_cintura = reader.get_joint_positions(CINTURA)
    #     posiciones_completas.update(pos_cintura)
    #     vista_previa_parcial("cintura", pos_cintura, contador - 1)
    
    # # Asignar duración
    # dur = solicitar_duracion()
    
    # pasos[-1]["duracion"] = solicitar_duracion()
    # print(f"Duración asignada: {pasos[-1]['duracion']} segundos")
    # #return contador + 1

    # # Crear el paso
    # paso = {
    # "nombre": f"Paso {contador} (Mano y brazo)",
    # "posiciones": posiciones_completas,
    # "duracion": dur
    # }

    # pasos.append(paso)
    # print(f"\nPaso {contador} guardado con éxito. Duración: {dur} segundos")
    
    # return contador + 1

def repetir_pasos(pasos, contador):
    try:
        n = int(input("¿Cuántos pasos anteriores quiere repetir?: "))
        veces = int(input("¿Cuántas veces desea repetirlos?: "))
        if n <= 0 or veces <= 0:
            print("Valores inválidos. Deben ser mayores a cero.")
            return contador
        if n > len(pasos):
            print("No hay suficientes pasos para repetir.")
            return contador

        mantener_duracion = input("¿Desea mantener la duración original de los pasos? [s/n]: ").strip().lower()

        if mantener_duracion == 'n':
            nueva_duracion = solicitar_duracion()
            print(f"Duración personalizada para pasos repetidos: {nueva_duracion} segundos")
        elif mantener_duracion != 's':
            print("Opción no válida. Se mantendrán las duraciones originales.")
            mantener_duracion = 's'

        seleccionados = pasos[-n:]
        for i in range(veces):
            for paso in seleccionados:
                nueva_pos = {k: v for k, v in paso["posiciones"].items()}
                duracion = paso["duracion"] if mantener_duracion == 's' else nueva_duracion
                nuevo_paso = {
                    "nombre": f"Paso {contador}",
                    "posiciones": nueva_pos,
                    "duracion": duracion
                }
                pasos.append(nuevo_paso)
                vista_previa_parcial("repetido", nueva_pos, contador - 1)
                print(f"Paso {contador} con duración: {duracion} segundos")
                contador += 1
    except ValueError:
        print("Entrada inválida. Asegúrese de ingresar números.")
    return contador

def modificar_paso(pasos, reader):
    if not pasos:
        print("No hay pasos para modificar.")
        return
    try:
        for i, paso in enumerate(pasos):
            print(f"{i+1}: {paso['nombre']} (Duración: {paso['duracion']} s)")
        idx = int(input("Ingrese el número del paso que desea modificar: ")) - 1
        if not (0 <= idx < len(pasos)):
            print("Índice fuera de rango.")
            return
        print("\nMODOS DE MODIFICACIÓN:")
        print("  1: Recapturar todo el paso (modo 1, 2 o 3).")
        print("  2: Capturar o actualizar solo la cintura.")
        print("  3: Modificar solo la duración.")
        opcion = input("Seleccione una opción: ").strip()
        if opcion == '1':
            modo = input("Modo (1, 2 o 3): ").strip()
            if modo == '1':
                pos = reader.get_joint_positions(BRAZOS_Y_CINTURA)
            elif modo == '2':
                pos = reader.get_joint_positions(BRAZO_IZQ)
                input("Captura brazo derecho. Presione Enter...")
                pos.update(reader.get_joint_positions(BRAZO_DER))
                incluir_cintura = input("¿Capturar cintura también? [s/n]: ").strip().lower()
                if incluir_cintura == 's':
                    pos.update(reader.get_joint_positions(CINTURA))
                else:
                    for j in CINTURA:
                        pos[j] = 0.0
            elif modo == '3':
                pos_izq = reader.get_joint_positions(BRAZO_IZQ)
                pos = {k: v for k, v in pos_izq.items()}
                for izq_id, (der_id, signo) in MIRROR_MAP.items():
                    if izq_id in pos:
                        pos[der_id] = pos[izq_id] * signo
                incluir_cintura = input("¿Capturar cintura también? [s/n]: ").strip().lower()
                if incluir_cintura == 's':
                    pos.update(reader.get_joint_positions(CINTURA))
                else:
                    for j in CINTURA:
                        pos[j] = 0.0
            else:
                print("Modo inválido.")
                return
            pasos[idx]["posiciones"] = pos
            vista_previa_parcial("modificado", pos, idx)
        elif opcion == '2':
            pos_cintura = reader.get_joint_positions(CINTURA)
            pasos[idx]["posiciones"].update(pos_cintura)
            vista_previa_parcial("solo cintura", pos_cintura, idx)
        elif opcion == '3':
            nueva_dur = solicitar_duracion()
            pasos[idx]["duracion"] = nueva_dur
            print(f"Duración actualizada a {nueva_dur} segundos.")
        else:
            print("Opción no válida.")
    except ValueError:
        print("Entrada inválida.")

def duplicar_paso(pasos, contador):
    if not pasos:
        print("No hay pasos para duplicar.")
        return contador
    try:
        for i, paso in enumerate(pasos):
            print(f"{i+1}: {paso['nombre']} (Duración: {paso['duracion']} s)")
        idx = int(input("Ingrese el número del paso que desea duplicar: ")) - 1
        if not (0 <= idx < len(pasos)):
            print("Índice fuera de rango.")
            return contador

        mantener_duracion = input("¿Desea mantener la duración original del paso? [s/n]: ").strip().lower()
        if mantener_duracion == 'n':
            nueva_duracion = solicitar_duracion()
        else:
            nueva_duracion = pasos[idx]["duracion"]

        paso_original = pasos[idx]
        nueva_pos = {k: v for k, v in paso_original["posiciones"].items()}

        nuevo_paso = {
            "nombre": f"Paso {contador}",
            "posiciones": nueva_pos,
            "duracion": nueva_duracion
        }

        pasos.append(nuevo_paso)
        vista_previa_parcial("duplicado", nueva_pos, contador - 1)
        print(f"Paso {contador} duplicado exitosamente con duración: {nueva_duracion} segundos")
        return contador + 1
    except ValueError:
        print("Entrada inválida.")
        return contador


def main():
    
      
    if len(sys.argv) < 2:
        print(f"Uso: python3 {sys.argv[0]} <interfaz_red>")
        sys.exit(1)

    ChannelFactoryInitialize(0, sys.argv[1])
    reader = ArmStateReader()
    reader.init()

    manos = Dex3_1_Reader()
    manos.init()


    print("Esperando conexión con el robot...")
    while not reader.first_update:
        time.sleep(0.1)
    print("Conexión establecida.")

    pasos = []
    contador = 1

    while True:
        print("\nMODOS DE CAPTURA DISPONIBLES:")
        print("  1: Grabar todos los motores.")
        print("  2: Capturar brazo izquierdo → derecho → cintura opcional.")
        print("  3: Capturar brazo izquierdo y generar espejo derecho.")
        print("  4: Capturar mano derecha → derecho → cintura opcional..")
        print("  r: Repetir últimos pasos.")
        print("  d: Duplicar un paso específico.")
        print("  m: Modificar un paso existente.")
        print("  e: Eliminar un paso.")
        print("  f: Finalizar y guardar.")

        modo = input("Seleccione un modo de grabación (1, 2, 3, r, e, m, f): ").strip().lower()

        if modo == '1':
            contador = grabar_modo_1(reader, pasos, contador)
        elif modo == '2':
            contador = grabar_modo_2(reader, pasos, contador)
        elif modo == '3':
            contador = grabar_modo_3(reader, pasos, contador)
        elif modo == '4':
            contador = grabar_modo_4(reader, pasos, contador, manos)
        elif modo == 'r':
            contador = repetir_pasos(pasos, contador)
        elif modo == 'd':
            contador = duplicar_paso(pasos, contador)
        elif modo == 'e':
            eliminar_ultimo_paso(pasos)
            contador = max(1, contador - 1)
        elif modo == 'm':
            modificar_paso(pasos, reader)
        elif modo == 'f':
            break
        else:
            print("Modo no reconocido.")

    if pasos:
        guardar_archivo(pasos)
    else:
        print("No se capturaron pasos.")

if __name__ == "__main__":
    main()