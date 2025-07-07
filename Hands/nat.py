def grabar_modo_4(reader, pasos, contador):
    print("\n--- Modo 4: Captura de mano y brazo ---")
    
    # Primero capturamos la mano
    print("\nCapturando posición de la MANO izquierda...")
    pos_izq = reader.get_joint_positions(MANO_IZQ)
    vista_previa_parcial("mano izquierda", pos_izq, contador - 1)
    
    # Preguntar si se desea capturar el brazo también
    capturar_brazo = input("¿Desea capturar también la posición del BRAZO izquierdo? [s/n]: ").strip().lower()
    pos_izq = {}

    if capturar_brazo == 's':
        pos_izq = reader.get_joint_positions(BRAZO_IZQ)
        vista_previa_parcial("brazo izquierdo", pos_izq, contador - 1)

    else:
            pos_izq = {j: pasos[-1]['posiciones'].get(j, 0.0) for j in BRAZO_IZQ}
            print("\nUsando posiciones anteriores de brazo izquierda:")
            vista_previa_parcial("brazo izquierda (anteriores)", pos_izq, contador - 1)
    
    # Mano derecha
    input("\nCapturar MANO derecha. Enter para continuar...")
    pos_der = reader.get_joint_positions(MANO_DER)
    vista_previa_parcial("mano derecha", pos_der, contador - 1)
    
     # Preguntar si se desea capturar el brazo también
    capturar_brazo = input("¿Desea capturar también la posición del BRAZO derecho [s/n]: ").strip().lower()
    pos_der = {}
    if capturar_brazo == 's':
        input("Preparado para capturar BRAZO derecho. Enter para continuar...")
        pos_der = reader.get_joint_positions(BRAZO_DER)
        vista_previa_parcial("brazo derecho", pos_der, contador - 1)
    else:
            pos_der = {j: pasos[-1]['posiciones'].get(j, 0.0) for j in BRAZO_DER}
            print("\nUsando posiciones anteriores de brazo derecho:")
            vista_previa_parcial("brazo derecha (anteriores)", pos_der, contador - 1)

    # Combinar todas las posiciones
    posiciones_completas = {**pos_izq, **pos_izq, **pos_der, **pos_der}
    
    # Preguntar por la cintura
    grabar_cintura = input("¿Capturar cintura para este paso? [s/n]: ").strip().lower()
    if grabar_cintura == 's':
        pos_cintura = reader.get_joint_positions(CINTURA)
        posiciones_completas.update(pos_cintura)
        vista_previa_parcial("cintura", pos_cintura, contador - 1)
    
    # Asignar duración
    dur = solicitar_duracion()
    
    pasos[-1]["duracion"] = solicitar_duracion()
    print(f"Duración asignada: {pasos[-1]['duracion']} segundos")
    return contador + 1

    # Crear el paso
    paso = {
        "nombre": f"Paso {contador} (Mano y brazo)",
        "posiciones": posiciones_completas,
        "duracion": dur
    }
    pasos.append(paso)
    print(f"\nPaso {contador} guardado con éxito. Duración: {dur} segundos")
    
    return contador + 1


#GUIA 2

def grabar_modo_4(reader, pasos, contador):
    print("\n--- Modo 4: Captura de mano y brazo ---")
    
    # Primero capturamos la mano
    print("\nCapturando posición de la MANO izquierda...")
    pos_mano_izq = reader.get_joint_positions(MANO_IZQ)
    vista_previa_parcial("mano izquierda", pos_mano_izq, contador - 1)
    
    # Preguntar si se desea capturar el brazo también
    capturar_brazo = input("¿Desea capturar también la posición del BRAZO izquierdo? [s/n]: ").strip().lower()
    pos_brazo_izq = {}
    
    if capturar_brazo == 's':
        pos_brazo_izq = reader.get_joint_positions(BRAZO_IZQ)
        vista_previa_parcial("brazo izquierdo", pos_brazo_izq, contador - 1)
    else:
            pos_mano_izq = {j: pasos[-1]['posiciones'].get(j, 0.0) for j in MANO_IZQ}
            print("\nUsando posiciones anteriores de mano izquierda:")
            vista_previa_parcial("mano izquierda (anteriores)", pos_mano_izq, contador - 1)
    
    # Mano derecha
    input("\nCapturar MANO derecha. Enter para continuar...")
    pos_mano_der = reader.get_joint_positions(MANO_DER)
    vista_previa_parcial("mano derecha", pos_mano_der, contador - 1)
    
    pos_brazo_der = {}
    if capturar_brazo == 's':
        input("Preparado para capturar BRAZO derecho. Enter para continuar...")
        pos_brazo_der = reader.get_joint_positions(BRAZO_DER)
        vista_previa_parcial("brazo derecho", pos_brazo_der, contador - 1)
    
    # Combinar todas las posiciones
    posiciones_completas = {**pos_mano_izq, **pos_brazo_izq, **pos_mano_der, **pos_brazo_der}
    
    # Preguntar por la cintura
    grabar_cintura = input("¿Capturar cintura para este paso? [s/n]: ").strip().lower()
    if grabar_cintura == 's':
        pos_cintura = reader.get_joint_positions(CINTURA)
        posiciones_completas.update(pos_cintura)
        vista_previa_parcial("cintura", pos_cintura, contador - 1)
    
    # Asignar duración
    dur = solicitar_duracion()
    
    # Crear el paso
    paso = {
        "nombre": f"Paso {contador} (Mano y brazo)",
        "posiciones": posiciones_completas,
        "duracion": dur
    }
    pasos.append(paso)
    print(f"\nPaso {contador} guardado con éxito. Duración: {dur} segundos")
    
    return contador + 1



#NUEV0

def grabar_modo_4(reader, pasos, contador):
    print("\n--- Modo 4: Captura de mano y brazo ---")
    posiciones_completas = {}
    
    # Verificar si hay pasos anteriores para ofrecer opción de reutilizar
    tiene_anteriores = len(pasos) > 0
    
    # Captura de BRAZO IZQUIERDO
    print("\n BRAZO IZQUIERDO:")
    capturar_brazo = input("¿Desea capturar/incluir brazo izquierdo? [s/n]: ").strip().lower()
    
    if capturar_brazo == 's':
        if tiene_anteriores:
            print("Opciones:")
            print("1. Usar posiciones anteriores de brazo izquierdo")
            print("2. Capturar nuevas posiciones")
            opcion = input("Seleccione opción (1/2): ").strip()
            
            if opcion == '1':
                pos_brazo_izq = {j: pasos[-1]['posiciones'].get(j, 0.0) for j in BRAZO_IZQ}
                print("\nUsando posiciones anteriores de brazo izquierdo:")
                vista_previa_parcial("brazo izquierdo (anteriores)", pos_brazo_izq, contador - 1)
            else:
                pos_brazo_izq = reader.get_joint_positions(BRAZO_IZQ)
                vista_previa_parcial("brazo izquierdo (nuevas)", pos_brazo_izq, contador - 1)
        else:
            pos_brazo_izq = reader.get_joint_positions(BRAZO_IZQ)
            vista_previa_parcial("brazo izquierdo", pos_brazo_izq, contador - 1)
        
        posiciones_completas.update(pos_brazo_izq)
    
    # Captura de BRAZO DERECHO
    if capturar_brazo == 's':
        print("\nBRAZO DERECHO:")
        input("Preparado para capturar BRAZO derecho. Presione Enter cuando esté listo...")
        
        if tiene_anteriores:
            print("Opciones:")
            print("1. Usar posiciones anteriores de brazo derecho")
            print("2. Capturar nuevas posiciones")
            opcion = input("Seleccione opción (1/2): ").strip()
            
            if opcion == '1':
                pos_brazo_der = {j: pasos[-1]['posiciones'].get(j, 0.0) for j in BRAZO_DER}
                print("\nUsando posiciones anteriores de brazo derecho:")
                vista_previa_parcial("brazo derecho (anteriores)", pos_brazo_der, contador - 1)
            else:
                pos_brazo_der = reader.get_joint_positions(BRAZO_DER)
                vista_previa_parcial("brazo derecho (nuevas)", pos_brazo_der, contador - 1)
        else:
            pos_brazo_der = reader.get_joint_positions(BRAZO_DER)
            vista_previa_parcial("brazo derecho", pos_brazo_der, contador - 1)
        
        posiciones_completas.update(pos_brazo_der)
    
    # Captura de CINTURA
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
