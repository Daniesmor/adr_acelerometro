import serial
import struct
import time
import os  # Necesario para manejar rutas de archivos

# --- CONFIGURACIÓN ---
# Asegúrate de que este sea el puerto correcto para tu placa.
# En Linux puede ser /dev/ttyUSB0, /dev/ttyACM0, etc.
# En Windows será algo como 'COM3', 'COM4', etc.
# En macOS será algo como '/dev/cu.usbserial-XXXX'
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# --- CONFIGURACIÓN DE EXPORTACIÓN ---
OUTPUT_FILENAME = 'sensor_data.csv'  # Nombre del archivo de salida


# Si quieres que el archivo se guarde en una carpeta específica, puedes hacer esto:
# OUTPUT_FOLDER = 'sensor_logs'
# OUTPUT_FILENAME = os.path.join(OUTPUT_FOLDER, 'sensor_data.csv')

def main():
    """
    Función principal para conectar, leer y mostrar los datos del sensor,
    y exportarlos a un archivo CSV.
    """
    ser = None  # Inicializamos la variable del puerto serie
    output_file = None  # Inicializamos la variable del archivo de salida

    try:
        print(f"Intentando conectar al puerto {SERIAL_PORT} a {BAUD_RATE} baudios...")
        # Abrimos la conexión con el puerto serie
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

        # Pequeña pausa para que el Arduino se reinicie si es necesario
        time.sleep(2)
        ser.flushInput()  # Limpiamos cualquier dato basura en el buffer de entrada

        print("Conexión establecida. Escuchando datos...")
        print(f"Los datos se guardarán en '{OUTPUT_FILENAME}'")
        print("Presiona Ctrl+C para salir.")

        # --- Preparar el archivo de salida ---
        # Si usas una carpeta específica, asegúrate de que exista
        # if 'OUTPUT_FOLDER' in locals() and not os.path.exists(OUTPUT_FOLDER):
        #     os.makedirs(OUTPUT_FOLDER)

        output_file = open(OUTPUT_FILENAME, 'w')  # 'w' para escribir (sobrescribe si existe)
        output_file.write("timestamp_python,acc_x_ms2,acc_y_ms2,acc_z_ms2\n")  # Encabezado CSV
        output_file.flush()  # Asegura que el encabezado se escriba inmediatamente

        # El formato del struct en Python debe coincidir con el de Arduino.
        # '<' indica little-endian (común en microcontroladores).
        # 'fff' indica tres números de punto flotante (float).
        packet_format = '<fff'
        packet_size = struct.calcsize(packet_format)

        while True:
            # 1. Sincronizar: Esperar hasta encontrar el byte de inicio '<'
            if ser.read(1) == b'<':

                # 2. Leer la carga útil: el número exacto de bytes del paquete
                packet_bytes = ser.read(packet_size)

                # 3. Verificar el fin: leer el byte de fin '>'
                end_byte = ser.read(1)

                # Comprobamos que hemos recibido un paquete completo y válido
                if len(packet_bytes) == packet_size and end_byte == b'>':
                    # 4. Desempaquetar los bytes para obtener los floats
                    ax, ay, az = struct.unpack(packet_format, packet_bytes)

                    # Obtener el timestamp actual en Python
                    current_time = time.time()

                    # 5. Mostrar los datos en la terminal de forma ordenada.
                    print(f"AX: {ax: 8.4f} m/s² | AY: {ay: 8.4f} m/s² | AZ: {az: 8.4f} m/s²", end='\r')

                    # 6. Escribir los datos en el archivo CSV
                    output_file.write(f"{current_time},{ax},{ay},{az}\n")
                    output_file.flush()  # Fuerza la escritura al disco para no perder datos en caso de fallo

    except serial.SerialException as e:
        print(f"\nError: No se pudo abrir o leer el puerto serie '{SERIAL_PORT}'.")
        print(f"Detalle: {e}")
        print("Asegúrate de que:")
        print("  - La placa está conectada.")
        print("  - El nombre del puerto es correcto.")
        print("  - No hay otro programa (como el Monitor Serie de Arduino) usando el puerto.")

    except KeyboardInterrupt:
        # El usuario presionó Ctrl+C
        print("\n\nPrograma detenido por el usuario.")

    except Exception as e:
        print(f"\nOcurrió un error inesperado: {e}")

    finally:
        # Este bloque se ejecuta siempre, asegurando que cerremos el puerto y el archivo.
        if ser and ser.is_open:
            ser.close()
            print("Puerto serie cerrado.")
        if output_file:
            output_file.close()
            print(f"Archivo '{OUTPUT_FILENAME}' cerrado.")


if __name__ == '__main__':
    main()