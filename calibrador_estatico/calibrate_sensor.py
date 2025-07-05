import serial
import numpy as np
import time
import json
from pathlib import Path

# --- CONFIGURACIÓN ---
SERIAL_PORT = '/dev/ttyUSB0'   # Cambia según tu puerto
BAUD_RATE = 115200
NUM_SAMPLES = 500 # Used for calibration
G_VALUE = 1.0  # 1g = 9.80665 m/s² (Your ESP32 converts to g before sending)
CALIB_FILE = "sensor_calibration.json"

def save_calibration(data, filename=CALIB_FILE):
    """Guarda los parámetros de calibración en un archivo JSON."""
    try:
        # Conversión segura a tipos serializables
        def convert(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, (np.float32, np.float64)):
                return float(obj)
            if isinstance(obj, dict):
                return {k: convert(v) for k, v in obj.items()}
            return obj
        
        with open(filename, 'w') as f:
            json.dump(convert(data), f, indent=4)
        print(f"✓ Calibración guardada en {filename}")
        return True
    except Exception as e:
        print(f"✗ Error al guardar calibración: {e}")
        return False

def load_calibration(filename=CALIB_FILE):
    """Carga los parámetros de calibración desde un archivo JSON."""
    try:
        if not Path(filename).exists():
            print(f"⚠ Archivo de calibración '{filename}' no encontrado. Se usarán valores por defecto.")
            return None
            
        with open(filename, 'r') as f:
            data = json.load(f)
            
            # Validar estructura básica
            required_keys = {'accel': ['offset', 'gain'], 'gyro': ['bias']}
            for main_key, sub_keys in required_keys.items():
                if main_key not in data:
                    raise ValueError(f"Falta clave '{main_key}' en calibración")
                for sub_key in sub_keys:
                    if sub_key not in data[main_key]:
                        raise ValueError(f"Falta '{main_key}.{sub_key}' en calibración")
                        
            print(f"✓ Calibración existente cargada desde {filename}")
            return data
    except Exception as e:
        print(f"⚠ Error al cargar calibración: {str(e)}")
        print("Se usarán valores por defecto.")
        return None

def get_avg_measurements(ser, num_samples=NUM_SAMPLES):
    """Lee y promedia num_samples muestras."""
    print(f"\nTomando {num_samples} muestras...")
    measurements = []
    start_time = time.time()
    
    # Clear any residual data in the serial buffer
    ser.flushInput()
    
    while len(measurements) < num_samples:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                values = list(map(float, line.split(',')))
                if len(values) == 6: # Ensure all 6 values (ax, ay, az, gx, gy, gz) are present
                    measurements.append(values)
                    
                    # Mostrar progreso cada 100 muestras
                    if len(measurements) % 100 == 0:
                        print(f"{len(measurements)}/{num_samples} muestras...")
        except (ValueError, UnicodeDecodeError, serial.SerialException) as e:
            # print(f"Warning: Could not decode line or serial error: {e}")
            continue # Skip invalid lines or try again on serial error
    
    if not measurements:
        raise ValueError("No se obtuvieron mediciones válidas. Asegúrate de que el ESP32 esté enviando datos.")
    
    avg = np.mean(measurements, axis=0)
    std = np.std(measurements, axis=0)
    
    print(f"→ Tiempo total: {time.time()-start_time:.2f}s")
    print(f"→ Promedio: {avg}")
    print(f"→ Desviación estándar: {std}")
    
    return avg

def calibrate_gyroscope_advanced(ser, current_accel_cal):
    """Calibración mejorada del giroscopio usando nivel de tasa cero.
    Nota: Aunque el script del ESP32 aplica calibración ANTES de enviar,
    para el giroscopio solo necesitamos el 'bias' (offset),
    así que solo promediaremos los valores 3,4,5.
    """
    print("\n" + "="*50)
    print("CALIBRACIÓN AVANZADA DEL GIROSCOPIO".center(50))
    print("="*50)
    print("\nINSTRUCCIONES:")
    print("1. Coloca el sensor en cualquier posición ESTABLE")
    print("2. No es necesario alinearlo con ningún eje")
    print("3. Evita cualquier movimiento o vibración")
    print(f"4. Se tomarán {NUM_SAMPLES*2} muestras para mayor precisión\n")
    
    input("Deja el sensor COMPLETAMENTE QUIETO y pulsa Enter...")
    time.sleep(1)  # Tiempo para asegurar quietud
    
    # We pass the current accelerometer calibration so the ESP32 is sending
    # "calibrated" accel data, but we only care about the raw gyro data for bias.
    avg_all_sensors = get_avg_measurements(ser, NUM_SAMPLES*2)
    
    return {
        "bias": {
            "x": float(avg_all_sensors[3]),
            "y": float(avg_all_sensors[4]),
            "z": float(avg_all_sensors[5])
        },
        "bias_units": "°/s",
        "method": "zero-rate_level"
    }

def calibrate_accelerometer_6_position(ser):
    """Calibración del acelerómetro usando el método de 6 puntos.
    Basado en los datos RAW que el ESP32 enviaría si no se aplicara calibración
    antes de enviar, pero como sí se aplica, el script del ESP32 ya nos da los
    valores 'corregidos' que debemos usar para calcular los offsets y ganancias
    de la calibración. Es decir, los valores que recibimos en Python *ya tienen*
    el offset y la ganancia actual, por lo que el proceso de calibración debe
    encontrar los nuevos.
    
    Sin embargo, la forma más robusta es que el ESP32 envíe datos CRUDOS
    para la calibración y luego aplicar la calibración en Python.
    Dado el código actual de ESP32, asumo que estamos calibrando sobre
    los datos YA PROCESADOS por el `apply_calibration` del ESP32,
    lo cual es inusual pero factible si entendemos que el ESP32 ya "corrige"
    la Z.
    
    Dado que el ESP32 aplica la calibración ANTES de enviar, lo que necesitamos
    aquí son los valores "sin calibrar" efectivos para calcular los nuevos
    parámetros. La forma más sencilla es que el ESP32 envíe siempre los raw
    data para calibración y luego los valores calibrados para uso normal.
    
    Si el ESP32 ya aplica la calibración, los valores que recibimos ya están
    "calibrados" por los valores actuales. Para calcular los nuevos, necesitamos
    los valores brutos o "brutos efectivos".
    
    Vamos a asumir que los valores que recibimos del ESP32 en este punto
    (durante la calibración) *no* tienen los offsets y ganancias aplicados
    por el ESP32 (es decir, asumen que los `accel_offset_x` y `accel_gain_x`
    en el ESP32 son 0.0 y 1.0 respectivamente, como al inicio).
    Si no es así, la lógica aquí se complicaría.

    *Revisión clave:* Tu ESP32 *sí* aplica `apply_calibration` antes de `Serial.print`.
    Esto significa que los valores que recibimos aquí ya han sido 'procesados'
    por la calibración *actualmente cargada* en el ESP32.
    Esto hace que la calibración aquí sea recursiva/problemática.

    **La solución ideal es que tu ESP32 tenga un modo de "raw data" para calibración.**
    Dado que no lo tiene, vamos a asumir que para calibrar, los `offset` y `gain`
    en el ESP32 son 0 y 1 respectivamente, y que los valores recibidos son
    'casi crudos' (solo convertidos a G).
    
    La calibración de 6 puntos calcula el offset como el promedio de (+1g) y (-1g)
    y la ganancia como la mitad del rango entre (+1g) y (-1g).
    """
    print("\n" + "="*50)
    print("CALIBRACIÓN DEL ACELERÓMETRO (6 PUNTOS)".center(50))
    print("="*50)
    print("\nINSTRUCCIONES:")
    print("1. Usa una superficie perfectamente plana y nivelada.")
    print("2. Asegura el sensor firmemente sin vibraciones.")
    print("3. Espera 2 segundos después de cada posición.")
    print("4. Alinea cada eje con la gravedad (+1g o -1g).\n")
    
    readings = {}
    axes = ['X', 'Y', 'Z']
    
    # In order to get the "raw" readings for calibration, we need to temporarily
    # bypass the apply_calibration function in the ESP32.
    # Since we can't do that remotely, we assume that for the purpose of THIS calibration
    # run, the ESP32's current calibration parameters are defaults (0 offset, 1 gain).
    # If the user has already loaded calibration, the Python script will try to load it,
    # but for accel calibration, we are essentially re-calculating from the readings.

    # Collect measurements assuming ESP32 sends values where axis aligns with gravity
    # and has been converted to 'g' but NOT yet compensated for offset/gain from ESP32's internal calibration parameters.
    # The current ESP32 code structure makes this assumption necessary if we want
    # a proper 6-point calibration to calculate new offsets/gains.

    for i, axis in enumerate(axes):
        # Positive direction
        input(f"\n① Coloca el sensor con el eje {axis} hacia ARRIBA (+1g) y pulsa Enter...")
        time.sleep(2)  # Allow time to stabilize
        # Get raw data from ESP32 (which is already converted to 'g' by ESP32, and potentially Z-inverted)
        # We need the values directly from the ESP32's Serial.print
        raw_pos_avg = get_avg_measurements(ser)
        readings[f"{axis}_pos"] = raw_pos_avg[i] # Store the relevant axis reading
        
        # Negative direction
        input(f"② Coloca el sensor con el eje {axis} hacia ABAJO (-1g) y pulsa Enter...")
        time.sleep(2)
        raw_neg_avg = get_avg_measurements(ser)
        readings[f"{axis}_neg"] = raw_neg_avg[i] # Store the relevant axis reading
    
    # The ESP32 is already applying the Z-axis inversion when sending data for Z-axis.
    # So for Z_pos, when Z is up, ESP32 sends a negative value if it's correctly inverted.
    # And for Z_neg, when Z is down, ESP32 sends a positive value.
    # We need to consider this in our calculation here.

    # Calculate offset and gain for each axis
    # The formula for offset is (Max_reading + Min_reading) / 2
    # The formula for gain is (Max_reading - Min_reading) / (2 * Ideal_Range)
    
    # For X and Y, 'up' is +1g, 'down' is -1g
    offset_x = (readings['X_pos'] + readings['X_neg']) / 2
    gain_x = (readings['X_pos'] - readings['X_neg']) / (2 * G_VALUE)
    
    offset_y = (readings['Y_pos'] + readings['Y_neg']) / 2
    gain_y = (readings['Y_pos'] - readings['Y_neg']) / (2 * G_VALUE)
    
    # For Z, if ESP32's `apply_calibration` already has `az = -(az - offset)/gain;`
    # then when Z is physically UP (+1g), ESP32's `az` output will be approx -1.0
    # and when Z is physically DOWN (-1g), ESP32's `az` output will be approx +1.0
    # So, the 'positive' reading is when the sensor is Z_down, and 'negative' is Z_up.
    # Therefore, to compute the offset/gain, we need to swap which reading is considered "positive" and "negative" for the Z-axis,
    # or handle the sign.
    # Let's verify the ESP32 code and what it sends:
    # If Z is physically UP (gravity points +Z): raw reading should be ~+9.81 m/s^2, converted to +1.0g by ESP32.
    # BUT, if `az = -(az - accel_offset_z) / accel_gain_z;` is applied, this becomes -1.0g.
    # So, readings['Z_pos'] (Z physically UP) should be around -1.0
    # And readings['Z_neg'] (Z physically DOWN) should be around +1.0
    
    # Let's adjust the Z-axis calculation to account for this expected inversion
    # The 'positive' 1g equivalent from the ESP32 is Z_neg_reading (physically Z down)
    # The 'negative' 1g equivalent from the ESP32 is Z_pos_reading (physically Z up)
    
    # To compute offset: (reading when physically -1g + reading when physically +1g) / 2
    # So for Z: (readings['Z_neg'] + readings['Z_pos']) / 2
    offset_z = (readings['Z_pos'] + readings['Z_neg']) / 2
    
    # To compute gain: (reading when physically +1g - reading when physically -1g) / (2 * G_VALUE)
    # Since Z_pos_reading (Z up) is effectively -1g, and Z_neg_reading (Z down) is effectively +1g:
    # gain_z = (readings['Z_neg'] - readings['Z_pos']) / (2 * G_VALUE)
    # However, since the Python script generates `az = -(az - offset) / gain;` for ESP32,
    # the 'gain' here should be calculated from the absolute difference and then matched.
    # Let's keep the formula consistent for offset and gain, but expect the gain_z to be negative.
    gain_z = (readings['Z_pos'] - readings['Z_neg']) / (2 * G_VALUE)
    
    # Auto-correction of gains (normalize to ~1.0) - this is crucial
    # This step assumes that the average gain should be close to 1.0 (or -1.0 for Z if inverted).
    # We will compute the average of the absolute gains and use that to normalize.
    
    avg_gain_abs = (abs(gain_x) + abs(gain_y) + abs(gain_z)) / 3
    correction_factor = 1.0 / avg_gain_abs if avg_gain_abs != 0 else 1.0
    
    cal_data = {
        "offset": {
            "x": float(offset_x),
            "y": float(offset_y),
            "z": float(offset_z)
        },
        "gain": {
            "x": float(gain_x * correction_factor),
            "y": float(gain_y * correction_factor),
            "z": float(gain_z * correction_factor) # Keep the sign for Z as calculated
        },
        "correction_factor": float(correction_factor),
        "method": "6-point_with_auto_correction"
    }
    
    # Automatic diagnosis
    print("\n" + "DIAGNÓSTICO DE CALIBRACIÓN".center(50, '-'))
    print(f"Factor de corrección aplicado: {correction_factor:.4f}")
    print("\nGanancias originales (sin corrección):")
    print(f"X: {gain_x:.6f} | Y: {gain_y:.6f} | Z: {gain_z:.6f}")
    print("\nGanancias corregidas (para guardar y usar en ESP32):")
    print(f"X: {cal_data['gain']['x']:.6f} | Y: {cal_data['gain']['y']:.6f} | Z: {cal_data['gain']['z']:.6f}")
    
    return cal_data

def verify_calibration(ser, calibration):
    """Verificación mejorada de la calibración."""
    print("\n" + "="*50)
    print("VERIFICACIÓN DE CALIBRACIÓN".center(50))
    print("="*50)
    
    # Ideal values for accel after applying calibration in ESP32, assuming Z inversion
    # When Z is physically UP, ESP32 outputs -1.0 for Z after its internal calibration
    # When Z is physically DOWN, ESP32 outputs +1.0 for Z after its internal calibration
    # For X up, ESP32 outputs +1.0 for X
    # For Y up, ESP32 outputs +1.0 for Y
    
    positions = [
        ("Horizontal (Z arriba)", [0, 0, -1]),  # If ESP32 inverts Z, then Z-up becomes -1.0
        ("Horizontal (Z abajo)", [0, 0, 1]),   # If ESP32 inverts Z, then Z-down becomes +1.0
        ("Lateral (X arriba)", [1, 0, 0]),
        ("Lateral (Y arriba)", [0, 1, 0])
    ]
    
    for desc, ideal_accel in positions:
        input(f"\nColoca el sensor en posición {desc} y pulsa Enter...")
        time.sleep(1)
        
        # Get data already processed by ESP32's apply_calibration function
        readings = get_avg_measurements(ser, NUM_SAMPLES//5) # Use fewer samples for verification
        
        ax = readings[0]
        ay = readings[1]
        az = readings[2]
        
        gx = readings[3]
        gy = readings[4]
        gz = readings[5]
        
        print("\nRESULTADOS:")
        print(f"Acelerómetro (g): X={ax:.4f} (ideal: {ideal_accel[0]:.1f}) | Y={ay:.4f} (ideal: {ideal_accel[1]:.1f}) | Z={az:.4f} (ideal: {ideal_accel[2]:.1f})")
        print(f"Giroscopio (°/s): X={gx:.4f} | Y={gy:.4f} | Z={gz:.4f} (todos ideales: 0.00)")
        
        # Calculate errors based on ideal values for the *processed* data
        accel_error = np.sqrt((ax-ideal_accel[0])**2 + (ay-ideal_accel[1])**2 + (az-ideal_accel[2])**2)
        gyro_error = np.sqrt(gx**2 + gy**2 + gz**2)
        print(f"→ Error total acelerómetro: {accel_error:.4f}g")
        print(f"→ Error total giroscopio: {gyro_error:.4f}°/s")

def generate_arduino_code(calibration):
    """Genera código listo para copiar al firmware del ESP32."""
    print("\n" + "CÓDIGO PARA ESP32".center(50, '='))
    print("\n// Parámetros de calibración generados automáticamente")
    print(f"const float accel_offset_x = {calibration['accel']['offset']['x']:.6f};")
    print(f"const float accel_offset_y = {calibration['accel']['offset']['y']:.6f};")
    print(f"const float accel_offset_z = {calibration['accel']['offset']['z']:.6f};")
    print(f"const float accel_gain_x = {calibration['accel']['gain']['x']:.6f};")
    print(f"const float accel_gain_y = {calibration['accel']['gain']['y']:.6f};")
    print(f"const float accel_gain_z = {calibration['accel']['gain']['z']:.6f};")
    print(f"const float gyro_bias_x = {calibration['gyro']['bias']['x']:.6f};")
    print(f"const float gyro_bias_y = {calibration['gyro']['bias']['y']:.6f};")
    print(f"const float gyro_bias_z = {calibration['gyro']['bias']['z']:.6f};")
    
    print("\n// Función para aplicar calibración (incluye corrección eje Z)")
    print('''void apply_calibration(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  // Aplicar calibración al acelerómetro
  ax = (ax - accel_offset_x) / accel_gain_x;
  ay = (ay - accel_offset_y) / accel_gain_y;
  az = -(az - accel_offset_z) / accel_gain_z;  // ¡Inversión del eje Z ya aplicada!
  
  // Aplicar calibración al giroscopio
  gx -= gyro_bias_x;
  gy -= gyro_bias_y;
  gz -= gyro_bias_z;
}''')
    print("\nCopia y pega estos valores y la función 'apply_calibration' en tu código del ESP32.")
    print("Asegúrate de que la línea `az = -(az - accel_offset_z) / accel_gain_z;` esté en tu ESP32.")


def main():
    print("\n" + " SISTEMA DE CALIBRACIÓN LSM6DSOX ".center(50, '='))
    
    # Initialize serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2) # Give ESP32 time to reset and start
        ser.flushInput() # Clear any old data
        print(f"\n✓ Conectado al puerto serie {SERIAL_PORT} a {BAUD_RATE} baudios.")
    except serial.SerialException as e:
        print(f"\n✗ Error al conectar con el puerto serie: {str(e)}")
        print(f"  - Asegúrate que el puerto '{SERIAL_PORT}' sea correcto y esté disponible.")
        print("  - En Linux/macOS, revisa 'ls /dev/tty*' o 'ls /dev/cu.*'.")
        print("  - En Windows, revisa el Administrador de Dispositivos para ver los COM Ports.")
        print("  - Asegúrate de que ningún otro programa (como el Serial Monitor de Arduino IDE) esté usando el puerto.")
        return
    except Exception as e:
        print(f"\n✗ Ocurrió un error inesperado al conectar: {str(e)}")
        return
    
    # Load or initialize calibration
    calibration = {
        "accel": {
            "offset": {"x": 0.0, "y": 0.0, "z": 0.0},
            "gain": {"x": 1.0, "y": 1.0, "z": 1.0}
        },
        "gyro": {
            "bias": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
    }
    
    loaded_cal = load_calibration()
    if loaded_cal:
        # We only update the existing calibration dictionary.
        # This assumes the structure matches, which it should if it's from this script.
        calibration.update(loaded_cal)
    
    # Main menu
    while True:
        print("\n" + " MENÚ PRINCIPAL ".center(50, '-'))
        print("1. Calibrar acelerómetro (6 puntos)")
        print("2. Calibrar giroscopio (avanzado)")
        print("3. Verificar calibración actual")
        print("4. Generar código para ESP32")
        print("5. Salir y guardar")
        
        choice = input("\nSelecciona una opción (1-5): ")
        
        if choice == '1':
            # Pass the serial object to calibration function
            calibration["accel"] = calibrate_accelerometer_6_position(ser)
        elif choice == '2':
            # Pass the current accelerometer calibration to gyro calibration
            # so ESP32 sends calibrated accel data during gyro calibration (though not strictly needed for gyro bias)
            calibration["gyro"] = calibrate_gyroscope_advanced(ser, calibration["accel"])
        elif choice == '3':
            verify_calibration(ser, calibration)
        elif choice == '4':
            generate_arduino_code(calibration)
        elif choice == '5':
            if save_calibration(calibration):
                print("\n✓ Configuración guardada exitosamente. ¡Hasta luego!")
            break
        else:
            print("\n✗ Opción no válida. Intenta nuevamente.")
    
    ser.close()
    print("\n" + " CALIBRACIÓN COMPLETADA ".center(50, '='))

if __name__ == "__main__":
    main()