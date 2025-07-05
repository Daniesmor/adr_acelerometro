import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import numpy as np
import threading
from playsound import playsound
import os
from scipy.signal import butter, filtfilt
from scipy.integrate import cumulative_trapezoid  # NUEVO: Importar para integración numérica
import struct

# --- CONFIGURACIÓN GENERAL ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
MAX_LEN = 150  # Longitud máxima de los datos a mostrar en el gráfico
CALIBRATION_SAMPLES = 300

# --- Rutas de Sonido ---
SOUND_START_BEEP = '/home/daniel/Downloads/start_beep.wav'
SOUND_REP_BEEP = '/home/daniel/Downloads/my_beep.wav'

# --- DATOS COMPARTIDOS ENTRE HILOS ---
# Usaremos una lista para almacenar las nuevas muestras y un Lock para la seguridad del hilo
# Cada elemento será una tupla (acc_x, acc_y, acc_z, timestamp)
new_samples_buffer = []
buffer_lock = threading.Lock()

# Deques para los datos en vivo (se llenarán con las muestras procesadas)
acc_x_live = deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
acc_y_live = deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
acc_z_live = deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
# vel_z_live ahora mostrará 0.0 o se podría eliminar si no se necesita velocidad en vivo
vel_z_live = deque([0.0] * MAX_LEN, maxlen=MAX_LEN)

is_running = True

# --- Variables para almacenar el Bias del acelerómetro ---
acc_bias_x = 0.0
acc_bias_y = 0.0
acc_bias_z = 0.0

FILTER_ORDER = 4
CUTOFF_FREQ = 50.0
SAMPLING_RATE = 208.0  # Frecuencia de muestreo del Arduino en Hz (todavía útil para filtros Butterworth)


# ===================================================================
# --- CLASE "CEREBRO": LÓGICA DE DETECCIÓN (BASADA EN REPOSO POR ACELERACIÓN) ---
# ===================================================================
class RepetitionCounter:
    def __init__(self):
        self.state = 'WAITING'
        self.rep_count = 0

        # --- Parámetros de Detección (Ajustados para la nueva lógica) ---
        # AJUSTADO: Umbral de inicio reducido. La señal limpia tiene picos más bajos pero claros.
        self.START_ACCEL_THRESHOLD = 1
        self.CONSECUTIVE_SAMPLES_TO_START = 5

        ## LÓGICA MEJORADA: Parámetros para el fin de la fase concéntrica ##
        # NUEVO: Parámetro para la detección robusta del fin de la concéntrica (aceleración negativa sostenida)
        self.CONSECUTIVE_SAMPLES_FOR_ACCEL_NEGATIVE = 10  # Requiere 3 muestras consecutivas de aceleración negativa

        ## LÓGICA MEJORADA: Parámetros para el fin de la fase excéntrica ##
        self.rest_buffer = deque(maxlen=25)
        # AJUSTADO: Umbrales de reposo mucho más estrictos. La señal limpia es muy estable en reposo.
        self.REST_STD_DEV_THRESHOLD = 0.015
        self.REST_ABS_ACCEL_THRESHOLD = 0.08
        # La condición de velocidad en reposo ahora dependerá de que la velocidad integrada sea 0 al final de la repetición
        # o se puede eliminar si solo se usa la aceleración para detectar el reposo.
        # Para simplificar, la eliminamos de la condición de fin de excéntrica si no hay velocidad en vivo.
        # self.ECCENTRIC_END_VELOCITY_THRESHOLD = 0.07 # Ya no se usa para detección en vivo

        ## LÓGICA MEJORADA: Timeouts de seguridad (en segundos) ##
        self.MAX_REP_DURATION = 7.0  # Si una repetición dura más que esto, se resetea.
        self.state_enter_time = 0.0

        # --- Variables internas de estado ---
        self.start_trigger_counter = 0
        self.concentric_end_counter = 0  # Reutilizado para CONSECUTIVE_SAMPLES_FOR_ACCEL_NEGATIVE
        # self.current_rep_peak_velocity = 0.0 # Ya no se actualiza en vivo, se calcula post-integración

        # --- Buffers y métricas ---
        self.accelerations_in_lift = []  # Almacena aceleraciones para integración
        self.timestamps_in_lift = []  # Almacena timestamps para integración
        self.velocities_in_lift = []  # Este buffer se llenará post-integración

        self.last_rep_avg_speed, self.last_rep_peak_speed = 0.0, 0.0
        self.last_rep_velocities_data = []
        self.last_rep_accelerations_data = []

        # --- NUEVOS PARÁMETROS Y VARIABLES PARA ZUPT PROACTIVO ---
        self.CONSECUTIVE_REST_SAMPLES_FOR_ZUPT = 3  # Cuántas muestras consecutivas de "reposo por aceleración" para aplicar ZUPT
        self.rest_zup_counter = 0  # Contador para las muestras de reposo en ZUPT

    def _is_at_rest(self, acceleration_z):
        self.rest_buffer.append(acceleration_z)
        if len(self.rest_buffer) < self.rest_buffer.maxlen: return False
        std_dev = np.std(list(self.rest_buffer))
        return std_dev < self.REST_STD_DEV_THRESHOLD and abs(acceleration_z) < self.REST_ABS_ACCEL_THRESHOLD

    ## LÓGICA MEJORADA: La función principal ahora gestiona los timeouts ##
    def process_sample(self, acceleration_z,
                       sample_timestamp):  # Ahora recibe el timestamp real de la muestra
        # Comprobación de Timeout de seguridad
        if self.state != 'WAITING' and (time.time() - self.state_enter_time > self.MAX_REP_DURATION):
            print(f"!!! TIMEOUT de {self.MAX_REP_DURATION}s alcanzado. Reseteando estado.!!!")
            self.state = 'WAITING'
            # Resetear contador ZUPT si hay timeout
            self.rest_zup_counter = 0
            return (True, True)  # Forzar reseteo completo

        if self.state == 'WAITING':
            return self._handle_waiting_state(acceleration_z, sample_timestamp)
        elif self.state == 'CONCENTRIC':
            return self._handle_concentric_state(acceleration_z, sample_timestamp)
        elif self.state == 'ECCENTRIC':
            return self._handle_eccentric_state(acceleration_z, sample_timestamp)
        return (False, False)

    def _handle_waiting_state(self, acceleration_z, sample_timestamp):
        is_at_rest = self._is_at_rest(acceleration_z)

        # Resetear contador ZUPT al entrar en WAITING
        self.rest_zup_counter = 0

        # Lógica de inicio de fase concéntrica
        if acceleration_z > self.START_ACCEL_THRESHOLD:
            self.start_trigger_counter += 1
        else:
            # Si la aceleración cae por debajo del umbral, reiniciamos el contador
            self.start_trigger_counter = 0

        if self.start_trigger_counter >= self.CONSECUTIVE_SAMPLES_TO_START:
            print("\nINICIO FASE CONCÉNTRICA")
            # --- TRAZA SOLICITADA ---
            print(f"  > Aceleración Z al inicio: {acceleration_z:.3f} m/s^2")
            print(f"  > Timestamp al inicio:   {sample_timestamp:.6f} s")  # Mostrar el timestamp real
            # --- FIN TRAZA ---
            if os.path.exists(SOUND_START_BEEP): playsound(SOUND_START_BEEP, block=False)
            self.state = 'CONCENTRIC'
            self.state_enter_time = time.time()  ## LÓGICA MEJORADA: Iniciar temporizador
            self.accelerations_in_lift.clear()  # Limpiar para la nueva repetición
            self.timestamps_in_lift.clear()  # Limpiar para la nueva repetición
            self.velocities_in_lift.clear()  # Limpiar para la nueva repetición (se llenará post-integración)
            self.start_trigger_counter = 0
            self.concentric_end_counter = 0
            return (False, False)

        return (is_at_rest, False)

    def _handle_concentric_state(self, acceleration_z, sample_timestamp): # Ahora recibe el timestamp real
        self.accelerations_in_lift.append(acceleration_z)  # Guardar aceleración
        self.timestamps_in_lift.append(sample_timestamp)  # ¡Usar el timestamp real de la muestra!

        # AÑADIDO: Imprimir la aceleración Z en cada instante de la fase concéntrica
        print(f"  > CONCENTRIC: Accel Z = {acceleration_z:.3f} m/s^2")

        # Resetear contador ZUPT al entrar en CONCENTRIC
        self.rest_zup_counter = 0

        # La velocidad pico se calculará post-integración, no se actualiza en vivo aquí.
        # if velocity_z_placeholder > self.current_rep_peak_velocity:
        #     self.current_rep_peak_velocity = velocity_z_placeholder

        # ===================================================================
        # === LÓGICA DE FIN DE FASE CONCÉNTRICA (Aceleración Negativa Sostenida) ===
        # ===================================================================
        if acceleration_z < 0:  # O un pequeño umbral como -0.1 para evitar ruido
            self.concentric_end_counter += 1
        else:
            self.concentric_end_counter = 0  # Resetear si la aceleración vuelve a ser positiva

        if self.concentric_end_counter >= self.CONSECUTIVE_SAMPLES_FOR_ACCEL_NEGATIVE:
            # Fin inmediato de la fase concéntrica
            self.rep_count += 1
            if os.path.exists(SOUND_REP_BEEP): playsound(SOUND_REP_BEEP, block=False)

            # === INTEGRACIÓN NUMÉRICA DE LA VELOCIDAD ===
            if len(self.accelerations_in_lift) > 1:  # Necesitamos al menos 2 puntos para integrar
                # --- INICIO DEPURACIÓN ---
                print(f"DEBUG INTEGRACIÓN: Longitud aceleraciones: {len(self.accelerations_in_lift)}")
                print(f"DEBUG INTEGRACIÓN: Longitud timestamps: {len(self.timestamps_in_lift)}")
                if len(self.timestamps_in_lift) > 1:
                    total_duration = self.timestamps_in_lift[-1] - self.timestamps_in_lift[0]
                    print(f"DEBUG INTEGRACIÓN: Duración total de la fase: {total_duration:.6f} segundos")
                    # Calcular el dt promedio
                    avg_dt = total_duration / (len(self.timestamps_in_lift) - 1) if len(
                        self.timestamps_in_lift) > 1 else 0
                    print(f"DEBUG INTEGRACIÓN: dt promedio: {avg_dt:.6f} segundos")
                    # Mostrar algunos valores de aceleración y sus timestamps
                    print(f"DEBUG INTEGRACIÓN: Primeros 5 (accel, ts):")
                    for i in range(min(5, len(self.accelerations_in_lift))):
                        print(
                            f"  [{i}] Accel: {self.accelerations_in_lift[i]:.3f}, TS: {self.timestamps_in_lift[i]:.6f}")
                    print(f"DEBUG INTEGRACIÓN: Últimos 5 (accel, ts):")
                    for i in range(max(0, len(self.accelerations_in_lift) - 5), len(self.accelerations_in_lift)):
                        print(
                            f"  [{i}] Accel: {self.accelerations_in_lift[i]:.3f}, TS: {self.timestamps_in_lift[i]:.6f}")
                # --- FIN DEPURACIÓN ---

                # Integrar aceleración para obtener velocidad
                # La velocidad inicial de la repetición se asume 0 (desde el reposo)
                integrated_velocities = cumulative_trapezoid(
                    self.accelerations_in_lift,
                    x=self.timestamps_in_lift,
                    initial=0.0
                )
                # --- INICIO DEPURACIÓN DE RESULTADOS ---
                print(f"DEBUG INTEGRACIÓN: Velocidades integradas (primeros 5): {integrated_velocities[:5]}")
                print(f"DEBUG INTEGRACIÓN: Velocidades integradas (últimos 5): {integrated_velocities[-5:]}")
                print(f"DEBUG INTEGRACIÓN: Velocidad pico calculada: {np.max(integrated_velocities):.3f}")
                # --- FIN DEPURACIÓN DE RESULTADOS ---

                self.velocities_in_lift = list(integrated_velocities)  # Llenar el buffer de velocidades
                self.last_rep_velocities_data = list(integrated_velocities)  # Datos para el gráfico de la última rep

                # Calcular métricas de velocidad a partir de las velocidades integradas
                self.last_rep_peak_speed = np.max(integrated_velocities)

                # Calcular velocidad media solo de la fase propulsiva (velocidades positivas)
                positive_integrated_velocities = [v for v in integrated_velocities if v > 0]
                if positive_integrated_velocities:
                    self.last_rep_avg_speed = np.mean(positive_integrated_velocities)
                else:
                    self.last_rep_avg_speed = 0.0
            else:  # No hay suficientes datos para integrar
                self.velocities_in_lift = []
                self.last_rep_velocities_data = []
                self.last_rep_peak_speed = 0.0
                self.last_rep_avg_speed = 0.0
            # === FIN INTEGRACIÓN NUMÉRICA ===

            self.last_rep_accelerations_data = list(
                self.accelerations_in_lift)  # Datos para el gráfico de la última rep

            # === CÓDIGO PARA GUARDAR LOS DATOS DE LA REPETICIÓN ===
            try:
                if not os.path.exists('rep_data'):
                    os.makedirs('rep_data')
                filename = f'rep_data/rep_{self.rep_count}.npz'
                np.savez(filename,
                         velocities=self.last_rep_velocities_data,
                         accelerations=self.last_rep_accelerations_data,
                         timestamps=self.timestamps_in_lift)  # Guardar también los timestamps
                # print(f"\n  > Datos de la repetición guardados en: {filename}") # Opcional
            except Exception as e:
                print(f"\n  > Error al guardar los datos de la repetición: {e}")
            # ===================================================================

            print("\n" + "-" * 30)
            print(f"FIN CONCÉNTRICA REP #{self.rep_count}")
            print(f"  > Vel. Media: {self.last_rep_avg_speed:.3f} m/s")
            print(f"  > Vel. Pico:  {self.last_rep_peak_speed:.3f} m/s")
            print("-" * 30)

            self.state = 'ECCENTRIC'
            self.state_enter_time = time.time()  # Reiniciar temporizador para la excéntrica

            # Limpiar buffers para la siguiente repetición
            self.accelerations_in_lift.clear()
            self.timestamps_in_lift.clear()

            return (False, False)  # Salir para evitar que se procese más en este estado

        # ===================================================================
        # === FIN DE LA LÓGICA DE FIN DE FASE CONCÉNTRICA ===
        # ===================================================================

        return (False, False)  # Si la condición de fin no se cumple, la fase concéntrica continúa

    ## LÓGICA MEJORADA: Detección del fin de la excéntrica más estricta ##
    def _handle_eccentric_state(self, acceleration_z, sample_timestamp): # Ahora recibe el timestamp real
        # Condición 1: El sensor debe estar cinemáticamente en reposo (aceleración estable).
        is_accel_at_rest = self._is_at_rest(acceleration_z)

        # Condición 2: La velocidad integrada debe haber vuelto cerca de cero.
        # Como ya no tenemos velocidad en vivo, esta condición se basa en la aceleración
        # o se puede eliminar si la detección de reposo solo usa aceleración.
        # Para esta implementación, asumimos que si la aceleración está en reposo, la velocidad también lo estará.
        is_velocity_at_rest = True  # Placeholder, ya no se usa la velocidad en vivo para esto

        # --- Lógica de ZUPT Proactivo ---
        if is_accel_at_rest:
            self.rest_zup_counter += 1
        else:
            self.rest_zup_counter = 0  # Resetear si el reposo se interrumpe

        if self.rest_zup_counter >= self.CONSECUTIVE_REST_SAMPLES_FOR_ZUPT:
            # print(f"\n!!! ZUPT PROACTIVO: {self.CONSECUTIVE_REST_SAMPLES_FOR_ZUPT} muestras de reposo. Forzando velocidad a cero.!!!") # Opcional
            self.rest_zup_counter = 0  # Resetear el contador después de aplicar ZUPT
            # Retornamos (False, True) para indicar que se debe resetear la velocidad
            # (que ahora es implícitamente 0 al no haber integración en vivo)
            return (False, True)
            # --- Fin Lógica de ZUPT Proactivo ---

        if is_accel_at_rest and is_velocity_at_rest:  # is_velocity_at_rest siempre es True ahora
            print("\nCiclo de repetición completado (dispositivo en reposo total).")
            self.state = 'WAITING'
            # Resetear contador ZUPT al salir de ECCENTRIC
            self.rest_zup_counter = 0
            return (True, True)  # Retorna True para indicar que se debe resetear la velocidad

        return (False, False)


# ===================================================================
# --- CLASE FILTRO DE KALMAN 1D (ELIMINADA) ---
# ===================================================================
# La clase KalmanFilter1D ha sido eliminada.


# --- Inicialización del Filtro de Kalman (ELIMINADA) ---
# kf = KalmanFilter1D(...) ha sido eliminada.
# Ya no se necesita el objeto kf

rep_counter = RepetitionCounter()


def apply_zero_phase_filter(data, cutoff, fs, order):
    """Aplica un filtro Butterworth de fase cero."""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y


# --- HILO LECTOR DE PUERTO SERIE (VERSIÓN BINARIA ROBUSTA) ---
def serial_reader_thread():
    global ser, is_running, acc_bias_x, acc_bias_y, acc_bias_z, new_samples_buffer, buffer_lock

    # El tamaño esperado de nuestro paquete de datos: 3 floats de 4 bytes cada uno.
    packet_size = struct.calcsize('<fff')  # '<fff' significa: little-endian, float, float, float

    while is_running:
        try:
            # 1. Esperar y leer el byte de inicio '<'
            ser.read_until(b'<')

            # 2. Leer el número exacto de bytes de nuestro paquete de datos
            packet_bytes = ser.read(packet_size)
            if len(packet_bytes) != packet_size:
                continue  # Si no recibimos el paquete completo, lo descartamos

            # 3. Leer el byte de fin '>' (opcional pero bueno para verificar)
            end_byte = ser.read(1)
            if end_byte != b'>':
                continue  # Si no es el byte de fin, el paquete está corrupto, lo descartamos

            # 4. Si todo es correcto, desempaquetamos los bytes para obtener los floats
            x, y, z = struct.unpack('<fff', packet_bytes)

            # El resto de la lógica es la misma que antes
            # NOTA: La calibración del acelerómetro ahora se hace aquí, no al inicio.
            corrected_x = x - acc_bias_x
            corrected_y = y - acc_bias_y
            corrected_z = z - acc_bias_z

            with buffer_lock:
                # Almacenamos el timestamp exacto de la lectura
                new_samples_buffer.append((corrected_x, corrected_y, corrected_z, time.time()))

        except serial.SerialException as e:
            print(f"Error en el hilo lector: {e}", end='\r')
            time.sleep(0.1)
        except (struct.error, Exception) as e:
            print(f"Error inesperado en hilo lector: {e}")
            # No detenemos el hilo, solo ignoramos el paquete corrupto
            pass


# --- Variables globales para la integración de velocidad (ELIMINADAS/SIMPLIFICADAS) ---
# current_velocity_z = 0.0 # Ya no se usa
# previous_acceleration_z = 0.0 # Ya no se usa
# last_processed_timestamp = time.time() # Ya no se usa para integración en vivo


# --- CONFIGURACIÓN DE GRÁFICOS ---
fig, axs = plt.subplots(3, 2, figsize=(16, 12), sharex='col')
fig.suptitle("Análisis de Press de Banca (Lógica Robusta)", fontsize=16)

ax_acc_x = axs[0, 0]
ax_acc_x.set_title("Aceleración Eje X")
ax_acc_x.set_ylabel(r"$m/s^2$")
ax_acc_x.set_ylim(-15, 15)
ax_acc_x.grid(True)
line_acc_x, = ax_acc_x.plot([], [], 'r')

ax_acc_y = axs[1, 0]
ax_acc_y.set_title("Aceleración Eje Y")
ax_acc_y.set_ylabel(r"$m/s^2$")
ax_acc_y.set_ylim(-15, 15)
ax_acc_y.grid(True)
line_acc_y, = ax_acc_y.plot([], [], 'g')

ax_acc_z = axs[2, 0]
ax_acc_z.set_title("Aceleración Eje Z (Vertical)")
ax_acc_z.set_ylabel(r"$m/s^2$")
ax_acc_z.set_xlabel("Muestras")
ax_acc_z.set_ylim(-15, 15)
ax_acc_z.grid(True)
line_acc_z, = ax_acc_z.plot([], [], 'b')

ax_vel_live = axs[0, 1]
ax_vel_live.set_title("Velocidad en Vivo (Eje Z) - No Integrada")  # Título actualizado
ax_vel_live.set_ylabel("m/s")
ax_vel_live.set_ylim(-2.0, 2.0)
ax_vel_live.grid(True)
line_vel_live, = ax_vel_live.plot([], [], 'm')
info_text = ax_vel_live.text(0.98, 0.95, '', transform=ax_vel_live.transAxes, fontsize=12, ha='right', va='top',
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
ax_vel_rep = axs[1, 1]
ax_vel_rep.set_title("Perfil Velocidad (Última Rep. - Integrado)")  # Título actualizado
ax_vel_rep.set_ylabel("m/s")
ax_vel_rep.set_ylim(-3.0, 3.0)
ax_vel_rep.set_xlim(0, 200)
ax_vel_rep.grid(True)
line_vel_rep, = ax_vel_rep.plot([], [], 'c-', marker='.')

ax_acc_rep = axs[2, 1]
ax_acc_rep.set_title("Perfil Aceleración Z (Última Rep.)")
ax_acc_rep.set_ylabel(r"$m/s^2$")
ax_acc_rep.set_xlabel("Muestras en Rep.")
ax_acc_rep.set_ylim(-15, 15)
ax_acc_rep.set_xlim(0, 200)
ax_acc_rep.grid(True)
line_acc_rep, = ax_acc_rep.plot([], [], 'b-', marker='.')


# --- FUNCIÓN DE ACTUALIZACIÓN DE GRÁFICOS (VERSIÓN CON INTEGRACIÓN NUMÉRICA) ---
def update(frame):
    global new_samples_buffer, buffer_lock  # kf ya no es global

    samples_to_process = []
    with buffer_lock:
        if new_samples_buffer:
            samples_to_process = list(new_samples_buffer)
            new_samples_buffer.clear()

    if not samples_to_process:
        info_text.set_text(f'REPS: {rep_counter.rep_count}\n'
                           f'Estado: {rep_counter.state}\n'
                           f'Vel. Media: {rep_counter.last_rep_avg_speed:.3f}\n'
                           f'Vel. Pico: {rep_counter.last_rep_peak_speed:.3f}')
        return line_acc_x, line_acc_y, line_acc_z, line_vel_live, line_vel_rep, line_acc_rep, info_text

    # Procesar cada nueva muestra
    for i, (ax, ay, az, sample_timestamp) in enumerate(samples_to_process): # Recibe el timestamp real
        a_current = az

        # ===================================================================
        # === LÓGICA DE PROCESAMIENTO DE MUESTRAS ===
        # ===================================================================

        # Ya no hay filtro de Kalman en vivo. Pasamos el timestamp real al contador de repeticiones.
        (is_stopped, should_reset_velocity) = rep_counter.process_sample(a_current, sample_timestamp)

        # La lógica de resetear la velocidad (ZUPT) ahora solo implica que la integración futura comenzará desde 0.
        # No hay un estado de filtro de Kalman que resetear aquí.
        if should_reset_velocity or is_stopped:
            # Si se ordena un reseteo, la próxima integración de velocidad comenzará desde 0.
            # Los buffers de aceleración y timestamp ya se limpian en RepetitionCounter
            pass

            # ===================================================================
        # === FIN DE LA LÓGICA DE PROCESAMIENTO DE MUESTRAS ===
        # ===================================================================

        # Añadir los datos a los gráficos en vivo
        acc_x_live.append(ax)
        acc_y_live.append(ay)
        acc_z_live.append(az)
        vel_z_live.append(0.0)  # La velocidad en vivo ya no se calcula, se muestra 0.0

    # --- Actualización de gráficos ---
    line_acc_x.set_data(range(len(acc_x_live)), list(acc_x_live))
    line_acc_y.set_data(range(len(acc_y_live)), list(acc_y_live))
    line_acc_z.set_data(range(len(acc_z_live)), list(acc_z_live))
    line_vel_live.set_data(range(len(vel_z_live)), list(vel_z_live))

    if rep_counter.last_rep_velocities_data:
        rep_len = len(rep_counter.last_rep_velocities_data)
        # El filtro Butterworth se aplica a los datos de la repetición guardada si es necesario
        # Asegúrate de que los datos de la repetición sean lo suficientemente largos para el filtro
        required_len_for_filter = 3 * (FILTER_ORDER + 1)

        if rep_len > required_len_for_filter:
            velocities_for_plot = apply_zero_phase_filter(rep_counter.last_rep_velocities_data, CUTOFF_FREQ,
                                                          SAMPLING_RATE, FILTER_ORDER)
            accelerations_for_plot = apply_zero_phase_filter(rep_counter.last_rep_accelerations_data, CUTOFF_FREQ,
                                                             SAMPLING_RATE, FILTER_ORDER)
        else:
            velocities_for_plot = rep_counter.last_rep_velocities_data
            accelerations_for_plot = rep_counter.last_rep_accelerations_data

        line_vel_rep.set_data(range(rep_len), velocities_for_plot)
        line_acc_rep.set_data(range(rep_len), accelerations_for_plot)
        ax_vel_rep.set_xlim(0, max(1, rep_len))
        ax_acc_rep.set_xlim(0, max(1, rep_len))

    info_text.set_text(f'REPS: {rep_counter.rep_count}\n'
                       f'Estado: {rep_counter.state}\n'
                       f'Vel. Media: {rep_counter.last_rep_avg_speed:.3f}\n'
                       f'Vel. Pico: {rep_counter.last_rep_peak_speed:.3f}')

    return line_acc_x, line_acc_y, line_acc_z, line_vel_live, line_vel_rep, line_acc_rep, info_text


# --- BUCLE PRINCIPAL ---
ser = None
try:
    print(f"Intentando conectar al puerto {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    ser.dtr = False
    time.sleep(0.1)
    ser.dtr = True
    time.sleep(2)
    ser.flushInput()
    print("Conectado.")

    # ===================================================================
    # --- INICIO DE LA RUTINA DE CALIBRACIÓN (RE-ACTIVADA) ---
    # ===================================================================

    # Esta calibración es CRUCIAL para eliminar el bias residual de la
    # aceleración lineal que envía el Arduino, evitando la deriva de la velocidad.

    print("\n-------------------------------------")
    print("Iniciando calibración del acelerómetro...")
    print(f"Se tomarán {CALIBRATION_SAMPLES} muestras. Mantén el sensor completamente quieto.")

    temp_x, temp_y, temp_z = [], [], []

    packet_size = struct.calcsize('<fff')
    samples_collected = 0

    while samples_collected < CALIBRATION_SAMPLES:
        try:
            ser.read_until(b'<')
            packet_bytes = ser.read(packet_size)
            end_byte = ser.read(1)

            if len(packet_bytes) == packet_size and end_byte == b'>':
                x, y, z = struct.unpack('<fff', packet_bytes)
                temp_x.append(x)
                temp_y.append(y)
                temp_z.append(z)
                samples_collected += 1
                print(f"Tomando muestra {samples_collected}/{CALIBRATION_SAMPLES}", end='\r')
        except (struct.error, serial.SerialException):
            continue  # Ignorar paquetes corruptos durante la calibración

    print("\n")  # Nueva línea después del progreso

    # Calcular el promedio para obtener el bias
    if temp_x and temp_y and temp_z:
        acc_bias_x = np.mean(temp_x)
        acc_bias_y = np.mean(temp_y)
        acc_bias_z = np.mean(temp_z)

        print("Calibración completada.")
        print(f"  > Accel Bias X: {acc_bias_x:9.6f}")
        print(f"  > Accel Bias Y: {acc_bias_y:9.6f}")
        print(f"  > Accel Bias Z: {acc_bias_z:9.6f}")
    else:
        print("Error: No se pudieron tomar suficientes muestras para la calibración. Usando bias = 0.")

    print("-------------------------------------")

    # ===================================================================
    # --- FIN DE LA RUTINA DE CALIBRACIÓN ---
    # ===================================================================

    print("\nIniciando hilos y sistema de monitoreo...")
    reader_thread = threading.Thread(target=serial_reader_thread, daemon=True)
    reader_thread.start()

    print("¡Todo listo! Esperando levantamiento...")
    time.sleep(1)  # Pequeña pausa antes de mostrar el gráfico

    ani = animation.FuncAnimation(fig, update, blit=True, interval=5,
                                  cache_frame_data=False)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


except serial.SerialException as e:
    print(f"\nError: No se pudo abrir el puerto serie '{SERIAL_PORT}'.")
    print(f"Detalle: {e}")
    print("Asegúrate de que el dispositivo está conectado y el nombre del puerto es correcto.")
except Exception as e:
    print(f"Error principal: {e}")
finally:
    print("\nCerrando programa...")
    is_running = False
    if 'reader_thread' in locals() and reader_thread.is_alive():
        reader_thread.join(timeout=2)  # Aumentado el timeout para dar más tiempo al hilo
    if ser and ser.is_open:
        ser.close()
        print("Puerto serie cerrado.")