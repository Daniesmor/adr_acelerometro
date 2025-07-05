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
vel_z_live = deque([0.0] * MAX_LEN, maxlen=MAX_LEN)  # Velocidad integrada en vivo

is_running = True

# --- Variables para almacenar el Bias del acelerómetro ---
acc_bias_x = 0.0
acc_bias_y = 0.0
acc_bias_z = 0.0

FILTER_ORDER = 4
CUTOFF_FREQ = 50.0
SAMPLING_RATE = 208.0  # Frecuencia de muestreo del Arduino en Hz


# ===================================================================
# --- CLASE "CEREBRO": LÓGICA DE DETECCIÓN (BASADA EN REPOSO POR ACELERACIÓN) ---
# ===================================================================
class RepetitionCounter:
    def __init__(self):
        self.state = 'WAITING'
        self.rep_count = 0

        # --- Parámetros de Detección (Ajustados para la nueva lógica) ---
        self.START_ACCEL_THRESHOLD = 0.35
        self.CONSECUTIVE_SAMPLES_TO_START = 3

        ## LÓGICA MEJORADA: Parámetros para el fin de la fase concéntrica ##
        # La fase concéntrica termina si la velocidad cae por debajo del 70% de su pico.
        self.CONCENTRIC_END_PEAK_DROP_RATIO = 0.7
        self.CONSECUTIVE_SAMPLES_FOR_PEAK_DROP = 4

        ## LÓGICA MEJORADA: Parámetros para el fin de la fase excéntrica ##
        self.rest_buffer = deque(maxlen=15)
        self.REST_STD_DEV_THRESHOLD = 0.06
        self.REST_ABS_ACCEL_THRESHOLD = 0.15
        # Nueva condición: la velocidad también debe estar cerca de cero.
        self.ECCENTRIC_END_VELOCITY_THRESHOLD = 0.08

        ## LÓGICA MEJORADA: Timeouts de seguridad (en segundos) ##
        self.MAX_REP_DURATION = 7.0  # Si una repetición dura más que esto, se resetea.
        self.state_enter_time = 0.0

        # --- Variables internas de estado ---
        self.start_trigger_counter = 0
        self.concentric_end_counter = 0
        self.current_rep_peak_velocity = 0.0

        # --- Buffers y métricas ---
        self.velocities_in_lift, self.accelerations_in_lift = [], []
        self.last_rep_avg_speed, self.last_rep_peak_speed = 0.0, 0.0
        self.last_rep_velocities_data, self.last_rep_accelerations_data = [], []

        # --- NUEVOS PARÁMETROS Y VARIABLES PARA ZUPT PROACTIVO ---
        self.CONSECUTIVE_REST_SAMPLES_FOR_ZUPT = 5  # Cuántas muestras consecutivas de "reposo por aceleración" para aplicar ZUPT
        self.rest_zup_counter = 0  # Contador para las muestras de reposo en ZUPT

    def _is_at_rest(self, acceleration_z):
        # (Esta función es la misma que la versión robusta anterior)
        self.rest_buffer.append(acceleration_z)
        if len(self.rest_buffer) < self.rest_buffer.maxlen: return False
        std_dev = np.std(list(self.rest_buffer))
        return std_dev < self.REST_STD_DEV_THRESHOLD and abs(acceleration_z) < self.REST_ABS_ACCEL_THRESHOLD

    ## LÓGICA MEJORADA: La función principal ahora gestiona los timeouts ##
    def process_sample(self, acceleration_z, velocity_z):
        # Comprobación de Timeout de seguridad
        if self.state != 'WAITING' and (time.time() - self.state_enter_time > self.MAX_REP_DURATION):
            print(f"!!! TIMEOUT de {self.MAX_REP_DURATION}s alcanzado. Reseteando estado. !!!")
            self.state = 'WAITING'
            # Resetear contador ZUPT si hay timeout
            self.rest_zup_counter = 0
            return (True, True)  # Forzar reseteo completo

        if self.state == 'WAITING':
            return self._handle_waiting_state(acceleration_z, velocity_z)
        elif self.state == 'CONCENTRIC':
            return self._handle_concentric_state(acceleration_z, velocity_z)
        elif self.state == 'ECCENTRIC':
            return self._handle_eccentric_state(acceleration_z, velocity_z)
        return (False, False)

    def _handle_waiting_state(self, acceleration_z, velocity_z):
        is_at_rest = self._is_at_rest(acceleration_z)

        # Resetear contador ZUPT al entrar en WAITING
        self.rest_zup_counter = 0

        if acceleration_z > self.START_ACCEL_THRESHOLD:
            self.start_trigger_counter += 1
        else:
            self.start_trigger_counter = 0

        if self.start_trigger_counter >= self.CONSECUTIVE_SAMPLES_TO_START:
            print("\nINICIO FASE CONCÉNTRICA")
            if os.path.exists(SOUND_START_BEEP): playsound(SOUND_START_BEEP, block=False)
            self.state = 'CONCENTRIC'
            self.state_enter_time = time.time()  ## LÓGICA MEJORADA: Iniciar temporizador
            self.velocities_in_lift.clear();
            self.accelerations_in_lift.clear()
            self.start_trigger_counter = 0;
            self.concentric_end_counter = 0
            self.current_rep_peak_velocity = 0.0  # Resetear pico de la rep actual
            return (False, False)

        return (is_at_rest, is_at_rest)

    ## LÓGICA MEJORADA: Detección del fin de la concéntrica basada en la caída desde el pico ##
    def _handle_concentric_state(self, acceleration_z, velocity_z):
        self.velocities_in_lift.append(velocity_z)
        self.accelerations_in_lift.append(acceleration_z)

        # Resetear contador ZUPT al entrar en CONCENTRIC
        self.rest_zup_counter = 0

        # Actualizar constantemente la velocidad pico de esta repetición
        if velocity_z > self.current_rep_peak_velocity:
            self.current_rep_peak_velocity = velocity_z

        # Condición para terminar: la velocidad ha caído de forma sostenida
        # por debajo de un porcentaje de su pico.
        if self.current_rep_peak_velocity > 0.1 and \
                velocity_z < (self.current_rep_peak_velocity * self.CONCENTRIC_END_PEAK_DROP_RATIO):
            self.concentric_end_counter += 1
        else:
            self.concentric_end_counter = 0

        if self.concentric_end_counter >= self.CONSECUTIVE_SAMPLES_FOR_PEAK_DROP:
            # Fin de la fase concéntrica
            self.rep_count += 1
            if os.path.exists(SOUND_REP_BEEP): playsound(SOUND_REP_BEEP, block=False)

            self.last_rep_velocities_data = list(self.velocities_in_lift)
            self.last_rep_accelerations_data = list(self.accelerations_in_lift)

            # El cálculo de métricas ahora usa los datos almacenados
            positive_velocities = [v for v in self.last_rep_velocities_data if v > 0]
            if positive_velocities:
                self.last_rep_avg_speed = np.mean(positive_velocities)
                # El pico es el valor máximo guardado
                self.last_rep_peak_speed = self.current_rep_peak_velocity
            else:
                self.last_rep_avg_speed, self.last_rep_peak_speed = 0.0, 0.0

            print("-" * 30);
            print(f"FIN CONCÉNTRICA REP #{self.rep_count}")
            print(f"  > Vel. Media: {self.last_rep_avg_speed:.3f} m/s")
            print(f"  > Vel. Pico:  {self.last_rep_peak_speed:.3f} m/s");
            print("-" * 30)

            self.state = 'ECCENTRIC'
            self.state_enter_time = time.time()  # Reiniciar temporizador para la excéntrica
            return (False, False)

        return (False, False)

    ## LÓGICA MEJORADA: Detección del fin de la excéntrica más estricta ##
    def _handle_eccentric_state(self, acceleration_z, velocity_z):
        # Condición 1: El sensor debe estar cinemáticamente en reposo (aceleración estable).
        is_accel_at_rest = self._is_at_rest(acceleration_z)

        # Condición 2: La velocidad integrada debe haber vuelto cerca de cero.
        is_velocity_at_rest = abs(velocity_z) < self.ECCENTRIC_END_VELOCITY_THRESHOLD

        # --- Lógica de ZUPT Proactivo ---
        if is_accel_at_rest:
            self.rest_zup_counter += 1
        else:
            self.rest_zup_counter = 0  # Resetear si el reposo se interrumpe

        if self.rest_zup_counter >= self.CONSECUTIVE_REST_SAMPLES_FOR_ZUPT:
            print(
                f"!!! ZUPT PROACTIVO: {self.CONSECUTIVE_REST_SAMPLES_FOR_ZUPT} muestras de reposo. Forzando velocidad a cero. !!!")
            self.rest_zup_counter = 0  # Resetear el contador después de aplicar ZUPT
            # Retornamos (False, True) para indicar que se debe resetear la velocidad
            # pero la fase ECCENTRIC aún no ha terminado.
            # Esto permite que la integración se corrija y luego la fase termine naturalmente.
            return (False, True)
            # --- Fin Lógica de ZUPT Proactivo ---

        if is_accel_at_rest and is_velocity_at_rest:
            print("Ciclo de repetición completado (dispositivo en reposo total).")
            self.state = 'WAITING'
            # Resetear contador ZUPT al salir de ECCENTRIC
            self.rest_zup_counter = 0
            return (True, True)  # Retorna True para indicar que se debe resetear la velocidad

        return (False, False)


rep_counter = RepetitionCounter()


def apply_zero_phase_filter(data, cutoff, fs, order):
    """Aplica un filtro Butterworth de fase cero."""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y


# --- HILO LECTOR DE PUERTO SERIE ---
def serial_reader_thread():
    global ser, is_running, acc_bias_x, acc_bias_y, acc_bias_z, new_samples_buffer, buffer_lock
    while is_running:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line and line.count(',') == 2:
                parts = line.split(',')
                x, y, z = map(float, parts)

                # Aplicar la corrección de bias inmediatamente
                corrected_x = x - acc_bias_x
                corrected_y = y - acc_bias_y
                corrected_z = z - acc_bias_z

                # Almacenar la muestra corregida con su timestamp para procesamiento en lote
                with buffer_lock:
                    new_samples_buffer.append((corrected_x, corrected_y, corrected_z, time.time()))
        except (serial.SerialException, ValueError, UnicodeDecodeError) as e:
            print(f"Error en el hilo lector: {e}", end='\r')
            time.sleep(0.01)  # Pequeña pausa para evitar sobrecargar la CPU
        except Exception as e:
            print(f"Error inesperado en hilo lector: {e}")
            is_running = False


# --- Variables globales para la integración de velocidad ---
current_velocity_z = 0.0
previous_acceleration_z = 0.0  # Última aceleración Z procesada
last_processed_timestamp = time.time()  # Timestamp de la última muestra procesada

# --- CONFIGURACIÓN DE GRÁFICOS ---
fig, axs = plt.subplots(3, 2, figsize=(16, 12), sharex='col')
fig.suptitle("Análisis de Press de Banca (Lógica Robusta)", fontsize=16)
ax_acc_x = axs[0, 0];
ax_acc_x.set_title("Aceleración Eje X");
ax_acc_x.set_ylabel(r"$m/s^2$");
ax_acc_x.set_ylim(-15, 15);
ax_acc_x.grid(True)
line_acc_x, = ax_acc_x.plot([], [], 'r')
ax_acc_y = axs[1, 0];
ax_acc_y.set_title("Aceleración Eje Y");
ax_acc_y.set_ylabel(r"$m/s^2$");
ax_acc_y.set_ylim(-15, 15);
ax_acc_y.grid(True)
line_acc_y, = ax_acc_y.plot([], [], 'g')
ax_acc_z = axs[2, 0];
ax_acc_z.set_title("Aceleración Eje Z (Vertical)");
ax_acc_z.set_ylabel(r"$m/s^2$");
ax_acc_z.set_xlabel("Muestras");
ax_acc_z.set_ylim(-15, 15);
ax_acc_z.grid(True)
line_acc_z, = ax_acc_z.plot([], [], 'b')
ax_vel_live = axs[0, 1];
ax_vel_live.set_title("Velocidad en Vivo (Eje Z)");
ax_vel_live.set_ylabel("m/s");
ax_vel_live.set_ylim(-2.0, 2.0);
ax_vel_live.grid(True)
line_vel_live, = ax_vel_live.plot([], [], 'm')
info_text = ax_vel_live.text(0.98, 0.95, '', transform=ax_vel_live.transAxes, fontsize=12, ha='right', va='top',
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
ax_vel_rep = axs[1, 1];
ax_vel_rep.set_title("Perfil Velocidad (Última Rep.)");
ax_vel_rep.set_ylabel("m/s");
ax_vel_rep.set_ylim(-3.0, 3.0);
ax_vel_rep.set_xlim(0, 200);
ax_vel_rep.grid(True)
line_vel_rep, = ax_vel_rep.plot([], [], 'c-', marker='.')
ax_acc_rep = axs[2, 1];
ax_acc_rep.set_title("Perfil Aceleración Z (Última Rep.)");
ax_acc_rep.set_ylabel(r"$m/s^2$");
ax_acc_rep.set_xlabel("Muestras en Rep.");
ax_acc_rep.set_ylim(-15, 15);
ax_acc_rep.set_xlim(0, 200);
ax_acc_rep.grid(True)
line_acc_rep, = ax_acc_rep.plot([], [], 'b-', marker='.')


# --- FUNCIÓN DE ACTUALIZACIÓN DE GRÁFICOS (CON INTEGRACIÓN TRAPEZOIDAL) ---
def update(frame):
    global current_velocity_z, previous_acceleration_z, last_processed_timestamp, new_samples_buffer, buffer_lock

    # 1. Obtener todas las nuevas muestras del buffer de forma segura
    samples_to_process = []
    with buffer_lock:
        if new_samples_buffer:
            samples_to_process = list(new_samples_buffer)
            new_samples_buffer.clear()  # Limpiar el buffer después de copiar las muestras

    # Si no hay nuevas muestras, no hay nada que integrar ni actualizar en los gráficos en vivo
    if not samples_to_process:
        # Solo actualizamos el texto de información y devolvemos las líneas existentes
        # Esto es importante para blit=True, ya que espera que se devuelva algo
        info_text.set_text(f'REPS: {rep_counter.rep_count}\n'
                           f'Estado: {rep_counter.state}\n'
                           f'Vel. Media: {rep_counter.last_rep_avg_speed:.3f}\n'
                           f'Vel. Pico: {rep_counter.last_rep_peak_speed:.3f}')
        return line_acc_x, line_acc_y, line_acc_z, line_vel_live, line_vel_rep, line_acc_rep, info_text

    # 2. Procesar cada nueva muestra para la integración de velocidad
    for i, (ax, ay, az, timestamp) in enumerate(samples_to_process):
        # Usar la aceleración del eje Z para la integración de velocidad
        a_current = az

        # Calcular dt para esta muestra específica.
        # Usamos el SAMPLING_RATE fijo del Arduino para una integración más consistente.
        dt_sample = 1.0 / SAMPLING_RATE

        # Procesar la muestra con la lógica del contador de repeticiones
        (is_stopped, should_reset_velocity) = rep_counter.process_sample(a_current, current_velocity_z)

        if should_reset_velocity or is_stopped:
            current_velocity_z = 0.0
            previous_acceleration_z = 0.0  # Resetear para un inicio limpio de la próxima repetición
        else:
            # Integración trapezoidal para esta muestra
            current_velocity_z += ((previous_acceleration_z + a_current) / 2.0) * dt_sample

        previous_acceleration_z = a_current  # Actualizar para el siguiente paso de integración

        # Añadir la aceleración y velocidad actuales a los deques en vivo para el gráfico
        acc_x_live.append(ax)
        acc_y_live.append(ay)
        acc_z_live.append(az)
        vel_z_live.append(current_velocity_z)

    # 3. Actualizar el timestamp de la última muestra procesada
    last_processed_timestamp = samples_to_process[-1][3]

    # 4. Actualizar los datos de las líneas de los gráficos (solo una vez por llamada a update)
    line_acc_x.set_data(range(len(acc_x_live)), list(acc_x_live))
    line_acc_y.set_data(range(len(acc_y_live)), list(acc_y_live))
    line_acc_z.set_data(range(len(acc_z_live)), list(acc_z_live))
    line_vel_live.set_data(range(len(vel_z_live)), list(vel_z_live))

    # 5. Actualizar los gráficos de la última repetición (si hay datos)
    if rep_counter.last_rep_velocities_data:
        rep_len = len(rep_counter.last_rep_velocities_data)

        # El filtro de fase cero (filtfilt) necesita que la longitud de los datos
        # sea estrictamente mayor que padlen, donde padlen = 3 * (orden_filtro + 1).
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

        ax_vel_rep.set_xlim(0, max(1, rep_len));
        ax_acc_rep.set_xlim(0, max(1, rep_len))

    # 6. Actualizar el texto de información
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
    ser.dtr = False;
    time.sleep(0.1);
    ser.dtr = True;
    time.sleep(2)
    ser.flushInput()
    print("Conectado.")

    # ===================================================================
    # --- INICIO DE LA RUTINA DE CALIBRACIÓN ---
    # ===================================================================
    print("\n-------------------------------------")
    print("Iniciando calibración del acelerómetro...")
    print(f"Se tomarán {CALIBRATION_SAMPLES} muestras. Mantén el sensor completamente quieto.")

    temp_x, temp_y, temp_z = [], [], []

    # Descartar las primeras lecturas para estabilizar el sensor
    for _ in range(50):
        ser.readline()

    for i in range(CALIBRATION_SAMPLES):
        try:
            line = ser.readline().decode('utf-8').strip()
            print(f"Intento [{i + 1}/{CALIBRATION_SAMPLES}]: Python ha leído -> '{line}'")
            if line and line.count(',') == 2:
                parts = line.split(',')
                x, y, z = map(float, parts)
                temp_x.append(x)
                temp_y.append(y)
                temp_z.append(z)
                # Imprimir progreso para que el usuario sepa que algo está pasando
                print(f"Tomando muestra {i + 1}/{CALIBRATION_SAMPLES}", end='\r')
        except (ValueError, UnicodeDecodeError):
            # Ignorar líneas corruptas durante la calibración
            continue

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
                                  cache_frame_data=False)  # O interval=1 para la máxima fluidez
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
        reader_thread.join(timeout=1)
    if ser and ser.is_open:
        ser.close()
        print("Puerto serie cerrado.")