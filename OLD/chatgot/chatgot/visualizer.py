import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, find_peaks

# ⚙️ Configuración
PORT = '/dev/ttyUSB0'   # Cambia por tu puerto
BAUD = 115200
FS = 100        # Frecuencia de muestreo

# 🧽 Filtros
def butter_lowpass(cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low')
    return b, a

def apply_filter(data, cutoff=5):
    b, a = butter_lowpass(cutoff, FS)
    if len(data) < 10:  # Evitar errores con datos pequeños
        return data
    return filtfilt(b, a, data)

# 🛰️ Abrir puerto
ser = serial.Serial(PORT, BAUD)
print("Conectado...")

# 🔢 Inicializar buffers
buffer_size = 500
timestamps = []
accZ = []

# 🔴 Preparar gráfica
plt.ion()
fig, axs = plt.subplots(2, 1, figsize=(10, 6))

line1, = axs[0].plot([], [], label='Acc Z')
axs[0].set_title('Aceleración eje Z')
axs[0].set_ylabel('m/s²')
axs[0].legend()

line2, = axs[1].plot([], [], label='Velocidad Z')
line_peaks, = axs[1].plot([], [], 'rx', label='Reps')
axs[1].set_title('Velocidad eje Z')
axs[1].set_ylabel('m/s')
axs[1].set_xlabel('Tiempo (s)')
axs[1].legend()

# 🔁 Bucle en tiempo real
try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        try:
            data = line.split(',')
            if len(data) < 4:
                continue
            t = int(data[0])
            az = float(data[3])

            timestamps.append(t)
            accZ.append(az)

            if len(accZ) > buffer_size:
                timestamps = timestamps[-buffer_size:]
                accZ = accZ[-buffer_size:]

            # Procesamiento
            time_s = (np.array(timestamps) - timestamps[0]) / 1e6
            accZ_filtered = apply_filter(np.array(accZ))

            dt = np.gradient(time_s) if len(time_s) > 1 else np.array([1/FS])
            velZ = np.cumsum(accZ_filtered * dt)

            velZ_filtered = apply_filter(velZ)

            # Detectar repeticiones
            if len(velZ_filtered) > 10:
                peaks, _ = find_peaks(velZ_filtered, height=0.2, distance=FS/2)
            else:
                peaks = []

            # Actualizar gráfica
            line1.set_data(time_s, accZ_filtered)
            line2.set_data(time_s, velZ_filtered)
            line_peaks.set_data(time_s[peaks], velZ_filtered[peaks])

            axs[0].relim()
            axs[0].autoscale_view()

            axs[1].relim()
            axs[1].autoscale_view()

            plt.pause(0.01)

        except Exception as e:
            print(f"Error: {e}")
            continue

except KeyboardInterrupt:
    print("Terminando...")
    ser.close()
    plt.ioff()
    plt.show()
