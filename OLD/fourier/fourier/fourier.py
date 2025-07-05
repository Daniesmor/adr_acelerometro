import numpy as np
import matplotlib.pyplot as plt

# --- Configuración (DEBE COINCIDIR CON EL CÓDIGO DEL ESP32) ---
FILE_PATH = 'datos_aceleracion.txt' # Nombre del archivo donde pegarás los datos
SAMPLING_RATE = 200.0  # Frecuencia de muestreo en Hz
# Calcula automáticamente el número de muestras leyendo las líneas del archivo
# En tu caso, tu archivo tiene 512 líneas de datos.
# Puedes ponerlo manualmente como:
NUM_SAMPLES = 512 # <--- ¡CAMBIADO A 512!

# O, para que sea más robusto si tu archivo cambia de tamaño:
# with open(FILE_PATH, 'r') as f:
#     # Contar líneas que no sean comentarios (asumiendo que tus datos no inician con '-')
#     NUM_SAMPLES = sum(1 for line in f if not line.strip().startswith('-') and line.strip())

capture_duration_sec = NUM_SAMPLES / SAMPLING_RATE # Recalcula la duración basada en las muestras reales

def analyze_fft(file_path, sampling_rate, num_samples, capture_duration_sec):
    """
    Lee datos de aceleración de un archivo, calcula la FFT y grafica los resultados.
    """
    try:
        # Cargar los datos desde el archivo de texto
        # Ignoramos líneas que comienzan con '#' o '-' (en caso de que queden mensajes)
        data = np.genfromtxt(file_path, delimiter=',', comments='#') # Cambiado comments a '#' para ser más genérico

        # Verificación del número de muestras
        if data.shape[0] != num_samples:
            print(f"Advertencia: El número de filas leídas ({data.shape[0]}) no coincide con NUM_SAMPLES esperado ({num_samples}).")
            print("Ajustando NUM_SAMPLES al número de filas reales para el cálculo de la FFT.")
            num_samples = data.shape[0] # Ajustar NUM_SAMPLES para que coincida con la cantidad de filas leídas
            capture_duration_sec = num_samples / sampling_rate # Recalcular duración

        # Separar los datos por eje
        accel_x = data[:, 0]
        accel_y = data[:, 1]
        accel_z = data[:, 2]

        # --- Gráfica de la Señal en el Tiempo ---
        plt.figure(figsize=(12, 10))

        ax1 = plt.subplot(2, 1, 1) # 2 filas, 1 columna, primer gráfico
        time_axis = np.arange(0, num_samples / sampling_rate, 1/sampling_rate)
        ax1.plot(time_axis, accel_x, label='Accel X')
        ax1.plot(time_axis, accel_y, label='Accel Y')
        ax1.plot(time_axis, accel_z, label='Accel Z')
        ax1.set_title('Señal de Aceleración Lineal Bruta en el Tiempo')
        ax1.set_xlabel('Tiempo (s)')
        ax1.set_ylabel('Aceleración (m/s^2)')
        ax1.legend()
        ax1.grid(True)

        # --- Cálculo y Gráfica de la FFT ---
        fft_x = np.fft.fft(accel_x)
        fft_y = np.fft.fft(accel_y)
        fft_z = np.fft.fft(accel_z)

        # Calcular las frecuencias correspondientes
        frequencies = np.fft.fftfreq(num_samples, d=1/sampling_rate)
        positive_frequencies = frequencies[:num_samples // 2] # Solo nos interesan las frecuencias positivas

        # Calcular la magnitud (amplitud) del espectro para las frecuencias positivas
        # Multiplicar por 2/N para obtener la amplitud real de una señal real
        magnitude_x = np.abs(fft_x[:num_samples // 2]) * 2 / num_samples
        magnitude_y = np.abs(fft_y[:num_samples // 2]) * 2 / num_samples
        magnitude_z = np.abs(fft_z[:num_samples // 2]) * 2 / num_samples

        ax2 = plt.subplot(2, 1, 2) # 2 filas, 1 columna, segundo gráfico
        ax2.plot(positive_frequencies, magnitude_x, label='FFT Accel X')
        ax2.plot(positive_frequencies, magnitude_y, label='FFT Accel Y')
        ax2.plot(positive_frequencies, magnitude_z, label='FFT Accel Z')
        ax2.set_title(f'Espectro de Frecuencia (FFT) - Fs: {sampling_rate}Hz, N: {num_samples} ({capture_duration_sec:.2f}s)')
        ax2.set_xlabel('Frecuencia (Hz)')
        ax2.set_ylabel('Amplitud')
        ax2.legend()
        ax2.grid(True)
        ax2.set_xlim(0, sampling_rate / 2) # Limitar el eje X a la frecuencia de Nyquist

        plt.tight_layout() # Ajustar el diseño para evitar superposiciones
        plt.show()

        # Opcional: Imprimir las frecuencias dominantes
        # Para el eje Z que suele ser el más relevante en un press de banca (movimiento vertical)
        if len(magnitude_z) > 0: # Asegurarse de que no esté vacío
            idx_dominant_z = np.argmax(magnitude_z)
            dominant_freq_z = positive_frequencies[idx_dominant_z]
            print(f"\nFrecuencia dominante en el eje Z (movimiento principal): {dominant_freq_z:.2f} Hz (con amplitud {magnitude_z[idx_dominant_z]:.2f})")
        else:
            print("\nNo se pudieron calcular frecuencias dominantes para el eje Z.")

    except FileNotFoundError:
        print(f"Error: El archivo '{file_path}' no se encontró.")
        print("Asegúrate de haber guardado los datos del Monitor Serie en ese archivo.")
    except Exception as e:
        print(f"Ocurrió un error al procesar los datos: {e}")
        print("Asegúrate de que el formato de los datos en el archivo sea correcto (X,Y,Z por línea).")
        print("Contenido parcial del archivo que causó el error:")
        # Intentar imprimir las últimas líneas del archivo para depuración
        with open(file_path, 'r') as f:
            lines = f.readlines()
            print("Últimas 10 líneas:")
            for line in lines[-10:]:
                print(line.strip())

# --- Ejecutar el análisis ---
if __name__ == "__main__":
    analyze_fft(FILE_PATH, SAMPLING_RATE, NUM_SAMPLES, capture_duration_sec)