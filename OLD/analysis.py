import numpy as np
import matplotlib.pyplot as plt
import sys # Importamos el módulo sys para acceder a los argumentos de línea de comandos


def analizar_repeticion(file_path):
    """
    Carga y analiza los datos de una repetición específica desde un path de archivo dado.
    """
    try:
        data = np.load(file_path)
        velocities = data['velocities']
        accelerations = data['accelerations']
    except FileNotFoundError:
        print(f"Error: No se encontró el archivo {file_path}")
        return
    except Exception as e:
        print(f"Error al cargar o procesar el archivo {file_path}: {e}")
        return

    # Extraer el número de repetición del nombre del archivo para el título del gráfico
    # Esto asume que el nombre del archivo es 'rep_X.npz'
    rep_number = "Desconocida"
    try:
        # Obtener solo el nombre del archivo (sin la ruta)
        base_filename = file_path.split('/')[-1]
        # Extraer el número (ej. 'rep_5.npz' -> '5')
        if base_filename.startswith('rep_') and base_filename.endswith('.npz'):
            rep_number = base_filename.replace('rep_', '').replace('.npz', '')
    except:
        pass # Si falla la extracción, se queda como "Desconocida"


    # --- ANÁLISIS DE DATOS ---

    # 1. Encontrar la velocidad pico y su posición
    peak_velocity = np.max(velocities)
    peak_velocity_index = np.argmax(velocities)

    # 2. Calcular los umbrales de caída de velocidad para diferentes ratios
    drop_ratio_70 = peak_velocity * 0.70
    drop_ratio_60 = peak_velocity * 0.60
    drop_ratio_50 = peak_velocity * 0.50
    drop_ratio_40 = peak_velocity * 0.40

    # --- VISUALIZACIÓN ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f'Análisis Detallado de la Repetición #{rep_number} (Archivo: {file_path.split("/")[-1]})', fontsize=16)

    # Gráfico de Velocidad
    ax1.plot(velocities, 'c-', marker='.', label='Perfil de Velocidad')
    ax1.axhline(y=peak_velocity, color='g', linestyle='--', label=f'Pico Vel: {peak_velocity:.2f} m/s')
    ax1.axhline(y=drop_ratio_70, color='r', linestyle=':', label='Caída al 70%')
    ax1.axhline(y=drop_ratio_60, color='m', linestyle=':', label='Caída al 60%')
    ax1.axhline(y=drop_ratio_50, color='y', linestyle=':', label='Caída al 50%')
    ax1.axhline(y=drop_ratio_40, color='k', linestyle=':', label='Caída al 40%')
    ax1.scatter(peak_velocity_index, peak_velocity, color='g', s=100, zorder=5)
    ax1.set_title('Perfil de Velocidad')
    ax1.set_ylabel('Velocidad (m/s)')
    ax1.legend()
    ax1.grid(True)

    # Gráfico de Aceleración
    ax2.plot(accelerations, 'b-', marker='.', label='Perfil de Aceleración')
    ax2.axhline(0, color='grey', linestyle='--')
    ax2.set_title('Perfil de Aceleración')
    ax2.set_ylabel('Aceleración (m/s^2)')
    ax2.set_xlabel('Muestras')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()


if __name__ == '__main__':
    # sys.argv es una lista de argumentos de línea de comandos.
    # sys.argv[0] es el nombre del script (analysis.py)
    # sys.argv[1] sería el primer argumento que le pasemos.
    if len(sys.argv) < 2:
        print("Uso: python3 analysis.py <ruta_al_archivo.npz>")
        print("Ejemplo: python3 analysis.py rep_data/rep_5.npz")
        sys.exit(1) # Salir con un código de error

    # El path del archivo es el primer argumento después del nombre del script
    file_to_analyze = sys.argv[1]
    analizar_repeticion(file_to_analyze)