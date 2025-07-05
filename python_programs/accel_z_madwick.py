import sys
import serial
import time
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget,
    QLabel, QMessageBox
)
from PySide6.QtCore import Qt, QTimer, Slot
# Después importa matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

class RealtimePlotWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Gráfico en Tiempo Real de IMU (ESP32)")
        self.setGeometry(100, 100, 1000, 700) # x, y, width, height

        # --- Configuración del puerto serial ---
        self.serial_port = '/dev/ttyUSB0' # Asegúrate de que este sea el puerto correcto
        self.baud_rate = 921600          # Debe coincidir con el baudrate de tu ESP32
        self.ser = None # Objeto serial

        # --- Datos para el gráfico ---
        # Usaremos una ventana deslizante para mostrar solo los últimos N puntos
        self.max_points = 200 # Número máximo de puntos a mostrar en el gráfico
        self.data_counter = 0 # Para el eje X (tiempo/muestras)
        self.acc_x_data = []
        self.acc_y_data = []
        self.acc_z_data = []

        # --- Configuración de la interfaz de usuario ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.title_label = QLabel("Lecturas de Acelerómetro del ESP32 (ACC_X, ACC_Y, ACC_Z)")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 18px; font-weight: bold; margin-bottom: 10px;")
        self.layout.addWidget(self.title_label)

        # --- Configuración de Matplotlib ---
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        
        # Inicializar las líneas del gráfico
        self.line_x, = self.ax.plot([], [], 'r-', label='ACC_X') # 'r-' para línea roja
        self.line_y, = self.ax.plot([], [], 'g-', label='ACC_Y') # 'g-' para línea verde
        self.line_z, = self.ax.plot([], [], 'b-', label='ACC_Z') # 'b-' para línea azul
        
        self.ax.set_xlabel("Muestras")
        self.ax.set_ylabel("Aceleración (m/s²)")
        self.ax.set_title("Datos de Acelerómetro en Tiempo Real")
        self.ax.grid(True)
        self.ax.legend()
        self.fig.tight_layout()

        # --- Integrar el gráfico de Matplotlib en PySide6 ---
        # Usamos el nombre completo FigureCanvasQTAgg para mayor explicitud
        self.canvas = FigureCanvasQTAgg(self.fig)
        
        # Añadir el widget del gráfico directamente al layout principal
        self.layout.addWidget(self.canvas)
        
        # --- Configuración del temporizador para lectura serial ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_serial_data)
        # El intervalo del temporizador debe ser menor que el intervalo de envío del ESP32.
        # Si el ESP32 envía a 833Hz (cada 1.2ms), un temporizador de 1ms o 5ms es razonable.
        # Usar 1ms puede ser muy agresivo para el sistema, 5-10ms es un buen punto de partida.
        self.timer_interval_ms = 5 # ms
        
        # Iniciar la comunicación serial y el temporizador
        self.init_serial()
        self.start_data_acquisition()

    def init_serial(self):
        """Inicializa la conexión serial."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.ser.reset_input_buffer() # Limpiar el buffer al inicio
            print(f"Puerto serial {self.serial_port} abierto a {self.baud_rate} baudios.")
        except serial.SerialException as e:
            QMessageBox.critical(self, "Error de Puerto Serial", 
                                 f"No se pudo abrir el puerto serial {self.serial_port}:\n{e}\n"
                                 "Asegúrate de que el ESP32 esté conectado y los permisos sean correctos.")
            print(f"Error al abrir el puerto serial: {e}")
            sys.exit(1) # Salir de la aplicación si no se puede abrir el puerto

    def start_data_acquisition(self):
        """Inicia el temporizador para la lectura de datos."""
        if self.ser and self.ser.is_open:
            self.timer.start(self.timer_interval_ms)
            print(f"Adquisición de datos iniciada con temporizador de {self.timer_interval_ms} ms.")
        else:
            print("El puerto serial no está abierto. No se puede iniciar la adquisición de datos.")

    @Slot()
    def read_serial_data(self):
        """Lee datos del puerto serial y actualiza el gráfico."""
        try:
            # Lee una línea completa hasta el carácter de nueva línea
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line:
                # print(f"Raw data: {line}") # Para depuración
                try:
                    # Parsear los valores flotantes
                    # Esperamos "ACC_X,ACC_Y,ACC_Z"
                    parts = line.split(',')
                    if len(parts) == 3: # Asegurarse de que tenemos 3 valores
                        ax = float(parts[0])
                        ay = float(parts[1])
                        az = float(parts[2])

                        # Añadir los datos a las listas
                        self.data_counter += 1
                        self.acc_x_data.append(ax)
                        self.acc_y_data.append(ay)
                        self.acc_z_data.append(az)

                        # Mantener solo los últimos 'max_points'
                        if len(self.acc_x_data) > self.max_points:
                            self.acc_x_data.pop(0)
                            self.acc_y_data.pop(0)
                            self.acc_z_data.pop(0)
                            # Ajustar el contador para el eje X si los datos se desplazan
                            # Esto asegura que el eje X siempre muestre el rango correcto de muestras
                            # en la ventana deslizante.
                            self.ax.set_xlim(self.data_counter - self.max_points, self.data_counter)
                        else:
                            self.ax.set_xlim(0, self.max_points) # Rango inicial si no hay suficientes puntos

                        # Actualizar el gráfico
                        self.update_plot()
                    else:
                        print(f"Formato de datos inesperado: {line}")
                except ValueError as ve:
                    print(f"Error al parsear datos: {line} - {ve}")
                except IndexError as ie:
                    print(f"Error de índice al parsear datos: {line} - {ie}")

        except serial.SerialTimeoutException:
            # No se recibieron datos en el tiempo de espera
            pass 
        except serial.SerialException as e:
            print(f"Error de lectura serial: {e}")
            self.timer.stop() # Detener el temporizador en caso de error grave
            QMessageBox.warning(self, "Error de Lectura Serial", 
                                f"Se perdió la conexión con el puerto serial:\n{e}")
            self.close() # Cerrar la aplicación

    def update_plot(self):
        """Actualiza los datos de las líneas del gráfico y redibuja."""
        # El eje X para la ventana deslizante
        x_values = range(self.data_counter - len(self.acc_x_data) + 1, self.data_counter + 1)

        self.line_x.set_data(x_values, self.acc_x_data)
        self.line_y.set_data(x_values, self.acc_y_data)
        self.line_z.set_data(x_values, self.acc_z_data)

        # Ajustar los límites del eje Y automáticamente
        # Se puede hacer más sofisticado si los datos tienen un rango muy amplio
        self.ax.relim()
        self.ax.autoscale_view()

        self.canvas.draw() # Redibujar el canvas

    def closeEvent(self, event):
        """Sobrescribe el evento de cierre de ventana para cerrar el puerto serial."""
        if self.ser and self.ser.is_open:
            print("Cerrando puerto serial...")
            self.ser.close()
        self.timer.stop()
        event.accept() # Aceptar el evento de cierre

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RealtimePlotWindow()
    window.show()
    sys.exit(app.exec())
