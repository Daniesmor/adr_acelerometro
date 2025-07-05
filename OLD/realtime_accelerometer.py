import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QGridLayout, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg
import serial
import numpy as np
import time

# --- PARÁMETRO DE DETECCIÓN CORREGIDO ---
CONCENTRIC_START_V_THRESHOLD = 0.9   # m/s, umbral de inicio realista
CONCENTRIC_END_V_THRESHOLD = 0.05    # m/s, fin de la fase concéntrica

# --- OTROS PARÁMETROS ---
SAMPLING_RATE = 208.0
STABILIZATION_WINDOW_SIZE = 25
STABILIZATION_MEAN_THRESHOLD = 0.5
STABILIZATION_STD_THRESHOLD = 2.4

# Se elimina la clase CalibrationDialog porque ya no es necesaria

class RealTimePlotter(QMainWindow):
    # --- MODIFICADO: __init__ ya no necesita 'offsets' ---
    def __init__(self, serial_port, baud_rate):
        super().__init__()
        # self.offsets = offsets  <-- Eliminado
        self.setWindowTitle("VBT: Sistema Cinemático Calibrado")
        self.setGeometry(100, 100, 1400, 900)
        # ... (El resto del __init__ es igual, lo incluyo por completitud)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QGridLayout(self.central_widget)
        self.layout.setColumnStretch(0, 3); self.layout.setColumnStretch(1, 3); self.layout.setColumnStretch(2, 1)
        self.plot_x = pg.PlotWidget(title="Aceleración X"); self.plot_y = pg.PlotWidget(title="Aceleración Y"); self.plot_z = pg.PlotWidget(title="Aceleración Z")
        self.layout.addWidget(self.plot_x, 0, 0); self.layout.addWidget(self.plot_y, 1, 0); self.layout.addWidget(self.plot_z, 2, 0)
        self.plot_vel_x = pg.PlotWidget(title="Velocidad X"); self.plot_vel_y = pg.PlotWidget(title="Velocidad Y"); self.plot_vel_z = pg.PlotWidget(title="Velocidad Z (Vertical)")
        self.layout.addWidget(self.plot_vel_x, 0, 1); self.layout.addWidget(self.plot_vel_y, 1, 1); self.layout.addWidget(self.plot_vel_z, 2, 1)
        all_plots = [self.plot_x, self.plot_y, self.plot_z, self.plot_vel_x, self.plot_vel_y, self.plot_vel_z]
        for p in all_plots: p.showGrid(x=True, y=True); p.addLegend()
        for p in [self.plot_x, self.plot_y, self.plot_z]: p.setYRange(-15, 15)
        for p in [self.plot_vel_x, self.plot_vel_y, self.plot_vel_z]: p.setYRange(-2, 2)
        self.curve_x = self.plot_x.plot(pen='r', name='Filtrada'); self.raw_curve_x = self.plot_x.plot(pen=pg.mkPen((255,0,0,100), style=Qt.DotLine), name='Cruda')
        self.curve_y = self.plot_y.plot(pen='g', name='Filtrada'); self.raw_curve_y = self.plot_y.plot(pen=pg.mkPen((0,255,0,100), style=Qt.DotLine), name='Cruda')
        self.curve_z = self.plot_z.plot(pen='b', name='Filtrada'); self.raw_curve_z = self.plot_z.plot(pen=pg.mkPen((0,0,255,100), style=Qt.DotLine), name='Cruda')
        self.curve_vel_x = self.plot_vel_x.plot(pen='r', name='Velocidad X'); self.curve_vel_y = self.plot_vel_y.plot(pen='g', name='Velocidad Y'); self.curve_vel_z = self.plot_vel_z.plot(pen='b', name='Velocidad Z')
        info_layout = QVBoxLayout()
        self.velocity_label = QLabel("Vel. Media Rep: -- m/s"); self.velocity_label.setFont(QFont('Arial', 18, QFont.Bold))
        info_layout.addWidget(self.velocity_label, alignment=Qt.AlignCenter)
        self.inst_velocity_label = QLabel("Vel. Inst. Z: 0.00 m/s"); self.inst_velocity_label.setFont(QFont('Arial', 14))
        info_layout.addWidget(self.inst_velocity_label, alignment=Qt.AlignCenter)
        self.rep_state_label = QLabel("Estado: EN ESPERA"); self.rep_state_label.setFont(QFont('Arial', 14, QFont.Bold))
        info_layout.addWidget(self.rep_state_label, alignment=Qt.AlignCenter)
        self.pause_button = QPushButton("Pausa"); info_layout.addStretch(); info_layout.addWidget(self.pause_button)
        self.layout.addLayout(info_layout, 0, 2, 3, 1)
        self.pause_button.setCheckable(True); self.pause_button.clicked.connect(self.toggle_pause)
        self.ser = serial_port
        self.data_x, self.data_y, self.data_z = [], [], []; self.raw_data_x, self.raw_data_y, self.raw_data_z = [], [], []
        self.velocity_data_x, self.velocity_data_y, self.velocity_data_z = [], [], []
        self.current_velocity_x = 0.0; self.current_velocity_y = 0.0; self.current_velocity_z = 0.0
        self.max_len = 300
        self.timer = QTimer(); self.timer.setInterval(5); self.timer.timeout.connect(self.update_plot); self.timer.start()
        self.is_paused = False
        self.green_bar, self.red_bar = None, None
        self.stored_green_pos, self.stored_red_pos = None, None
        self.rep_state = "IDLE"
        self.rep_start_index = 0

    def calculate_mean_velocity_from_segment(self, start_index, end_index):
        start_index = int(start_index)
        end_index = int(end_index)
        if start_index > end_index: start_index, end_index = end_index, start_index
        start_index = max(0, start_index)
        end_index = min(len(self.velocity_data_z), end_index)
        if start_index >= end_index: self.velocity_label.setText("Vel. Media Rep: 0.00 m/s"); return 0.0
        velocity_segment = np.array(self.velocity_data_z[start_index:end_index])
        if velocity_segment.size == 0: self.velocity_label.setText("Vel. Media Rep: 0.00 m/s"); return 0.0
        mean_vel = np.mean(velocity_segment)
        self.velocity_label.setText(f"Vel. Media Rep: {mean_vel:.2f} m/s")
        return mean_vel

    def update_plot(self):
        if self.is_paused: return
        dt = 1.0 / SAMPLING_RATE
        new_data_count = 0
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line and len(line.split(',')) == 6:
                    raw_values = list(map(float, line.split(',')))
                    
                    # --- MODIFICADO: Leemos los datos directamente, sin restar offsets ---
                    acc_x = raw_values[0]
                    acc_y = raw_values[1]
                    acc_z = raw_values[2]
                    self.data_x.append(acc_x); self.data_y.append(acc_y); self.data_z.append(acc_z)
                    
                    raw_acc_x = raw_values[3]
                    raw_acc_y = raw_values[4]
                    raw_acc_z = raw_values[5]
                    self.raw_data_x.append(raw_acc_x); self.raw_data_y.append(raw_acc_y); self.raw_data_z.append(raw_acc_z)
                    
                    new_data_count += 1
                    self.current_velocity_x += acc_x * dt; self.current_velocity_y += acc_y * dt; self.current_velocity_z += acc_z * dt
                    self.velocity_data_x.append(self.current_velocity_x); self.velocity_data_y.append(self.current_velocity_y); self.velocity_data_z.append(self.current_velocity_z)

            except (ValueError, UnicodeDecodeError): continue
            
        if new_data_count == 0: return
        # ... (El resto de la función update_plot y toda la clase se mantienen igual) ...
        popped_count = 0
        while len(self.data_x) > self.max_len:
            self.data_x.pop(0); self.data_y.pop(0); self.data_z.pop(0)
            self.raw_data_x.pop(0); self.raw_data_y.pop(0); self.raw_data_z.pop(0)
            self.velocity_data_x.pop(0); self.velocity_data_y.pop(0); self.velocity_data_z.pop(0)
            popped_count += 1
        if popped_count > 0:
             if self.stored_green_pos is not None: self.stored_green_pos = max(0, self.stored_green_pos - popped_count)
             if self.stored_red_pos is not None: self.stored_red_pos = max(0, self.stored_red_pos - popped_count)
        if len(self.data_z) >= STABILIZATION_WINDOW_SIZE:
            accel_window = np.array(self.data_z[-STABILIZATION_WINDOW_SIZE:])
            is_stable = abs(np.mean(accel_window)) < STABILIZATION_MEAN_THRESHOLD and np.std(accel_window) < STABILIZATION_STD_THRESHOLD
            if is_stable:
                self.current_velocity_x = 0.0; self.current_velocity_y = 0.0; self.current_velocity_z = 0.0
                for i in range(min(len(self.velocity_data_z), STABILIZATION_WINDOW_SIZE)):
                    self.velocity_data_x[-(i+1)] = 0.0; self.velocity_data_y[-(i+1)] = 0.0; self.velocity_data_z[-(i+1)] = 0.0
        if self.rep_state == "IDLE":
            self.rep_state_label.setText("Estado: EN ESPERA"); self.rep_state_label.setStyleSheet("color: green;")
            if self.current_velocity_z > CONCENTRIC_START_V_THRESHOLD:
                self.rep_state = "CONCENTRIC"
                self.rep_start_index = len(self.velocity_data_z) - 1
                self.stored_green_pos = self.rep_start_index
                self.stored_red_pos = None
        elif self.rep_state == "CONCENTRIC":
            self.rep_state_label.setText("Estado: FASE CONCÉNTRICA"); self.rep_state_label.setStyleSheet("color: red;")
            if self.current_velocity_z < CONCENTRIC_END_V_THRESHOLD:
                rep_end_index = len(self.velocity_data_z) - 1
                self.stored_red_pos = rep_end_index
                self.calculate_mean_velocity_from_segment(self.rep_start_index, rep_end_index)
                self.rep_state = "IDLE"
        self.update_gui_elements()
        
    def update_gui_elements(self):
        x_axis = np.arange(len(self.data_x))
        self.curve_x.setData(x_axis, self.data_x); self.raw_curve_x.setData(x_axis, self.raw_data_x)
        self.curve_y.setData(x_axis, self.data_y); self.raw_curve_y.setData(x_axis, self.raw_data_y)
        self.curve_z.setData(x_axis, self.data_z); self.raw_curve_z.setData(x_axis, self.raw_data_z)
        vel_x_axis = np.arange(len(self.velocity_data_x))
        self.curve_vel_x.setData(vel_x_axis, self.velocity_data_x); self.curve_vel_y.setData(vel_x_axis, self.velocity_data_y); self.curve_vel_z.setData(vel_x_axis, self.velocity_data_z)
        self.inst_velocity_label.setText(f"Vel. Inst. Z: {self.current_velocity_z:.2f} m/s")
        self.draw_threshold_markers()

    def marker_moved(self, line):
        if line == self.green_bar: self.stored_green_pos = line.value()
        elif line == self.red_bar: self.stored_red_pos = line.value()
        self.recalculate_from_markers()

    def recalculate_from_markers(self):
        if self.stored_green_pos is not None and self.stored_red_pos is not None:
            self.calculate_mean_velocity_from_segment(self.stored_green_pos, self.stored_red_pos)

    def draw_threshold_markers(self):
        if self.green_bar: 
            try: self.plot_z.removeItem(self.green_bar)
            except: pass
        if self.red_bar: 
            try: self.plot_z.removeItem(self.red_bar)
            except: pass
        self.green_bar, self.red_bar = None, None
        is_movable = self.is_paused
        if self.stored_green_pos is not None:
            self.green_bar = pg.InfiniteLine(pos=self.stored_green_pos, angle=90, movable=is_movable, pen=pg.mkPen('g', width=3))
            self.plot_z.addItem(self.green_bar)
            if is_movable: self.green_bar.sigPositionChanged.connect(self.marker_moved)
        if self.stored_red_pos is not None:
            self.red_bar = pg.InfiniteLine(pos=self.stored_red_pos, angle=90, movable=is_movable, pen=pg.mkPen('r', width=3))
            self.plot_z.addItem(self.red_bar)
            if is_movable: self.red_bar.sigPositionChanged.connect(self.marker_moved)
                
    def toggle_pause(self):
        self.is_paused = not self.is_paused
        self.pause_button.setText("Reanudar" if self.is_paused else "Pausa")
        self.draw_threshold_markers()

    def closeEvent(self, event):
        self.timer.stop(); self.ser.close(); event.accept()

# --- MODIFICADO: El bloque principal ahora es mucho más simple ---
if __name__ == '__main__':
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 115200
    app = QApplication(sys.argv)

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Error fatal al abrir el puerto: {e}")
        print("Asegúrate de que el ESP32 está conectado y que el puerto es el correcto.")
        sys.exit(1)

    # Ya no se muestra el diálogo de calibración
    # Se asume que el ESP32 envía datos calibrados
    print("Conectado al ESP32. Asumiendo que los datos recibidos ya están calibrados.")
    main_window = RealTimePlotter(ser, BAUD_RATE)
    main_window.show()
    sys.exit(app.exec_())