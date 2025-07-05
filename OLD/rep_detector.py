# En tu archivo rep_detector.py

from enum import Enum
import numpy as np
from collections import deque

class RepEvent(Enum):
    NONE = 0
    LIFT_START = 1
    LIFT_COMPLETE = 2

class Phase(Enum):
    ARMED = "ARMADO"
    CONCENTRIC = "CONCÉNTRICA"

class RepDetectorV3_ZUPT:
    """
    Versión final con comunicación de back-dating para una visualización precisa.
    """
    def __init__(self, dt,
                 accel_rest_std_dev_threshold=0.3,
                 samples_for_rest=4,
                 samples_for_start=4,
                 filter_window_size=3,
                 start_accel_threshold=0.1):
        
        self.dt = dt
        self.accel_rest_std_dev_threshold = accel_rest_std_dev_threshold
        self.samples_for_rest = samples_for_rest
        self.samples_for_start = samples_for_start
        self.filter_window_size = filter_window_size
        self.start_accel_threshold = start_accel_threshold

        self.filter_buffer = deque(maxlen=self.filter_window_size)
        self.accel_buffer = deque(maxlen=self.samples_for_rest)
        self.start_data_buffer = deque(maxlen=self.samples_for_start)
        self.is_at_rest = False
        self.was_at_rest = False
        self.phase = Phase.ARMED
        self.rep_count = 0
        self.velocity = 0.0
        self.position = 0.0
        self.rest_counter = 0
        self.max_velocity_in_rep = 0.0
        self.initial_rest_confirmed_for_start_buffer = False # Nueva bandera

        self.accel_bias = 0.0

        print("\n--- DETECTOR INICIADO (V_BACKDATING_FINAL) ---")
        print(f"Ajustes: samples_start_buffer={self.samples_for_start}\n")

    def _update_kinematics(self, accel_sample):
        calibrated_accel = accel_sample - self.accel_bias
        self.velocity += calibrated_accel * self.dt
        self.position += self.velocity * self.dt

    def update(self, raw_accel_z, sample_num): # Added sample_num
        self.filter_buffer.append(raw_accel_z)
        if len(self.filter_buffer) < self.filter_buffer.maxlen:
            return RepEvent.NONE, None, self.phase, self.rep_count, self.velocity, self.position
        
        filtered_accel_z = np.mean(list(self.filter_buffer))
        self.accel_buffer.append(filtered_accel_z)
        
        old_rest_status = self.is_at_rest
        accel_std_dev = np.std(list(self.accel_buffer))
        if accel_std_dev < self.accel_rest_std_dev_threshold:
            self.rest_counter = min(self.rest_counter + 1, self.samples_for_rest)
        else:
            self.rest_counter = 0
        self.is_at_rest = self.rest_counter >= self.samples_for_rest
        if self.is_at_rest and not old_rest_status:
            self.accel_bias = np.mean(list(self.accel_buffer))
            print(f"  [{sample_num:06d}][TRAZA] RECALIBRADO. Nuevo sesgo: {self.accel_bias:+.3f}")
        
        event = RepEvent.NONE
        payload = None # El nuevo paquete de datos

        if self.phase == Phase.ARMED:
            calibrated_accel = filtered_accel_z - self.accel_bias
            # If acceleration is above the threshold, add to the buffer
            # This starts buffering immediately when acceleration rises,
            # regardless of the current 'is_at_rest' status.
            if calibrated_accel > self.start_accel_threshold:
                if not self.start_data_buffer and self.was_at_rest:
                    self.initial_rest_confirmed_for_start_buffer = True
                self.start_data_buffer.append(filtered_accel_z)
            else:
                # If acceleration drops below threshold, clear the buffer.
                # This ensures we only count *consecutive* samples above threshold.
                self.start_data_buffer.clear()
                self.initial_rest_confirmed_for_start_buffer = False



            # La fase CONCENTRIC se dispara si tenemos suficientes muestras consecutivas
            # Y si el inicio de esta secuencia fue confirmado como desde reposo.
            if (len(self.start_data_buffer) >= self.samples_for_start and 
                self.initial_rest_confirmed_for_start_buffer):


                print(f"\n--- [{sample_num:06d}] INICIO: CONCÉNTRICA (Confirmado y retrodatado) ---\n")
                self.phase = Phase.CONCENTRIC
                event = RepEvent.LIFT_START
                payload = self.samples_for_start # <-- Enviamos el tamaño del buffer como payload
                
                self.velocity = 0.0
                self.position = 0.0
                for accel_sample in self.start_data_buffer:
                    self._update_kinematics(accel_sample)
                
                self.start_data_buffer.clear() # Limpiar el buffer después de usarlo
                self.initial_rest_confirmed_for_start_buffer = False # Resetear la bandera
                self.max_velocity_in_rep = self.velocity
            else:
                # Si no se dispara la fase, y el buffer está vacío, reseteamos la cinemática.
                if not self.start_data_buffer:
                    self.velocity = 0.0
                    self.position = 0.0
        
        elif self.phase == Phase.CONCENTRIC:
            self._update_kinematics(filtered_accel_z)
            self.max_velocity_in_rep = max(self.max_velocity_in_rep, self.velocity)
            if self.max_velocity_in_rep > 0.1 and self.velocity < 0:
                self.rep_count += 1
                self.phase = Phase.ARMED
                self.rest_counter = 0
                event = RepEvent.LIFT_COMPLETE # This event is for the visualizer
                print(f"\n--- [{sample_num:06d}] FIN: CONCÉNTRICA (Repetición #{self.rep_count}) ---\n")
        
        
        self.was_at_rest = self.is_at_rest
        return event, payload, self.phase, self.rep_count, self.velocity, self.position # Return sample_num for debugging if needed
