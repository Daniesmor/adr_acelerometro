import numpy as np
from scipy.signal import butter, filtfilt

class LowPassFilter:
    """
    Una clase para aplicar un filtro paso bajo Butterworth de fase cero.

    Los coeficientes del filtro se calculan una sola vez durante la inicialización
    para mayor eficiencia.
    """
    def __init__(self, cutoff_freq, sampling_rate, order=4):
        """
        Inicializa el filtro.
        :param cutoff_freq: Frecuencia de corte del filtro en Hz.
        :param sampling_rate: Frecuencia de muestreo de la señal en Hz.
        :param order: Orden del filtro Butterworth.
        """
        nyq = 0.5 * sampling_rate
        normal_cutoff = cutoff_freq / nyq
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        self.min_data_len = 3 * order + 1

    def apply(self, data):
        """
        Aplica el filtro a un conjunto de datos.
        :param data: Un array o lista de numpy con los datos de la señal.
        :return: Los datos filtrados.
        """
        if len(data) < self.min_data_len:
            return data # Devuelve los datos originales si no hay suficientes puntos para filtrar
        return filtfilt(self.b, self.a, data)

