import serial
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# -------- CONFIGURACIÓN --------
PORT = '/dev/ttyUSB0'  # Cambia esto según tu puerto (COMx en Windows o '/dev/ttyUSB0' en Linux)
BAUDRATE = 115200
SAMPLING_RATE = 208.0  # Hz
DT = 1.0 / SAMPLING_RATE
WINDOW_SIZE = 500  # Número de muestras que se muestran en la gráfica

# -------- INICIALIZACIÓN --------
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

# Buffers circulares para plot
accX, accY, accZ = deque(maxlen=WINDOW_SIZE), deque(maxlen=WINDOW_SIZE), deque(maxlen=WINDOW_SIZE)
velX, velY, velZ = deque(maxlen=WINDOW_SIZE), deque(maxlen=WINDOW_SIZE), deque(maxlen=WINDOW_SIZE)
dispX, dispY, dispZ = deque(maxlen=WINDOW_SIZE), deque(maxlen=WINDOW_SIZE), deque(maxlen=WINDOW_SIZE)

# Estado actual de velocidad y desplazamiento
vX = vY = vZ = 0.0
sX = sY = sZ = 0.0

# -------- CONFIGURACIÓN PLOT --------
plt.ion()
fig, axs = plt.subplots(3, 1, figsize=(10, 8))

axs[0].set_title('Aceleración (m/s²)')
axs[1].set_title('Velocidad (m/s)')
axs[2].set_title('Desplazamiento (m)')

for ax in axs:
    ax.grid(True)

# -------- LOOP PRINCIPAL --------
try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        try:
            data = list(map(float, line.split(',')))
            if len(data) != 6:
                continue

            # Usamos los datos filtrados (los tres primeros)
            aX, aY, aZ = data[0], data[1], data[2]

            # Integración simple (rectangular)
            vX += aX * DT
            vY += aY * DT
            vZ += aZ * DT

            sX += vX * DT
            sY += vY * DT
            sZ += vZ * DT

            # Guardar en buffer para graficar
            accX.append(aX)
            accY.append(aY)
            accZ.append(aZ)

            velX.append(vX)
            velY.append(vY)
            velZ.append(vZ)

            dispX.append(sX)
            dispY.append(sY)
            dispZ.append(sZ)

            # Limpiar los ejes
            for ax in axs:
                ax.cla()

            # Dibujar aceleración
            axs[0].plot(accX, label='X')
            axs[0].plot(accY, label='Y')
            axs[0].plot(accZ, label='Z')
            axs[0].legend()
            axs[1].set_ylabel('m/s²')

            # Dibujar velocidad
            axs[1].plot(velX, label='X')
            axs[1].plot(velY, label='Y')
            axs[1].plot(velZ, label='Z')
            axs[1].legend()
            axs[1].set_ylabel('m/s')

            # Dibujar desplazamiento
            axs[2].plot(dispX, label='X')
            axs[2].plot(dispY, label='Y')
            axs[2].plot(dispZ, label='Z')
            axs[2].legend()
            axs[2].set_ylabel('m')
            axs[2].set_xlabel('Muestras')

            plt.pause(0.001)

        except ValueError:
            continue

except KeyboardInterrupt:
    print('Terminando...')
    ser.close()

