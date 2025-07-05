from machine import Pin, I2C
import time

# Dirección I2C del LSM6DSOX (usualmente 0x6A o 0x6B)
LSM6DSOX_ADDR = 0x6A

# Registros principales
WHO_AM_I = 0x0F
CTRL1_XL = 0x10
OUTX_L_XL = 0x28

# Inicializa I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# Leer el registro WHO_AM_I para confirmar comunicación
whoami = i2c.readfrom_mem(LSM6DSOX_ADDR, WHO_AM_I, 1)
print("WHO_AM_I:", whoami[0])

# Configurar acelerómetro (por ejemplo, 104 Hz, +- 2g)
i2c.writeto_mem(LSM6DSOX_ADDR, CTRL1_XL, bytes([0x40]))

def read_accel():
    data = i2c.readfrom_mem(LSM6DSOX_ADDR, OUTX_L_XL, 6)
    x = int.from_bytes(data[0:2], 'little', signed=True)
    y = int.from_bytes(data[2:4], 'little', signed=True)
    z = int.from_bytes(data[4:6], 'little', signed=True)
    return (x, y, z)

while True:
    ax, ay, az = read_accel()
    print("Accel X:", ax, "Y:", ay, "Z:", az)
    time.sleep(0.5)
