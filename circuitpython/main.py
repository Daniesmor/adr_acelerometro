import board
import busio
import adafruit_lsm6ds

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm6ds.LSM6DSOX(i2c)

while True:
    accel_x, accel_y, accel_z = sensor.acceleration
    gyro_x, gyro_y, gyro_z = sensor.gyro
    print("Accel: ({0:0.2f}, {1:0.2f}, {2:0.2f}) m/s^2".format(accel_x, accel_y, accel_z))
    print("Gyro: ({0:0.2f}, {1:0.2f}, {2:0.2f}) rad/s".format(gyro_x, gyro_y, gyro_z))
