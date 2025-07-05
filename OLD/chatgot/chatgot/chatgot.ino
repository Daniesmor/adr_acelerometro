#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <MadgwickAHRS.h>

Adafruit_LSM6DSOX lsm6dsox;
Madgwick filter;

unsigned long microsPrev;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!lsm6dsox.begin_I2C()) {
    Serial.println("No se encontró el LSM6DSOX");
    while (1) delay(10);
  }

  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  filter.begin(104);
  microsPrev = micros();
}

void loop() {
  sensors_event_t accel, gyro, temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  unsigned long microsNow = micros();
  float deltaT = (microsNow - microsPrev) / 1e6;
  microsPrev = microsNow;

  // Actualizar filtro Madgwick
  filter.updateIMU(
    gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
    accel.acceleration.x, accel.acceleration.y, accel.acceleration.z
  );

  // Obtener matriz de rotación desde cuaterniones
  float q0 = filter.q0;
  float q1 = filter.q1;
  float q2 = filter.q2;
  float q3 = filter.q3;

  // Transformar aceleración del sistema local al marco global
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // Fórmulas de rotación (rotar vector aceleración)
  float gx = ax * (1 - 2 * (q2 * q2 + q3 * q3)) + ay * (2 * (q1 * q2 - q0 * q3)) + az * (2 * (q1 * q3 + q0 * q2));
  float gy = ax * (2 * (q1 * q2 + q0 * q3)) + ay * (1 - 2 * (q1 * q1 + q3 * q3)) + az * (2 * (q2 * q3 - q0 * q1));
  float gz = ax * (2 * (q1 * q3 - q0 * q2)) + ay * (2 * (q2 * q3 + q0 * q1)) + az * (1 - 2 * (q1 * q1 + q2 * q2));

  // Corregir gravedad (en marco global, gravedad en eje Z)
  gz -= 9.80665;

  // Enviar datos
  Serial.print(microsNow); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);

  delay(5);
}
