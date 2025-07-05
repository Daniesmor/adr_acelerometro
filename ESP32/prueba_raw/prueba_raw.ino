#include <Adafruit_LSM6DSOX.h>

Adafruit_LSM6DSOX lsm6dsox;

// Parámetros de calibración
const float accel_offset_x = 0.0;
const float accel_offset_y = 0.0;
const float accel_offset_z = 0.0;
const float accel_gain_x = 1.0;
const float accel_gain_y = 1.0;
const float accel_gain_z = 1.0;

const float gyro_bias_x = 0.0;
const float gyro_bias_y = 0.0;
const float gyro_bias_z = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("No se encontró LSM6DSOX");
    while (1) delay(10);
  }

  lsm6dsox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);  
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
  
  Serial.println("Sensor LSM6DSOX listo.");
}

void apply_calibration(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  ax = (ax - accel_offset_x) / accel_gain_x;
  ay = (ay - accel_offset_y) / accel_gain_y;
  az = (az - accel_offset_z) / accel_gain_z;
  
  gx -= gyro_bias_x;
  gy -= gyro_bias_y;
  gz -= gyro_bias_z;
}

void loop() {
  sensors_event_t accel, gyro, temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  // Aceleración en m/s^2 directamente
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  
  // Giroscopio en grados/segundo
  float gx = gyro.gyro.x;
  float gy = gyro.gyro.y;
  float gz = gyro.gyro.z;

  // Aplicar calibración
  apply_calibration(ax, ay, az, gx, gy, gz);

  // Mostrar en formato legible
  Serial.println("==============================");
  Serial.println("   Aceleración (m/s^2)");
  Serial.print("X: "); Serial.println(ax, 4);
  Serial.print("Y: "); Serial.println(ay, 4);
  Serial.print("Z: "); Serial.println(az, 4);
  Serial.println("   Giroscopio (°/s)");
  Serial.print("X: "); Serial.println(gx, 4);
  Serial.print("Y: "); Serial.println(gy, 4);
  Serial.print("Z: "); Serial.println(gz, 4);
  Serial.println("==============================");
  Serial.println();

  delay(100); // 10 Hz
}
