#include <Adafruit_LSM6DSOX.h>
#include <MadgwickAHRS.h>

Adafruit_LSM6DSOX lsm6dsox;
Madgwick filter;

unsigned long previousMicros = 0;
const unsigned long intervalMicros = 1000;  // 1 ms = 1 kHz

unsigned long sampleCount = 0;
unsigned long totalTime = 0;

void setup() {
  Serial.begin(115200);
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("No se encontró LSM6DSOX");
    while (1) delay(10);
  }

  lsm6dsox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);  
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);

  filter.begin(1000);  // Inicializa filtro a 1000 Hz
  // No uses setBeta() porque no existe en esta versión
}

void loop() {
  unsigned long currentMicros = micros();

  if (currentMicros - previousMicros >= intervalMicros) {
    previousMicros = currentMicros;

    sensors_event_t accel, gyro, temp;
    lsm6dsox.getEvent(&accel, &gyro, &temp);

    float gx = gyro.gyro.x;
    float gy = gyro.gyro.y;
    float gz = gyro.gyro.z;

    float ax = accel.acceleration.x / 9.81;
    float ay = accel.acceleration.y / 9.81;
    float az = accel.acceleration.z / 9.81;

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    sampleCount++;
    totalTime += intervalMicros;
  }

  static unsigned long lastPrintMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastPrintMillis >= 1000) {
    float actualFrequency = (sampleCount * 1000000.0) / totalTime;
    Serial.print("Frecuencia de muestreo real: ");
    Serial.print(actualFrequency, 2);
    Serial.println(" Hz");

    float roll  = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw   = filter.getYaw();

    Serial.print("Roll: "); Serial.print(roll, 2);
    Serial.print(", Pitch: "); Serial.print(pitch, 2);
    Serial.print(", Yaw: "); Serial.println(yaw, 2);

    sampleCount = 0;
    totalTime = 0;

    lastPrintMillis = currentMillis;
  }
}
