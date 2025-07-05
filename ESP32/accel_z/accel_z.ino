#include <Adafruit_LSM6DSOX.h>
#include <MadgwickAHRS.h>
#include <Wire.h>   // Necesario para I2C
#include <cstdio>   // Necesario para snprintf

// Definiciones de pines I2C para ESP32 (cambia si usas pines diferentes)
// Por defecto en muchos ESP32 (ej. ESP32 DevKitC) son GPIO21 para SDA y GPIO22 para SCL
#ifndef I2C_SDA
#define I2C_SDA 21
#endif
#ifndef I2C_SCL
#define I2C_SCL 22
#endif

Adafruit_LSM6DSOX lsm6dsox;
Madgwick filter;

const float G_MPS2 = 9.80665;
// El intervalo de envío se establece en 1ms para acercarse lo más posible a 833Hz
// (1000ms / 833Hz = ~1.2ms). Un valor de 1ms hará que se intente enviar
// en cada ciclo posible del loop si el procesamiento lo permite.
const unsigned long SEND_INTERVAL_MS = 1; 

// Parámetros de calibración (los que me proporcionaste)
const float accel_bias_x = 0.008985;
const float accel_bias_y = -0.009384;
const float accel_bias_z = -0.005929;
const float accel_scale_x = 0.999961;
const float accel_scale_y = 1.000446;
const float accel_scale_z = -0.999593;

const float gyro_bias_x = 0.004037;
const float gyro_bias_y = -0.002313;
const float gyro_bias_z = -0.003543;

void setup() {
  // Establece el baud rate de la comunicación serial a una velocidad alta.
  // 921600 es la más común para alta velocidad en ESP32 y adaptadores USB-a-serial.
  Serial.begin(921600);
  while (!Serial) delay(10); // Espera a que el puerto serial esté listo

  // Inicializa la comunicación I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  // Establece la velocidad del bus I2C a 400 kHz (Fast Mode) para lecturas rápidas del sensor
  Wire.setClock(400000);

  // Inicializa el sensor LSM6DSOX
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("Error: No se encontró LSM6DSOX. ¡Verifica las conexiones!");
    while (1) delay(10); // Detiene el programa si el sensor no se encuentra
  }

  // Configura la tasa de datos del acelerómetro y giroscopio a 833 Hz
  // LSM6DS_RATE_833_HZ es la opción más cercana y superior a 800Hz.
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_833_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_833_HZ);
  
  // Configura el filtro Madgwick con la frecuencia de muestreo del sensor
  filter.begin(833);
  
  delay(100); // Pequeña pausa para estabilización

  Serial.println("IMU Ready");
  // Formato de salida que el script Python espera (si descomentas todo):
  Serial.println("FORMATO: ACC_X,ACC_Y,ACC_Z,GYR_X,GYR_Y,GYR_Z,ROLL,PITCH,YAW");
}

void loop() {
  static unsigned long lastSend = 0;
  
  // Intenta enviar datos cada SEND_INTERVAL_MS.
  // Con 1ms, esto intentará enviar tan rápido como sea posible,
  // acercándose a la tasa de muestreo del sensor.
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis(); // Actualiza el último tiempo de envío

    // Obtiene los nuevos eventos del sensor
    sensors_event_t accel, gyro, temp;
    lsm6dsox.getEvent(&accel, &gyro, &temp);

    // Calibración acelerómetro
    // Ajusta la aceleración de m/s^2 a G para la calibración, luego de vuelta a m/s^2
    float ax = (accel.acceleration.x / G_MPS2 - accel_bias_x) / accel_scale_x * G_MPS2;
    float ay = (accel.acceleration.y / G_MPS2 - accel_bias_y) / accel_scale_y * G_MPS2;
    float az = -(accel.acceleration.z / G_MPS2 - accel_bias_z) / accel_scale_z * G_MPS2; // Nota el signo negativo en az

    // Calibración giroscopio
    // Asegúrate de que DEG_TO_RAD esté definido si lo necesitas,
    // pero la biblioteca Madgwick ya espera radianes.
    float gx = gyro.gyro.x - (gyro_bias_x * DEG_TO_RAD);
    float gy = gyro.gyro.y - (gyro_bias_y * DEG_TO_RAD);
    float gz = gyro.gyro.z - (gyro_bias_z * DEG_TO_RAD);

    // Actualiza el filtro Madgwick. Los acelerómetros se normalizan a G.
    filter.updateIMU(gx, gy, gz, ax/G_MPS2, ay/G_MPS2, az/G_MPS2);

    // Prepara la cadena de datos completa usando snprintf para eficiencia.
    // Envía solo ACC_X, ACC_Y, ACC_Z con 2 decimales para empezar.
    char buffer[100]; // Un búfer lo suficientemente grande para la cadena de datos
    snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f\n", ax, ay, az);
    
    Serial.print(buffer); // Envía la cadena completa de una sola vez

    /*
    // Si deseas enviar todos los datos (ACC, GYRO, ROLL, PITCH, YAW), descomenta lo siguiente:
    // Asegúrate de que el buffer sea lo suficientemente grande.
    // Recuerda que Madgwick::getRoll(), getPitch(), getYaw() devuelven grados.
    snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
             ax, ay, az,
             gx / DEG_TO_RAD, gy / DEG_TO_RAD, gz / DEG_TO_RAD, // Convierte radianes/s a grados/s para imprimir
             filter.getRoll(), filter.getPitch(), filter.getYaw());
    Serial.print(buffer);
    */
  }
}
