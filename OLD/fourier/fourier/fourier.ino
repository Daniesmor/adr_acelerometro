#include <Adafruit_LSM6DSOX.h>
#include <MadgwickAHRS.h>

// --- Parámetros de Configuración ---
#define SAMPLING_RATE 200.0f // Frecuencia de muestreo en Hz
#define CAPTURE_DURATION_SEC 5 // Duración de la captura en segundos
// Calcula automáticamente el número de muestras basado en la duración y frecuencia de muestreo
#define NUM_SAMPLES ((int)(SAMPLING_RATE * CAPTURE_DURATION_SEC))

// --- Instancias de Hardware y Librerías ---
Adafruit_LSM6DSOX sox;
Madgwick filter;

// --- Variables de Control ---
unsigned int sampleCount = 0;      // Contador de muestras
unsigned long lastUpdate = 0;      // Última vez que se tomó una muestra
unsigned long captureStartTime = 0; // Tiempo en que inició la captura actual
bool captureCompleted = false;     // Bandera para indicar que la captura ha terminado

// Calcular el intervalo de tiempo entre muestras en microsegundos para precisión
const unsigned long loop_interval_micros = (unsigned long)(1000000.0f / SAMPLING_RATE);

void setup() {
  Serial.begin(115200); // Velocidad de comunicación serial

  // Pequeña pausa para asegurar que el Monitor Serie esté listo
  delay(100);
  Serial.println("--- Iniciando ESP32 ---");

  // Inicializar el sensor LSM6DSOX
  if (!sox.begin_I2C()) {
    Serial.println("ERROR: No se encontró el chip LSM6DSOX. Revisa las conexiones.");
    while (1) {
      delay(100); // Bucle infinito si el sensor no inicializa
    }
  }

  // Inicializar el filtro Madgwick con la frecuencia de muestreo
  filter.begin(SAMPLING_RATE);

  Serial.print("Preparado para capturar ");
  Serial.print(NUM_SAMPLES);
  Serial.print(" muestras durante ");
  Serial.print(CAPTURE_DURATION_SEC);
  Serial.println(" segundos.");
  Serial.println("Por favor, inicia tu movimiento ahora.");
  Serial.println("--- COMIENZO DE DATOS ---");

  captureStartTime = millis(); // Registrar el tiempo de inicio de la captura
  lastUpdate = micros();       // Sincronizar el temporizador de muestreo
}

void loop() {
  if (captureCompleted) {
    // Si la captura ha terminado, no hacemos nada más en el loop
    delay(1000); // Pequeña pausa para no saturar el CPU
    return;
  }

  unsigned long currentMicros = micros();

  // Asegurar la frecuencia de muestreo deseada
  if (currentMicros - lastUpdate >= loop_interval_micros) {
    lastUpdate = currentMicros;

    sensors_event_t accel, gyro, temp;
    sox.getEvent(&accel, &gyro, &temp);

    // Convertir giroscopios a grados/segundo para Madgwick
    float gx = gyro.gyro.x * 180 / PI;
    float gy = gyro.gyro.y * 180 / PI;
    float gz = gyro.gyro.z * 180 / PI;

    // Actualizar el filtro Madgwick para obtener la orientación y poder restar la gravedad
    filter.updateIMU(gx, gy, gz, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

    // Obtener cuaternión y calcular la aceleración lineal (sin filtrar, solo sin gravedad)
    float q0 = filter.q0;
    float q1 = filter.q1;
    float q2 = filter.q2;
    float q3 = filter.q3;

    // Calcular y restar la componente de la gravedad
    float gravityX = 2 * (q1 * q3 - q0 * q2);
    float gravityY = 2 * (q0 * q1 + q2 * q3);
    float gravityZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float linearAccelX = accel.acceleration.x - gravityX * 9.81;
    float linearAccelY = accel.acceleration.y - gravityY * 9.81;
    float linearAccelZ = accel.acceleration.z - gravityZ * 9.81;

    // Imprimir la muestra directamente al puerto serie
    Serial.print(linearAccelX, 4); // Imprimir con 4 decimales de precisión
    Serial.print(",");
    Serial.print(linearAccelY, 4);
    Serial.print(",");
    Serial.println(linearAccelZ, 4);

    sampleCount++;
  }

  // Comprobar si se ha recolectado el número deseado de muestras o si ha transcurrido el tiempo
  if (sampleCount >= NUM_SAMPLES || (millis() - captureStartTime >= (CAPTURE_DURATION_SEC * 1000UL))) {
    if (!captureCompleted) { // Asegurarse de que solo se ejecuta una vez al finalizar
      Serial.println("--- FIN DE DATOS ---");
      Serial.print("Captura finalizada. Se recolectaron ");
      Serial.print(sampleCount);
      Serial.println(" muestras.");
      Serial.println("Puedes copiar los datos del Monitor Serie ahora.");
      Serial.println("Reinicia el ESP32 para una nueva captura.");
      captureCompleted = true; // Establecer la bandera para detener el bucle
    }
  }
}
