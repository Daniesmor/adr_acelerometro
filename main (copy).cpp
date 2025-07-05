#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

// --- OBJETOS GLOBALES ---
Adafruit_LSM6DSOX imu;
Madgwick          filter;

// --- VARIABLES PARA LA LÓGICA DEL ENCODER ---
enum EncoderState { CALIBRATING, WAITING, CONCENTRIC, ECCENTRIC };
EncoderState currentState = CALIBRATING;

// --- Contador para el disparador de inicio
int start_trigger_counter = 0;

float velocity = 0.0;
float position = 0.0;
float previous_velocity = 0.0; 
float previous_filtered_az = 0.0;

// Variables para métricas de la repetición
float rep_max_velocity = 0.0;
float rep_rom = 0.0;
int rep_count = 0;

// Variables de calibración
float gyro_bias_x = 0.0, gyro_bias_y = 0.0, gyro_bias_z = 0.0;
const int CALIBRATION_SAMPLES = 2000;
const float GRAVITY_MPS2 = 9.81f;

// Variables de tiempo
unsigned long lastUpdateTime = 0;
unsigned long liftStartTime = 0; // MEJORA: Necesitamos registrar el inicio del levantamiento
unsigned long liftStopTime = 0;

// --- PARÁMETROS Y UMBRALES (PARA AJUSTAR) ---
const int sampleRate = 208;
const float CUTOFF_FREQUENCY = 10.0; // Frecuencia de corte en Hz. ¡AJUSTA ESTE VALOR! 5-15 Hz suele ser un buen rango.
const float START_THRESHOLD_MPS2 = 2.0; 
const float STOP_VELOCITY_THRESHOLD_MPS = 0.08;
const int STOP_DURATION_MS = 200;

// --- PARÁMETROS PARA EL DISPARADOR DE INICIO DE REPETICIÓN
const int CONSECUTIVE_SAMPLES_TO_START = 5; // Requiere 5 muestras (~24ms) sobre el umbral

// --- PARÁMETROS para refinar la lógica ---
const float CONCENTRIC_END_VELOCITY_THRESHOLD = 0.05; // m/s. Umbral para confirmar fin de concéntrica.
const int CONCENTRIC_END_SAMPLES = 5; // ~24ms. Muestras consecutivas para confirmar.
int concentric_end_counter = 0; // Contador para el fin de la fase concéntrica.

// --- VARIABLES para el cálculo de MPV ---
float propulsive_velocity_sum = 0.0;
int propulsive_sample_count = 0;
float mpv = 0.0;

// --- BÚFERES PARA PROCESAMIENTO OFFLINE ---
const int MAX_REP_SAMPLES = 500; // Máximo de muestras por repetición. ¡AJUSTAR CON CUIDADO POR LA RAM!
float raw_az_buffer[MAX_REP_SAMPLES];
float dt_buffer[MAX_REP_SAMPLES];
int rep_sample_count = 0;

class ButterworthFilter {
public:
    // Constructor que acepta los parámetros del filtro
    ButterworthFilter(float cutoff_frequency, float sample_rate) {
        // --- Cálculo de coeficientes para Butterworth IIR de 2º orden ---
        const float a = tan(M_PI * cutoff_frequency / sample_rate);
        const float a2 = a * a;
        const float sqrt2a = 1.414213562f * a; // Usar 'f' para literales float
        
        // Coeficientes del numerador (b) y denominador (a)
        b0 = a2 / (1.0f + sqrt2a + a2);
        b1 = 2.0f * b0;
        b2 = b0;
        
        float a0_inv = 1.0f / (1.0f + sqrt2a + a2);
        a1 = -2.0f * (a2 - 1.0f) * a0_inv;
        a2_coeff = (1.0f - sqrt2a + a2) * a0_inv;

        // Inicializar el estado del filtro (historial de muestras)
        x1 = x2 = y1 = y2 = 0.0f;
    }

    // Aplica el filtro a una nueva muestra de entrada
    float apply(float input) {
        // Ecuación de diferencia del filtro IIR
        // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2_coeff * y2;

        // Actualizar el historial de muestras para la siguiente iteración
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;

        return output;
    }

    // MÉTODO PARA FILTRADO DE FASE CERO
    void apply_filtfilt(float* data, int num_samples) {
        if (num_samples == 0) return;

        float temp_buffer[num_samples];

        // 1. PASO HACIA ADELANTE (FORWARD PASS)
        reset_state(); // Reseteamos el estado interno del filtro
        for (int i = 0; i < num_samples; i++) {
            temp_buffer[i] = this->apply(data[i]);
        }

        // 2. PASO HACIA ATRÁS (BACKWARD PASS)
        reset_state(); // Reseteamos de nuevo para el segundo paso
        for (int i = num_samples - 1; i >= 0; i--) {
            // Leemos del buffer temporal y escribimos en el buffer original
            data[i] = this->apply(temp_buffer[i]);
        }
    }


private:
    // Coeficientes del filtro
    float b0, b1, b2, a1, a2_coeff;
    // Estado del filtro (muestras anteriores)
    float x1, x2, y1, y2; 
    // Función de ayuda para resetear el estado del filtro
    void reset_state() {
        x1 = x2 = y1 = y2 = 0.0f;
    }
};
// =================================================================================


ButterworthFilter acceleration_filter(CUTOFF_FREQUENCY, sampleRate);


// --- Variables para la calibración dinámica ---
// Umbrales para detectar el estado de reposo. ¡Deberás ajustarlos!
const float GYRO_STILL_THRESHOLD = 0.1; // Radianes/segundo. Umbral para el giroscopio.
const float ACCEL_STILL_THRESHOLD = 0.5; // m/s^2. Umbral para el acelerómetro.
const unsigned long STILL_TIME_THRESHOLD = 2000; // 2 segundos en milisegundos.

// Variables para el seguimiento del estado de reposo
bool isStationary = false;
unsigned long stationaryStartTime = 0;

// Factor de suavizado para el filtro paso bajo (0.0 a 1.0)
// Un valor bajo actualiza el bias más lentamente.
const float BIAS_UPDATE_ALPHA = 0.01;


void setup() {
    Serial.begin(115200);
    
    if (!imu.begin_I2C()) {
        Serial.println("Error al iniciar LSM6DSOX");
        while (1);
    }
    Serial.println("Sensor LSM6DSOX encontrado!");

    imu.setAccelRange(LSM6DSOX_ACCEL_RANGE_16_G);
    imu.setGyroRange(LSM6DSOX_GYRO_RANGE_2000_DPS);
    imu.setAccelDataRate(LSM6DSOX_RATE_208_HZ);
    imu.setGyroDataRate(LSM6DSOX_RATE_208_HZ);

    filter.begin(sampleRate);
    
    // --- INICIO DE LA RUTINA DE CALIBRACIÓN DEL GIROSCOPIO ---
    Serial.println("Iniciando calibración del giroscopio... Mantén el sensor completamente quieto.");

    // Usaremos una variable temporal para las lecturas del sensor
    sensors_event_t accel, gyro, temp;

    gyro_bias_x = 0.0;
    gyro_bias_y = 0.0;
    gyro_bias_z = 0.0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        imu.getEvent(&accel, &gyro, &temp); // Leer datos del sensor
        gyro_bias_x += gyro.gyro.x;
        gyro_bias_y += gyro.gyro.y;
        gyro_bias_z += gyro.gyro.z;
        delay(2); // Pequeña pausa para asegurar lecturas distintas
    }

    // Calcular el promedio de las lecturas para obtener el bias
    gyro_bias_x /= CALIBRATION_SAMPLES;
    gyro_bias_y /= CALIBRATION_SAMPLES;
    gyro_bias_z /= CALIBRATION_SAMPLES;

    Serial.println("Calibración completada.");
    Serial.print("Gyro Bias X: "); Serial.println(gyro_bias_x, 6);
    Serial.print("Gyro Bias Y: "); Serial.println(gyro_bias_y, 6);
    Serial.print("Gyro Bias Z: "); Serial.println(gyro_bias_z, 6);
    Serial.println("-------------------------------------");
    Serial.println("Sistema listo. Esperando levantamiento...");
    
    // --- FIN DE LA RUTINA DE CALIBRACIÓN ---
    currentState = WAITING; // Omitiendo la calibración real por ahora
    lastUpdateTime = millis();
}

// FUNCIÓN para procesar los datos al final de la repetición
void processRepetitionData() {
    if (rep_sample_count == 0) return;

    // 1. APLICAR FILTRADO DE FASE CERO
    // raw_az_buffer ahora contendrá la aceleración filtrada sin desfase
    acceleration_filter.apply_filtfilt(raw_az_buffer, rep_sample_count);

    // 2. RECALCULAR TODAS LAS MÉTRICAS DESDE CERO CON LOS DATOS FILTRADOS
    // Reiniciamos las variables de estado para el cálculo
    velocity = 0.0;
    position = 0.0;
    previous_velocity = 0.0;
    previous_filtered_az = 0.0;
    rep_max_velocity = 0.0;
    rep_rom = 0.0;
    propulsive_velocity_sum = 0.0;
    propulsive_sample_count = 0;
    mpv = 0.0;
    
    // Iteramos sobre los datos filtrados para calcular todo
    for (int i = 0; i < rep_sample_count; i++) {
        float current_filtered_az = raw_az_buffer[i];
        float current_dt = dt_buffer[i];

        // Integración trapezoidal para velocidad y posición
        previous_velocity = velocity;
        velocity += ((previous_filtered_az + current_filtered_az) / 2.0f) * current_dt;
        position += ((previous_velocity + velocity) / 2.0f) * current_dt;
        previous_filtered_az = current_filtered_az; // Actualizar para el próximo ciclo

        // Métricas
        if (velocity > rep_max_velocity) rep_max_velocity = velocity;
        if (position > rep_rom) rep_rom = position;
        if (current_filtered_az > 0) { // Lógica de MPV
            propulsive_velocity_sum += velocity;
            propulsive_sample_count++;
        }
    }
    
    // Cálculos finales
    float liftDuration = (millis() - liftStartTime) / 1000.0f;
    float averageVelocity = (liftDuration > 0) ? (rep_rom / liftDuration) : 0;
    if (propulsive_sample_count > 0) {
        mpv = propulsive_velocity_sum / propulsive_sample_count;
    }

    // 3. IMPRIMIR RESULTADOS FINALES
    Serial.println("-------------------------------------");
    Serial.print("--- FIN REP #"); Serial.print(rep_count); Serial.println(" (PROCESADO) ---");
    Serial.print("Velocidad Media Propulsiva (MPV): "); Serial.print(mpv, 3); Serial.println(" m/s");
    Serial.print("Velocidad Máxima:                 "); Serial.print(rep_max_velocity, 3); Serial.println(" m/s");
    Serial.print("Distancia (ROM):                  "); Serial.print(rep_rom * 100, 2); Serial.println(" cm");
    Serial.println("-------------------------------------");
}



/**
 * @brief Verifica si el dispositivo está en reposo y, si es así,
 * actualiza suavemente el bias del giroscopio para compensar la deriva.
 * @param current_gyro La lectura actual del giroscopio (sin corregir).
 * @param current_accel La lectura actual del acelerómetro.
 */
void updateGyroBiasInBackground(const sensors_event_t& current_gyro, const sensors_event_t& current_accel) {
    // 1. Comprobar si el giroscopio está quieto
    bool gyroIsStill = (abs(current_gyro.gyro.x - gyro_bias_x) < GYRO_STILL_THRESHOLD) &&
                       (abs(current_gyro.gyro.y - gyro_bias_y) < GYRO_STILL_THRESHOLD) &&
                       (abs(current_gyro.gyro.z - gyro_bias_z) < GYRO_STILL_THRESHOLD);

    // 2. Comprobar si el acelerómetro está quieto (magnitud cercana a 9.8 m/s^2)
    float accelMagnitude = sqrt(current_accel.acceleration.x * current_accel.acceleration.x +
                            current_accel.acceleration.y * current_accel.acceleration.y +
                            current_accel.acceleration.z * current_accel.acceleration.z);
    bool accelIsStill = abs(accelMagnitude - GRAVITY_MPS2) < ACCEL_STILL_THRESHOLD;

    // 3. Lógica para determinar el estado de reposo sostenido
    if (gyroIsStill && accelIsStill) {
        if (!isStationary) {
            // El dispositivo acaba de entrar en estado de reposo, iniciar temporizador
            isStationary = true;
            stationaryStartTime = millis();
        } else {
            // El dispositivo sigue en reposo, comprobar si ha pasado el tiempo suficiente
            if (millis() - stationaryStartTime >= STILL_TIME_THRESHOLD) {
                // ¡Sí! Ha estado quieto el tiempo suficiente. Actualicemos el bias.
                Serial.println("Dispositivo en reposo. Actualizando bias del giroscopio...");

                // Aplicar un filtro paso bajo para actualizar suavemente el bias.
                // nuevo_bias = (1 - alpha) * bias_anterior + alpha * lectura_actual
                gyro_bias_x = (1 - BIAS_UPDATE_ALPHA) * gyro_bias_x + BIAS_UPDATE_ALPHA * current_gyro.gyro.x;
                gyro_bias_y = (1 - BIAS_UPDATE_ALPHA) * gyro_bias_y + BIAS_UPDATE_ALPHA * current_gyro.gyro.y;
                gyro_bias_z = (1 - BIAS_UPDATE_ALPHA) * gyro_bias_z + BIAS_UPDATE_ALPHA * current_gyro.gyro.z;

                // Opcional: Imprimir el nuevo bias para depuración
                Serial.print("Nuevo Bias X: "); Serial.println(gyro_bias_x, 6);
            }
        }
    } else {
        // El dispositivo se ha movido, resetear el estado de reposo
        if (isStationary) {
            isStationary = false;
            Serial.println("Movimiento detectado. Pausando recalibración.");
        }
    }
}


void handleWaitingStage(float linear_az, unsigned long currentTime) {
    // Usamos la aceleración cruda para detectar el inicio
    if (linear_az > START_THRESHOLD_MPS2) {
        start_trigger_counter++;
    } else {
        start_trigger_counter = 0;
    }

    if (start_trigger_counter >= CONSECUTIVE_SAMPLES_TO_START) {
        // INICIO DE REPETICIÓN CONFIRMADO
        // 1. Resetear TODAS las variables de la repetición
        position = 0.0;
        velocity = 0.0;
        previous_velocity = 0.0;
        previous_filtered_az = 0.0;
        rep_max_velocity = 0.0;
        rep_rom = 0.0;
        liftStartTime = currentTime;
        
        // Contadores y sumatorios
        start_trigger_counter = 0; // Solo se necesita una vez
        concentric_end_counter = 0;
        propulsive_velocity_sum = 0.0;
        propulsive_sample_count = 0;
        mpv = 0.0;
        
        // ---- CORRECCIÓN CRÍTICA ----
        // Resetear el contador de muestras del búfer
        rep_sample_count = 0;
        
        // 3. Imprimir mensaje y cambiar de estado
        Serial.println("INICIO FASE CONCÉNTRICA");
        currentState = CONCENTRIC;
    }
}

void handleConcentricState(float linear_az, float dt) {
    // 1. GUARDAR DATOS CRUDOS EN EL BÚFER
    if (rep_sample_count < MAX_REP_SAMPLES) {
        raw_az_buffer[rep_sample_count] = linear_az;
        dt_buffer[rep_sample_count] = dt;
        rep_sample_count++;
    }

    // 2. DETECCIÓN DE FIN DE REP EN TIEMPO REAL
    // Usamos una integración de Euler simple (no la trapezoidal) solo para tener una
    // estimación de la velocidad en tiempo real. Su única finalidad es detectar el fin del movimiento.
    // La velocidad 'real' se calculará después con alta precisión.
    velocity += linear_az * dt;
    
    // Usamos la lógica robusta de confirmación que ya tenías
    if (velocity < CONCENTRIC_END_VELOCITY_THRESHOLD) {
        concentric_end_counter++;
    } else {
        concentric_end_counter = 0;
    }

    if (concentric_end_counter >= CONCENTRIC_END_SAMPLES) {
        rep_count++;
        
        // 3. LLAMAR A LA FUNCIÓN DE PROCESAMIENTO
        // ¡Aquí ocurre la magia! Todos los cálculos de alta precisión se hacen ahora aquí.
        processRepetitionData();

        // 4. CAMBIAR DE ESTADO
        // Reseteamos la velocidad para la fase excéntrica
        velocity = 0.0; 
        previous_velocity = 0.0;
        Serial.println("INICIO FASE EXCÉNTRICA");
        currentState = ECCENTRIC;
    }
}


void handleEccentricState(float linear_az, float dt, unsigned long currentTime) {
    // Seguimos estimando la velocidad de forma simple para saber cuándo se detiene el movimiento.
    velocity += linear_az * dt;
    
    if (abs(velocity) < STOP_VELOCITY_THRESHOLD_MPS) {
        if (liftStopTime == 0) {
            liftStopTime = currentTime;
        }
        if ((currentTime - liftStopTime) >= STOP_DURATION_MS) {
            Serial.println("\nRepetición completada. Esperando siguiente levantamiento...");
            currentState = WAITING;
        }
    } else {
        liftStopTime = 0;
    }
}

void loop() {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    updateGyroBiasInBackground(gyro, accel);

    float corrected_gyro_x = gyro.gyro.x - gyro_bias_x;
    float corrected_gyro_y = gyro.gyro.y - gyro_bias_y;
    float corrected_gyro_z = gyro.gyro.z - gyro_bias_z;

    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    if (dt <= 0.001) return;

    filter.updateIMU(corrected_gyro_x, corrected_gyro_y, corrected_gyro_z, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

    float q[4];
    filter.getQuaternion(q, q + 1, q + 2, q + 3);
    float gravity_z = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    float linear_az = accel.acceleration.z - (gravity_z * GRAVITY_MPS2);

    // ---- CAMBIO IMPORTANTE ----
    // Ya NO filtramos aquí. Pasamos la aceleración lineal cruda a los manejadores de estado.
    // float filtered_az = acceleration_filter.apply(linear_az); // <<-- LÍNEA ELIMINADA

    switch (currentState) {
        case CALIBRATING: {
            break;
        }
        // Pasamos 'linear_az' en lugar de 'filtered_az'
        case WAITING:   handleWaitingStage(linear_az, currentTime);        break;
        // Pasamos 'linear_az' y solo los parámetros que necesita
        case CONCENTRIC:handleConcentricState(linear_az, dt);                break;
        // La fase excéntrica puede seguir usando una lógica simple con la aceleración cruda
        case ECCENTRIC: handleEccentricState(linear_az, dt, currentTime);    break;
    }
}