/* Includes ---------------------------------------------------------------- */
#include <clasificacion_imu_inferencing.h>
#include <Arduino_LSM9DS1.h>

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2     9.80665f
#define MAX_ACCEPTED_RANGE   2.0f
#define CONFIDENCE_THRESHOLD 0.90f // Umbral mínimo de confianza (60%) para encender el LED

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;

/**
 * @brief Controla el LED RGB integrado del Arduino Nano 33 BLE Sense
 *        (lógica inversa: 0 = encendido, 255 = apagado).
 */
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
    analogWrite(LEDR, 255 - r);
    analogWrite(LEDG, 255 - g);
    analogWrite(LEDB, 255 - b);
}

/**
 * @brief Ilumina el LED según la inclinación detectada
 */
void actualizarLedPorPosicion(const char* label_cstr, float confidence) {
    if (confidence < CONFIDENCE_THRESHOLD) {
        setLedColor(0, 0, 0);
        return;
    }

    String label = String(label_cstr);
    label.toLowerCase();

    bool isPos = (label.indexOf("pos") >= 0 || label.indexOf("+") >= 0);
    bool isNeg = (label.indexOf("neg") >= 0 || label.indexOf("-") >= 0);

    if (label.indexOf("roll") >= 0) {
        if (isPos) {
            setLedColor(255, 0, 0);       // Roll positivo: Rojo
        } else if (isNeg) {
            setLedColor(255, 30, 90);     // Roll negativo: Rosa
        }
    } 
    else if (label.indexOf("pitch") >= 0) {
        if (isPos) {
            setLedColor(0, 255, 0);       // Pitch positivo: Verde
        } else if (isNeg) {
            setLedColor(255, 160, 0);     // Pitch negativo: Amarillo
        }
    } 
    else if (label.indexOf("yaw") >= 0) {
        if (isPos) {
            setLedColor(0, 0, 255);       // Yaw positivo: Azul
        } else if (isNeg) {
            setLedColor(120, 0, 255);     // Yaw negativo: Morado
        }
    } 
    else {
        setLedColor(0, 0, 0);             // Reposo u otra clase: Apagado
    }
}

/**
 * @brief Return the sign of the number
 */
float ei_get_sign(float number) {
    return (number >= 0.0f) ? 1.0f : -1.0f;
}

/**
 * @brief Arduino setup function
 */
void setup()
{
    Serial.begin(115200);
    // while (!Serial); // Descomenta si quieres esperar a abrir el Monitor Serie

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    setLedColor(0, 0, 0);

    Serial.println("Edge Impulse Inferencing Demo - LED RGB");

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized (Ejes por muestra: %d)\r\n", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3 &&
        EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 6 &&
        EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 9) {
        ei_printf("ERR: Numero de ejes no soportado (%d)\n", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
        return;
    }
}

/**
 * @brief Get data and run inferencing
 */
void loop()
{
    delay(200);

    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        // Acelerómetro (ejes 0, 1, 2)
        float accX, accY, accZ;
        IMU.readAcceleration(accX, accY, accZ);

        if (fabs(accX) > MAX_ACCEPTED_RANGE) accX = ei_get_sign(accX) * MAX_ACCEPTED_RANGE;
        if (fabs(accY) > MAX_ACCEPTED_RANGE) accY = ei_get_sign(accY) * MAX_ACCEPTED_RANGE;
        if (fabs(accZ) > MAX_ACCEPTED_RANGE) accZ = ei_get_sign(accZ) * MAX_ACCEPTED_RANGE;

        buffer[ix + 0] = accX * CONVERT_G_TO_MS2;
        buffer[ix + 1] = accY * CONVERT_G_TO_MS2;
        buffer[ix + 2] = accZ * CONVERT_G_TO_MS2;

        // Giroscopio si el modelo tiene 6 o 9 ejes (ejes 3, 4, 5)
        if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME >= 6) {
            float gyrX, gyrY, gyrZ;
            IMU.readGyroscope(gyrX, gyrY, gyrZ);

            buffer[ix + 3] = gyrX;
            buffer[ix + 4] = gyrY;
            buffer[ix + 5] = gyrZ;
        }

        // Magnetómetro si el modelo tiene 9 ejes (ejes 6, 7, 8)
        if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME == 9) {
            float magX, magY, magZ;
            IMU.readMagneticField(magX, magY, magZ);

            buffer[ix + 6] = magX;
            buffer[ix + 7] = magY;
            buffer[ix + 8] = magZ;
        }

        int64_t sleep_us = next_tick - micros();
        if (sleep_us > 0) {
            delayMicroseconds(sleep_us);
        }
    }

    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    size_t max_idx = 0;
    float max_val = 0.0f;

    ei_printf("Predictions:\n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        if (result.classification[ix].value > max_val) {
            max_val = result.classification[ix].value;
            max_idx = ix;
        }
    }

    actualizarLedPorPosicion(result.classification[max_idx].label, max_val);

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}

#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION)
#error "Invalid model for current sensor"
#endif