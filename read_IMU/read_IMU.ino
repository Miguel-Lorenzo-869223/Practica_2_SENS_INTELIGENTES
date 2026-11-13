// ------------------------------------------------------------
//  Author: Miguel A.Lorenzo
//  Date: 13/11/2026
//  Subject: Smart Sensors
//  Master: MSc in Electronic Engineering
//  University: University of Zaragoza EINA/UNIZAR
// ------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_LSM9DS1.h>

#define CONVERT_G_TO_MS2 9.80665f

// Dynamic data variables
float ax, ay, az;
float gx, gy, gz;

unsigned long pv_millis_sample = 0;
const long interval_sample = 20; // 50 Hz (20 ms)

// FUNCTIONS DECLARATION
void read_imu();
void printer(unsigned long timestamp);

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial && millis() < 4000);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Cabecera exacta de 7 columnas
  Serial.println("timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ");
}

void loop() {
  unsigned long current_millis = millis();

  if (current_millis - pv_millis_sample >= interval_sample) {
    pv_millis_sample = current_millis;
    read_imu();
    printer(current_millis);
  }
}

void read_imu() {
  // 1. Read Accelerometer (converted to m/s^2)
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    ax *= CONVERT_G_TO_MS2;
    ay *= CONVERT_G_TO_MS2;
    az *= CONVERT_G_TO_MS2;
  }
  // 2. Read Gyroscope (dps)
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
}

void printer(unsigned long timestamp) {
  Serial.print(timestamp); Serial.print(",");
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.println(gz, 4);
}