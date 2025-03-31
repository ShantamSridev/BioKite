#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

// Sensor objects
MPU6050 mpu(Wire);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// Timing
unsigned long timer = 0;

// Altitude reference
float base_altitude = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // MPU6050 Setup
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 init failed with code: ");
    Serial.println(status);
    while (1) delay(10);
  }

  Serial.println(F("Calculating gyro offsets, do not move the MPU6050..."));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("Done calibrating MPU6050!\n");

  // AHT20 Setup
  if (!aht.begin()) {
    Serial.println("Failed to find AHT20 sensor");
    while (1) delay(10);
  }

  // BMP280 Setup at address 0x77
  if (!bmp.begin(0x77)) {
    Serial.println("Failed to find BMP280 sensor at 0x77");
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // Calibrate ground level
  delay(500);  // Let the sensor stabilize a bit
  base_altitude = bmp.readAltitude(1013.25);  // You can adjust the sea level pressure if known

  Serial.println("All sensors initialized and altitude calibrated.\n");
}

void loop() {
  mpu.update();  // Update MPU6050 data

  if ((millis() - timer) > 1000) {  // Print every 1 second
    Serial.println("----- Sensor Data -----");

    // MPU6050
    float roll  = mpu.getAngleX();
    float pitch = mpu.getAngleY();
    float yaw   = mpu.getAngleZ();

    Serial.println("MPU6050 (Filtered Angles):");
    Serial.printf("  Roll  (°): %.2f\n", roll);
    Serial.printf("  Pitch (°): %.2f\n", pitch);
    Serial.printf("  Yaw   (°): %.2f\n", yaw);

    // AHT20 readings
    sensors_event_t humidity, temp_aht;
    aht.getEvent(&humidity, &temp_aht);

    // BMP280 readings
    float temp_bmp = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float current_altitude = bmp.readAltitude(1013.25);
    float relative_altitude = current_altitude - base_altitude;

    // Fused temperature
    float fused_temp = (temp_aht.temperature + temp_bmp) / 2.0;

    Serial.println("Environment:");
    Serial.printf("  Fused Temp     (°C): %.2f\n", fused_temp);
    Serial.printf("  Humidity       (%%): %.2f\n", humidity.relative_humidity);
    Serial.printf("  Pressure       (Pa): %.2f\n", pressure);
    Serial.printf("  Altitude Change (m): %.2f\n", relative_altitude);

    Serial.println("------------------------\n");

    timer = millis();
  }
}
