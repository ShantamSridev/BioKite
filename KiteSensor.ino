#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>

// MPU, AHT20, BMP280
MPU6050 mpu(Wire);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// Timing
unsigned long timer = 0;
float base_altitude = 0;

// Structure to send
typedef struct struct_message {
  float pitch;
  float roll;
  float yaw;
  float temp;
  float rel_alt;
} SensorData;

SensorData dataToSend;

// Receiver MAC Address
uint8_t receiverAddress[] = {0x30, 0xC9, 0x22, 0x12, 0xED, 0xE0};

// ESP-NOW send status callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 init failed with code: ");
    Serial.println(status);
    while (1) delay(10);
  }

  Serial.println(F("Calculating gyro offsets..."));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("MPU6050 ready!");

  // Initialize AHT20
  if (!aht.begin()) {
    Serial.println("AHT20 not found");
    while (1) delay(10);
  }

  // Initialize BMP280 at 0x77
  if (!bmp.begin(0x77)) {
    Serial.println("BMP280 not found at 0x77");
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  delay(500); // Allow sensor to stabilize
  base_altitude = bmp.readAltitude(1013.25);

  // Init Wi-Fi in Station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("Peer added");
  } else {
    Serial.println("Failed to add peer");
  }

  Serial.println("System initialized and ready to send data!\n");
}

void loop() {
  mpu.update();

  if ((millis() - timer) > 1000) {
    timer = millis();

    // Read angles from MPU6050
    dataToSend.roll  = mpu.getAngleX();
    dataToSend.pitch = mpu.getAngleY();
    dataToSend.yaw   = mpu.getAngleZ();

    // Read temperature from AHT20
    sensors_event_t humidity, temp_aht;
    aht.getEvent(&humidity, &temp_aht);

    // BMP280 readings
    float temp_bmp = bmp.readTemperature();
    float current_alt = bmp.readAltitude(1013.25);
    dataToSend.rel_alt = current_alt - base_altitude;

    // Fused temperature
    dataToSend.temp = (temp_aht.temperature + temp_bmp) / 2.0;

    // Send via ESP-NOW
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));

    Serial.println("Sending Data:");
    Serial.printf("  Pitch: %.2f  Roll: %.2f  Yaw: %.2f\n", dataToSend.pitch, dataToSend.roll, dataToSend.yaw);
    Serial.printf("  Temp:  %.2f  Rel Alt: %.2f\n\n", dataToSend.temp, dataToSend.rel_alt);
  }
}
