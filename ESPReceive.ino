#include <WiFi.h>
#include <esp_now.h>

// Struct to match sender
typedef struct struct_message {
  float pitch;
  float roll;
  float yaw;
  float temp;
  float rel_alt;
  float battery;
} SensorData;

SensorData incomingData;

// ESP-NOW receive callback
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, data, sizeof(incomingData));

    // // Print to Serial Monitor
    // Serial.println("----- Data Received via ESP-NOW -----");
    // Serial.printf("Pitch       (°): %.2f\n", incomingData.pitch);
    // Serial.printf("Roll        (°): %.2f\n", incomingData.roll);
    // Serial.printf("Yaw         (°): %.2f\n", incomingData.yaw);
    // Serial.printf("Temperature (°C): %.2f\n", incomingData.temp);
    // Serial.printf("Altitude    (m): %.2f\n", incomingData.rel_alt);
    // Serial.printf("Battery     (V): %.2f\n", incomingData.battery);
    // Serial.println("-------------------------------------\n");

    // Send formatted data to Serial2 (loopback to Serial1)
    Serial.printf("<PITCH:%.2f,ROLL:%.2f,YAW:%.2f,TEMP:%.2f,ALT:%.2f,BATT:%.2f>\n",
                   incomingData.pitch,
                   incomingData.roll,
                   incomingData.yaw,
                   incomingData.temp,
                   incomingData.rel_alt,
                   incomingData.battery);
  } else {
    Serial.println("Received data of unexpected length.");
  }
}

void setup() {
  Serial.begin(115200);  // USB serial monitor

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP32 receiver ready. Loopback test active...");
}

void loop() {
}
