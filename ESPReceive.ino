#include <WiFi.h>
#include <esp_now.h>

// Struct to match sender
typedef struct struct_message {
  float pitch;
  float roll;
  float yaw;
  float temp;
  float rel_alt;
} SensorData;

SensorData incomingData;

// New-style ESP-NOW callback for Arduino Core 3.x
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, data, sizeof(incomingData));

    // Print to Serial Monitor (debug)
    Serial.println("----- Data Received -----");
    Serial.printf("Pitch       (°): %.2f\n", incomingData.pitch);
    Serial.printf("Roll        (°): %.2f\n", incomingData.roll);
    Serial.printf("Yaw         (°): %.2f\n", incomingData.yaw);
    Serial.printf("Temperature (°C): %.2f\n", incomingData.temp);
    Serial.printf("Altitude    (m): %.2f\n", incomingData.rel_alt);
    Serial.println("--------------------------\n");

    // Send formatted data packet to the Pico via UART
    Serial1.printf("<PITCH:%.2f,ROLL:%.2f,YAW:%.2f,TEMP:%.2f,ALT:%.2f>\n",
                   incomingData.pitch,
                   incomingData.roll,
                   incomingData.yaw,
                   incomingData.temp,
                   incomingData.rel_alt);
  } else {
    Serial.println("Received data of unexpected length.");
  }
}

void setup() {
  Serial.begin(115200);    // USB serial for debugging
  Serial1.begin(115200);   // UART to Pico (e.g., TX = GPIO17, RX = GPIO16)

  // Init Wi-Fi and ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP32 receiver ready. Forwarding to Pico...");
}

void loop() {
  // Nothing needed — data is handled in callback
}
