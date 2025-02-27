#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
    float data;
} struct_message;

struct_message myData;

unsigned long lastTime = 0;
int packetCount = 0;

// Updated Callback Function
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    
    packetCount++;
    unsigned long currentTime = millis();
    float timeElapsed = (currentTime - lastTime) / 1000.0;

    if (timeElapsed >= 1.0) {  // Update frequency every second
        float frequency = packetCount / timeElapsed;
        Serial.print("Data Frequency: ");
        Serial.print(frequency);
        Serial.println(" packets/sec");

        lastTime = currentTime;
        packetCount = 0;
    }

    Serial.print("Received Float: ");
    Serial.println(myData.data);
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Use the updated callback signature
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    // Nothing needed here, ESP-NOW handles data reception via callback
}
