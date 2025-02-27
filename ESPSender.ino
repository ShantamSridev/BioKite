#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xd4, 0xd4, 0xda, 0x59, 0xd3, 0xa4};

// Structure to send only one float value
typedef struct struct_message {
    float data;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    myData.data = random(0, 100) / 10.0; // Generate random float value

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result != ESP_OK) {
        Serial.println("Error sending data");
    }

    delay(1); // Minimize delay to maximize transmission speed
}
