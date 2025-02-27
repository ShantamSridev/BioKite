#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C  // I2C address for SSD1306

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Data structure matching the sender
typedef struct struct_message {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    unsigned long timestamp;
} struct_message;

struct_message myData;

unsigned long lastTime = 0;
int packetCount = 0;

// Callback function for receiving data
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));

    packetCount++;
    unsigned long currentTime = millis();
    float timeElapsed = (currentTime - lastTime) / 1000.0;

    if (timeElapsed >= 1.0) {  // Every second
        float frequency = packetCount / timeElapsed;
        Serial.print("ESP-NOW Data Frequency: ");
        Serial.print(frequency);
        Serial.println(" packets/sec");

        lastTime = currentTime;
        packetCount = 0;
    }

    Serial.println("Received IMU Data:");
    Serial.print("Accel X: "); Serial.println(myData.accelX);
    Serial.print("Accel Y: "); Serial.println(myData.accelY);
    Serial.print("Accel Z: "); Serial.println(myData.accelZ);
    Serial.print("Gyro X: "); Serial.println(myData.gyroX);
    Serial.print("Gyro Y: "); Serial.println(myData.gyroY);
    Serial.print("Gyro Z: "); Serial.println(myData.gyroZ);

    // Display on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("IMU Data:");

    display.setCursor(0, 10);
    display.print("AX: "); display.print(myData.accelX, 2);

    display.setCursor(0, 20);
    display.print("AY: "); display.print(myData.accelY, 2);

    display.setCursor(0, 30);
    display.print("AZ: "); display.print(myData.accelZ, 2);

    display.setCursor(0, 40);
    display.print("GX: "); display.print(myData.gyroX, 2);

    display.setCursor(0, 50);
    display.print("GY: "); display.print(myData.gyroY, 2);

    display.display();
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        return;
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Waiting for Data...");
    display.display();

    lastTime = millis();
}

void loop() {
    // Nothing here, ESP-NOW handles receiving data asynchronously
}
