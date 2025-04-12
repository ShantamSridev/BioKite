#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// ----- Workaround for conflicting sensor_t -----
// Before including the ESP32 camera header, redefine sensor_t.
#define sensor_t esp32_camera_sensor_t
#include "esp_camera.h"
#undef sensor_t
// --------------------------------------------------

#include <Adafruit_Sensor.h>

// CAMERA PINS for XIAO ESP32S3 Sense
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39

#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

const char *ssid = "WifiNetwork1";
const char *password = "simplePassword";


// SENSORS
MPU6050 mpu(Wire);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// TIMING
unsigned long timer = 0;
float base_altitude = 0;

// BATTERY
const int BATTERY_PIN = A0;
const float VOLTAGE_DIVIDER_RATIO = 2.0;
const float ADC_MAX = 4095.0;
const float REF_VOLTAGE = 3.3;

// STRUCTURE TO SEND
typedef struct struct_message {
  float pitch;
  float roll;
  float yaw;
  float temp;
  float rel_alt;
  float battery;
} SensorData;

SensorData dataToSend;

// ESP-NOW Receiver MAC Address
uint8_t receiverAddress[] = {0x30, 0xC9, 0x22, 0x12, 0xED, 0xE0};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// CAMERA STREAMING: Web server on port 80
WebServer server(80);

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Choose a lower resolution for faster streaming if needed.
  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return false;
  }
  return true;
}

void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) continue;

    response = "--frame\r\n";
    response += "Content-Type: image/jpeg\r\n\r\n";
    server.sendContent(response);
    server.sendContent((const char *)fb->buf, fb->len);
    server.sendContent("\r\n");
    
    esp_camera_fb_return(fb);
    delay(50); // roughly limit to ~20 FPS
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // INIT SENSORS
  if (mpu.begin() != 0) {
    Serial.println("MPU6050 init failed");
    while (1) delay(10);
  }
  Serial.println("Calculating gyro offsets...");
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("MPU6050 ready!");

  if (!aht.begin()) {
    Serial.println("AHT20 not found");
    while (1) delay(10);
  }

  if (!bmp.begin(0x77)) {
    Serial.println("BMP280 not found");
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  delay(500);
  base_altitude = bmp.readAltitude(1013.25);

  // INIT WIFI (for HTTP streaming)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // Replace with your Wi-Fi credentials
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // INIT ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("ESP-NOW peer added");
  } else {
    Serial.println("ESP-NOW peer add failed");
  }

  // INIT CAMERA
  if (!initCamera()) {
    Serial.println("Camera failed to initialize");
  } else {
    server.on("/stream", HTTP_GET, handleStream);
    server.begin();
    Serial.println("Stream available at http://<device-ip>/stream");
  }

  Serial.println("System initialized and ready!");
}

void loop() {
  mpu.update();
  server.handleClient(); // Handle incoming HTTP stream requests

  if ((millis() - timer) > 1000) {
    timer = millis();

    // SENSOR READS
    dataToSend.roll  = mpu.getAngleX();
    dataToSend.pitch = mpu.getAngleY();
    dataToSend.yaw   = mpu.getAngleZ();

    sensors_event_t humidity, temp_aht;
    aht.getEvent(&humidity, &temp_aht);

    float temp_bmp = bmp.readTemperature();
    float current_alt = bmp.readAltitude(1013.25);
    dataToSend.rel_alt = current_alt - base_altitude;
    dataToSend.temp = (temp_aht.temperature + temp_bmp) / 2.0;

    int adc_reading = analogRead(BATTERY_PIN);
    dataToSend.battery = (adc_reading / ADC_MAX) * REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO;

    esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));

    // DEBUG OUTPUT
    Serial.println("Sending Data:");
    Serial.printf("  Pitch: %.2f  Roll: %.2f  Yaw: %.2f\n", dataToSend.pitch, dataToSend.roll, dataToSend.yaw);
    Serial.printf("  Temp:  %.2f  Rel Alt: %.2f\n", dataToSend.temp, dataToSend.rel_alt);
    Serial.printf("  Battery Voltage: %.2f V\n\n", dataToSend.battery);
  }
}
