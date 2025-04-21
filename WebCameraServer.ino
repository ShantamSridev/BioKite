// —————— Rename camera driver’s sensor_t → esp_sensor_t ——————
#define sensor_t esp_sensor_t
#include "esp_camera.h"
#undef sensor_t

#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_light.h>

// —————— Rename Adafruit Unified Sensor’s sensor_t → adafruit_sensor_t ——————
#define sensor_t adafruit_sensor_t
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#undef sensor_t

#define CAMERA_MODEL_XIAO_ESP32S3
#include "camera_pins.h"
#include <esp_now.h>


const char* ssid = "ESP32-CAM";
const char* password = "123456789";

void startCameraServer();
void setupLedFlash(int pin);

// MPU, AHT20, BMP280
MPU6050 mpu(Wire);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// Timing
unsigned long timer = 0;
float base_altitude = 0;

// Battery voltage reading
const int BATTERY_PIN = A0;
const float VOLTAGE_DIVIDER_RATIO = 2.0; // Because 220k / (220k + 220k) = 0.5
const float ADC_MAX = 4095.0;
const float REF_VOLTAGE = 3.3; // Reference voltage for ESP32S3 ADC


// Structure to send
typedef struct struct_message {
  float pitch;
  float roll;
  float yaw;
  float temp;
  float rel_alt;
  float battery; // New field for battery voltage
} SensorData;

// Receiver MAC Address
uint8_t receiverAddress[] = {0x30, 0xC9, 0x22, 0x12, 0xED, 0xE0};

// ESP-NOW send status callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void sensor_setup(){
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

void sensorTask(){
    SensorData data;
    // Read angles from MPU6050
    mpu.update();
    data.roll  = mpu.getAngleX();
    data.pitch = mpu.getAngleY();
    data.yaw   = mpu.getAngleZ();

    // Read temperature from AHT20
    sensors_event_t humidity, temp_aht;
    aht.getEvent(&humidity, &temp_aht);

    // BMP280 readings
    float temp_bmp = bmp.readTemperature();
    float current_alt = bmp.readAltitude(1013.25);
    data.rel_alt = current_alt - base_altitude;

    // Fused temperature
    data.temp = (temp_aht.temperature + temp_bmp) / 2.0;

    // Read battery voltage
    int adc_reading = analogRead(BATTERY_PIN);
    data.battery = (adc_reading / ADC_MAX) * REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO;

    // Send via ESP-NOW
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&data, sizeof(data));

    Serial.println("Sending Data:");
    Serial.printf("  Pitch: %.2f  Roll: %.2f  Yaw: %.2f\n", data.pitch, data.roll, data.yaw);
    Serial.printf("  Temp:  %.2f  Rel Alt: %.2f\n", data.temp, data.rel_alt);
    Serial.printf("  Battery Voltage: %.2f V\n\n", data.battery);
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  esp_sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password, 1);
  sensor_setup();
  startCameraServer();

  Serial.println("Camera server ready at: http://192.168.4.1");
  //Serial.print(WiFi.localIP());
}

void loop() {
  static unsigned long lastSensorSend = 0;

  // Always update MPU
  mpu.update();

  // Send other sensor data every 500ms
  if (millis() - lastSensorSend >= 500) {
    lastSensorSend = millis();
    sensorTask();  // This still calls mpu.getAngleX/Y/Z — fine
  }
}
