#include "Arduino.h"
#include "esp_camera.h"
#include "fb_gfx.h"
#include "dl_image.hpp"
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "face_recognition_tool.hpp"
#include "face_recognition_112_v1_s8.hpp"
#include "String.h"
#include <FS.h>
#include <Firebase_ESP_Client.h>
#include <LittleFS.h>
#include "SD_MMC.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>
#include "esp_task_wdt.h"
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <HardwareSerial.h>  
// ThingsBoard Configuration
const char* mqttServer = "thingsboard.cloud";
const char* token = "LLmv2iTL8jMUQczlMte0"; // Replace with your device token

#define UART_TX_PIN 21
#define UART_RX_PIN 47

// MQTT and WiFi clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
HardwareSerial SerialPort(2);  // Use UART1

// Sensor Data Buffer
float sensorData[6];

#define RELAY_PIN 16

unsigned long sendDataPrevMillis = 0;
String id = "";
// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  
Adafruit_BMP280 bmp; // I2C

#define SD_MMC_CMD 38 //Please do not modify it.
#define SD_MMC_CLK 39 //Please do not modify it.
#define SD_MMC_D0  40 //Please do not modify it.

// Camera configuration (adjust for your module)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

// Face recognition configuration
#define TWO_STAGE 1
#define FACE_ID_SAVE_NUMBER 7
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00

#if TWO_STAGE
HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
#else
HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
#endif

FaceRecognition112V1S8 recognizer;

int8_t recognition_enabled = 1;
bool faceRecognized = true;

SemaphoreHandle_t dataMutex;
SemaphoreHandle_t cameraMutex;
volatile float sharedTemp = 0.0;
float hx711Data[4] = {0.0, 0.0, 0.0, 0.0};
String faceEnrollID = "";
bool pillboxFlags[4] = {false, false, false, false};

bool pillboxTaskRunning = false;
bool registerFlag = false;
bool startButtonPressed = false;
static bool pillboxTrigger = false;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t pillboxTaskHandle = NULL;
TaskHandle_t faceTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;
TaskHandle_t thingsboardTaskHandle = NULL;
volatile bool faceTaskRunning = false;

// UART communication
QueueHandle_t uartCmdQueue;
QueueHandle_t uartRespQueue;

// Forward Declarations
void thingsboardTask(void*);
void sensorTask(void*);
void pillboxControlTask(void*);
void faceRecognitionTask(void*);
void uartCommunicationTask(void*);
void parseHX711Data(const String&);
std::list<dl::detect::result_t> detect_faces(camera_fb_t*, uint8_t*);
bool recognize_faces(uint8_t*, int, int, std::list<dl::detect::result_t>&, String);
void enroll_faces_from_sd(String);
String sendCommandAndWait(const String&, uint32_t timeout = 3000);
void setupWiFi();
void reconnectMQTT();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
// Enhanced recognition function

void setup() {
    Serial.begin(115200);
  Serial.println("Starting ESP32-S3 MedCare System with ThingsBoard...");
  
  // Initialize mutexes (same as original)
  dataMutex = xSemaphoreCreateMutex();
  cameraMutex = xSemaphoreCreateMutex();
  
  if (!dataMutex || !cameraMutex) {
    Serial.println("Failed to create mutexes!");
    while(1) delay(1000);
  }
  
  // Pre-allocate image buffers (same as original)
  rgbBuffer = (uint8_t*)heap_caps_malloc(RGB_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  jpgBuffer = (uint8_t*)heap_caps_malloc(JPG_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  
  if (!rgbBuffer || !jpgBuffer) {
    Serial.println("PSRAM allocation failed! Using internal RAM");
    if (rgbBuffer) heap_caps_free(rgbBuffer);
    if (jpgBuffer) heap_caps_free(jpgBuffer);
    
    rgbBuffer = (uint8_t*)malloc(RGB_BUF_SIZE);
    jpgBuffer = (uint8_t*)malloc(JPG_BUF_SIZE);
  }
  
  if (!rgbBuffer || !jpgBuffer) {
    Serial.println("CRITICAL: Memory allocation failed!");
    while(1) delay(1000);
  }
  
  // Camera initialization (same as original camera_config_t setup)
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
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  Serial.println("Camera initialized");
  
  // Initialize I2C, LCD, BMP280 (same as original)
  Wire.begin(45, 48);
  Wire.setClock(100000);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.println("LCD initialized");
  
  // WiFi connection
  setupWiFi();
  
  // ThingsBoard MQTT setup
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(onMqttMessage);
  
  // Initialize BMP280 (same as original)
  if (!bmp.begin(0x76)) {
    Serial.println(F("BMP280 sensor not found"));
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 initialized");
  }
  
  // Initialize UART (same as original)
  SerialPort.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART initialized");
  
  // Initialize file systems (same as original)
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  
  if(!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
  } else {
    Serial.println("LittleFS mounted");
  }
  
  if(!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed");
  } else {
    Serial.println("SD Card mounted");
  }
  
  // Create UART queues 
  uartCmdQueue = xQueueCreate(5, 50);
  uartRespQueue = xQueueCreate(5, 50);
  
  // Create tasks
  BaseType_t result;
  
  result = xTaskCreatePinnedToCore(thingsboardTask, "ThingsBoard", 8192, NULL, 1, &thingsboardTaskHandle, 1);
  if (result != pdPASS) Serial.println("Failed to create ThingsBoard task");
  
  result = xTaskCreatePinnedToCore(sensorTask, "Sensor", 8192, NULL, 2, &sensorTaskHandle, 1);
  if (result != pdPASS) Serial.println("Failed to create Sensor task");
  
  result = xTaskCreatePinnedToCore(uartCommunicationTask, "UART", 4096, NULL, 3, &uartTaskHandle, 1);
  if (result != pdPASS) Serial.println("Failed to create UART task");
  
  Serial.println("Setup completed!");
}

void loop() {
    static uint32_t lastDisplay = 0;
  
  if (millis() - lastDisplay > 5000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(sharedTemp, 1);
    lcd.print("C");
    
    lastDisplay = millis();
  }
  
  // Face recognition trigger - when register or start button pressed
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10))) {
    bool shouldStartFace = !faceTaskRunning && (registerFlag || startButtonPressed);
    xSemaphoreGive(dataMutex);

    if (shouldStartFace) {
      Serial.println("Starting face recognition task");
      faceTaskRunning = true;
      
      BaseType_t result = xTaskCreatePinnedToCore(
        faceRecognitionTask, 
        "FaceRecog", 
        32768,
        NULL, 
        4,
        &faceTaskHandle, 
        1
      );
      
      if (result != pdPASS) {
        Serial.println("Failed to create face recognition task");
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
          faceTaskRunning = false;
          registerFlag = false;
          startButtonPressed = false;
          xSemaphoreGive(dataMutex);
        }
      }
    }
    if (pillboxTrigger && !pillboxTaskRunning) {
  pillboxTaskRunning = true;
  pillboxTrigger = false;

  BaseType_t result = xTaskCreatePinnedToCore(
    pillboxControlTask,
    "PillboxControl",
    8192,
    NULL,
    2,
    &pillboxTaskHandle,
    1
  );

  if (result != pdPASS) {
    Serial.println("Failed to create PillboxControl task from loop");
    pillboxTaskRunning = false;
  } else {
    Serial.println("PillboxControl task created from loop");
  }
}
  }
  
  esp_task_wdt_reset();
  delay(100);
  }

// WiFi setup function
void setupWiFi() {
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print("WiFi....");
  
  WiFiManager wm;
  wm.setConnectTimeout(60);
  wm.setConfigPortalTimeout(180);
  
  bool res = wm.autoConnect("ESP32_AP", "12345678");
  
  if (!res) {
    Serial.println("Failed to connect.");
    lcd.clear();
    lcd.print("WiFi Failed!");
    delay(3000);
    ESP.restart();
  } else {
    Serial.println("Connected!");
    Serial.println(WiFi.localIP());
    lcd.clear();
    lcd.print("Connected!");
    delay(1000);
  }
}

// MQTT reconnection function
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), token, "")) {
      Serial.println("connected");
      // Subscribe to RPC commands
      mqttClient.subscribe("v1/devices/me/rpc/request/+");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - trying again in 5 seconds");
      delay(5000);
    }
  }
}

// MQTT message callback - handles register and start button commands
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = String((char*)payload);
  String topicStr = String(topic);
  
  Serial.println("Received: " + topicStr + " - " + message);
  
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, message);
  
  String method = doc["method"];
  
  if (method == "register") {
    String username = doc["params"]["username"];
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
      faceEnrollID = username;
      registerFlag = true;
      xSemaphoreGive(dataMutex);
    }
    
    // Send response
    String responsePayload = "{\"success\":true}";
    String responseTopic = topicStr;
    responseTopic.replace("request", "response");
    mqttClient.publish(responseTopic.c_str(), responsePayload.c_str());
    
  } else if (method == "startButton") {
    int pillboxId = doc["params"]["pillboxId"];
    String username = doc["params"]["username"];
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
      faceEnrollID = username;
      startButtonPressed = true;
      for (int i = 0; i < 4; i++) pillboxFlags[i] = false;
      if (pillboxId >= 1 && pillboxId <= 4) {
        pillboxFlags[pillboxId - 1] = true;
      }
      xSemaphoreGive(dataMutex);
    }
    
    // Send response
    String responsePayload = "{\"success\":true}";
    String responseTopic = topicStr;
    responseTopic.replace("request", "response");
    mqttClient.publish(responseTopic.c_str(), responsePayload.c_str());
  }
  else if (method == "getUserList") {
  DynamicJsonDocument respDoc(512);
  JsonArray arr = respDoc.to<JsonArray>();

  File root = SD_MMC.open("/");
  if (root && root.isDirectory()) {
    File file = root.openNextFile();
    while (file) {
      if (file.isDirectory()) {
    String folderName = String(file.name()).substring(1); // remove leading slash
    if (folderName.startsWith("user_")) {
        arr.add(folderName.substring(5)); // Remove "user_" prefix
    }
}

      file = root.openNextFile();
    }
  }

  String responsePayload;
  serializeJson(respDoc, responsePayload);

  String responseTopic = topicStr;
  responseTopic.replace("request", "response");
  mqttClient.publish(responseTopic.c_str(), responsePayload.c_str());
}
}

// ThingsBoard task - replaces Firebase task
void thingsboardTask(void* pv) {
  for (;;) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();
    
    // Upload sensor data every 5 seconds
    static uint32_t lastUpload = 0;
    if (millis() - lastUpload > 5000) { 
      float tempCopy;
      float hxCopy[4];
      
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
        tempCopy = sharedTemp;
        memcpy(hxCopy, hx711Data, sizeof(hxCopy));
        xSemaphoreGive(dataMutex);
      } else {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        continue;
      }
      
      // Create JSON payload
      char payload[256];
      snprintf(payload, sizeof(payload), 
        "{\"temperature\":%.2f,\"pillbox1\":%.2f,\"pillbox2\":%.2f,\"pillbox3\":%.2f,\"pillbox4\":%.2f}",
        tempCopy, hxCopy[0], hxCopy[1], hxCopy[2], hxCopy[3]);
      
      if (mqttClient.publish("v1/devices/me/telemetry", payload)) {
        Serial.println("Telemetry data published");
      } else {
        Serial.println("Publish failed");
      }
      
      lastUpload = millis();
    }
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// Sensor task (same as original)
void sensorTask(void* pv) {
  for (;;) {
    float temp = 0.0;
    if (bmp.begin(0x76)) {
      temp = bmp.readTemperature();
    }
    
    String response = sendCommandAndWait("REQ_HX711", 3000);
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
      sharedTemp = temp;
      if (!response.isEmpty() && response != "TIMEOUT") {
        parseHX711Data(response);
      }
      xSemaphoreGive(dataMutex);
    }
    Serial.printf("Temperatue: %.2f", temp);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
// Face recognition task - modified registration mode
void faceRecognitionTask(void* pv) {
  // Suspend other tasks for better performance
  if (thingsboardTaskHandle != NULL) {
    vTaskSuspend(thingsboardTaskHandle);
    Serial.println("Thingsboard task suspended");
  }
  if (sensorTaskHandle != NULL) {
    vTaskSuspend(sensorTaskHandle);
    Serial.println("Sensor task suspended");
  }
  Serial.println("Face recognition task started");
  
  bool isRegisterMode = registerFlag;
  
  if (isRegisterMode) {
    // Register mode - capture and save 7 JPEG images ONLY when face is detected
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Registration");
    lcd.setCursor(0, 1);
    lcd.print("Mode");
    delay(2000);

    // Create directory for user
    String userDir = "/user_" + faceEnrollID;
    Serial.println("Checking SD folder: " + userDir);
    if (!SD_MMC.exists(userDir)) {
      SD_MMC.mkdir(userDir);
    }

    int capturedImages = 0;
    const uint32_t registrationTimeout = 60000; // 60 seconds timeout
    uint32_t startTime = millis();

    while (capturedImages < 7 && (millis() - startTime < registrationTimeout)) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Look at camera");
      lcd.setCursor(0, 1);
      lcd.print("Images: ");
      lcd.print(capturedImages);
      lcd.print("/7");
      Serial.println("Capturing image...");

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        continue;
      }

      // Convert frame to RGB for face detection
      uint8_t *rgb_buf = (uint8_t *)malloc(fb->width * fb->height * 3);
      if (!rgb_buf) {
        esp_camera_fb_return(fb);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        continue;
      }

      bool faceDetected = false;
      if (fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf)) {
        auto faces = detect_faces(fb, rgb_buf);

        if (!faces.empty()) {
          faceDetected = true;
          Serial.println("Face detected for registration!");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Face Detected!");
          lcd.setCursor(0, 1);
          lcd.print("Saving...");

          // Convert to JPEG before saving
          uint8_t *jpg_buf = NULL;
          size_t jpg_len = 0;
          bool success = frame2jpg(fb, 80, &jpg_buf, &jpg_len); // 80 = JPEG quality

          if (success) {
            String filename = userDir + "/face" + String(capturedImages) + ".jpg";
            File file = SD_MMC.open(filename, FILE_WRITE);
            if (file) {
              file.write(jpg_buf, jpg_len);
              file.close();
              Serial.printf("Saved JPEG: %s (%d bytes)\n", filename.c_str(), jpg_len);

              capturedImages++;

              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Image ");
              lcd.print(capturedImages);
              lcd.print(" saved!");
              lcd.setCursor(0, 1);
              lcd.print("Move slightly");
              delay(1500);
            } else {
              Serial.println("Failed to open file for writing: " + filename);
            }
            free(jpg_buf);
          } else {
            Serial.println("JPEG conversion failed");
          }
        }
      }

      free(rgb_buf);
      esp_camera_fb_return(fb);

      if (!faceDetected) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
      } else {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    }

    // Registration result
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Registration");
    if (capturedImages >= 7) {
      lcd.setCursor(0, 1);
      lcd.print("Complete!");
      Serial.println("Registration completed successfully with " + String(capturedImages) + " images");
// Alternative: Direct directory listing approach
String allUsers = "";

// Method 1: Using listDir function if available
Serial.println("Scanning SD card root directory...");

File root = SD_MMC.open("/");
if (!root) {
  Serial.println("Failed to open root directory");
} else if (!root.isDirectory()) {
  Serial.println("Root is not a directory");
  root.close();
} else {
  Serial.println("Root directory opened successfully");
  
  // Reset and scan
  root.rewindDirectory();
  
  File file = root.openNextFile();
  int fileCount = 0;
  
  while (file) {
    fileCount++;
    String fileName = String(file.name());
    Serial.printf("File %d: %s (isDir: %s)\n", fileCount, fileName.c_str(), file.isDirectory() ? "YES" : "NO");
    
    if (file.isDirectory()) {
      // Remove leading slash if present
      String folderName = fileName;
      if (folderName.startsWith("/")) {
        folderName = folderName.substring(1);
      }
      
      // Check for user_ prefix
      if (folderName.startsWith("user_") && folderName.length() > 5) {
        String username = folderName.substring(5);
        Serial.println("Extracting username: " + username);
        
        if (!allUsers.isEmpty()) allUsers += ", ";
        allUsers += username;
      }
    }
    
    file.close();
    file = root.openNextFile();
  }
  
  root.close();
  Serial.printf("Scanned %d items total\n", fileCount);
}

// Debug output
Serial.println("Final allUsers string: '" + allUsers + "'");

// Now send as telemetry to ThingsBoard
String telemetryPayload = "{\"userList\":\"" + allUsers + "\", \"last_registered_user\":\"" + faceEnrollID + "\"}";
mqttClient.publish("v1/devices/me/telemetry", telemetryPayload.c_str());


    } else {
      lcd.setCursor(0, 1);
      lcd.print("Timeout!");
      Serial.println("Registration timeout - only captured " + String(capturedImages) + " images");
    }

    delay(3000);
  } else {
    // Recognition mode - same as original recognize logic
    enroll_faces_from_sd(faceEnrollID);
    
    camera_fb_t *fb;
    uint8_t *buf;
    bool ok = false;
    const uint32_t timeout = 30000;
    uint32_t startTime = millis();
    
    Serial.println("Starting live face recognition...");
    
    while (!ok && (millis() - startTime < timeout)) {
      fb = esp_camera_fb_get();
      if (!fb) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        continue;
      }
      
      buf = (uint8_t*)malloc(fb->width * fb->height * 3);
      if (!buf) {
        esp_camera_fb_return(fb);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        continue;
      }
      
      if (fmt2rgb888(fb->buf, fb->len, fb->format, buf)) {
        auto faces = detect_faces(fb, buf);
        
        if (!faces.empty()) {
          Serial.println("Face Detected!");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Face Detected");
          lcd.setCursor(0, 1);
          lcd.print("Recognizing...");
          
          ok = recognize_faces(buf, fb->width, fb->height, faces, faceEnrollID);
          
          if (ok) {
            Serial.println("Face recognition successful!");
          }
        }
      }
      
      free(buf);
      esp_camera_fb_return(fb);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    // Cleanup enrolled faces
    for (int i = 0; i < 7; i++) {
      recognizer.delete_id(i);
    }
    
    if (ok && startButtonPressed) {
      pillboxTrigger = true;
    }

  }
  
  // Reset flags
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000))) {

    faceEnrollID = "";
    faceTaskRunning = false;
    registerFlag = false;
    startButtonPressed = false;
    xSemaphoreGive(dataMutex);
  }
  // Resume suspended tasks
  if (thingsboardTaskHandle != NULL) {
    vTaskResume(thingsboardTaskHandle);
    Serial.println("Thingsboard task resumed");
  }
  if (sensorTaskHandle != NULL) {
    vTaskResume(sensorTaskHandle);
    Serial.println("Sensor task resumed");
  }
  
  Serial.println("Face recognition task completed");
  
  // Clear task handle and delete task
  faceTaskHandle = NULL;
  vTaskDelete(NULL);
}
// Simplified Pillbox Control Task
void pillboxControlTask(void* pv) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dispensing");
  lcd.setCursor(0, 1);
  lcd.print("Pills...");
  Serial.println("Dispensing pills");
  for (int i = 0; i < 4; i++) {
    if (pillboxFlags[i]) {
      Serial.printf("Dispensing from pillbox %d\n", i+1);
      
      String command = "PILLBOX_" + String(i+1);
      String response = sendCommandAndWait(command, 5000);
      
      if (response == "COMPLETE") {
        Serial.printf("Pillbox %d dispensed successfully\n", i+1);
      } else {
        Serial.println("Dispense failed: " + response);
      }
    }
  }
  
  for (int i = 0; i < 4; i++) pillboxFlags[i] = false;

  Serial.println("Dispensing Complete");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dispensing");
  lcd.setCursor(0, 1);
  lcd.print("Complete");
  delay(3000);
  
  pillboxTaskRunning = false;
  vTaskDelete(NULL);
}

// Keep original UART, face detection, and utility functions...
// (uartCommunicationTask, sendCommandAndWait, parseHX711Data, 
//  detect_faces, recognize_faces, enroll_faces_from_sd functions remain the same)
// ======================== UART COMMUNICATION ======================== //

void uartCommunicationTask(void* pv) {
  char cmdBuf[50];
  char respBuf[50];
  
  for (;;) {
    if (xQueueReceive(uartCmdQueue, cmdBuf, portMAX_DELAY)) {
      SerialPort.println(cmdBuf);
      Serial.println("UART Sent: " + String(cmdBuf));
      
      // Wait for response
      uint32_t startTime = millis();
      bool responseReceived = false;
      String response = "";
      
      while (millis() - startTime < 3000 && !responseReceived) {
        if (SerialPort.available()) {
          response = SerialPort.readStringUntil('\n');
          response.trim();
          if (!response.isEmpty()) {
            response.toCharArray(respBuf, sizeof(respBuf));
            xQueueSend(uartRespQueue, respBuf, 0);
            responseReceived = true;
            Serial.println("UART Received: " + response);
          }
        }
        delay(10);
      }
      
      if (!responseReceived) {
        strcpy(respBuf, "TIMEOUT");
        xQueueSend(uartRespQueue, respBuf, 0);
      }
    }
  }
}

String sendCommandAndWait(const String& cmd, uint32_t timeout) {
  char cmdBuf[50];
  char respBuf[50];
  cmd.toCharArray(cmdBuf, sizeof(cmdBuf));
  
  String response = "TIMEOUT";
  
  if (xQueueSend(uartCmdQueue, cmdBuf, pdMS_TO_TICKS(100))) {
    if (xQueueReceive(uartRespQueue, respBuf, pdMS_TO_TICKS(timeout))) {
      response = String(respBuf);
    }
  }
  
  return response;
}
std::list<dl::detect::result_t> detect_faces(camera_fb_t *fb,uint8_t *buf){
  std::list<dl::detect::result_t> r;
  auto c = s1.infer(buf,{fb->height,fb->width,3});
    r = s2.infer(buf,{fb->height,fb->width,3},c);
  return r;
}
bool recognize_faces(uint8_t *frame_data,int w,int h,std::list<dl::detect::result_t> &faces, String id){

  Tensor<uint8_t> t; t.set_element(frame_data).set_shape({h,w,3}).set_auto_free(false);
  for(auto &f:faces){
    face_info_t info = recognizer.recognize(t,f.keypoint);
    if(info.id>=0){
      Serial.printf("Recognized ID %s (%.2f)\n", id,info.similarity);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Face Detected");
      lcd.setCursor(0, 1);
      lcd.print("ID:");
      lcd.print(id);
      lcd.print(" C:");
      lcd.printf("%.2f", info.similarity); // Show confidence as int
      delay(30);
      return true;
    }
    else{
      Serial.println("Unknown face");
    }
  }
  return false;
}


void enroll_faces_from_sd(String folder) {
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  // Create folder if not exists
  if (!SD_MMC.exists("/user_" + folder)) {
    SD_MMC.mkdir("/user_" + folder);
  }
  
  for (int i = 0; i < 7; ++i) {
    String imagePath = "/user_" + folder + "/face" + i + ".jpg";
    
    if (!SD_MMC.exists(imagePath)) {
      Serial.printf("File not found: %s\n", imagePath.c_str());
      continue;
    }
    
    File file = SD_MMC.open(imagePath);
    if (!file) {
      Serial.printf("Failed to open %s\n", imagePath.c_str());
      continue;
    }
    
    size_t file_size = file.size();
    if (file_size > JPG_BUF_SIZE) {
      Serial.printf("File too large: %s (%d bytes)\n", imagePath.c_str(), file_size);
      file.close();
      continue;
    }
    
    file.read(jpgBuffer, file_size);
    file.close();
    
    // Convert JPEG to RGB
    if (!fmt2rgb888(jpgBuffer, file_size, PIXFORMAT_JPEG, rgbBuffer)) {
      Serial.println("JPEG decode failed");
      continue;
    }
    
    // Detect faces
    auto candidates = s1.infer(rgbBuffer, {240, 320, 3});
    auto results = s2.infer(rgbBuffer, {240, 320, 3}, candidates);
    
    if (results.empty()) {
      Serial.printf("No face detected in %s\n", imagePath.c_str());
      continue;
    }
    
    // Enroll first face found
    Tensor<uint8_t> tensor;
    tensor.set_element(rgbBuffer).set_shape({240, 320, 3}).set_auto_free(false);
    
    for (auto &face : results) {
      int enrolled_id = recognizer.enroll_id(tensor, face.keypoint, "", true);
        Serial.printf("Enrolled face from %s as ID: %d\n", imagePath.c_str(), enrolled_id);
        face_info_t recognition = recognizer.recognize(tensor, face.keypoint);
        if (recognition.id >= 0) {
            Serial.printf("Recognized Face ID: %d (Confidence: %.2f)\n",
                        recognition.id, recognition.similarity);
        } else {
            Serial.println("Unknown Face Detected");
        }
    }
  }
  SD_MMC.end();
}
void parseHX711Data(const String& dataLine) {
  if (!dataLine.startsWith("DATA,")) {
    Serial.println("Invalid HX711 data format");
    return;
  }
  
  int startIdx = 5; // Skip "DATA,"
  int sensorIndex = 0;
  int lastComma = startIdx;
  
  while (sensorIndex < 4 && lastComma < dataLine.length()) {
    int nextComma = dataLine.indexOf(',', lastComma + 1);
    if (nextComma == -1) nextComma = dataLine.length();
    
    String valueStr = dataLine.substring(lastComma, nextComma);
    hx711Data[sensorIndex++] = valueStr.toFloat();
    lastComma = nextComma + 1;
  }
}
