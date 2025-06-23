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
// UART Configuration
#define UART_TX_PIN 21  // GPIO17 (TX)
#define UART_RX_PIN 47  // GPIO16 (RX)
HardwareSerial SerialPort(2);  // Use UART1

// Sensor Data Buffer
float sensorData[6];

#define RELAY_PIN 16

const char* ssid = "YUNG";
const char* password = "yung0608";

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "yyss08686@gmail.com"
#define USER_PASSWORD "yung0608"

// Insert Firebase project API Key
#define API_KEY "AIzaSyDB70uJdPvJjHi8gvLtqR4VFpUaQzOw8f8"
// Insert Firebase storage bucket ID e.g bucket-name.appspot.com
#define STORAGE_BUCKET_ID "medcare-92416.firebasestorage.app"
// Define Firebase Data object
#define DATABASE_URL "https://medcare-92416-default-rtdb.asia-southeast1.firebasedatabase.app/"
/* 4. Define the Firebase storage bucket ID e.g bucket-name.appspot.com */


FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config2;

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

// Add right after includes
typedef struct {
    size_t size;
    size_t index;
    size_t count;
    int sum;
    int *values;
} ra_filter_t;

#define RA_FILTER_SIZE 10
static ra_filter_t ra_filter;

static ra_filter_t* ra_filter_init(ra_filter_t *filter, size_t sample_size) {
    memset(filter, 0, sizeof(ra_filter_t));
    filter->values = (int *)malloc(sample_size * sizeof(int));
    if (!filter->values) return NULL;
    memset(filter->values, 0, sample_size * sizeof(int));
    filter->size = sample_size;
    return filter;
}
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

// LED configuration
#if CONFIG_LED_ILLUMINATOR_ENABLED
int led_duty = 0;
bool isStreaming = false;
#endif
// Add these functions before setup()
std::list<dl::detect::result_t> detect_faces(camera_fb_t *fb, uint8_t *converted_buf) {
  std::list<dl::detect::result_t> results;
    
  if (fb->format == PIXFORMAT_RGB565) {
    #if TWO_STAGE
      auto candidates = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
      results = s2.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3}, candidates);
    #else
      results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
    #endif
  } else if (converted_buf) {
    #if TWO_STAGE
      auto candidates = s1.infer(converted_buf, {(int)fb->height, (int)fb->width, 3});
      results = s2.infer(converted_buf, {(int)fb->height, (int)fb->width, 3}, candidates);
    #else
      results = s1.infer(converted_buf, {(int)fb->height, (int)fb->width, 3});
    #endif
  }
  return results;
}

// Enhanced recognition function
bool recognize_faces(uint8_t *frame_data, int width, int height, std::list<dl::detect::result_t> &faces) {
  // Create tensor from frame data
  Tensor<uint8_t> tensor;
  tensor.set_element(frame_data)
        .set_shape({height, width, 3})
        .set_auto_free(false);

  for (auto &face : faces) {
    face_info_t recognition = recognizer.recognize(tensor, face.keypoint);
    if (recognition.id >= 0) {
      lcd.printf("Recognized Face ID: %d (Confidence: %.2f)\n",
                recognition.id, recognition.similarity);
      return false;
    } 
    else {
      lcd.println("Unknown Face Detected");
      return true;
    }
  }
}


#define FACE_ID_SAVE_NUMBER 7  // adjust if needed

void enroll_faces_from_sd(String folder) {
    if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 1)) {
        Serial.println("Card Mount Failed");
        return;
    }
    Serial.println("Card Mounted");

    for (int i = 0; i < 7; ++i) {
        if (recognizer.get_enrolled_id_num() >= FACE_ID_SAVE_NUMBER) {
          Serial.println("Max enrolled faces reached");
          break;
        }

        // Construct the image path: /sdcard/yung/face1.jpg ... face7.jpg
        String imagePath = "/" + folder + "/face" + i + ".jpg";
        Serial.printf("Opening %s\n", imagePath.c_str());

        File file = SD_MMC.open(imagePath.c_str());
        if (!file) {
            Serial.printf("Failed to open %s\n", imagePath.c_str());
            continue;
        }

        size_t file_size = file.size();
        uint8_t *jpg_buf = (uint8_t *)malloc(file_size);
        if (!jpg_buf) {
            Serial.println("Failed to allocate JPG buffer");
            file.close();
            continue;
        }

        file.read(jpg_buf, file_size);
        file.close();
        Serial.println("File read and closed");

        uint8_t *rgb_buf = (uint8_t *)malloc(320 * 240 * 3);
        if (!rgb_buf) {
            Serial.println("Failed to allocate RGB buffer");
            free(jpg_buf);
            continue;
        }

        if (!fmt2rgb888(jpg_buf, file_size, PIXFORMAT_JPEG, rgb_buf)) {
            Serial.println("JPEG decode failed");
            free(jpg_buf);
            free(rgb_buf);
            continue;
        }

        // Detect face
        std::list<dl::detect::result_t> &candidates = s1.infer(rgb_buf, {240, 320, 3});
        std::list<dl::detect::result_t> &results = s2.infer(rgb_buf, {240, 320, 3}, candidates);
        if (results.empty()) {
            Serial.printf("No face detected in %s\n", imagePath.c_str());
            free(jpg_buf);
            free(rgb_buf);
            continue;
        }

        // Prepare tensor
        Tensor<uint8_t> tensor;
        tensor.set_element(rgb_buf)
              .set_shape({240, 320, 3})
              .set_auto_free(false);

        // Enroll face
        int enrolled_id = recognizer.enroll_id(tensor, results.front().keypoint, "", true);
        Serial.printf("Enrolled face from %s as ID: %d\n", imagePath.c_str(), enrolled_id);

        free(jpg_buf);
        free(rgb_buf);
    }

    SD_MMC.end();
    Serial.println("SD Card unmounted");
}

// The Firebase Storage download callback function
void fcsDownloadCallback(FCS_DownloadStatusInfo info)
{
    if (info.status == firebase_fcs_download_status_init)
    {
        Serial.printf("Downloading file %s (%d) to %s\n", info.remoteFileName.c_str(), info.fileSize, info.localFileName.c_str());
    }
    else if (info.status == firebase_fcs_download_status_download)
    {
        Serial.printf("Downloaded %d%s, Elapsed time %d ms\n", (int)info.progress, "%", info.elapsedTime);
    }
    else if (info.status == firebase_fcs_download_status_complete)
    {
        Serial.println("Download completed\n");
    }
    else if (info.status == firebase_fcs_download_status_error)
    {
        Serial.printf("Download failed, %s\n", info.errorMsg.c_str());
    }
}
bool copyFile(fs::FS &srcFS, String srcPath, fs::FS &dstFS, String dstPath, String id) {
  if(!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 1)) {  // Mount SD card
    Serial.println("Card Mount Failed");
  }
  uint8_t cardType = SD_MMC.cardType();  // Get SD card type
  if(cardType == CARD_NONE){
    Serial.println("No SD_MMC card attached");
  }  
  File srcFile = srcFS.open(srcPath);
  if (!srcFile || srcFile.isDirectory()) {
    Serial.println("Source file open failed");
    return false;
  }
  if (!SD_MMC.exists("/"+id)) {
    if (SD_MMC.mkdir("/"+id)) {
      Serial.println("Created folder /"+id);
    } else {
      Serial.println("Failed to create /"+id +"folder");
    }
  }

  File dstFile = dstFS.open(dstPath, FILE_WRITE);
  if (!dstFile) {
    Serial.println("Destination file open failed");
    srcFile.close();
    return false;
  }

  Serial.println("📦 Copying...");
  uint8_t buf[512];
  size_t bytesRead;
  while ((bytesRead = srcFile.read(buf, sizeof(buf))) > 0) {
    dstFile.write(buf, bytesRead);
  }

  srcFile.close();
  dstFile.close();
  SD_MMC.end();
  Serial.println("SD Card unmounted");
  return true;
}
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  SerialPort.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
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
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  Wire.begin(4, 5);
  Wire.setClock(100000); // Explicitly set 100 kHz
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */   
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  pinMode(RELAY_PIN, OUTPUT); 

  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
    // set cursor to first column, first row
  lcd.setCursor(0, 0);
  // print message
  lcd.print("Connecting to Wi-Fi");
  delay(1000);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
 
  /* Assign the api key (required) */
  config2.api_key = API_KEY;
  config2.database_url = DATABASE_URL;
  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  /* Assign the callback function for the long running token generation task */
  config2.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
  Firebase.reconnectNetwork(true);
  config2.timeout.serverResponse = 30000;
  config2.fcs.download_buffer_size = 4096;
  fbdo.setBSSLBufferSize(4096, 1024);

  Firebase.begin(&config2, &auth);
  Firebase.reconnectWiFi(true);


  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;  
  }
  Serial.println("LittleFS Mount");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);  // Set SD card pins
  // Initialize face recognition filter
  Serial.printf("App Partition Size: %dMB\n", ESP.getFlashChipSize()/(1024*1024));
  ra_filter_init(&ra_filter, RA_FILTER_SIZE);
}
void loop() {
  if (Firebase.ready() && (millis()- sendDataPrevMillis > 5000 || sendDataPrevMillis ==0)){
    Serial.println("Reading data from firebase");
    sendDataPrevMillis = millis();
    //-----read register start?
    if(Firebase.RTDB.getBool(&fbdo, "/Medcare/register")){
      if(fbdo.dataType()=="boolean" && fbdo.boolData()){ 
        Serial.print("New Register/n");
        if(Firebase.RTDB.getString(&fbdo, "/Medcare/Userface/id")){
          if(fbdo.dataType()=="string"){
            id = fbdo.stringData();
            Serial.printf("Download %s data.../n", id);
            for(int i = 0; i <7; i++){
              String firebasePath = id + "/" + "face" + i;
              String localPath = "/" + id + "/" + "face" + i + ".jpg";                
                  // Let Firebase handle file operations automatically
              if (!Firebase.Storage.download(&fbdo, STORAGE_BUCKET_ID, 
                                                         firebasePath.c_str(), 
                                                            localPath.c_str(), 
                                               mem_storage_type_flash, 
                                                fcsDownloadCallback)){
                Serial.println(firebasePath + " download fail: " + fbdo.errorReason());
              }
              if (copyFile(LittleFS, localPath, SD_MMC, localPath, id)){
                Serial.println("Image copied to SD card");
                if (LittleFS.remove(localPath)) {
                  Serial.println("Deleted image from LittleFS");
                } 
                else {
                  Serial.println("Failed to delete image from LittleFS");
                }
              } 
              else {
                  Serial.println("❌ Copy failed");
              } 
              // Verify download after completion
              File file = SD_MMC.open(localPath);
              if(file) {
                Serial.printf("Final file size: %d bytes\n", file.size());
                file.close();
              }
            }
            if(Firebase.RTDB.setBool(&fbdo, "Medcare/register", false)){
              Serial.printf("%s info downloaded", id);
            }
          }
        }      
      }
    }
  }
  if(Firebase.RTDB.getBool(&fbdo, "Medcare/ForceStart")){
    if(fbdo.dataType() == "boolean" && fbdo.boolData()){
      Serial.println("ForcedStarting.........");
      camera_fb_t *fb = NULL;
      uint8_t *converted_buf = NULL;
      bool buffer_converted = false;
      if(Firebase.RTDB.getString(&fbdo, "Medcare/id")){
        //if(fbdo.dataType() == "String"){
          id = fbdo.stringData();
          enroll_faces_from_sd(id);
          faceRecognized = true;
          while(faceRecognized==true){
            Serial.println("Start");
            fb = esp_camera_fb_get();
            if (!fb) {
              Serial.print("no image");
              continue;}
            lcd.clear();
            // set cursor to first column, first row
            lcd.setCursor(0, 0);
            // print message
            lcd.print("Capturing");
            Serial.println("Capturing....");
            delay(10);
            // Convert frame if necessary
            if (fb->format != PIXFORMAT_RGB888) {
              converted_buf = (uint8_t*)malloc(fb->width * fb->height * 3);
              if (fmt2rgb888(fb->buf, fb->len, fb->format, converted_buf)) {
              buffer_converted = true;
              Serial.println("done converting");
              }
            }
            Serial.print("detecting face.....");
            // Step 1: Face Detection
            auto detected_faces = detect_faces(fb, converted_buf); 
            Serial.print("Done detect");
            if (!detected_faces.empty()) {
              Serial.print("face detected......");
              lcd.clear();
              // set cursor to first column, first row
              lcd.setCursor(0, 0);
              // print message
              lcd.printf("Detcted faces\n");
              delay(1000);    
              // Step 2: Face Recognition
              if (recognition_enabled) {
                faceRecognized = recognize_faces(
                buffer_converted ? converted_buf : fb->buf,
                fb->width,
                fb->height,
                detected_faces
                );
              }
            }
            else{
              Serial.print("No face Detected");
              faceRecognized = true;
            }
            Serial.println("After recognizing");
            // Cleanup
            if (buffer_converted) {
              free(converted_buf);
              buffer_converted = false;
            }
            esp_camera_fb_return(fb);
            delay(100);
            Serial.println("after free");
            Serial.println("Repeat");
          }
          for(int i = 0; i <7; i++){
            if(recognizer.delete_id(i) != ESP_FAIL){
              Serial.printf("id %d deteleted\n", i);
            }
          }
              if(Firebase.RTDB.getBool(&fbdo, "Medcare/TakePillbox1")){
                if(fbdo.boolData()){
                  sendCommand("RUN_MOTOR1,1,300");
                  waitForAck();
                  digitalWrite(RELAY_PIN, HIGH);
                  delay(100);
                  sendCommand("RUN_MOTOR2,1,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR2,0,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR1,0,300");
                  waitForAck();
                  digitalWrite(RELAY_PIN, LOW);
                  delay(100);
                }
              }
              if(Firebase.RTDB.getBool(&fbdo, "Medcare/TakePillbox2")){
                if(fbdo.boolData()){
                  sendCommand("RUN_MOTOR1,1,200");
                  waitForAck();
                  digitalWrite(RELAY_PIN, HIGH);
                  delay(100);
                  sendCommand("RUN_MOTOR2,1,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR2,0,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR1,0,200");
                  waitForAck();
                  digitalWrite(RELAY_PIN, LOW);
                  delay(100);
                }
              }
              if(Firebase.RTDB.getBool(&fbdo, "Medcare/TakePillbox3")){
                if(fbdo.boolData()){
                  sendCommand("RUN_MOTOR1,1,100");
                  waitForAck();
                  digitalWrite(RELAY_PIN, HIGH);
                  delay(100);
                  sendCommand("RUN_MOTOR2,1,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR2,0,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR1,0,100");
                  waitForAck();
                  digitalWrite(RELAY_PIN, LOW);
                  delay(100);
                }
              }
              if(Firebase.RTDB.getBool(&fbdo, "Medcare/TakePillbox4")){
                if(fbdo.boolData()){
                  sendCommand("RUN_MOTOR1,0,100");
                  waitForAck();
                  digitalWrite(RELAY_PIN, HIGH);
                  delay(100);
                  sendCommand("RUN_MOTOR2,1,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR2,0,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR1,1,100");
                  waitForAck();
                  digitalWrite(RELAY_PIN, LOW);
                  delay(100);
                }
              }
              if(Firebase.RTDB.getBool(&fbdo, "Medcare/TakePillbox5")){
                if(fbdo.boolData()){
                  sendCommand("RUN_MOTOR1,0,200");
                  waitForAck();
                  digitalWrite(RELAY_PIN, HIGH);
                  delay(100);
                  sendCommand("RUN_MOTOR2,1,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR2,0,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR1,1,200");
                  waitForAck();
                  digitalWrite(RELAY_PIN, LOW);
                  delay(100);
                }
              }
              if(Firebase.RTDB.getBool(&fbdo, "Medcare/TakePillbox6")){
                if(fbdo.boolData()){
                  sendCommand("RUN_MOTOR1,0,300");
                  waitForAck();
                  digitalWrite(RELAY_PIN, HIGH);
                delay(100);
                  sendCommand("RUN_MOTOR2,1,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR2,0,200");
                  waitForAck();
                  sendCommand("RUN_MOTOR1,1,300");
                  waitForAck();
                  digitalWrite(RELAY_PIN, LOW);
                  delay(100);
                }
              }              
          if(Firebase.RTDB.setBool(&fbdo, "Medcare/ForceStart", false)){
            Serial.print("Done yung");
          }
          delay(1000);
       // }
      }    
    }
  }
  if(Firebase.RTDB.setFloat(&fbdo, "Medcare/Temperature", bmp.readTemperature())){
    Serial.println("Uploaded Temperature");
  } 
  delay(100);
  sendCommand("REQ_HX711");
  // Wait for and parse the response
  String data = receiveData();
  if (data.length() > 0) {
    Serial.println("Received: " + data);
    if (data.startsWith("DATA,")) {
      uploadToFirebase(data);
    }
  }


}
void sendCommand(const String& cmd) {
  Serial.print("Sending: "); Serial.println(cmd);
  SerialPort.println(cmd);
}

String receiveData() {
  unsigned long start = millis();
  Serial.print("waiting.....");
  while (millis() - start < 3000) {
    if (SerialPort.available()) {
      Serial.print("receiving.....");
      String line = SerialPort.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) return line;
    }
  }
  return "";
}

void uploadToFirebase(const String& dataLine) {
  // Format: "DATA,123.45,67.89,45.67,90.12,34.56,12.34"
  int startIdx =5; // after "DATA,"
  int sensorIndex = 1;
  Serial.println("Uploading....");
  while (sensorIndex <= 1) {
    Serial.println("Uploading2....");
    int endIdx = dataLine.indexOf(',', startIdx);
    if (endIdx == -1 && sensorIndex < 1) break; // malformed

    String value;
    if (sensorIndex < 6) {
      value = dataLine.substring(startIdx, endIdx);
      startIdx = endIdx + 1;
    } else {
      value = dataLine.substring(startIdx);
    }
    Serial.println(value);
    String path = "Medcare/pillbox" + String(sensorIndex);
    if (Firebase.RTDB.setFloat(&fbdo, path, value.toFloat())) {
      Serial.println("Updated " + path + ": " + value);
    } else {
      Serial.println("Firebase error at " + path + ": " + fbdo.errorReason());
    }

    sensorIndex++;
  }
}

void waitForAck() {
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (SerialPort.available()) {
      String ack = SerialPort.readStringUntil('\n');
      if (ack == "ACK") {
        Serial.println("Motor operation acknowledged.");
        return;
      }
    }
  }
  Serial.println("ACK not received (timeout).");
}
