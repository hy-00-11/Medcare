#include "esp_camera.h"
#include "dl_image.hpp"
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "face_recognition_112_v1_s8.hpp"
#include "fb_gfx.h"
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
bool is_enrolling = true;
int8_t recognition_enabled = 1;



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
void recognize_faces(uint8_t *frame_data, int width, int height, std::list<dl::detect::result_t> &faces) {
    // Create tensor from frame data
    Tensor<uint8_t> tensor;
    tensor.set_element(frame_data)
          .set_shape({height, width, 3})
          .set_auto_free(false);

    for (auto &face : faces) {
        if (is_enrolling && recognizer.get_enrolled_id_num() < FACE_ID_SAVE_NUMBER) {
            int new_id = recognizer.enroll_id(tensor, face.keypoint, "", true);
            Serial.printf("Enrolled New Face ID: %d\n", new_id);
        }

        face_info_t recognition = recognizer.recognize(tensor, face.keypoint);
        if (recognition.id >= 0) {
            Serial.printf("Recognized Face ID: %d (Confidence: %.2f)\n",
                        recognition.id, recognition.similarity);
        } else {
            Serial.println("Unknown Face Detected");
        }
    }
}
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Initialize camera
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
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_RGB565;
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

  // Initialize face recognition filter
  ra_filter_init(&ra_filter, RA_FILTER_SIZE);
}
void loop() {
    camera_fb_t *fb = NULL;
    uint8_t *converted_buf = NULL;
    bool buffer_converted = false;

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) continue;

        // Convert frame if necessary
        if (fb->format != PIXFORMAT_RGB888) {
            converted_buf = (uint8_t*)malloc(fb->width * fb->height * 3);
            if (fmt2rgb888(fb->buf, fb->len, fb->format, converted_buf)) {
                buffer_converted = true;
            }
        }

        // Step 1: Face Detection
        auto detected_faces = detect_faces(fb, converted_buf);
        
        if (!detected_faces.empty()) {
            Serial.printf("Detected %d faces\n", detected_faces.size());
            
            // Step 2: Face Recognition
            if (recognition_enabled) {
                recognize_faces(
                    buffer_converted ? converted_buf : fb->buf,
                    fb->width,
                    fb->height,
                    detected_faces
                );
            }
        }

        // Cleanup
        if (buffer_converted) {
            free(converted_buf);
            buffer_converted = false;
        }
        esp_camera_fb_return(fb);
    }
}