#include "esp_camera.h"

// ── Pin definitions DFR1154 V1.1 ────────────────────────────────────────────
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    5
#define SIOD_GPIO_NUM    8
#define SIOC_GPIO_NUM    9
#define Y9_GPIO_NUM      4
#define Y8_GPIO_NUM      6
#define Y7_GPIO_NUM      7
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     17
#define Y4_GPIO_NUM     21
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM     16
#define VSYNC_GPIO_NUM   1
#define HREF_GPIO_NUM    2
#define PCLK_GPIO_NUM   15

// ── UART to motor ESP32 ──────────────────────────────────────────────────────
#define MOTOR_UART      Serial1
#define MOTOR_UART_TX   43
#define MOTOR_UART_RX   44
#define MOTOR_BAUD      115200

// ── Calibrated values ─────────────────────────────────
#define BRIGHT_THRESHOLD    120
#define MIN_BLOB_AREA        50
#define PX_TO_MM          1.936f
#define BOARD_ORIGIN_PX_X   127
#define BOARD_ORIGIN_PX_Y   229
#define CAM_TO_ARM_OFFSET_X 0.0f
#define CAM_TO_ARM_OFFSET_Y 0.0f

// ── ROI ───────────────────────────────────────────────
#define ROI_X_START_PCT  0.25f
#define ROI_X_END_PCT    0.75f
#define ROI_Y_START_PCT  0.30f
#define ROI_Y_END_PCT    0.75f

bool initCamera();
bool detectPuck(camera_fb_t *fb, int &cx, int &cy);
void pixelToArmCoords(int px, int py, float &arm_x, float &arm_y);

void setup() {
  Serial.begin(115200);
  MOTOR_UART.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_UART_RX, MOTOR_UART_TX);
  if (!initCamera()) {
    while (true) delay(500);
  }
  Serial.println("[SYS] Ready. Waiting for 'R' command...");
}

void loop() {
  // Only process when 'R' is received from the Arm
  if (MOTOR_UART.available() && MOTOR_UART.read() == 'R') {
    
    // Grab and discard 2 frames to ensure the sensor buffer is fresh
    for(int i = 0; i < 2; i++) {
        camera_fb_t *temp_fb = esp_camera_fb_get();
        if(temp_fb) esp_camera_fb_return(temp_fb);
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return;

    int px = 0, py = 0;
    if (detectPuck(fb, px, py)) {
      float ax = 0.0f, ay = 0.0f;
      pixelToArmCoords(px, py, ax, ay);
      
      // Send 8-byte packet (float x, float y)
      uint8_t buf[8];
      memcpy(&buf[0], &ax, 4);
      memcpy(&buf[4], &ay, 4);
      MOTOR_UART.write(buf, 8);
      
      Serial.printf("[DATA] Sent Arm X:%.2f Y:%.2f\n", ax, ay);
    }
    esp_camera_fb_return(fb);
  }
}

// ── Pixel → arm coordinates ──────────────────────
void pixelToArmCoords(int px, int py, float &arm_x, float &arm_y) {
  arm_x = -((py - BOARD_ORIGIN_PX_Y) * PX_TO_MM) + CAM_TO_ARM_OFFSET_X;
  arm_y =  ((px - BOARD_ORIGIN_PX_X) * PX_TO_MM) + CAM_TO_ARM_OFFSET_Y;
}

// ── Puck detection ──────────────────────────────
bool detectPuck(camera_fb_t *fb, int &cx, int &cy) {
  uint8_t *pixels = fb->buf;
  int w = fb->width;
  int h = fb->height;

  int x_start = (int)(w * ROI_X_START_PCT);
  int x_end   = (int)(w * ROI_X_END_PCT);  
  int y_start = (int)(h * ROI_Y_START_PCT);
  int y_end   = (int)(h * ROI_Y_END_PCT);  

  long sum_x = 0, sum_y = 0, count = 0;
  for (int y = y_start; y < y_end; y++) {
    for (int x = x_start; x < x_end; x++) {
      if (pixels[y * w + x] >= BRIGHT_THRESHOLD) {
        sum_x += x;
        sum_y += y;
        count++;
      }
    }
  }

  if (count < MIN_BLOB_AREA) return false;

  cx = (int)(sum_x / count);
  cy = (int)(sum_y / count);
  return true;
}

// ── Camera init ───────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  // Manual exposure/gain values from original code
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_aec_value(s, 400);
  s->set_gain_ctrl(s, 0);
  s->set_agc_gain(s, 0);
  
  return true;
}