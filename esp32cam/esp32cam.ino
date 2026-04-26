#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include <plastic-bottle-detection_inferencing.h>

/* ===================== WIFI AP (FIXED IP) ===================== */
static const char *AP_SSID = "ESP32-CAM-BIN";
static const char *AP_PASS = "12345678";     // >= 8 chars
static const int   AP_CH   = 6;
static const int   AP_MAX_CONN = 2;

// Fixed server IP for hotspot
static const IPAddress AP_IP      (192, 168, 4, 1);
static const IPAddress AP_GATEWAY (192, 168, 4, 1);
static const IPAddress AP_SUBNET  (255, 255, 255, 0);

/* ===================== UART TO UNO ===================== */
static const int UNO_TX_PIN = 15;     // ESP32 GPIO15 (TX1) -> UNO D10 (RX)
static const uint32_t UNO_BAUD = 9600;

/* ===================== BOOT + FILTERS ===================== */
static const unsigned long WARMUP_TIME_MS = 2000;
static const int REQUIRED_STABLE_COUNT = 1;
static const unsigned long HEARTBEAT_MS = 1200;

static unsigned long bootTimeMs = 0;
static String lastDecisionSeen = "";
static int stableCounter = 0;

static String lastSent = "";
static unsigned long lastSentMs = 0;

/* ===================== AI THINKER PINS ===================== */
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

/* ===================== GLOBAL AI RESULT (for UI) ===================== */
static volatile float g_plastic = 0.0f;
static volatile float g_empty   = 0.0f;
static volatile uint32_t g_last_ms = 0;
static String g_decision = "booting";

/* ===================== EI SIZE ===================== */
static constexpr int EI_W = EI_CLASSIFIER_INPUT_WIDTH;
static constexpr int EI_H = EI_CLASSIFIER_INPUT_HEIGHT;

/* ===================== BUFFERS ===================== */
static uint8_t *ei_buf = nullptr;
static size_t   ei_buf_len = 0;

static int ei_get_data(size_t offset, size_t length, float *out_ptr) {
  for (size_t i = 0; i < length; i++) out_ptr[i] = (float)ei_buf[offset + i];
  return 0;
}

/* ===================== CAMERA MUTEX ===================== */
static SemaphoreHandle_t camMutex;

/* ===================== HTTP SERVER ===================== */
static httpd_handle_t httpd = NULL;

/* ===================== SIMPLE WEB UI ===================== */
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32-CAM Plastic Bottle Detection</title>
  <style>
    body{font-family:Arial, sans-serif; margin:16px; background:#111; color:#eee;}
    .card{max-width:820px; margin:auto; background:#1b1b1b; border-radius:14px; padding:14px;}
    h2{margin:0 0 10px 0; font-size:18px;}
    img{width:100%; border-radius:12px; background:#000; display:block;}
    .row{display:flex; gap:10px; flex-wrap:wrap; margin-top:10px;}
    .pill{padding:8px 10px; border-radius:999px; background:#2a2a2a; font-size:14px;}
    .ok{background:#173a1f;}
    .warn{background:#3a2c17;}
    .bad{background:#3a1717;}
    .small{opacity:.9; font-size:13px;}
  </style>
</head>
<body>
  <div class="card">
    <h2>ESP32-CAM + Edge Impulse</h2>
    <img id="stream" src="/stream" />
    <div class="row">
      <div id="decision" class="pill">decision: ...</div>
      <div id="scores" class="pill small">scores: ...</div>
      <div id="age" class="pill small">age: ...</div>
    </div>
  </div>

<script>
function colorizeDecision(el, d){
  el.classList.remove('ok','warn','bad');
  if(d === 'plastic_bottle') el.classList.add('ok');
  else if(d === 'empty') el.classList.add('warn');
  else el.classList.add('bad');
}
async function refreshResult(){
  try{
    const r = await fetch('/result.json?t=' + Date.now());
    const j = await r.json();
    const d = document.getElementById('decision');
    d.textContent = 'decision: ' + j.decision;
    colorizeDecision(d, j.decision);
    document.getElementById('scores').textContent =
      `scores: plastic=${j.plastic.toFixed(3)} empty=${j.empty.toFixed(3)}`;
    document.getElementById('age').textContent = `age: ${j.age_ms} ms`;
  }catch(e){}
}
setInterval(refreshResult, 800);
refreshResult();
</script>
</body>
</html>
)HTML";

/* ===================== CAMERA INIT ===================== */
static bool init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  // MJPEG stream needs JPEG frames
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count     = 2;

  return (esp_camera_init(&config) == ESP_OK);
}

/* ===================== RESIZE CENTER CROP RGB888 ===================== */
static void resize_center_rgb888(uint8_t *src, int sw, int sh,
                                 uint8_t *dst, int dw, int dh) {
  int crop = min(sw, sh);
  int xo = (sw - crop) / 2;
  int yo = (sh - crop) / 2;

  for (int y = 0; y < dh; y++) {
    for (int x = 0; x < dw; x++) {
      int sx = xo + (x * crop) / dw;
      int sy = yo + (y * crop) / dh;

      int si = (sy * sw + sx) * 3;
      int di = (y * dw + x) * 3;

      dst[di + 0] = src[si + 0];
      dst[di + 1] = src[si + 1];
      dst[di + 2] = src[si + 2];
    }
  }
}

/* ===================== RGB -> GRAYSCALE ===================== */
static void rgb_to_gray(uint8_t *rgb, uint8_t *gray, int w, int h) {
  for (int i = 0; i < w * h; i++) {
    int r = rgb[i*3 + 0];
    int g = rgb[i*3 + 1];
    int b = rgb[i*3 + 2];
    gray[i] = (uint8_t)((77*r + 150*g + 29*b) >> 8);
  }
}

/* ===================== CAPTURE -> EI BUFFER (AUTO RGB/GRAY) ===================== */
static bool capture_to_ei_auto() {
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(800)) != pdTRUE) return false;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    xSemaphoreGive(camMutex);
    return false;
  }

  const int sw = fb->width;
  const int sh = fb->height;

  size_t full_len = (size_t)sw * (size_t)sh * 3;
  uint8_t *full_rgb = (uint8_t*)malloc(full_len);
  if (!full_rgb) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);
    return false;
  }

  bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, full_rgb);
  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);

  if (!ok) { free(full_rgb); return false; }

  // resize to EI_W x EI_H RGB first
  size_t rgb_needed = (size_t)EI_W * (size_t)EI_H * 3;
  uint8_t *rgb_small = (uint8_t*)malloc(rgb_needed);
  if (!rgb_small) { free(full_rgb); return false; }

  resize_center_rgb888(full_rgb, sw, sh, rgb_small, EI_W, EI_H);
  free(full_rgb);

  // determine grayscale
  int expected_samples =
#ifdef EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME
    EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME
#else
    EI_W * EI_H * 3
#endif
  ;

#ifdef EI_CLASSIFIER_GRAYSCALE
  const bool wantsGray = (EI_CLASSIFIER_GRAYSCALE == 1);
#else
  const bool wantsGray = (expected_samples == (EI_W * EI_H));
#endif

  if (wantsGray) {
    size_t needed = (size_t)EI_W * (size_t)EI_H;
    if (!ei_buf || ei_buf_len != needed) {
      if (ei_buf) free(ei_buf);
      ei_buf = (uint8_t*)malloc(needed);
      if (!ei_buf) { ei_buf_len = 0; free(rgb_small); return false; }
      ei_buf_len = needed;
    }
    rgb_to_gray(rgb_small, ei_buf, EI_W, EI_H);
    free(rgb_small);
  } else {
    size_t needed = rgb_needed;
    if (!ei_buf || ei_buf_len != needed) {
      if (ei_buf) free(ei_buf);
      ei_buf = (uint8_t*)malloc(needed);
      if (!ei_buf) { ei_buf_len = 0; free(rgb_small); return false; }
      ei_buf_len = needed;
    }
    memcpy(ei_buf, rgb_small, needed);
    free(rgb_small);
  }

  return true;
}

/* ===================== DECISION ===================== */
static const char *decide(const ei_impulse_result_t &result, float &plastic, float &empty) {
  int idx_plastic = -1, idx_empty = -1;

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (strcmp(result.classification[i].label, "plastic_bottle") == 0) idx_plastic = (int)i;
    if (strcmp(result.classification[i].label, "empty") == 0) idx_empty = (int)i;
  }

  plastic = (idx_plastic >= 0) ? result.classification[idx_plastic].value : 0.0f;
  empty   = (idx_empty   >= 0) ? result.classification[idx_empty].value   : 0.0f;

  if (plastic >= 0.65f && empty <= 0.30f) return "plastic_bottle";
  return "empty";
}

/* ===================== SEND (stability + heartbeat) ===================== */
static void sendToUnoStableHeartbeat(const String &decision) {
  unsigned long now = millis();

  if (now - bootTimeMs < WARMUP_TIME_MS) return;

  if (decision == lastDecisionSeen) stableCounter++;
  else { lastDecisionSeen = decision; stableCounter = 1; }

  if (stableCounter < REQUIRED_STABLE_COUNT) return;

  bool changed = (decision != lastSent);
  bool heartbeatDue = (now - lastSentMs >= HEARTBEAT_MS);
  if (!changed && !heartbeatDue) return;

  Serial1.println(decision);
  Serial.print("UART->UNO: "); Serial.println(decision);

  lastSent = decision;
  lastSentMs = now;
}

/* ===================== HTTP HANDLERS ===================== */
static esp_err_t root_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t result_handler(httpd_req_t *req) {
  uint32_t age = millis() - g_last_ms;

  char buf[220];
  snprintf(buf, sizeof(buf),
           "{\"decision\":\"%s\",\"plastic\":%.6f,\"empty\":%.6f,\"age_ms\":%u}",
           g_decision.c_str(), (double)g_plastic, (double)g_empty, (unsigned)age);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stream_handler(httpd_req_t *req) {
  static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
  static const char* BOUNDARY = "\r\n--frame\r\n";
  static const char* PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  char part_buf[64];

  while (true) {
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);

    if (!fb) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    if (httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY)) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    int hlen = snprintf(part_buf, sizeof(part_buf), PART, (unsigned)fb->len);
    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    if (httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(60));
  }

  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static void start_http_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.stack_size = 8192;

  if (httpd_start(&httpd, &config) != ESP_OK) {
    Serial.println("HTTP server start failed");
    return;
  }

  httpd_uri_t uri_root   = { .uri="/",           .method=HTTP_GET, .handler=root_handler,   .user_ctx=NULL };
  httpd_uri_t uri_json   = { .uri="/result.json",.method=HTTP_GET, .handler=result_handler, .user_ctx=NULL };
  httpd_uri_t uri_stream = { .uri="/stream",     .method=HTTP_GET, .handler=stream_handler, .user_ctx=NULL };

  httpd_register_uri_handler(httpd, &uri_root);
  httpd_register_uri_handler(httpd, &uri_json);
  httpd_register_uri_handler(httpd, &uri_stream);

  Serial.println("HTTP server started: /  /stream  /result.json");
}

/* ===================== TASK ===================== */
void inferenceTask(void *pv) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!capture_to_ei_auto()) continue;

    ei::signal_t signal;
    signal.total_length = ei_buf_len;
    signal.get_data = ei_get_data;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR rc = run_classifier(&signal, &result, false);
    if (rc != EI_IMPULSE_OK) continue;

    float plastic, empty;
    const char *d = decide(result, plastic, empty);

    g_decision = d;
    g_plastic  = plastic;
    g_empty    = empty;
    g_last_ms  = millis();

    Serial.println("------ RESULT ------");
    Serial.print("Decision: "); Serial.println(d);
    Serial.print("Plastic:  "); Serial.println(plastic, 4);
    Serial.print("Empty:    "); Serial.println(empty, 4);
    Serial.println("--------------------");

    sendToUnoStableHeartbeat(String(d));
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  bootTimeMs = millis();

  Serial1.begin(UNO_BAUD, SERIAL_8N1, -1, UNO_TX_PIN);

  camMutex = xSemaphoreCreateMutex();
  if (!camMutex) {
    Serial.println("Mutex fail");
    while (1) delay(1000);
  }

  if (!init_camera()) {
    Serial.println("Camera init FAILED!");
    while (1) delay(1000);
  }

  // ===== HOTSPOT MODE (FIXED IP) =====
  WiFi.mode(WIFI_AP);

  if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET)) {
    Serial.println("softAPConfig FAILED!");
    while (1) delay(1000);
  }

  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CH, false, AP_MAX_CONN);
  if (!ok) {
    Serial.println("softAP FAILED!");
    while (1) delay(1000);
  }

  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP:   "); Serial.println(WiFi.softAPIP());
  Serial.println("Open:    http://192.168.4.1/");

  start_http_server();

  xTaskCreatePinnedToCore(inferenceTask, "inference", 10 * 1024, NULL, 1, NULL, 1);
}

void loop() {
  delay(1000);
}