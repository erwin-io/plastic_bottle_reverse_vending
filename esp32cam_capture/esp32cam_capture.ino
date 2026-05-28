/*
  ESP32-S3 CAM - FULL .INO CODE
  BALANCED BRIGHTNESS + SHARPER IMAGE + STABLE HTTP UPLOAD + WEB SERVER

  Camera:
    FRAMESIZE_XGA
    JPEG_QUALITY = 7

  API:
    http://192.168.254.113:8000/upload

  ESP32 Web:
    http://ESP32_IP/
    http://ESP32_IP/status
    http://ESP32_IP/capture.jpg

  UNO output:
    plastic_bottle | empty | invalid
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <ArduinoJson.h>

/* ===================== WiFi ===================== */
const char* WIFI_SSID = "Olly Kitchen";
const char* WIFI_PASS = "feby2020";

/* ===================== API TARGET ===================== */
static IPAddress API_IP(192, 168, 254, 113);
const uint16_t API_PORT = 8000;
const char* API_PATH = "/upload";

/* ===================== UART TO ARDUINO UNO ===================== */
HardwareSerial ToUNO(1);
static const int UNO_TX_PIN = 14;
static const uint32_t UNO_BAUD = 9600;

/* ===================== Camera Settings ===================== */
static framesize_t FRAME_SIZE = FRAMESIZE_SXGA;
static int JPEG_QUALITY = 10;
static const int XCLK_HZ = 20000000;

/*
  Balanced image tuning:
  Previous version became too dark.
  This version keeps mango visible but avoids overexposure.
*/
static const int CAM_BRIGHTNESS = 1;
static const int CAM_CONTRAST = 1;
static const int CAM_SATURATION = 0;
static const int CAM_SHARPNESS = 2;
static const int CAM_DENOISE = 0;
static const int CAM_AE_LEVEL = -1;

/* ===================== Timing ===================== */
static const uint32_t SEND_PERIOD_MS = 3000;
static const uint32_t HTTP_TOTAL_TIMEOUT_MS = 45000;
static const uint32_t HTTP_WRITE_STALL_TIMEOUT_MS = 20000;

static const uint32_t UNO_MIN_GAP_MS = 150;
static const uint32_t UNO_SAME_TYPE_COOLDOWN_MS = 2500;

/* =========================================================
   ESP32-S3 CAMERA PIN MAP
   ========================================================= */
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1

#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11

#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

/* ===================== Web Server ===================== */
static httpd_handle_t httpd = NULL;

/* ===================== Camera Mutex ===================== */
static SemaphoreHandle_t cameraMutex = NULL;

/* ===================== Runtime State ===================== */
static String gLastUnoWord = "";
static String gLastMessage = "invalid";
static String gLastBody = "";
static String gLastError = "";

static uint32_t gLastUnoSendMs = 0;
static uint32_t gLastPostMs = 0;
static uint32_t gSeq = 0;

static uint32_t gLastJpegBytes = 0;
static uint32_t gLastCaptureMs = 0;
static uint32_t gLastPostTotalMs = 0;
static bool gLastPostOk = false;

/* ===================== HTML Page ===================== */
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-S3 CAM</title>
<style>
  body{margin:0;background:#0b0b0b;color:white;font-family:Arial}
  .bar{padding:14px;background:#111;border-bottom:1px solid #222}
  .wrap{padding:16px;display:grid;grid-template-columns:1fr;gap:16px}
  .card{background:#111;border:1px solid #222;border-radius:12px;padding:14px}
  img{width:100%;max-width:900px;border-radius:12px;border:1px solid #333;background:#000}
  code{background:#000;border:1px solid #333;padding:2px 6px;border-radius:6px}
  pre{white-space:pre-wrap;word-break:break-word;background:#000;border:1px solid #333;padding:10px;border-radius:8px}
  .ok{color:#76ff76}
  .bad{color:#ff6b6b}
</style>
</head>
<body>
<div class="bar">
  <b>ESP32-S3 CAM</b> — XGA / Q7 / balanced brightness / sharper / stable upload
</div>

<div class="wrap">
  <div class="card">
    <h3>Status</h3>
    <div>POST: <b id="postState">-</b></div>
    <div>Message: <code id="msg">-</code></div>
    <div>Seq: <code id="seq">0</code></div>
    <div>JPEG bytes: <code id="bytes">0</code></div>
    <div>Capture ms: <code id="cap">0</code></div>
    <div>POST total ms: <code id="total">0</code></div>
    <div>Free heap: <code id="heap">0</code></div>
    <div>Free PSRAM: <code id="psram">0</code></div>
    <div>IP: <code id="ip">-</code></div>

    <h4>Last body</h4>
    <pre id="body">-</pre>

    <h4>Last error</h4>
    <pre id="err">-</pre>
  </div>

  <div class="card">
    <h3>Live Capture</h3>
    <p>Captured live from <code>/capture.jpg</code>.</p>
    <img id="img" src="/capture.jpg">
  </div>
</div>

<script>
async function updateStatus(){
  try {
    const r = await fetch('/status?ts=' + Date.now(), {cache:'no-store'});
    const j = await r.json();

    document.getElementById('postState').innerHTML =
      j.last_post_ok ? '<span class="ok">OK</span>' : '<span class="bad">FAIL</span>';

    document.getElementById('msg').textContent = j.last_message;
    document.getElementById('seq').textContent = j.seq;
    document.getElementById('bytes').textContent = j.last_jpeg_bytes;
    document.getElementById('cap').textContent = j.last_capture_ms;
    document.getElementById('total').textContent = j.last_post_total_ms;
    document.getElementById('heap').textContent = j.free_heap;
    document.getElementById('psram').textContent = j.free_psram;
    document.getElementById('ip').textContent = j.ip;
    document.getElementById('body').textContent = j.last_body || '-';
    document.getElementById('err').textContent = j.last_error || '-';
  } catch(e) {}
}

function refreshImage(){
  document.getElementById('img').src = '/capture.jpg?ts=' + Date.now();
}

setInterval(updateStatus, 1000);
setInterval(refreshImage, 3000);

updateStatus();
</script>
</body>
</html>
)rawliteral";

/* ===================== Helpers ===================== */
static void delayMsSmart(uint32_t ms) {
  uint32_t endAt = millis() + ms;

  while ((int32_t)(millis() - endAt) < 0) {
    delay(10);
  }
}

static String extractHttpBody(const String& response) {
  int idx = response.indexOf("\r\n\r\n");

  if (idx < 0) {
    return "";
  }

  return response.substring(idx + 4);
}

static String normalizeMessage(String msg) {
  msg.trim();
  msg.toLowerCase();

  if (msg == "plastic_bottle") return "plastic_bottle";
  if (msg == "empty") return "empty";
  if (msg == "invalid") return "invalid";

  return "invalid";
}

static String parseApiMessage(const String& body) {
  if (body.length() == 0) {
    return "empty";
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, body);

  if (err) {
    Serial.print("[JSON] Parse error: ");
    Serial.println(err.c_str());
    return "invalid";
  }

  const char* msg = doc["message"];

  if (!msg) {
    return "invalid";
  }

  return normalizeMessage(String(msg));
}

static void sendToUno(const String& word) {
  uint32_t now = millis();
  uint32_t gap = now - gLastUnoSendMs;

  if (gap < UNO_MIN_GAP_MS) {
    delayMsSmart(UNO_MIN_GAP_MS - gap);
  }

  if (gLastUnoWord.length() && word == gLastUnoWord) {
    delayMsSmart(UNO_SAME_TYPE_COOLDOWN_MS);
  }

  ToUNO.println(word);

  Serial.print("[UNO<<] ");
  Serial.println(word);

  gLastUnoWord = word;
  gLastUnoSendMs = millis();
}

/* ===================== Camera Init ===================== */
static bool initCamera() {
  esp_camera_deinit();
  delay(200);

  camera_config_t config;
  memset(&config, 0, sizeof(config));

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

  /*
    If compile error:
      camera_config_t has no member named pin_sscb_sda

    change these two to:
      config.pin_sccb_sda = SIOD_GPIO_NUM;
      config.pin_sccb_scl = SIOC_GPIO_NUM;
  */
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;

  if (psramFound()) {
    Serial.println("[CAMERA] PSRAM found. Using PSRAM framebuffer.");
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("[CAMERA] No PSRAM. Using DRAM framebuffer.");
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  /*
    Keep 1 framebuffer for stability.
    This avoids camera/web/upload fighting over memory.
  */
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  Serial.println("[CAMERA] Starting init...");
  Serial.printf("[MEM] Free heap before camera init: %u\n", ESP.getFreeHeap());
  Serial.printf("[MEM] psramFound(): %s\n", psramFound() ? "YES" : "NO");
  Serial.printf("[MEM] PSRAM size: %u\n", ESP.getPsramSize());
  Serial.printf("[MEM] Free PSRAM: %u\n", ESP.getFreePsram());

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("[CAMERA] Init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();

  if (s) {
    Serial.printf("[CAMERA] Sensor PID: 0x%04x\n", s->id.PID);

    s->set_framesize(s, FRAME_SIZE);
    s->set_quality(s, JPEG_QUALITY);

    /*
      Balanced image tuning:
      - not too dark
      - not washed out
      - sharper object edges
      - better color for mango / bio objects
    */
    s->set_brightness(s, CAM_BRIGHTNESS);
    s->set_contrast(s, CAM_CONTRAST);
    s->set_saturation(s, CAM_SATURATION);
    s->set_sharpness(s, CAM_SHARPNESS);
    s->set_denoise(s, CAM_DENOISE);

    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);

    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, CAM_AE_LEVEL);

    s->set_gain_ctrl(s, 1);

    /*
      4 is balanced:
      - brighter than previous value 2
      - less noisy/washed out than too high gain
    */
    s->set_gainceiling(s, (gainceiling_t)4);

    s->set_lenc(s, 1);
    s->set_special_effect(s, 0);

    /*
      Disable digital crop/zoom for now.
      Your object already appears large. Clarity is more important.
    */
    s->set_dcw(s, 0);

    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
  }

  Serial.println("[CAMERA] Init OK");
  Serial.printf("[MEM] Free heap after camera init: %u\n", ESP.getFreeHeap());
  Serial.printf("[MEM] Free PSRAM after camera init: %u\n", ESP.getFreePsram());

  return true;
}

/* ===================== Capture Frame Safely ===================== */
static camera_fb_t* captureFrameLocked() {
  if (!cameraMutex) {
    return NULL;
  }

  if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
    Serial.println("[CAMERA] Mutex timeout");
    return NULL;
  }

  camera_fb_t* fb = NULL;

  for (int i = 0; i < 10; i++) {
    fb = esp_camera_fb_get();

    if (fb) {
      return fb;
    }

    delay(30);
  }

  xSemaphoreGive(cameraMutex);
  return NULL;
}

static void releaseFrameLocked(camera_fb_t* fb) {
  if (fb) {
    esp_camera_fb_return(fb);
  }

  if (cameraMutex) {
    xSemaphoreGive(cameraMutex);
  }
}

/* ===================== POST JPEG Directly - STABLE VERSION ===================== */
static bool postJpegDirect(
  camera_fb_t* fb,
  String& outMessage,
  String& outBody,
  String& outError
) {
  outMessage = "invalid";
  outBody = "";
  outError = "";

  if (!fb || !fb->buf || fb->len == 0) {
    outError = "invalid camera frame";
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    outError = "wifi disconnected";
    return false;
  }

  WiFiClient client;
  client.setNoDelay(true);
  client.setTimeout(15);

  uint32_t deadline = millis() + HTTP_TOTAL_TIMEOUT_MS;

  auto timedOut = [&]() -> bool {
    return (int32_t)(millis() - deadline) >= 0;
  };

  Serial.printf("[POST] Connecting to %s:%u\n", API_IP.toString().c_str(), API_PORT);

  if (!client.connect(API_IP, API_PORT)) {
    outError = "connect failed";
    return false;
  }

  client.print("POST ");
  client.print(API_PATH);
  client.println(" HTTP/1.1");

  client.print("Host: ");
  client.println(API_IP.toString());

  client.println("User-Agent: ESP32S3-CAM");
  client.println("Connection: close");
  client.println("Content-Type: image/jpeg");

  client.print("Content-Length: ");
  client.println(fb->len);

  client.println();

  size_t sent = 0;
  const size_t CHUNK = 1024;

  uint32_t sendStart = millis();
  uint32_t lastProgressMs = millis();
  uint32_t zeroWriteCount = 0;

  while (sent < fb->len) {
    if (timedOut()) {
      client.stop();
      outError = "timeout while sending";
      return false;
    }

    if (!client.connected()) {
      client.stop();
      outError = "server closed connection while sending";
      return false;
    }

    size_t remaining = fb->len - sent;
    size_t toSend = remaining > CHUNK ? CHUNK : remaining;

    size_t written = client.write(fb->buf + sent, toSend);

    if (written > 0) {
      sent += written;
      lastProgressMs = millis();
      zeroWriteCount = 0;

      if ((sent % 8192) < CHUNK) {
        Serial.printf("[POST] Progress: %u / %u bytes\n",
                      (unsigned)sent,
                      (unsigned)fb->len);
      }
    } else {
      zeroWriteCount++;

      if (millis() - lastProgressMs > HTTP_WRITE_STALL_TIMEOUT_MS) {
        client.stop();

        outError = "write stalled too long";

        Serial.printf("[POST] Write stalled. Sent %u / %u bytes. zeroWriteCount=%u\n",
                      (unsigned)sent,
                      (unsigned)fb->len,
                      (unsigned)zeroWriteCount);

        return false;
      }

      /*
        Longer delay gives TCP/WiFi buffer time to recover.
      */
      delay(20);
    }

    delay(0);
  }

  client.flush();

  uint32_t sendMs = millis() - sendStart;

  Serial.printf("[POST] Sent %u bytes in %u ms\n",
                (unsigned)sent,
                (unsigned)sendMs);

  String response;
  response.reserve(1200);

  uint32_t waitStart = millis();

  while (client.connected() && !client.available()) {
    if (timedOut()) {
      client.stop();
      outError = "timeout waiting response";
      return false;
    }

    delay(1);
  }

  uint32_t waitMs = millis() - waitStart;

  while (client.connected() || client.available()) {
    if (timedOut()) {
      client.stop();
      outError = "timeout reading response";
      return false;
    }

    while (client.available()) {
      char c = (char)client.read();

      if (response.length() < 1200) {
        response += c;
      }
    }

    delay(1);
  }

  client.stop();

  outBody = extractHttpBody(response);
  outBody.trim();

  outMessage = parseApiMessage(outBody);

  Serial.printf("[POST] Wait response: %u ms\n", (unsigned)waitMs);
  Serial.print("[POST] Body: ");
  Serial.println(outBody);
  Serial.print("[POST] Message: ");
  Serial.println(outMessage);

  return true;
}

/* ===================== One Full POST Cycle ===================== */
static void capturePostSendCycle() {
  Serial.println();
  Serial.println("====================================");
  Serial.printf("[CYCLE] #%u\n", (unsigned)gSeq);
  Serial.printf("[MEM] Free heap before capture: %u\n", ESP.getFreeHeap());
  Serial.printf("[MEM] Free PSRAM: %u\n", ESP.getFreePsram());

  uint32_t t0 = millis();

  camera_fb_t* fb = captureFrameLocked();

  if (!fb) {
    Serial.println("[CAMERA] Capture failed");

    gLastError = "capture failed";
    gLastPostOk = false;
    gLastMessage = "invalid";
    gLastBody = "";

    sendToUno("invalid");
    gSeq++;

    return;
  }

  uint32_t captureMs = millis() - t0;
  gLastJpegBytes = fb->len;

  Serial.printf("[CAMERA] Captured JPEG: %u bytes\n", (unsigned)fb->len);
  Serial.printf("[CAMERA] Capture time: %u ms\n", (unsigned)captureMs);
  Serial.printf("[MEM] Free heap after capture: %u\n", ESP.getFreeHeap());

  String message;
  String body;
  String error;

  bool ok = postJpegDirect(fb, message, body, error);

  releaseFrameLocked(fb);
  fb = NULL;

  if (!ok) {
    Serial.print("[POST] Failed: ");
    Serial.println(error);
    message = "invalid";
  }

  sendToUno(message);

  uint32_t totalMs = millis() - t0;

  gLastPostOk = ok;
  gLastMessage = message;
  gLastBody = body;
  gLastError = error;
  gLastCaptureMs = captureMs;
  gLastPostTotalMs = totalMs;

  Serial.printf("[CYCLE] Total: %u ms\n", (unsigned)totalMs);
  Serial.printf("[MEM] Free heap after release: %u\n", ESP.getFreeHeap());
  Serial.println("====================================");

  gSeq++;
}

/* ===================== Web Handlers ===================== */
static esp_err_t index_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t* req) {
  StaticJsonDocument<1024> doc;

  doc["seq"] = gSeq;
  doc["last_post_ok"] = gLastPostOk;
  doc["last_message"] = gLastMessage;
  doc["last_body"] = gLastBody;
  doc["last_error"] = gLastError;
  doc["last_jpeg_bytes"] = gLastJpegBytes;
  doc["last_capture_ms"] = gLastCaptureMs;
  doc["last_post_total_ms"] = gLastPostTotalMs;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["psram_found"] = psramFound();
  doc["free_psram"] = ESP.getFreePsram();
  doc["ip"] = WiFi.localIP().toString();

  String out;
  serializeJson(doc, out);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  return httpd_resp_send(req, out.c_str(), out.length());
}

static esp_err_t capturejpg_handler(httpd_req_t* req) {
  camera_fb_t* fb = captureFrameLocked();

  if (!fb) {
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Capture failed or camera busy.", HTTPD_RESP_USE_STRLEN);
  }

  gLastJpegBytes = fb->len;

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);

  releaseFrameLocked(fb);

  return res;
}

static void startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_open_sockets = 4;
  config.stack_size = 8192;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
  };

  httpd_uri_t jpg_uri = {
    .uri = "/capture.jpg",
    .method = HTTP_GET,
    .handler = capturejpg_handler,
    .user_ctx = NULL
  };

  esp_err_t err = httpd_start(&httpd, &config);

  if (err == ESP_OK) {
    httpd_register_uri_handler(httpd, &index_uri);
    httpd_register_uri_handler(httpd, &status_uri);
    httpd_register_uri_handler(httpd, &jpg_uri);

    Serial.println("[WEB] Server started");
    Serial.print("[WEB] Open: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.printf("[WEB] Server failed to start: 0x%x\n", err);
  }
}

/* ===================== Setup ===================== */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32-S3 CAM Upload Client");
  Serial.println("BALANCED BRIGHTNESS + SHARPER VERSION");
  Serial.println("Frame size: FRAMESIZE_XGA");
  Serial.println("JPEG quality: 7");
  Serial.println("====================================");

  cameraMutex = xSemaphoreCreateMutex();

  if (!cameraMutex) {
    Serial.println("[SYSTEM] Failed to create camera mutex");
  }

  ToUNO.begin(UNO_BAUD, SERIAL_8N1, -1, UNO_TX_PIN);

  Serial.print("[UART] UNO TX GPIO");
  Serial.println(UNO_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  Serial.print("[WIFI] Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("[WIFI] Connected. ESP32-S3 IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("[API] URL: http://");
  Serial.print(API_IP);
  Serial.print(":");
  Serial.print(API_PORT);
  Serial.println(API_PATH);

  Serial.printf("[CHECK] psramFound(): %s\n", psramFound() ? "YES" : "NO");
  Serial.printf("[CHECK] ESP.getPsramSize(): %u\n", ESP.getPsramSize());
  Serial.printf("[CHECK] ESP.getFreePsram(): %u\n", ESP.getFreePsram());
  Serial.printf("[CHECK] ESP.getFreeHeap(): %u\n", ESP.getFreeHeap());

  if (!initCamera()) {
    Serial.println("[SYSTEM] Camera init failed. Restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  startWebServer();

  Serial.println("[SYSTEM] Ready.");
}

/* ===================== Main Loop ===================== */
void loop() {
  uint32_t now = millis();

  if (now - gLastPostMs >= SEND_PERIOD_MS) {
    capturePostSendCycle();

    // Set after cycle finishes.
    // This prevents instant retry after failed/slow upload.
    gLastPostMs = millis();
  }

  delay(10);
}