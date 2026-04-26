/* 
  ESP32-CAM (AI Thinker) - CONTINUOUS PREPARE (for web preview) + SEND 1 IMAGE EVERY 3 SECONDS (FOREVER)
  API is FIXED IP:
    http://192.168.254.103:8000/upload

  OUTPUTS (ONLY):
    message:  plastic_bottle | empty | invalid
    category: PLASTIC_BOTTLE | EMPTY | INVALID
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <ArduinoJson.h>

/* ===================== WiFi ===================== */
const char* WIFI_SSID = "HG8145V5_D0A04";
const char* WIFI_PASS = "p75z~${Tn2Iy";

/* ===================== API TARGET (FIXED IP) ===================== */
static IPAddress API_IP(192, 168, 254, 203);
const uint16_t API_PORT = 8000;
const char* API_PATH = "/upload";

/* ===================== UART TO ARDUINO UNO ===================== */
HardwareSerial ToUNO(1);
static const int UNO_TX_PIN = 14;
static const uint32_t UNO_BAUD = 9600;

/* ===================== Camera tuning ===================== */
static framesize_t FRAME_SIZE = FRAMESIZE_VGA;   // 640x480
static int JPEG_QUALITY = 12;
static const int XCLK_HZ = 8000000;

static const int FB_RETRY_COUNT = 10;
static const int FB_RETRY_DELAY_MS = 20;

/* ===================== Timing ===================== */
static const uint32_t SEND_PERIOD_MS = 3000;     // ✅ send ONLY 1 image per 3 seconds (forever)
static const uint32_t CAPTURE_INTERVAL_MS = 450;
static const uint32_t HTTP_TOTAL_TIMEOUT_MS = 10000;

static const uint32_t HTTP_PAUSE_MS = 0;         // optional extra pause after a send (0 because period already enforces)
static const uint32_t UNO_SAME_TYPE_COOLDOWN_MS = 2500;
static const uint32_t UNO_MIN_GAP_MS = 150;

/* ===================== AI Thinker pins ===================== */
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

static httpd_handle_t httpd = NULL;

/* ===================== Stats ===================== */
struct PostStats {
  uint32_t send_ms = 0;
  uint32_t wait_first_byte_ms = 0;
  uint32_t read_ms = 0;
  size_t   sent_bytes = 0;
};

/* ===================== Shared State ===================== */
static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t gSeq = 0;
static volatile bool gSending = false;

static volatile uint32_t gLastSendStartMs = 0;
static volatile uint32_t gLastSendEndMs = 0;

static volatile uint32_t gLastCapMs = 0;
static volatile uint32_t gLastJpegBytes = 0;

static volatile uint32_t gLastSendMs = 0;
static volatile uint32_t gLastWaitMs = 0;
static volatile uint32_t gLastReadMs = 0;
static volatile uint32_t gLastTotalMs = 0;

static volatile bool gLastOk = false;
static String gLastJsonBody = "";
static String gLastMessage = "invalid";
static String gLastCategory = "INVALID";
static String gLastError = "";

/* Prepared JPEG buffer (latest captured) */
static uint8_t* gPrepJpeg = nullptr;
static size_t   gPrepLen  = 0;
static volatile uint32_t gPrepUpdatedMs = 0;

/* Cooldown tracking */
static String gLastUnoWord = "";
static volatile uint32_t gLastUnoSendMs = 0;
static volatile uint32_t gLastHttpDoneMs = 0;

/* ===================== Helpers ===================== */
static String mapMessageToCategory(String msg) {
  msg.trim();
  msg.toLowerCase();

  if (msg == "plastic_bottle") return "PLASTIC_BOTTLE";
  if (msg == "empty")          return "EMPTY";
  return "INVALID";
}

static String extractHttpBody(const String &resp) {
  int idx = resp.indexOf("\r\n\r\n");
  if (idx < 0) return "";
  return resp.substring(idx + 4);
}

// UNO ONLY RECEIVES: plastic_bottle | empty | invalid
static String mapMessageToUnoWord(String msg) {
  msg.trim();
  msg.toLowerCase();

  if (msg == "plastic_bottle") return "plastic_bottle";
  if (msg == "empty")          return "empty";
  return "invalid";
}

static void delayMsSmart(uint32_t ms) {
  uint32_t endAt = millis() + ms;
  while ((int32_t)(millis() - endAt) < 0) vTaskDelay(pdMS_TO_TICKS(10));
}

static void sendToUnoWithRules(const String &unoWord) {
  uint32_t now = millis();
  uint32_t gap = now - gLastUnoSendMs;
  if (gap < UNO_MIN_GAP_MS) delayMsSmart(UNO_MIN_GAP_MS - gap);
  if (gLastUnoWord.length() && unoWord == gLastUnoWord) delayMsSmart(UNO_SAME_TYPE_COOLDOWN_MS);

  ToUNO.println(unoWord);
  Serial.print("[UNO<<] "); Serial.println(unoWord);

  gLastUnoWord = unoWord;
  gLastUnoSendMs = millis();
}

static void setPreparedJpeg(const uint8_t* data, size_t len) {
  uint8_t* p = (uint8_t*)malloc(len);
  if (!p) return;
  memcpy(p, data, len);

  portENTER_CRITICAL(&gMux);
  uint8_t* old = gPrepJpeg;
  gPrepJpeg = p;
  gPrepLen  = len;
  gPrepUpdatedMs = millis();
  gLastJpegBytes = (uint32_t)len;
  portEXIT_CRITICAL(&gMux);

  if (old) free(old);
}

/* ===================== Camera init ===================== */
static bool initCamera() {
  esp_camera_deinit();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size   = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAME_SIZE);
    s->set_quality(s, JPEG_QUALITY);
    s->set_aec2(s, 1);
    s->set_ae_level(s, 0);
    s->set_brightness(s, 0);
  }
  return true;
}

static camera_fb_t* getFrameSafe() {
  for (int i = 0; i < FB_RETRY_COUNT; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) return fb;
    delay(FB_RETRY_DELAY_MS);
  }
  return NULL;
}

/* ===================== HTTP POST (cancel >10s) ===================== */
static bool postJpegToApi_10sCancel(
  const uint8_t* data, size_t len,
  String &outJsonBody,
  String &outMessage,
  String &outCategory,
  String &outError,
  PostStats &st
) {
  outJsonBody = "";
  outMessage = "invalid";
  outCategory = "INVALID";
  outError = "";

  WiFiClient client;
  client.setNoDelay(true);
  client.setTimeout(2);

  uint32_t deadline = millis() + HTTP_TOTAL_TIMEOUT_MS;
  auto timedOut = [&]()->bool { return (int32_t)(millis() - deadline) >= 0; };

  if (!client.connect(API_IP, API_PORT)) {
    outError = "connect() failed to fixed IP";
    return false;
  }
  if (timedOut()) { client.stop(); outError = "timeout during connect"; return false; }

  client.print("POST "); client.print(API_PATH); client.println(" HTTP/1.1");
  client.print("Host: "); client.println(API_IP.toString());
  client.println("Connection: close");
  client.println("Content-Type: image/jpeg");
  client.print("Content-Length: "); client.println(len);
  client.println();

  uint32_t tSend0 = millis();
  size_t sent = 0;
  const size_t CHUNK = 4096;
  while (sent < len) {
    if (timedOut()) { client.stop(); outError = "timeout while sending"; return false; }
    size_t n = (len - sent > CHUNK) ? CHUNK : (len - sent);
    size_t w = client.write(data + sent, n);
    if (w == 0) { client.stop(); outError = "write() returned 0"; return false; }
    sent += w;
    delay(0);
  }
  client.flush();
  st.send_ms = millis() - tSend0;
  st.sent_bytes = sent;

  uint32_t tWait0 = millis();
  while (client.connected() && !client.available()) {
    if (timedOut()) { client.stop(); outError = "timeout waiting first byte"; return false; }
    delay(1);
  }
  st.wait_first_byte_ms = millis() - tWait0;

  uint32_t tRead0 = millis();
  String resp;
  while (client.connected() || client.available()) {
    if (timedOut()) { client.stop(); outError = "timeout while reading"; return false; }
    while (client.available()) resp += (char)client.read();
    delay(1);
  }
  st.read_ms = millis() - tRead0;

  client.stop();

  outJsonBody = extractHttpBody(resp);
  outJsonBody.trim();

  // empty body => "empty"
  if (outJsonBody.length() == 0) {
    outError = "empty body";
    outMessage = "empty";
    outCategory = "EMPTY";
    return true;
  }

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, outJsonBody)) {
    outError = "json parse error";
    outMessage = "invalid";
    outCategory = "INVALID";
    return true;
  }

  const char* m = doc["message"];
  if (!m) {
    outError = "missing message field";
    outMessage = "invalid";
    outCategory = "INVALID";
    return true;
  }

  outMessage = String(m);
  outMessage.trim();
  outMessage.toLowerCase();

  if (outMessage != "plastic_bottle" && outMessage != "empty" && outMessage != "invalid") {
    outMessage = "invalid";
  }
  outCategory = mapMessageToCategory(outMessage);
  return true;
}

/* ===================== Web UI ===================== */
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM</title>
<style>
  html,body{margin:0;padding:0;background:#0b0b0b;color:#fff;font-family:Arial}
  .bar{padding:14px 16px;background:#111;position:sticky;top:0;border-bottom:1px solid #222}
  .wrap{padding:16px;display:flex;flex-direction:column;gap:12px}
  .row{display:flex;flex-wrap:wrap;gap:12px;align-items:flex-start}
  .card{background:#111;border:1px solid #222;border-radius:12px;padding:12px;min-width:300px;max-width:560px}
  .muted{opacity:.75;font-size:13px}
  code{background:#0e0e0e;border:1px solid #222;border-radius:8px;padding:2px 6px}
  img{width:100%;max-width:560px;border-radius:12px;border:1px solid #222;background:#000}
  .ok{color:#7CFC00}
  .bad{color:#ff6b6b}
  pre{white-space:pre-wrap;word-break:break-word;background:#0e0e0e;border:1px solid #222;border-radius:12px;padding:12px}
</style>
</head>
<body>
  <div class="bar">
    <b>ESP32-CAM</b> <span class="muted">Continuous capture • Sends 1 image every 3 seconds</span>
  </div>

  <div class="wrap">
    <div class="row">
      <div class="card">
        <div class="muted">Live status</div>
        <div id="l1" style="margin-top:8px;font-size:14px;">Loading...</div>
        <div id="l2" class="muted" style="margin-top:6px;">—</div>

        <div style="margin-top:10px;">
          <div><b>message:</b> <code id="msg">-</code></div>
          <div style="margin-top:6px;"><b>category:</b> <code id="cat">-</code></div>
        </div>

        <div style="margin-top:10px;" class="muted">
          seq: <span id="seq">0</span> • jpeg: <span id="bytes">0</span> bytes
        </div>

        <div style="margin-top:10px;" class="muted">
          cap: <span id="cap">0</span>ms • send: <span id="send">0</span>ms • wait: <span id="wait">0</span>ms • read: <span id="read">0</span>ms • total: <span id="tot">0</span>ms
        </div>

        <div class="muted" style="margin-top:10px;">Last JSON body</div>
        <pre id="json">(empty)</pre>
        <div class="muted">Last error</div>
        <pre id="err">(none)</pre>
      </div>

      <div class="card">
        <div class="muted">Prepared image (/last.jpg)</div>
        <div style="margin-top:10px;">
          <img id="im" src="/last.jpg" />
        </div>
      </div>
    </div>
  </div>

<script>
async function tick(){
  try{
    const r = await fetch('/status?ts='+Date.now(), {cache:'no-store'});
    const j = await r.json();

    const ok = j.last_ok;
    const sending = j.sending;

    document.getElementById('l1').innerHTML =
      `State: <b>${sending ? 'SENDING' : 'IDLE'}</b> • Last: <b class="${ok ? 'ok':'bad'}">${ok ? 'OK':'FAIL'}</b>`;

    document.getElementById('l2').textContent =
      `LastSendStart(ms): ${j.last_send_start_ms} • LastSendEnd(ms): ${j.last_send_end_ms} • PreparedAge(ms): ${j.prepared_age_ms}`;

    document.getElementById('seq').textContent = j.seq;
    document.getElementById('bytes').textContent = j.jpeg_bytes;

    document.getElementById('cap').textContent = j.capture_ms;
    document.getElementById('send').textContent = j.send_ms;
    document.getElementById('wait').textContent = j.wait_ms;
    document.getElementById('read').textContent = j.read_ms;
    document.getElementById('tot').textContent = j.total_ms;

    document.getElementById('msg').textContent = j.message || '-';
    document.getElementById('cat').textContent = j.category || '-';

    document.getElementById('json').textContent = j.json_body || '(empty)';
    document.getElementById('err').textContent = j.error || '(none)';

    const im = document.getElementById('im');
    if (!im.dataset.seq || im.dataset.seq != String(j.seq)) {
      im.dataset.seq = String(j.seq);
      im.src = '/last.jpg?ts=' + Date.now();
    }
  }catch(e){
    document.getElementById('l1').textContent = 'Status fetch failed';
    document.getElementById('l2').textContent = String(e);
  }
}
setInterval(tick, 400);
tick();
</script>
</body>
</html>
)rawliteral";

/* ===================== Web handlers ===================== */
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req) {
  StaticJsonDocument<1600> doc;

  portENTER_CRITICAL(&gMux);
  doc["seq"] = (uint32_t)gSeq;
  doc["sending"] = gSending;

  doc["last_send_start_ms"] = (uint32_t)gLastSendStartMs;
  doc["last_send_end_ms"] = (uint32_t)gLastSendEndMs;

  doc["capture_ms"] = (uint32_t)gLastCapMs;
  doc["send_ms"] = (uint32_t)gLastSendMs;
  doc["wait_ms"] = (uint32_t)gLastWaitMs;
  doc["read_ms"] = (uint32_t)gLastReadMs;
  doc["total_ms"] = (uint32_t)gLastTotalMs;

  doc["jpeg_bytes"] = (uint32_t)gLastJpegBytes;
  doc["last_ok"] = gLastOk;

  doc["message"] = gLastMessage;
  doc["category"] = gLastCategory;
  doc["json_body"] = gLastJsonBody;
  doc["error"] = gLastError;

  uint32_t now = millis();
  uint32_t age = (gPrepUpdatedMs == 0) ? 999999 : (now - gPrepUpdatedMs);
  doc["prepared_age_ms"] = age;

  doc["send_period_ms"] = SEND_PERIOD_MS;
  doc["last_uno_send_ms"] = (uint32_t)gLastUnoSendMs;
  doc["last_http_done_ms"] = (uint32_t)gLastHttpDoneMs;

  portEXIT_CRITICAL(&gMux);

  String out;
  serializeJson(doc, out);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, out.c_str(), out.length());
}

static esp_err_t lastjpg_handler(httpd_req_t *req) {
  uint8_t* p = nullptr;
  size_t len = 0;

  portENTER_CRITICAL(&gMux);
  p = gPrepJpeg;
  len = gPrepLen;
  portEXIT_CRITICAL(&gMux);

  if (!p || len == 0) {
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "No image yet.", HTTPD_RESP_USE_STRLEN);
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, (const char*)p, len);
}

static void startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port   = 32768;
  config.max_open_sockets = 4;

  httpd_uri_t index_uri  = { .uri="/",         .method=HTTP_GET, .handler=index_handler,  .user_ctx=NULL };
  httpd_uri_t status_uri = { .uri="/status",   .method=HTTP_GET, .handler=status_handler, .user_ctx=NULL };
  httpd_uri_t jpg_uri    = { .uri="/last.jpg", .method=HTTP_GET, .handler=lastjpg_handler,.user_ctx=NULL };

  if (httpd_start(&httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(httpd, &index_uri);
    httpd_register_uri_handler(httpd, &status_uri);
    httpd_register_uri_handler(httpd, &jpg_uri);
  }
}

/* ===================== Task A: continuous capture (prepare) ===================== */
static void captureTask(void* pv) {
  (void)pv;
  for (;;) {
    uint32_t t0 = millis();
    camera_fb_t* fb = getFrameSafe();
    if (!fb) {
      initCamera();
      fb = getFrameSafe();
    }
    if (fb) {
      setPreparedJpeg(fb->buf, fb->len);
      esp_camera_fb_return(fb);

      portENTER_CRITICAL(&gMux);
      gLastCapMs = (millis() - t0);
      portEXIT_CRITICAL(&gMux);
    }
    vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));
  }
}

/* ===================== Task B: send 1 frame every 3 seconds (FOREVER) ===================== */
static void sendTask(void* pv) {
  (void)pv;

  uint32_t nextSend = millis() + SEND_PERIOD_MS;

  for (;;) {
    int32_t waitMs = (int32_t)(nextSend - millis());
    if (waitMs > 0) vTaskDelay(pdMS_TO_TICKS(waitMs));
    nextSend += SEND_PERIOD_MS;

    // extra pause guard (optional)
    uint32_t lastDone = gLastHttpDoneMs;
    if (HTTP_PAUSE_MS > 0 && lastDone != 0) {
      uint32_t sinceDone = millis() - lastDone;
      if (sinceDone < HTTP_PAUSE_MS) delayMsSmart(HTTP_PAUSE_MS - sinceDone);
    }

    uint8_t* p = nullptr;
    size_t len = 0;

    portENTER_CRITICAL(&gMux);
    gSending = true;
    gLastSendStartMs = millis();
    p = gPrepJpeg;
    len = gPrepLen;
    portEXIT_CRITICAL(&gMux);

    uint32_t tCycle0 = millis();

    bool httpOk = false;
    String jsonBody, msg, cat, err;
    PostStats st;

    if (!p || len == 0) {
      httpOk = false;
      msg = "invalid";
      cat = "INVALID";
      err = "no prepared frame yet";
    } else {
      httpOk = postJpegToApi_10sCancel(p, len, jsonBody, msg, cat, err, st);
    }

    gLastHttpDoneMs = millis();

    // UNO ONLY GETS: plastic_bottle | empty | invalid
    String unoWord = mapMessageToUnoWord(msg);
    sendToUnoWithRules(unoWord);

    portENTER_CRITICAL(&gMux);
    gLastOk = httpOk;

    gLastSendMs = st.send_ms;
    gLastWaitMs = st.wait_first_byte_ms;
    gLastReadMs = st.read_ms;
    gLastTotalMs = (millis() - tCycle0);

    gLastJsonBody = jsonBody;
    gLastMessage = msg;
    gLastCategory = cat;
    gLastError = err;

    gLastSendEndMs = millis();
    gSending = false;
    gSeq++;
    portEXIT_CRITICAL(&gMux);
  }
}

/* ===================== Setup ===================== */
void setup() {
  Serial.begin(115200);
  delay(300);

  ToUNO.begin(UNO_BAUD, SERIAL_8N1, -1, UNO_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP32 URL: http://");
  Serial.println(WiFi.localIP());

  Serial.print("API URL: http://");
  Serial.print(API_IP);
  Serial.print(":");
  Serial.print(API_PORT);
  Serial.println(API_PATH);

  if (!initCamera()) {
    Serial.println("Camera init failed. Check power/ribbon.");
  }

  startWebServer();

  xTaskCreatePinnedToCore(captureTask, "captureTask", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(sendTask,    "sendTask",    8192, NULL, 1, NULL, 1);

  Serial.println("ESP32-CAM ready. Sending 1 image every 3 seconds (forever).");
}

void loop() {
  delay(1000);
}
