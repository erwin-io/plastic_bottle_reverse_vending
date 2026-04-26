#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <plastic-bottle-detection_inferencing.h>

HardwareSerial UnoSerial(1);
WebServer server(80);

// =============================
// WIFI STA
// =============================
static const char* WIFI_SSID = "HG8145V5_D0A04";
static const char* WIFI_PASS = "p75z~${Tn2Iy";

// =============================
// ESP32-S3-WROOM N16R8 CAM + OV3660
// Safe DRAM mode pin map
// =============================
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

// =============================
// UART TO UNO
// =============================
static const int UNO_RX_PIN = 44;
static const int UNO_TX_PIN = 43;
static const uint32_t UNO_BAUD = 9600;

// =============================
// DECISION THRESHOLD
// =============================
static const float TH_PLASTIC = 0.70f;

// =============================
// GLOBAL STATE
// =============================
static uint8_t* ei_buf = nullptr;
static size_t ei_buf_len = 0;

static bool wifiConnected = false;
static bool serverStarted = false;
static bool cameraReady = false;

static unsigned long lastWifiRetry = 0;
static unsigned long lastStatusPrint = 0;
static unsigned long lastInferDurationMs = 0;
static unsigned long lastLoopRun = 0;

static float lastPlasticScore = 0.0f;
static String lastLabel = "empty";
static String lastCameraMessage = "Camera not initialized yet";
static uint32_t inferCount = 0;

// =============================
// HTML UI
// =============================
static String htmlPage() {
    String html;
    html += "<!doctype html><html><head><meta charset='utf-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>ESP32-S3 Plastic Bottle Detection</title>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:20px;}";
    html += ".wrap{max-width:1100px;margin:auto;}";
    html += ".grid{display:grid;grid-template-columns:360px 1fr;gap:16px;}";
    html += "@media(max-width:900px){.grid{grid-template-columns:1fr;}}";
    html += ".card{background:#111827;border:1px solid #1f2937;border-radius:16px;padding:18px;box-shadow:0 4px 18px rgba(0,0,0,.25);}";
    html += ".title{font-size:26px;font-weight:800;margin-bottom:8px;}";
    html += ".muted{color:#94a3b8;font-size:14px;margin-bottom:10px;}";
    html += ".pill{display:inline-block;padding:10px 14px;border-radius:999px;font-weight:700;margin-bottom:10px;}";
    html += ".green{background:#14532d;color:#bbf7d0;}";
    html += ".gray{background:#374151;color:#e5e7eb;}";
    html += ".big{font-size:34px;font-weight:800;margin:6px 0 16px 0;}";
    html += ".kv{display:grid;grid-template-columns:140px 1fr;gap:10px;font-size:14px;}";
    html += ".preview{width:100%;border-radius:12px;border:1px solid #334155;background:#020617;min-height:220px;object-fit:contain;}";
    html += ".info{color:#cbd5e1;word-break:break-word;}";
    html += "button{background:#2563eb;color:#fff;border:none;padding:10px 14px;border-radius:10px;cursor:pointer;font-weight:700;margin-top:14px;margin-right:8px;}";
    html += "</style></head><body>";
    html += "<div class='wrap'><div class='grid'>";

    html += "<div class='card'>";
    html += "<div class='title'>ESP32-S3 Plastic Bottle Detection</div>";
    html += "<div class='muted'>Edge Impulse + WiFi STA + Web Server + UNO Serial</div>";
    html += "<div id='labelPill' class='pill gray'>empty</div>";
    html += "<div id='score' class='big'>0.000</div>";
    html += "<div class='kv'>";
    html += "<div>IP Address</div><div id='ip'>-</div>";
    html += "<div>WiFi</div><div id='wifi'>-</div>";
    html += "<div>Inference Count</div><div id='count'>-</div>";
    html += "<div>Last Infer</div><div id='lastinfer'>-</div>";
    html += "<div>Camera</div><div id='camera'>-</div>";
    html += "<div>Camera Info</div><div id='camerainfo' class='info'>-</div>";
    html += "</div>";
    html += "<button onclick='loadData()'>Refresh Status</button>";
    html += "<button onclick='reloadPreview()'>Refresh Preview</button>";
    html += "</div>";

    html += "<div class='card'>";
    html += "<div class='title' style='font-size:20px'>Camera Preview</div>";
    html += "<div class='muted'>Latest captured frame</div>";
    html += "<img id='preview' class='preview' src='' alt='preview'>";
    html += "</div>";

    html += "</div>";

    html += "<script>";
    html += "let lastCameraReady = false;";
    html += "async function loadData(){";
    html += "try{";
    html += "const r=await fetch('/json');";
    html += "const j=await r.json();";
    html += "lastCameraReady = j.camera_ready;";
    html += "document.getElementById('score').textContent=Number(j.plastic_score).toFixed(3);";
    html += "document.getElementById('ip').textContent=j.ip;";
    html += "document.getElementById('wifi').textContent=j.wifi_connected ? 'CONNECTED' : 'NOT CONNECTED';";
    html += "document.getElementById('count').textContent=j.inference_count;";
    html += "document.getElementById('lastinfer').textContent=j.last_infer_ms + ' ms';";
    html += "document.getElementById('camera').textContent=j.camera_ready ? 'READY' : 'NOT READY';";
    html += "document.getElementById('camerainfo').textContent=j.camera_message;";
    html += "const pill=document.getElementById('labelPill');";
    html += "pill.textContent=j.label;";
    html += "pill.className='pill ' + (j.label==='plastic_bottle' ? 'green' : 'gray');";
    html += "if(!j.camera_ready){document.getElementById('preview').removeAttribute('src');}";
    html += "}catch(e){}";
    html += "}";
    html += "function reloadPreview(){";
    html += "if(!lastCameraReady){return;}";
    html += "document.getElementById('preview').src='/capture.jpg?t=' + Date.now();";
    html += "}";
    html += "setInterval(loadData,1000);";
    html += "setInterval(function(){ if(lastCameraReady){ reloadPreview(); } },3000);";
    html += "loadData();";
    html += "</script></body></html>";
    return html;
}

// =============================
// WEB HANDLERS
// =============================
void handleRoot() {
    server.send(200, "text/html", htmlPage());
}

void handleJson() {
    String ip = wifiConnected ? WiFi.localIP().toString() : String("0.0.0.0");

    String json = "{";
    json += "\"label\":\"" + lastLabel + "\",";
    json += "\"plastic_score\":" + String(lastPlasticScore, 4) + ",";
    json += "\"wifi_connected\":";
    json += (wifiConnected ? "true" : "false");
    json += ",";
    json += "\"camera_ready\":";
    json += (cameraReady ? "true" : "false");
    json += ",";
    json += "\"camera_message\":\"";
    String safeMsg = lastCameraMessage;
    safeMsg.replace("\\", "\\\\");
    safeMsg.replace("\"", "\\\"");
    json += safeMsg;
    json += "\",";
    json += "\"ip\":\"" + ip + "\",";
    json += "\"inference_count\":" + String(inferCount) + ",";
    json += "\"last_infer_ms\":" + String(lastInferDurationMs);
    json += "}";
    server.send(200, "application/json", json);
}

void handleCaptureJpg() {
    if (!cameraReady) {
        server.send(503, "text/plain", "Camera not ready");
        return;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Capture failed");
        return;
    }

    if (fb->format != PIXFORMAT_JPEG) {
        esp_camera_fb_return(fb);
        server.send(500, "text/plain", "Frame is not JPEG");
        return;
    }

    WiFiClient client = server.client();
    server.setContentLength(fb->len);
    server.send(200, "image/jpeg", "");
    client.write(fb->buf, fb->len);

    esp_camera_fb_return(fb);
}

void handleFavicon() {
    server.send(204);
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

void startServerIfNeeded() {
    if (!wifiConnected || serverStarted) return;

    server.on("/", HTTP_GET, handleRoot);
    server.on("/json", HTTP_GET, handleJson);
    server.on("/capture.jpg", HTTP_GET, handleCaptureJpg);
    server.on("/favicon.ico", HTTP_GET, handleFavicon);
    server.onNotFound(handleNotFound);
    server.begin();

    serverStarted = true;

    Serial.println("Web server started");
    Serial.print("Open: http://");
    Serial.println(WiFi.localIP());
}

// =============================
// WIFI
// =============================
void connectWifi() {
    Serial.println("Starting WiFi...");
    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    delay(300);

    WiFi.mode(WIFI_STA);
    delay(200);

    WiFi.setSleep(false);
    delay(100);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lastWifiRetry = millis();
}

void maintainWifi() {
    wl_status_t st = WiFi.status();
    wifiConnected = (st == WL_CONNECTED);

    if (!wifiConnected) {
        if (millis() - lastWifiRetry >= 10000) {
            Serial.println("Retrying WiFi...");
            WiFi.disconnect();
            delay(100);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            lastWifiRetry = millis();
        }
        return;
    }

    startServerIfNeeded();
}

// =============================
// CAMERA + EI
// =============================
static int ei_get_data(size_t offset, size_t length, float* out_ptr) {
    for (size_t i = 0; i < length; i++) out_ptr[i] = (float)ei_buf[offset + i];
    return 0;
}

bool init_camera() {
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

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;

    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_QVGA;   // sharper than QQVGA
    config.jpeg_quality = 10;               // lower = better JPEG quality
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location  = CAMERA_FB_IN_DRAM;

    Serial.println("Initializing camera...");
    Serial.printf("XCLK=%d SDA=%d SCL=%d PCLK=%d VSYNC=%d HREF=%d\n",
                  XCLK_GPIO_NUM, SIOD_GPIO_NUM, SIOC_GPIO_NUM,
                  PCLK_GPIO_NUM, VSYNC_GPIO_NUM, HREF_GPIO_NUM);
    Serial.printf("D0..D7 = %d %d %d %d %d %d %d %d\n",
                  Y2_GPIO_NUM, Y3_GPIO_NUM, Y4_GPIO_NUM, Y5_GPIO_NUM,
                  Y6_GPIO_NUM, Y7_GPIO_NUM, Y8_GPIO_NUM, Y9_GPIO_NUM);

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Camera init failed: 0x%x", (unsigned int)err);
        lastCameraMessage = String(buf);
        Serial.println(lastCameraMessage);
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (!s) {
        lastCameraMessage = "Camera sensor pointer is NULL";
        Serial.println(lastCameraMessage);
        return false;
    }

    // image tuning
    s->set_brightness(s, 1);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_sharpness(s, 1);
    s->set_denoise(s, 1);
    s->set_gainceiling(s, (gainceiling_t)4);
    s->set_quality(s, 10);
    s->set_special_effect(s, 0);

    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);

    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, 1);

    s->set_gain_ctrl(s, 1);
    // do NOT force agc_gain to 0
    // do NOT force aec_value while auto exposure is enabled

    s->set_lenc(s, 1);

    char pidBuf[80];
    snprintf(pidBuf, sizeof(pidBuf), "Camera initialized successfully. PID=0x%02X", s->id.PID);
    lastCameraMessage = String(pidBuf);
    Serial.println(lastCameraMessage);

    return true;
}

void resize_center_rgb888(uint8_t* src, int sw, int sh, uint8_t* dst, int dw, int dh) {
    int crop = min(sw, sh);
    int x0 = (sw - crop) / 2;
    int y0 = (sh - crop) / 2;

    for (int y = 0; y < dh; y++) {
        int sy = y0 + (y * crop) / dh;
        for (int x = 0; x < dw; x++) {
            int sx = x0 + (x * crop) / dw;
            int sidx = (sy * sw + sx) * 3;
            int didx = (y * dw + x) * 3;
            dst[didx + 0] = src[sidx + 0];
            dst[didx + 1] = src[sidx + 1];
            dst[didx + 2] = src[sidx + 2];
        }
    }
}

bool capture_to_ei(int w, int h) {
    if (!cameraReady) return false;

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return false;

    int fw = fb->width;
    int fh = fb->height;

    size_t full_len = (size_t)fw * (size_t)fh * 3;
    uint8_t* full_rgb = (uint8_t*)malloc(full_len);
    if (!full_rgb) {
        esp_camera_fb_return(fb);
        return false;
    }

    bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, full_rgb);
    esp_camera_fb_return(fb);

    if (!ok) {
        free(full_rgb);
        return false;
    }

    size_t needed = (size_t)w * (size_t)h * 3;
    if (!ei_buf || ei_buf_len != needed) {
        if (ei_buf) {
            free(ei_buf);
            ei_buf = nullptr;
        }
        ei_buf = (uint8_t*)malloc(needed);
        ei_buf_len = needed;
    }

    if (!ei_buf) {
        free(full_rgb);
        return false;
    }

    resize_center_rgb888(full_rgb, fw, fh, ei_buf, w, h);
    free(full_rgb);
    return true;
}

float get_score(const ei_impulse_result_t& result, const char* label) {
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (strcmp(result.classification[i].label, label) == 0) {
            return result.classification[i].value;
        }
    }
    return 0.0f;
}

void runInferenceOnce() {
    const int W = EI_CLASSIFIER_INPUT_WIDTH;
    const int H = EI_CLASSIFIER_INPUT_HEIGHT;

    if (!capture_to_ei(W, H)) {
        Serial.println("capture_to_ei failed");
        lastLabel = "empty";
        lastPlasticScore = 0.0f;
        lastInferDurationMs = 0;
        UnoSerial.println("empty");
        return;
    }

    ei::signal_t signal;
    signal.total_length = (size_t)W * (size_t)H * 3;
    signal.get_data = ei_get_data;

    ei_impulse_result_t result = { 0 };

    unsigned long t0 = millis();
    EI_IMPULSE_ERROR rc = run_classifier(&signal, &result, false);
    lastInferDurationMs = millis() - t0;

    if (rc != EI_IMPULSE_OK) {
        Serial.printf("run_classifier error: %d\n", rc);
        lastLabel = "empty";
        lastPlasticScore = 0.0f;
        UnoSerial.println("empty");
        return;
    }

    float plastic = get_score(result, "plastic_bottle");
    lastPlasticScore = plastic;
    inferCount++;

    if (plastic >= TH_PLASTIC) {
        lastLabel = "plastic_bottle";
        UnoSerial.println("plastic_bottle");
        Serial.printf("plastic_bottle=%.3f -> TX plastic_bottle\n", plastic);
    } else {
        lastLabel = "empty";
        UnoSerial.println("empty");
        Serial.printf("plastic_bottle=%.3f -> TX empty\n", plastic);
    }
}

// =============================
// SETUP
// =============================
void setup() {
    Serial.begin(115200);
    delay(1500);

    UnoSerial.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);

    Serial.println();
    Serial.println("ESP32-S3 EI + WIFI + WEB + PREVIEW BOOT");
    Serial.println("Safe DRAM mode enabled");

    connectWifi();

    cameraReady = init_camera();
    if (!cameraReady) {
        Serial.println("Camera failed to start");
        UnoSerial.println("empty");
    } else {
        Serial.println("Camera ready");
    }
}

// =============================
// LOOP
// =============================
void loop() {
    maintainWifi();

    if (serverStarted) {
        server.handleClient();
    }

    if (cameraReady) {
        if (millis() - lastLoopRun >= 700) {
            lastLoopRun = millis();
            runInferenceOnce();
        }
    }

    if (millis() - lastStatusPrint >= 3000) {
        lastStatusPrint = millis();

        Serial.print("WiFi.status() = ");
        Serial.println((int)WiFi.status());

        if (wifiConnected) {
            Serial.print("IP: ");
            Serial.println(WiFi.localIP());
            Serial.print("RSSI: ");
            Serial.println(WiFi.RSSI());
        }

        Serial.print("cameraReady = ");
        Serial.println(cameraReady ? "true" : "false");
        Serial.print("lastCameraMessage = ");
        Serial.println(lastCameraMessage);
        Serial.print("lastLabel = ");
        Serial.println(lastLabel);
        Serial.print("lastPlasticScore = ");
        Serial.println(lastPlasticScore, 4);
        Serial.print("inferCount = ");
        Serial.println(inferCount);
        Serial.print("lastInferDurationMs = ");
        Serial.println(lastInferDurationMs);
    }

    delay(10);
}