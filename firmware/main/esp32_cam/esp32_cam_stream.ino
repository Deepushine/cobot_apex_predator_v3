/*
 * ═══════════════════════════════════════════════════════════════
 *  APEX_PREDATOR V3 — ESP32-CAM MJPEG Stream
 *  File: /firmware/esp32_cam/esp32_cam_stream.ino
 * ═══════════════════════════════════════════════════════════════

 *  FIRST BOOT:
 *    Connect to WiFi "APEX_CAM" password "12345678"
 *    Open http://192.168.4.1 → enter your WiFi SSID + password → save
 *    ESP restarts → Serial Monitor prints IP address
 *    Update config/robot_params.json → camera.esp32_stream_url
 *
 *  ARDUINO IDE SETTINGS:
 *    Board:     AI Thinker ESP32-CAM
 *    Partition: Huge APP (3MB No OTA)
 *    Speed:     115200
 * ═══════════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ── AI THINKER PIN MAP ────────────────────────────────────────────
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define FLASH_PIN       4   // active LOW

// ── STATIC IP — matches config/robot_params.json ─────────────────
IPAddress local_IP(192, 168, 1, 47);
IPAddress gateway(192, 168, 1,  1);
IPAddress subnet(255, 255, 255,  0);

// ── GLOBALS ───────────────────────────────────────────────────────
WebServer server(80);
String    ssid, password;
bool      configured = false;

// ── CAMERA INIT ───────────────────────────────────────────────────
bool initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk      = XCLK_GPIO_NUM;
  cfg.pin_pclk      = PCLK_GPIO_NUM;
  cfg.pin_vsync     = VSYNC_GPIO_NUM;
  cfg.pin_href      = HREF_GPIO_NUM;
  cfg.pin_sscb_sda  = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl  = SIOC_GPIO_NUM;
  cfg.pin_pwdn      = PWDN_GPIO_NUM;
  cfg.pin_reset     = RESET_GPIO_NUM;
  cfg.xclk_freq_hz  = 20000000;
  cfg.pixel_format  = PIXFORMAT_JPEG;
  cfg.frame_size    = FRAMESIZE_VGA;   // 640x480
  cfg.jpeg_quality  = 12;
  cfg.fb_count      = 2;

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("ERR: Camera init failed");
    return false;
  }
  Serial.println("✓ Camera ready (640x480)");
  return true;
}

// ── MJPEG STREAM ─────────────────────────────────────────────────
void handleStream() {
  WiFiClient client = server.client();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "multipart/x-mixed-replace;boundary=frame");

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { delay(10); continue; }

    client.print("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ");
    client.print(fb->len);
    client.print("\r\n\r\n");
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);
  }
}

// ── ROOT PAGE ─────────────────────────────────────────────────────
void handleRoot() {
  String ip = WiFi.localIP().toString();
  server.send(200, "text/html",
    "<h2 style='font-family:Arial'>APEX_PREDATOR Camera</h2>"
    "<img src='/stream' style='width:640px;border:2px solid #333'><br><br>"
    "<b>Stream URL:</b> http://" + ip + "/stream<br>"
    "<a href='/config'>Change WiFi</a> &nbsp;|&nbsp; "
    "<a href='javascript:fetch(\"/flash/on\")'>Flash ON</a> &nbsp;|&nbsp; "
    "<a href='javascript:fetch(\"/flash/off\")'>Flash OFF</a>"
  );
}

// ── WIFI CONFIG PORTAL ───────────────────────────────────────────
void handleConfigPage() {
  server.send(200, "text/html",
    "<h2 style='font-family:Arial'>APEX_CAM WiFi Setup</h2>"
    "<form method='POST' action='/save'>"
    "SSID: <input name='ssid' style='width:200px'><br><br>"
    "Password: <input name='pass' type='password' style='width:200px'><br><br>"
    "<input type='submit' value='Save and Restart'>"
    "</form>"
  );
}

void handleConfigSave() {
  Preferences prefs;
  prefs.begin("cam", false);
  prefs.putString("ssid", server.arg("ssid"));
  prefs.putString("pass", server.arg("pass"));
  prefs.end();
  server.send(200, "text/html",
    "<h2 style='font-family:Arial'>Saved! Restarting...</h2>"
    "<p>Find the IP in Serial Monitor after restart.</p>"
  );
  delay(1500);
  ESP.restart();
}

// ── SETUP ─────────────────────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable brownout
  Serial.begin(115200);
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, HIGH);  // flash OFF (active LOW)

  // Load stored WiFi credentials
  Preferences prefs;
  prefs.begin("cam", true);
  ssid     = "iykyk"; 
  password = "77777777";
  prefs.end();
  configured = ssid.length() > 0;

  if (!configured) {
    // ── First boot: AP mode ──────────────────────────────────────
    WiFi.softAP("APEX_CAM", "12345678");
    Serial.println("AP mode — no credentials stored");
    Serial.print("Config portal: http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("Connect to WiFi: APEX_CAM / 12345678");

    server.on("/",     handleConfigPage);
    server.on("/save", handleConfigSave);
    server.begin();

    // Blink flash to indicate AP mode
    while (true) {
      server.handleClient();
      digitalWrite(FLASH_PIN, LOW);  delay(300);
      digitalWrite(FLASH_PIN, HIGH); delay(300);
    }
  }

  // ── Normal boot: connect to WiFi ─────────────────────────────
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid.c_str(), password.c_str());

  Serial.print("Connecting to: "); Serial.println(ssid);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500); Serial.print(".");
    digitalWrite(FLASH_PIN, !digitalRead(FLASH_PIN));  // blink
    attempts++;
  }
  digitalWrite(FLASH_PIN, HIGH);  // flash off

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed — clearing credentials, restarting");
    Preferences p; p.begin("cam", false);
    p.putString("ssid", ""); p.end();
    delay(1000); ESP.restart();
  }

  Serial.println("\n✓ WiFi connected!");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
  Serial.print("Stream URL: http://"); Serial.print(WiFi.localIP());
  Serial.println("/stream");
  Serial.println("→ Update config/robot_params.json with this IP!");

  // Init camera
  initCamera();

  // Routes
  server.on("/",          handleRoot);
  server.on("/stream",    handleStream);
  server.on("/config",    handleConfigPage);
  server.on("/save",      handleConfigSave);
  server.on("/flash/on",  []{ digitalWrite(FLASH_PIN, LOW);  server.send(200, "text/plain", "ON");  });
  server.on("/flash/off", []{ digitalWrite(FLASH_PIN, HIGH); server.send(200, "text/plain", "OFF"); });
  server.on("/reset",     []{ server.send(200, "text/plain", "Restarting..."); delay(100); ESP.restart(); });

  server.begin();
  Serial.println("✓ Web server started");

  // 3 quick flashes = ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(FLASH_PIN, LOW);  delay(100);
    digitalWrite(FLASH_PIN, HIGH); delay(100);
  }
}

// ── LOOP ──────────────────────────────────────────────────────────
void loop() {
  server.handleClient();
}