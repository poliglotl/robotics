#include <WiFi.h>
#include <WebServer.h>

const char *AP_SSID = "Car";
const char *AP_PASS = "test1234";

WebServer server(80);
int apChannel = 1;

int pickChannel() {
  Serial.println("skan efira...");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  delay(100);

  int n = WiFi.scanNetworks();
  long load[14] = {0};

  for (int i = 0; i < n; i++) {
    int ch = WiFi.channel(i);
    if (ch < 1 || ch > 13) continue;
    long w = WiFi.RSSI(i) + 100;
    if (w < 1) w = 1;
    w = w * w;
    for (int c = ch - 2; c <= ch + 2; c++) {
      if (c >= 1 && c <= 13) load[c] += (c == ch) ? w : w / 3;
    }
  }
  WiFi.scanDelete();

  int best = 1;
  for (int c = 1; c <= 11; c++) {
    if (load[c] < load[best]) best = c;
  }
  Serial.printf("setey v efire: %d\n", n);
  for (int c = 1; c <= 11; c++) {
    Serial.printf("  ch %2d | zagruzka %7ld %s\n", c, load[c], c == best ? "<== VYBRAN" : "");
  }
  return best;
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>AP test</title></head><body style='font-family:sans-serif'>"
                "<h2>ESP32-CAM: AP rabotaet</h2>"
                "<p>Channel: " + String(apChannel) + "</p>"
                "<p>Uptime: " + String(millis() / 1000) + " c</p>"
                "<p>Clients: " + String(WiFi.softAPgetStationNum()) + "</p>"
                "<p>Free heap: " + String(ESP.getFreeHeap()) + "</p>"
                "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== AP autochannel ===");

  apChannel = pickChannel();

  WiFi.mode(WIFI_AP);
  delay(100);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS, apChannel, 0, 4);
  Serial.printf("softAP(\"%s\", ch %d): %s\n", AP_SSID, apChannel, ok ? "OK" : "FAIL");

  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);

  Serial.print("AP IP:   ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("TX power: %d (19.5 dBm = 78)\n", (int)WiFi.getTxPower());

  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started on port 80");
}

void loop() {
  server.handleClient();

  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    Serial.printf("[%6lu s] ch %d | clients: %u | heap: %u\n",
                  millis() / 1000, apChannel,
                  WiFi.softAPgetStationNum(),
                  ESP.getFreeHeap());
  }
}
