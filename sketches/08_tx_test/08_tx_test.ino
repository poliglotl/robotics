#include <WiFi.h>

// ===== ВПИШИТЕ СВОЙ ДОМАШНИЙ WI-FI, ТОЛЬКО 2.4 ГГц =====
const char *MY_SSID = "MyHomeWiFi";
const char *MY_PASS = "MyPassword";
// =======================================================

const char *statusText(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "0 IDLE";
    case WL_NO_SSID_AVAIL:   return "1 NO_SSID_AVAIL";
    case WL_CONNECTED:       return "3 CONNECTED";
    case WL_CONNECT_FAILED:  return "4 CONNECT_FAILED (parol?)";
    case WL_CONNECTION_LOST: return "5 CONNECTION_LOST";
    case WL_DISCONNECTED:    return "6 DISCONNECTED";
    default:                 return "drugoy";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== TX test ===");

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);

  Serial.println("skaniruyu efir...");
  int n = WiFi.scanNetworks();
  bool found = false;

  for (int i = 0; i < n && i < 40; i++) {
    bool mine = (WiFi.SSID(i) == MY_SSID);
    if (mine) found = true;
    Serial.printf("%2d | %4d dBm | %s %s\n", i + 1, WiFi.RSSI(i),
                  WiFi.SSID(i).c_str(), mine ? "<== VASHA SET" : "");
  }
  WiFi.scanDelete();

  if (!found) {
    Serial.printf("set \"%s\" NE naydena. Sverte imya so spiskom vyshe.\n", MY_SSID);
    return;
  }

  Serial.printf("podklyuchayus k \"%s\"", MY_SSID);
  WiFi.begin(MY_SSID, MY_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 25000) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("*** TX RABOTAET - peredatchik ispraven ***");
    Serial.print("IP:   ");
    Serial.println(WiFi.localIP());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.printf("*** NE PODKLYUCHILOS, status %s ***\n", statusText(WiFi.status()));
    Serial.println("Set vidna, a podklyuchitsya ne vyshlo - peredatchik ne rabotaet.");
  }
}

void loop() {
  delay(5000);
  Serial.printf("status: %s\n", statusText(WiFi.status()));
}
