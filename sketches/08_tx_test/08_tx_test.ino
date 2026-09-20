#include <WiFi.h>

// ===== Оставьте пустым - скетч сам найдёт открытую сеть.        =====
// ===== Или впишите своё имя сети и пароль, чтобы проверить её.  =====
const char *MY_SSID = "";
const char *MY_PASS = "";
// ====================================================================

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

  String target = MY_SSID;
  String pass   = MY_PASS;
  String bestOpen = "";
  int    bestRssi = -127;

  for (int i = 0; i < n && i < 40; i++) {
    bool isOpen = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    Serial.printf("%2d | %4d dBm | %-8s | %s\n", i + 1, WiFi.RSSI(i),
                  isOpen ? "OTKRYTA" : "s parolem", WiFi.SSID(i).c_str());
    if (isOpen && WiFi.RSSI(i) > bestRssi) {
      bestRssi = WiFi.RSSI(i);
      bestOpen = WiFi.SSID(i);
    }
  }
  WiFi.scanDelete();

  if (target.length() == 0) {
    if (bestOpen.length() == 0) {
      Serial.println("otkrytyh setey ne naydeno - vpishite svoyu set v MY_SSID");
      return;
    }
    target = bestOpen;
    pass   = "";
    Serial.printf("avtovybor: samaya silnaya OTKRYTAYA set \"%s\", %d dBm\n",
                  target.c_str(), bestRssi);
  }

  Serial.printf("podklyuchayus k \"%s\"", target.c_str());
  WiFi.begin(target.c_str(), pass.c_str());

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
    Serial.println("Set otkrytaya, parolya net - znachit delo ne v parole.");
  }
}

void loop() {
  delay(5000);
  Serial.printf("status: %s\n", statusText(WiFi.status()));
}
