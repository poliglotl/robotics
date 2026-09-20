/*
 * 13_selftest - самопроверка всего железа ESP32-CAM.
 *
 * Прогоняет по очереди процессор, флеш, PSRAM, фару, камеру, моторы и Wi-Fi,
 * в конце печатает таблицу PASS/FAIL.
 *
 * КОЛЁСА ВЫВЕСИТЬ - тест моторов их крутит.
 * АНТЕННУ ПРИКРУТИТЬ - тест Wi-Fi передаёт.
 *
 * Tools -> Partition Scheme: Huge APP   |   PSRAM: Enabled
 */

#include <WiFi.h>
#include "esp_camera.h"

#define TEST_MOTORS 1     // 0 = не крутить моторы

// пины (Keyestudio KS5024)
#define PIN_L_IN1 13
#define PIN_L_IN2 12
#define PIN_R_IN1 15
#define PIN_R_IN2 14
#define PIN_FLASH_LED 4

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define PWM_FREQ 1000
#define PWM_RES  8
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch)       ledcAttach((pin), PWM_FREQ, PWM_RES)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((pin), (duty))
#else
  #define PWM_SETUP(pin, ch)       do { ledcSetup((ch), PWM_FREQ, PWM_RES); \
                                        ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((ch), (duty))
#endif

#define MAX_RESULTS 10
static const char *resName[MAX_RESULTS];
static const char *resVal[MAX_RESULTS];
static int resCount = 0;

static void record(const char *name, bool ok, const char *note) {
  if (resCount < MAX_RESULTS) {
    resName[resCount] = name;
    resVal[resCount]  = ok ? "PASS" : "FAIL";
    resCount++;
  }
  Serial.printf("  -> %s  %s\n\n", ok ? "PASS" : "FAIL", note ? note : "");
}

// ------------------------------------------------------------------- 1. чип
static void testChip() {
  Serial.println("[1] PROTSESSOR");
  Serial.printf("  model:    %s rev %d, yader: %d, %d MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(),
                ESP.getChipCores(), ESP.getCpuFreqMHz());
  Serial.printf("  MAC:      %s\n", WiFi.macAddress().c_str());
  Serial.printf("  reset:    %d (1=power on, 6=brownout)\n", (int)esp_reset_reason());
  record("Protsessor", true, "");
}

// ------------------------------------------------------------------ 2. флеш
static void testFlash() {
  Serial.println("[2] FLESH-PAMYAT");
  uint32_t sz = ESP.getFlashChipSize();
  Serial.printf("  razmer:   %u bayt (%u MB)\n", sz, sz / (1024 * 1024));
  Serial.printf("  chastota: %u Hz\n", ESP.getFlashChipSpeed());
  Serial.printf("  skech:    %u iz %u bayt\n", ESP.getSketchSize(), ESP.getFreeSketchSpace());
  record("Flesh", sz >= 4 * 1024 * 1024, sz >= 4 * 1024 * 1024 ? "" : "menshe 4 MB!");
}

// ----------------------------------------------------------------- 3. PSRAM
static void testPsram() {
  Serial.println("[3] PSRAM");
  if (!psramFound()) {
    Serial.println("  ne naydena - vklyuchi Tools -> PSRAM -> Enabled");
    record("PSRAM", false, "ne naydena");
    return;
  }
  size_t total = ESP.getPsramSize();
  Serial.printf("  razmer:   %u bayt (%u MB)\n", total, total / (1024 * 1024));

  const size_t N = 64 * 1024;
  uint8_t *buf = (uint8_t *)ps_malloc(N);
  if (!buf) {
    record("PSRAM", false, "ps_malloc ne dal pamyat");
    return;
  }
  bool ok = true;
  for (size_t i = 0; i < N; i++) buf[i] = (uint8_t)(i * 31 + 7);
  for (size_t i = 0; i < N; i++) if (buf[i] != (uint8_t)(i * 31 + 7)) { ok = false; break; }
  free(buf);
  Serial.printf("  zapis/chtenie 64 KB: %s\n", ok ? "sovpalo" : "OSHIBKA");
  record("PSRAM", ok, "");
}

// ------------------------------------------------------------------ 4. фара
static void testLed() {
  Serial.println("[4] FARA (GPIO4)");
  Serial.println("  smotri na platu - belyy svetodiod dolzhen mignut 3 raza");
  pinMode(PIN_FLASH_LED, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_FLASH_LED, HIGH); delay(250);
    digitalWrite(PIN_FLASH_LED, LOW);  delay(250);
  }
  record("Fara", true, "proverit glazami");
}

// ---------------------------------------------------------------- 5. камера
static const char *sensorName(uint16_t pid) {
  switch (pid) {
    case 0x26:   return "OV2640";
    case 0x3660: return "OV3660";
    case 0x5640: return "OV5640";
    case 0x77:   return "OV7725";
    case 0x76:   return "OV7670";
    default:     return "neizvestnyy";
  }
}

static void testCamera() {
  Serial.println("[5] KAMERA");

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;   c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;   c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;   c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;   c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;   c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;     c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_QVGA;
  c.jpeg_quality = 12;
  c.fb_count     = 1;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("  esp_camera_init: oshibka 0x%x\n", err);
    Serial.println("  0x105 = shleyf; 0x101 = pamyat; 0x103 = pины");
    record("Kamera", false, "init failed");
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) Serial.printf("  sensor:   %s (PID 0x%04x)\n", sensorName(s->id.PID), s->id.PID);

  int good = 0;
  size_t bytes = 0;
  uint32_t t0 = millis();
  for (int i = 0; i < 10; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      good++;
      bytes += fb->len;
      if (i == 0) Serial.printf("  kadr:     %ux%u, %u bayt\n", fb->width, fb->height, fb->len);
      esp_camera_fb_return(fb);
    }
  }
  uint32_t dt = millis() - t0;
  Serial.printf("  snyato:   %d iz 10 kadrov za %lu ms\n", good, dt);
  if (good) Serial.printf("  sredniy:  %u bayt, ~%lu kadrov/sek\n", bytes / good, good * 1000UL / (dt ? dt : 1));

  record("Kamera", good >= 8, good >= 8 ? "" : "kadry teryayutsya");
}

// ---------------------------------------------------------------- 6. моторы
static void testMotors() {
  Serial.println("[6] MOTORY");
#if !TEST_MOTORS
  Serial.println("  propushcheno (TEST_MOTORS 0)");
  record("Motory", true, "ne proveryalis");
  return;
#else
  Serial.println("  KOLYOSA DOLZHNY BYT VYVESHENY!");
  PWM_SETUP(PIN_L_IN1, 2); PWM_SETUP(PIN_L_IN2, 3);
  PWM_SETUP(PIN_R_IN1, 4); PWM_SETUP(PIN_R_IN2, 5);

  struct { int pin; int ch; const char *what; } steps[] = {
    { PIN_L_IN1, 2, "LEVAYA vperyod" },
    { PIN_L_IN2, 3, "LEVAYA nazad"   },
    { PIN_R_IN1, 4, "PRAVAYA vperyod"},
    { PIN_R_IN2, 5, "PRAVAYA nazad"  },
  };

  for (int i = 0; i < 4; i++) {
    Serial.printf("  %s ...\n", steps[i].what);
    PWM_WRITE(steps[i].pin, steps[i].ch, 200);
    delay(800);
    PWM_WRITE(steps[i].pin, steps[i].ch, 0);
    delay(400);
  }
  record("Motory", true, "proverit glazami: 4 dvizheniya podryad");
#endif
}

// ------------------------------------------------------------- 7. Wi-Fi приём
static void testWifiRx() {
  Serial.println("[7] WI-FI PRIYOM");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);

  int n = WiFi.scanNetworks();
  int best = -127;
  for (int i = 0; i < n; i++) if (WiFi.RSSI(i) > best) best = WiFi.RSSI(i);
  WiFi.scanDelete();

  Serial.printf("  setey:    %d\n", n);
  Serial.printf("  luchshiy: %d dBm\n", best);
  Serial.println("  norma v gorode: 10+ setey, luchshiy luchshe -60 dBm");
  record("Wi-Fi priyom", n >= 3 && best > -75, "");
}

// ---------------------------------------------------- 8. Wi-Fi передача
static void testWifiTx() {
  Serial.println("[8] WI-FI PEREDACHA");
  WiFi.mode(WIFI_AP);
  delay(200);
  bool ok = WiFi.softAP("SelfTestAP", NULL, 1, 0, 4);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.printf("  softAP:   %s\n", ok ? "podnyalas" : "FAIL");
  Serial.printf("  TX power: %d (78 = max)\n", (int)WiFi.getTxPower());
  Serial.println();
  Serial.println("  ETOT TEST PLATA SAMA PROVERIT NE MOZHET.");
  Serial.println("  Voz'mi telefon ili noutbuk i poishchi set \"SelfTestAP\".");
  Serial.println("  Vidno  -> peredatchik rabotaet.");
  Serial.println("  Ne vidno v 20 sm -> peredatchik mertv.");
  record("Wi-Fi peredacha", ok, "proverit vneshnim ustroystvom");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("==========================================");
  Serial.println("      SAMOPROVERKA ZHELEZA ESP32-CAM      ");
  Serial.println("==========================================");
  Serial.println();

  testChip();
  testFlash();
  testPsram();
  testLed();
  testCamera();
  testMotors();
  testWifiRx();
  testWifiTx();

  Serial.println("==========================================");
  Serial.println("                 ITOGI                    ");
  Serial.println("==========================================");
  for (int i = 0; i < resCount; i++) {
    Serial.printf("  %-16s %s\n", resName[i], resVal[i]);
  }
  Serial.println("==========================================");
  Serial.printf("  heap svobodno: %u bayt\n", ESP.getFreeHeap());
  Serial.println();
  Serial.println("Fara i motory proveryayutsya glazami.");
  Serial.println("Peredacha - poiskom seti \"SelfTestAP\" s telefona.");
}

void loop() {
  delay(5000);
  Serial.printf("[%6lu s] AP klientov: %u | heap: %u\n",
                millis() / 1000, WiFi.softAPgetStationNum(), ESP.getFreeHeap());
}
