/*
 * 03_car_ap_camera - рабочая прошивка 4WD камера-робота на ESP32-CAM.
 *
 * Что делает:
 *   - поднимает СВОЮ точку доступа (SSID "Car", пароль "test1234")
 *   - на http://192.168.4.1        - веб-пульт управления
 *   - на http://192.168.4.1:81/stream - MJPEG видео с камеры
 *   - управляет 4 моторами через драйвер (L298N / TB6612 / DRV8833)
 *
 * ==========================  ПЕРЕД ЗАЛИВКОЙ  ==========================
 * 1. Tools -> Board:            "AI Thinker ESP32-CAM"
 * 2. Tools -> Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"   <-- обязательно
 * 3. Tools -> PSRAM:            Enabled  (если пункт есть)
 * 4. Tools -> Upload Speed:     115200 (если срывается - 57600)
 * 5. GPIO0 замкнуть на GND, нажать RESET, залить, ПОТОМ разомкнуть и RESET.
 * 6. Питание: отдельные 5V минимум 2A на плату. От USB-TTL камера+Wi-Fi+моторы
 *    не живут - будет "Brownout detector was triggered" и вечный ребут.
 *
 * !!! ПРОВЕРЬ ПИНЫ МОТОРОВ НИЖЕ ПОД СВОЮ ПЛАТУ - они у разных кит-ов разные !!!
 * ======================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"

// ---------------------------------------------------------------- настройки
const char *AP_SSID    = "Car";
const char *AP_PASS    = "test1234";   // >= 8 символов
const int   AP_CHANNEL = 1;

#define ENABLE_CAMERA 1     // поставь 0, чтобы отладить только моторы+Wi-Fi

// --------------------------------------------------------------- пины моторов
// Свободных GPIO у ESP32-CAM с включённой камерой почти нет.
// Обычно кит-ы разводят моторы на 12/13/14/15 (это пины SD-карты, она не нужна).
// Если твой робот дёргается не туда или не едет - меняй ЭТИ четыре числа.
//
// ВНИМАНИЕ: GPIO12 - strapping-пин. Если драйвер тянет его в HIGH на старте,
// плата может не загрузиться. Если ловишь циклический ребут - перенеси этот
// мотор на GPIO2 и проверь заново.
#define PIN_L_IN1   12    // левая сторона, вперёд
#define PIN_L_IN2   13    // левая сторона, назад
#define PIN_R_IN1   14    // правая сторона, вперёд
#define PIN_R_IN2   15    // правая сторона, назад

#define PIN_FLASH_LED 4   // белый светодиод-фара на плате AI-Thinker

// Если у робота один общий ENA/ENB - раскомментируй и укажи пины:
// #define PIN_L_EN  2
// #define PIN_R_EN  16

// --------------------------------------------------------------- пины камеры
// Раскладка AI-Thinker ESP32-CAM (самая распространённая).
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

// ----------------------------------------------------------------- ШИМ (LEDC)
// Камера занимает LEDC-канал 0 / таймер 0 под XCLK, поэтому моторам
// отдаём каналы 2..5 (таймеры 1 и 2).
#define PWM_FREQ  1000
#define PWM_RES   8        // 0..255
#define CH_L_IN1  2
#define CH_L_IN2  3
#define CH_R_IN1  4
#define CH_R_IN2  5

// В core 3.x API LEDC сменился - поддерживаем обе версии.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch)        ledcAttach((pin), PWM_FREQ, PWM_RES)
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((pin), (duty))
#else
  #define PWM_SETUP(pin, ch)        do { ledcSetup((ch), PWM_FREQ, PWM_RES); \
                                         ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((ch), (duty))
#endif

// --------------------------------------------------------------- состояние
static int      g_speed    = 200;   // 0..255
static uint32_t g_lastCmd  = 0;
static const uint32_t FAILSAFE_MS = 2000;  // нет команд -> стоп

// ------------------------------------------------------------------ моторы
static void sideDrive(int pinA, int chA, int pinB, int chB, int v) {
  if (v > 0) {                      // вперёд
    PWM_WRITE(pinA, chA, v);
    PWM_WRITE(pinB, chB, 0);
  } else if (v < 0) {               // назад
    PWM_WRITE(pinA, chA, 0);
    PWM_WRITE(pinB, chB, -v);
  } else {                          // стоп (выбег)
    PWM_WRITE(pinA, chA, 0);
    PWM_WRITE(pinB, chB, 0);
  }
}

static void drive(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  sideDrive(PIN_L_IN1, CH_L_IN1, PIN_L_IN2, CH_L_IN2, left);
  sideDrive(PIN_R_IN1, CH_R_IN1, PIN_R_IN2, CH_R_IN2, right);
}

static void motorsInit() {
  PWM_SETUP(PIN_L_IN1, CH_L_IN1);
  PWM_SETUP(PIN_L_IN2, CH_L_IN2);
  PWM_SETUP(PIN_R_IN1, CH_R_IN1);
  PWM_SETUP(PIN_R_IN2, CH_R_IN2);

#ifdef PIN_L_EN
  pinMode(PIN_L_EN, OUTPUT); digitalWrite(PIN_L_EN, HIGH);
#endif
#ifdef PIN_R_EN
  pinMode(PIN_R_EN, OUTPUT); digitalWrite(PIN_R_EN, HIGH);
#endif

  drive(0, 0);
}

static void applyCommand(const char *cmd) {
  g_lastCmd = millis();
  int s = g_speed;
  int turn = s * 3 / 4;   // на развороте чуть мягче

  if      (!strcmp(cmd, "forward"))  drive( s,  s);
  else if (!strcmp(cmd, "backward")) drive(-s, -s);
  else if (!strcmp(cmd, "left"))     drive(-turn,  turn);
  else if (!strcmp(cmd, "right"))    drive( turn, -turn);
  else if (!strcmp(cmd, "stop"))     drive(0, 0);
  else if (!strcmp(cmd, "led_on"))   digitalWrite(PIN_FLASH_LED, HIGH);
  else if (!strcmp(cmd, "led_off"))  digitalWrite(PIN_FLASH_LED, LOW);
  else Serial.printf("неизвестная команда: %s\n", cmd);
}

// ------------------------------------------------------------------ веб UI
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Car</title>
<style>
  body{margin:0;background:#15171c;color:#e6e8ee;font:16px/1.4 system-ui,sans-serif;
       text-align:center;padding:12px;-webkit-user-select:none;user-select:none}
  #cam{width:100%;max-width:480px;border-radius:10px;background:#000;aspect-ratio:4/3}
  .pad{display:grid;grid-template-columns:repeat(3,84px);gap:10px;
       justify-content:center;margin:18px auto}
  button{height:78px;border:0;border-radius:12px;background:#2b3040;color:#e6e8ee;
         font-size:26px;touch-action:manipulation}
  button:active{background:#4a71f0}
  .wide{grid-column:1/4;height:48px;font-size:16px}
  .row{max-width:300px;margin:0 auto;display:flex;gap:10px;align-items:center}
  input[type=range]{flex:1}
  #st{color:#8b93a7;font-size:13px;margin-top:10px}
</style></head><body>
<img id="cam" alt="нет видео">
<div class="pad">
  <span></span><button data-go="forward">&#9650;</button><span></span>
  <button data-go="left">&#9664;</button>
  <button data-go="stop">&#9632;</button>
  <button data-go="right">&#9654;</button>
  <span></span><button data-go="backward">&#9660;</button><span></span>
  <button class="wide" id="led">фара вкл/выкл</button>
</div>
<div class="row">скорость<input type="range" id="sp" min="80" max="255" value="200"><span id="spv">200</span></div>
<div id="st">&nbsp;</div>
<script>
  var st = document.getElementById('st');
  function send(q){
    fetch('/action?' + q).then(function(){ st.textContent = q; })
                         .catch(function(){ st.textContent = 'нет связи'; });
  }
  document.querySelectorAll('button[data-go]').forEach(function(b){
    var dir = b.dataset.go;
    var press = function(e){ e.preventDefault(); send('go=' + dir); };
    var release = function(e){ e.preventDefault(); if (dir !== 'stop') send('go=stop'); };
    b.addEventListener('touchstart', press, {passive:false});
    b.addEventListener('touchend', release);
    b.addEventListener('mousedown', press);
    b.addEventListener('mouseup', release);
    b.addEventListener('mouseleave', release);
  });
  var ledOn = false;
  document.getElementById('led').onclick = function(){
    ledOn = !ledOn; send('go=' + (ledOn ? 'led_on' : 'led_off'));
  };
  var sp = document.getElementById('sp');
  sp.oninput = function(){
    document.getElementById('spv').textContent = sp.value;
    send('speed=' + sp.value);
  };
  document.addEventListener('keydown', function(e){
    var m = {ArrowUp:'forward', ArrowDown:'backward', ArrowLeft:'left', ArrowRight:'right'};
    if (m[e.key]) { e.preventDefault(); send('go=' + m[e.key]); }
  });
  document.addEventListener('keyup', function(e){
    if (e.key.indexOf('Arrow') === 0) send('go=stop');
  });
  // видео отдаётся отдельным сервером на порту 81
  document.getElementById('cam').src = 'http://' + location.hostname + ':81/stream';
</script></body></html>
)HTML";

static httpd_handle_t ctrl_httpd   = NULL;
static httpd_handle_t stream_httpd = NULL;

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t action_handler(httpd_req_t *req) {
  size_t len = httpd_req_get_url_query_len(req) + 1;
  if (len <= 1) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no query");
    return ESP_FAIL;
  }

  char *query = (char *)malloc(len);
  if (!query) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_FAIL;
  }

  esp_err_t res = ESP_OK;
  if (httpd_req_get_url_query_str(req, query, len) == ESP_OK) {
    char value[24] = {0};
    if (httpd_query_key_value(query, "go", value, sizeof(value)) == ESP_OK) {
      applyCommand(value);
    } else if (httpd_query_key_value(query, "speed", value, sizeof(value)) == ESP_OK) {
      g_speed = constrain(atoi(value), 0, 255);
      Serial.printf("speed = %d\n", g_speed);
    } else {
      res = ESP_FAIL;
    }
  } else {
    res = ESP_FAIL;
  }
  free(query);

  if (res != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad query");
    return ESP_FAIL;
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "ok", 2);
}

#if ENABLE_CAMERA
#define PART_BOUNDARY "frameboundary"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char part[64];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("esp_camera_fb_get() failed");
      res = ESP_FAIL;
      break;
    }

    uint8_t *jpg     = fb->buf;
    size_t   jpg_len = fb->len;
    bool     owned   = false;    // true, если буфер выделили мы сами

    if (fb->format != PIXFORMAT_JPEG) {
      if (!frame2jpg(fb, 80, &jpg, &jpg_len)) {
        esp_camera_fb_return(fb);
        res = ESP_FAIL;
        break;
      }
      owned = true;
      esp_camera_fb_return(fb);
      fb = NULL;
    }

    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)jpg_len);
    res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg, jpg_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));

    if (owned) free(jpg);
    if (fb) esp_camera_fb_return(fb);
    if (res != ESP_OK) break;     // клиент отключился

    // отдаём время Wi-Fi/управлению, иначе пульт начинает лагать
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return res;
}

static bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;   c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;   c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;   c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;   c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.grab_mode    = CAMERA_GRAB_LATEST;   // свежий кадр вместо накопленной очереди

  if (psramFound()) {
    c.frame_size   = FRAMESIZE_VGA;      // 640x480
    c.jpeg_quality = 12;
    c.fb_count     = 2;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    // без PSRAM большие кадры не влезут в RAM
    c.frame_size   = FRAMESIZE_QVGA;     // 320x240
    c.jpeg_quality = 14;
    c.fb_count     = 1;
    c.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("esp_camera_init failed: 0x%x\n", err);
    Serial.println("Проверь: шлейф камеры, PSRAM=Enabled, питание 5V/2A.");
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    // для езды важнее плавность, чем разрешение
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
  }
  Serial.println("камера: OK");
  return true;
}
#endif  // ENABLE_CAMERA

static void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port   = 32768;
  cfg.max_uri_handlers = 8;
  cfg.lru_purge_enable = true;

  httpd_uri_t index_uri  = { "/",       HTTP_GET, index_handler,  NULL };
  httpd_uri_t action_uri = { "/action", HTTP_GET, action_handler, NULL };

  if (httpd_start(&ctrl_httpd, &cfg) == ESP_OK) {
    httpd_register_uri_handler(ctrl_httpd, &index_uri);
    httpd_register_uri_handler(ctrl_httpd, &action_uri);
    Serial.println("пульт:  http://192.168.4.1/");
  } else {
    Serial.println("не удалось запустить сервер на порту 80");
  }

#if ENABLE_CAMERA
  // Стрим держит поток httpd занятым, поэтому он на ОТДЕЛЬНОМ сервере/порту.
  httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
  scfg.server_port = 81;
  scfg.ctrl_port   = 32769;
  scfg.lru_purge_enable = true;

  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
  if (httpd_start(&stream_httpd, &scfg) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("видео:  http://192.168.4.1:81/stream");
  } else {
    Serial.println("не удалось запустить сервер стрима на порту 81");
  }
#endif
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(1000);
  Serial.println();
  Serial.println("=== ESP32-CAM 4WD car ===");
  Serial.printf("reset reason: %d\n", (int)esp_reset_reason());
  Serial.printf("PSRAM: %s\n", psramFound() ? "найден" : "НЕТ");

  pinMode(PIN_FLASH_LED, OUTPUT);
  digitalWrite(PIN_FLASH_LED, LOW);

  // Моторы инициализируем ДО камеры и Wi-Fi, чтобы они не дёргались на старте.
  motorsInit();

#if ENABLE_CAMERA
  if (!cameraInit()) {
    Serial.println("продолжаю без камеры - управление будет работать");
  }
#endif

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  bool ap = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4);
  WiFi.setSleep(false);

  Serial.printf("softAP(\"%s\"): %s\n", AP_SSID, ap ? "OK" : "FAIL");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  startServers();
  g_lastCmd = millis();
}

void loop() {
  // Failsafe: если пульт отвалился (закрыли страницу, ушли из зоны) - стоп.
  static bool stopped = false;
  if (millis() - g_lastCmd > FAILSAFE_MS) {
    if (!stopped) {
      drive(0, 0);
      stopped = true;
      Serial.println("failsafe: стоп (нет команд)");
    }
  } else {
    stopped = false;
  }

  static uint32_t last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("[%6lu s] клиентов: %u | heap: %u | speed: %d\n",
                  millis() / 1000, WiFi.softAPGetStationNum(),
                  ESP.getFreeHeap(), g_speed);
  }

  delay(20);
}
