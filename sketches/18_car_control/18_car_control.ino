/*
 * 18_car_control - главная прошивка: робот раздаёт Wi-Fi, телефон им управляет,
 * видео с камеры идёт в браузер. Новый модуль ESP32-CAM, передатчик живой.
 *
 * Сеть:  RobotCar  (без пароля)
 * Пульт: http://192.168.4.1
 * Видео: http://192.168.4.1:81/stream
 *
 * Tools -> Partition Scheme: Huge APP    |  Tools -> PSRAM: Enabled
 *
 * ПЕРЕД ЕЗДОЙ проверь разводку моторов кнопками "мотор A" / "мотор B"
 * на странице пульта и выставь флаги ниже. Колёса вывесить, антенна прикручена.
 */

#include <WiFi.h>
#include "esp_http_server.h"

// ------------------------------------------------------------- переключатели
#define ENABLE_CAMERA  1     // 0 = без камеры (отладка Wi-Fi и моторов)
#define ENABLE_MOTORS  1     // 0 = не трогать пины моторов вообще

// РАЗВОДКА МОТОРОВ. Драйвер даёт всего две группы выходов: A и B.
//   WIRING_SIDES 1 - A = левый борт, B = правый борт. Есть повороты.
//   WIRING_SIDES 0 - A = передние колёса, B = задние. ПОВОРОТОВ НЕТ,
//                    кнопки влево-вправо работать не будут физически.
// На нашем шасси 1: провода переставлены по схеме Keyestudio (оба правых
// мотора на OUT1/OUT2, оба левых на OUT3/OUT4), 19_wheel_sides подтвердил -
// 1 мигание левый борт, 2 мигания правый, все флаги ниже остаются 0.
#define WIRING_SIDES 1

// Группа A - это выходы OUT1/OUT2, группа B - OUT3/OUT4 (или наоборот,
// зависит от разводки платы-расширителя). Проверяется 19_wheel_sides.
// Кнопки влево-вправо работают наоборот -> SWAP_SIDES 1.
#define SWAP_SIDES 0

// Группа едет не в ту сторону - поменяй её флаг на 1.
#define INVERT_A 0
#define INVERT_B 0

#if ENABLE_CAMERA
  #include "esp_camera.h"
  #include "img_converters.h"
#endif

// ------------------------------------------------------------------ Wi-Fi
const char *AP_SSID = "RobotCar";   // без пароля
const int   AP_CHAN = 1;            // не подошёл - попробуй 6, потом 11

// --------------------------------------------------------------- пины моторов
// GPIO12 - strapping-пин. Ловишь циклический ребут - перенеси на GPIO2.
#define PIN_A_IN1   13
#define PIN_A_IN2   12
#define PIN_B_IN1   15
#define PIN_B_IN2   14

#define PIN_FLASH_LED 4   // белый светодиод-фара на плате AI-Thinker

// --------------------------------------------------------------- пины камеры
// Раскладка AI-Thinker ESP32-CAM.
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
// Камера занимает LEDC-канал 0 / таймер 0 под XCLK, моторам отдаём каналы 2..5.
#define PWM_FREQ  1000
#define PWM_RES   8        // 0..255
#define CH_A_IN1  2
#define CH_A_IN2  3
#define CH_B_IN1  4
#define CH_B_IN2  5

// В ядре 3.x API LEDC сменился - поддерживаем обе версии.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch)        ledcAttach((pin), PWM_FREQ, PWM_RES)
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((pin), (duty))
#else
  #define PWM_SETUP(pin, ch)        do { ledcSetup((ch), PWM_FREQ, PWM_RES); \
                                         ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((ch), (duty))
#endif

// --------------------------------------------------------------- состояние
static int      g_speed   = 200;    // 0..255
static uint32_t g_lastCmd = 0;
static volatile bool g_showRequest = false;   // нажали кнопку "номер"
static volatile bool g_showRunning = false;   // номер идёт, failsafe молчит
static const uint32_t FAILSAFE_MS = 2000;

// ------------------------------------------------------------------ моторы
static void groupDrive(int pinA, int chA, int pinB, int chB, int v) {
#if ENABLE_MOTORS
  if (v > 0) {
    PWM_WRITE(pinA, chA, v);
    PWM_WRITE(pinB, chB, 0);
  } else if (v < 0) {
    PWM_WRITE(pinA, chA, 0);
    PWM_WRITE(pinB, chB, -v);
  } else {
    PWM_WRITE(pinA, chA, 0);
    PWM_WRITE(pinB, chB, 0);
  }
#endif
}

// Низкий уровень: a - группа на пинах 13/12, b - группа на 15/14.
// Что это физически (борта или перед-зад) зависит от разводки проводов.
static void motors(int a, int b) {
#if INVERT_A
  a = -a;
#endif
#if INVERT_B
  b = -b;
#endif
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);
  groupDrive(PIN_A_IN1, CH_A_IN1, PIN_A_IN2, CH_A_IN2, a);
  groupDrive(PIN_B_IN1, CH_B_IN1, PIN_B_IN2, CH_B_IN2, b);
}

// Борта. Какая группа какой борт - решает разводка, а не код.
static void sides(int left, int right) {
#if SWAP_SIDES
  motors(right, left);
#else
  motors(left, right);
#endif
}

static void allStop() { motors(0, 0); }

static void motorsInit() {
#if ENABLE_MOTORS
  PWM_SETUP(PIN_A_IN1, CH_A_IN1);
  PWM_SETUP(PIN_A_IN2, CH_A_IN2);
  PWM_SETUP(PIN_B_IN1, CH_B_IN1);
  PWM_SETUP(PIN_B_IN2, CH_B_IN2);
  allStop();
  Serial.println("motors: init OK");
#else
  Serial.println("motors: DISABLED");
#endif
}

// Плавный разгон - робот не дёргается и не буксует.
static void ramp(int aFrom, int bFrom, int aTo, int bTo) {
  for (int i = 1; i <= 10; i++) {
    motors(aFrom + (aTo - aFrom) * i / 10,
           bFrom + (bTo - bFrom) * i / 10);
    delay(8);
  }
}

// ------------------------------------------------------------- номер по кнопке
// Крутится в loop(), а не в обработчике HTTP: иначе пульт бы завис на 20 секунд.
static void runShow() {
  const int spd = 170;

  digitalWrite(PIN_FLASH_LED, HIGH);
  delay(400);
  digitalWrite(PIN_FLASH_LED, LOW);

#if WIRING_SIDES
  // Квадрат с разворотами вокруг одного борта.
  for (int side = 0; side < 4; side++) {
    ramp(0, 0, spd, spd);  delay(1200);  ramp(spd, spd, 0, 0);  allStop();
    delay(250);
    ramp(0, 0, spd, 0);    delay(1300);  ramp(spd, 0, 0, 0);    allStop();
    delay(250);
  }
  // Покачивание.
  for (int i = 0; i < 3; i++) {
    ramp(0, 0, 0, spd);    delay(450);   ramp(0, spd, 0, 0);    allStop();
    delay(150);
    ramp(0, 0, spd, 0);    delay(900);   ramp(spd, 0, 0, 0);    allStop();
    delay(150);
  }
#else
  // Поворотов нет: проезд, откат, "шарканье" передней и задней парой.
  ramp(0, 0, spd, spd);    delay(1200);  ramp(spd, spd, 0, 0);  allStop();
  delay(400);
  ramp(0, 0, -spd, -spd);  delay(600);   ramp(-spd, -spd, 0, 0); allStop();
  delay(400);
  for (int i = 0; i < 6; i++) {
    motors(spd, 0);
    digitalWrite(PIN_FLASH_LED, HIGH);
    delay(220);
    allStop();
    digitalWrite(PIN_FLASH_LED, LOW);
    delay(120);
    motors(0, spd);
    delay(220);
    allStop();
    delay(120);
  }
  delay(300);
  ramp(0, 0, spd, spd);    delay(600);   ramp(spd, spd, 0, 0);  allStop();
#endif

  allStop();
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_FLASH_LED, HIGH);
    delay(150);
    digitalWrite(PIN_FLASH_LED, LOW);
    delay(150);
  }
}

// ---------------------------------------------------------------- команды
static void applyCommand(const char *cmd) {
  g_lastCmd = millis();
  int s = g_speed;

  if (!strcmp(cmd, "forward")) {
    sides(s, s);
  } else if (!strcmp(cmd, "backward")) {
    sides(-s, -s);
  } else if (!strcmp(cmd, "left")) {
#if WIRING_SIDES
    sides(0, s);           // правый борт толкает, левый стоит
#else
    Serial.println("povorot nevozmozhen: WIRING_SIDES 0");
#endif
  } else if (!strcmp(cmd, "right")) {
#if WIRING_SIDES
    sides(s, 0);           // левый борт толкает, правый стоит
#else
    Serial.println("povorot nevozmozhen: WIRING_SIDES 0");
#endif
  } else if (!strcmp(cmd, "stop")) {
    allStop();
  } else if (!strcmp(cmd, "test_a")) {
    motors(s, 0);          // только группа 13/12 - смотри, какие колёса
  } else if (!strcmp(cmd, "test_b")) {
    motors(0, s);          // только группа 15/14
  } else if (!strcmp(cmd, "show")) {
    g_showRequest = true;
  } else if (!strcmp(cmd, "led_on")) {
    digitalWrite(PIN_FLASH_LED, HIGH);
  } else if (!strcmp(cmd, "led_off")) {
    digitalWrite(PIN_FLASH_LED, LOW);
  } else {
    Serial.printf("unknown cmd: %s\n", cmd);
  }
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
  .row{max-width:300px;margin:0 auto 12px;display:flex;gap:10px;align-items:center}
  input[type=range]{flex:1}
  .diag{max-width:300px;margin:14px auto 0;display:flex;gap:8px}
  .diag button{height:42px;font-size:14px;flex:1;background:#232733}
  #st{color:#8b93a7;font-size:13px;margin-top:10px}
</style></head><body>
<img id="cam" alt="video off">
<div class="pad">
  <span></span><button data-go="forward">&#9650;</button><span></span>
  <button data-go="left">&#9664;</button>
  <button data-go="stop">&#9632;</button>
  <button data-go="right">&#9654;</button>
  <span></span><button data-go="backward">&#9660;</button><span></span>
  <button class="wide" id="led">фара вкл/выкл</button>
</div>
<div class="row">скорость<input type="range" id="sp" min="80" max="255" value="200"><span id="spv">200</span></div>
<div class="row"><button class="wide" id="show" style="height:44px;font-size:15px">показать номер</button></div>
<div class="diag">
  <button data-go="test_a">мотор A (13/12)</button>
  <button data-go="test_b">мотор B (15/14)</button>
</div>
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
  document.getElementById('show').onclick = function(){ send('go=show'); };
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
    bool     owned   = false;

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
    if (res != ESP_OK) break;      // клиент отключился

    vTaskDelay(pdMS_TO_TICKS(10)); // отдать время Wi-Fi и управлению
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
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    c.frame_size   = FRAMESIZE_VGA;
    c.jpeg_quality = 12;
    c.fb_count     = 2;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    c.frame_size   = FRAMESIZE_QVGA;
    c.jpeg_quality = 14;
    c.fb_count     = 1;
    c.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("esp_camera_init failed: 0x%x\n", err);
    Serial.println("proveri: shleyf kamery, PSRAM=Enabled, pitanie 5V/2A");
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("sensor PID: 0x%x\n", s->id.PID);
    s->set_framesize(s, FRAMESIZE_QVGA);   // для езды важнее плавность
    s->set_vflip(s, 1);                    // на этом шасси камера перевёрнута
    s->set_hmirror(s, 0);
  }
  Serial.println("camera: OK");
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
    Serial.println("pult:  http://192.168.4.1/");
  } else {
    Serial.println("port 80 server FAILED");
  }

#if ENABLE_CAMERA
  // Стрим занимает рабочий поток httpd целиком, поэтому он на отдельном порту.
  httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
  scfg.server_port = 81;
  scfg.ctrl_port   = 32769;
  scfg.lru_purge_enable = true;

  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
  if (httpd_start(&stream_httpd, &scfg) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("video: http://192.168.4.1:81/stream");
  } else {
    Serial.println("port 81 server FAILED");
  }
#endif
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== ESP32-CAM 4WD car ===");
  Serial.printf("reset reason: %d  (1=power on, 3=soft, 5=deepsleep, 6=brownout)\n",
                (int)esp_reset_reason());
  Serial.printf("PSRAM: %s\n", psramFound() ? "found" : "NOT FOUND");
  Serial.printf("wiring: %s\n", WIRING_SIDES ? "SIDES (povoroty est)"
                                             : "FRONT/REAR (povorotov net)");

  pinMode(PIN_FLASH_LED, OUTPUT);
  digitalWrite(PIN_FLASH_LED, LOW);

  // Моторы инициализируем первыми, чтобы они не дёргались на старте.
  motorsInit();

#if ENABLE_CAMERA
  if (!cameraInit()) {
    Serial.println("prodolzhayu bez kamery - upravlenie budet rabotat");
  }
#endif

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(200);

  bool ok = WiFi.softAP(AP_SSID, NULL, AP_CHAN, 0, 4);   // NULL = без пароля
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);

  Serial.printf("softAP(\"%s\", OTKRYTAYA, ch %d): %s\n",
                AP_SSID, AP_CHAN, ok ? "OK" : "FAIL");
  Serial.print("AP IP:    ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("TX power: %d (78 = 19.5 dBm, max)\n", (int)WiFi.getTxPower());

  startServers();
  g_lastCmd = millis();
}

void loop() {
  // Номер по кнопке. Блокирует loop(), но не веб-сервер: он в своём потоке.
  if (g_showRequest) {
    g_showRequest = false;
    g_showRunning = true;
    Serial.println("show: start");
    runShow();
    g_showRunning = false;
    g_lastCmd = millis();
    Serial.println("show: done");
  }

  // Failsafe: пульт отвалился - моторы стоп.
  static bool stopped = false;
  if (!g_showRunning && millis() - g_lastCmd > FAILSAFE_MS) {
    if (!stopped) {
      allStop();
      stopped = true;
      Serial.println("failsafe: stop");
    }
  } else {
    stopped = false;
  }

  static uint32_t last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("[%6lu s] ch %d | clients: %u | heap: %u | speed: %d\n",
                  millis() / 1000, AP_CHAN,
                  WiFi.softAPgetStationNum(),
                  ESP.getFreeHeap(), g_speed);
  }

  delay(20);
}
