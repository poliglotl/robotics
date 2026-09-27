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

// ----------------------------------------------------------- настройка камеры
// Картинка перевёрнута вверх ногами -> CAM_VFLIP 0.
// Картинка зеркальная (текст читается наоборот) -> CAM_HMIRROR 1.
#define CAM_VFLIP    0
#define CAM_HMIRROR  0

// Размер кадра. Меньше = меньше задержка.
//   FRAMESIZE_QVGA   320x240 - норма для езды
//   FRAMESIZE_HQVGA  240x176 - заметно шустрее
//   FRAMESIZE_QQVGA  160x120 - почти без задержки, но мелко
#define CAM_FRAME_SIZE FRAMESIZE_QVGA

// Сжатие JPEG: 10 = красиво и тяжело, 25 = мыло и легко.
// Задержка падает почти линейно с размером кадра.
#define CAM_QUALITY  16

// Пауза между кадрами. Меньше = плавнее, но Wi-Fi и пульту нужно время.
#define CAM_FRAME_GAP_MS 2

// ------------------------------------------------------------ серво (голова)
#define ENABLE_SERVOS 1

// GPIO2 свободен. Наклон на GPIO3 (RX), а не на GPIO1 (TX): так остаются
// живыми логи в Serial Monitor, а ввод в него нам всё равно не нужен.
#define PIN_PAN   2       // поворот влево-вправо
#define PIN_TILT  3       // наклон вверх-вниз
#define CH_PAN    6       // каналы 6 и 7 = таймер 3, моторы его не трогают
#define CH_TILT   7
#define SERVO_FREQ 50
#define SERVO_RES  16

// Импульс для 0 и 180 градусов. 1000-2000 безопасно для любого серво,
// 500-2500 это полный ход, но дешёвые SG90 упираются в стопор и воют.
#define PULSE_MIN_US 1000
#define PULSE_MAX_US 2000

// Пределы углов. Серво упирается и начинает выть - сузить эти числа.
#define PAN_MIN     60
#define PAN_CENTER  90
#define PAN_MAX    120
#define TILT_MIN    70
#define TILT_CENTER 90
#define TILT_MAX   110

// Скорость головы: один градус за столько миллисекунд, пока держишь кнопку.
#define SERVO_STEP_MS 15

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
  #define PWM_SETUP_F(pin, ch, freq, res) ledcAttach((pin), (freq), (res))
  #define PWM_WRITE(pin, ch, duty)        ledcWrite((pin), (duty))
#else
  #define PWM_SETUP_F(pin, ch, freq, res) do { ledcSetup((ch), (freq), (res)); \
                                               ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty)        ledcWrite((ch), (duty))
#endif
#define PWM_SETUP(pin, ch) PWM_SETUP_F((pin), (ch), PWM_FREQ, PWM_RES)

// --------------------------------------------------------------- состояние
static int      g_speed   = 200;    // 0..255
static uint32_t g_lastCmd = 0;
static volatile bool g_showRequest = false;   // нажали кнопку "номер"
static volatile bool g_showRunning = false;   // номер идёт, failsafe молчит
static const uint32_t FAILSAFE_MS = 2000;

#if ENABLE_SERVOS
// Направление, пока кнопка зажата: -1, 0 или +1. Шагает в loop().
static volatile int g_panDir  = 0;
static volatile int g_tiltDir = 0;
static int g_panNow  = PAN_CENTER;
static int g_tiltNow = TILT_CENTER;
#endif

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

// ------------------------------------------------------------------- серво
#if ENABLE_SERVOS
static uint32_t angleToDuty(int deg) {
  if (deg < 0)   deg = 0;
  if (deg > 180) deg = 180;
  uint32_t us = PULSE_MIN_US + (uint32_t)deg * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  return (us * 65536UL) / 20000UL;
}

static void servosInit() {
  PWM_SETUP_F(PIN_PAN,  CH_PAN,  SERVO_FREQ, SERVO_RES);
  PWM_SETUP_F(PIN_TILT, CH_TILT, SERVO_FREQ, SERVO_RES);
  PWM_WRITE(PIN_PAN,  CH_PAN,  angleToDuty(g_panNow));
  PWM_WRITE(PIN_TILT, CH_TILT, angleToDuty(g_tiltNow));
  Serial.println("servos: init OK");
}

// Двигает голову на один градус в заданную сторону. Вызывается из loop()
// по таймеру, пока кнопка зажата - получается плавное непрерывное движение.
static void servoTick() {
  static uint32_t last = 0;
  if (millis() - last < SERVO_STEP_MS) return;
  last = millis();

  if (g_panDir) {
    int t = g_panNow + g_panDir;
    if (t >= PAN_MIN && t <= PAN_MAX) {
      g_panNow = t;
      PWM_WRITE(PIN_PAN, CH_PAN, angleToDuty(g_panNow));
    }
  }
  if (g_tiltDir) {
    int t = g_tiltNow + g_tiltDir;
    if (t >= TILT_MIN && t <= TILT_MAX) {
      g_tiltNow = t;
      PWM_WRITE(PIN_TILT, CH_TILT, angleToDuty(g_tiltNow));
    }
  }
}

static void headCenter() {
  g_panDir = g_tiltDir = 0;
  g_panNow  = PAN_CENTER;
  g_tiltNow = TILT_CENTER;
  PWM_WRITE(PIN_PAN,  CH_PAN,  angleToDuty(g_panNow));
  PWM_WRITE(PIN_TILT, CH_TILT, angleToDuty(g_tiltNow));
}
#endif  // ENABLE_SERVOS

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
#if ENABLE_SERVOS
  } else if (!strcmp(cmd, "pan_l")) {
    g_panDir = -1;
  } else if (!strcmp(cmd, "pan_r")) {
    g_panDir = +1;
  } else if (!strcmp(cmd, "pan_s")) {
    g_panDir = 0;
  } else if (!strcmp(cmd, "tilt_u")) {
    g_tiltDir = +1;
  } else if (!strcmp(cmd, "tilt_d")) {
    g_tiltDir = -1;
  } else if (!strcmp(cmd, "tilt_s")) {
    g_tiltDir = 0;
  } else if (!strcmp(cmd, "head_c")) {
    headCenter();
#endif
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
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no,viewport-fit=cover">
<title>Car</title>
<style>
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  html,body{margin:0;height:100%;overflow:hidden;background:#000;
            color:#e6e8ee;font:15px/1.3 system-ui,sans-serif;
            -webkit-user-select:none;user-select:none;touch-action:none}

  /* видео на весь экран, под всеми кнопками */
  #cam{position:fixed;inset:0;width:100%;height:100%;object-fit:contain;
       background:#000;z-index:0}

  /* верхняя панель */
  #bar{position:fixed;top:0;left:0;right:0;z-index:2;display:flex;
       align-items:center;gap:8px;padding:8px 12px;
       padding-top:calc(8px + env(safe-area-inset-top));
       background:linear-gradient(#000c,#0000)}
  #bar button{height:34px;padding:0 12px;border:0;border-radius:8px;
              background:#ffffff26;color:#e6e8ee;font-size:13px}
  #bar button:active{background:#4a71f0}
  #sp{flex:1;max-width:160px}
  #st{margin-left:auto;font-size:12px;color:#ffffff8c;
      white-space:nowrap;overflow:hidden;text-overflow:ellipsis}

  /* два джойстика по нижним углам */
  .pad{position:fixed;bottom:calc(14px + env(safe-area-inset-bottom));z-index:2;
       display:grid;grid-template-columns:repeat(3,58px);
       grid-template-rows:repeat(3,52px);gap:6px}
  .pad.l{left:calc(14px + env(safe-area-inset-left))}
  .pad.r{right:calc(14px + env(safe-area-inset-right))}
  .pad button{border:0;border-radius:10px;background:#ffffff2e;
              color:#fff;font-size:20px;backdrop-filter:blur(3px)}
  .pad button:active{background:#4a71f0}
  .pad .lab{grid-column:1/4;display:flex;align-items:center;
            justify-content:center;font-size:11px;color:#ffffff8c;
            letter-spacing:.06em}
  .pad.r button{background:#ffffff1f}

  /* на широком экране джойстики не нужны - там клавиатура */
  @media (min-width:900px) and (pointer:fine){ .pad{opacity:.35} }
</style></head><body>

<img id="cam" alt="">

<div id="bar">
  <button id="fs">[ ] экран</button>
  <button id="led">фара</button>
  <button id="show">номер</button>
  <button id="ctr">центр</button>
  <input type="range" id="sp" min="80" max="255" value="200">
  <span id="st">WASD - колёса, стрелки - голова</span>
</div>

<div class="pad l">
  <div class="lab">КОЛЁСА</div>
  <span></span><button data-hold="forward">&#9650;</button><span></span>
  <button data-hold="left">&#9664;</button>
  <button data-tap="stop">&#9632;</button>
  <button data-hold="right">&#9654;</button>
  <span></span><button data-hold="backward">&#9660;</button><span></span>
</div>

<div class="pad r">
  <div class="lab">ГОЛОВА</div>
  <span></span><button data-hold="tilt_u" data-off="tilt_s">&#9650;</button><span></span>
  <button data-hold="pan_l" data-off="pan_s">&#9664;</button>
  <button data-tap="head_c">&#9678;</button>
  <button data-hold="pan_r" data-off="pan_s">&#9654;</button>
  <span></span><button data-hold="tilt_d" data-off="tilt_s">&#9660;</button><span></span>
</div>

<script>
  var st = document.getElementById('st');
  function send(q){
    fetch('/action?' + q).catch(function(){ st.textContent = 'нет связи'; });
  }

  // Кнопка "держать": нажал - поехали, отпустил - стоп.
  function bindHold(b){
    var on  = b.dataset.hold;
    var off = b.dataset.off || 'stop';
    var down = function(e){ e.preventDefault(); send('go=' + on); };
    var up   = function(e){ e.preventDefault(); send('go=' + off); };
    b.addEventListener('touchstart', down, {passive:false});
    b.addEventListener('touchend',   up);
    b.addEventListener('touchcancel',up);
    b.addEventListener('mousedown',  down);
    b.addEventListener('mouseup',    up);
    b.addEventListener('mouseleave', up);
  }
  document.querySelectorAll('button[data-hold]').forEach(bindHold);
  document.querySelectorAll('button[data-tap]').forEach(function(b){
    b.addEventListener('click', function(e){
      e.preventDefault(); send('go=' + b.dataset.tap);
    });
  });

  // Клавиатура: WASD колёса, стрелки голова.
  var KEY = {
    KeyW:['forward','stop'],  KeyS:['backward','stop'],
    KeyA:['left','stop'],     KeyD:['right','stop'],
    ArrowUp:['tilt_u','tilt_s'],   ArrowDown:['tilt_d','tilt_s'],
    ArrowLeft:['pan_l','pan_s'],   ArrowRight:['pan_r','pan_s']
  };
  var held = {};
  document.addEventListener('keydown', function(e){
    var k = KEY[e.code];
    if (!k) return;
    e.preventDefault();
    if (held[e.code]) return;      // не спамим на автоповторе
    held[e.code] = 1;
    send('go=' + k[0]);
  });
  document.addEventListener('keyup', function(e){
    var k = KEY[e.code];
    if (!k) return;
    e.preventDefault();
    held[e.code] = 0;
    send('go=' + k[1]);
  });
  // Ушли с вкладки с зажатой клавишей - на всякий случай стоп.
  window.addEventListener('blur', function(){
    held = {}; send('go=stop'); send('go=pan_s'); send('go=tilt_s');
  });

  document.getElementById('fs').onclick = function(){
    var d = document.documentElement;
    if (document.fullscreenElement) { document.exitFullscreen(); }
    else if (d.requestFullscreen)   { d.requestFullscreen(); }
    else if (d.webkitRequestFullscreen) { d.webkitRequestFullscreen(); }
  };

  var ledOn = false;
  document.getElementById('led').onclick = function(){
    ledOn = !ledOn; send('go=' + (ledOn ? 'led_on' : 'led_off'));
  };
  document.getElementById('show').onclick = function(){ send('go=show'); };
  document.getElementById('ctr').onclick  = function(){ send('go=head_c'); };

  var sp = document.getElementById('sp');
  sp.oninput = function(){ send('speed=' + sp.value); };

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

    vTaskDelay(pdMS_TO_TICKS(CAM_FRAME_GAP_MS)); // время Wi-Fi и пульту
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

  // Буферы сразу под тот размер, который стримим. Раньше тут была VGA,
  // которую потом уменьшали до QVGA - лишняя память и лишняя задержка.
  c.frame_size   = CAM_FRAME_SIZE;
  c.jpeg_quality = CAM_QUALITY;
  if (psramFound()) {
    c.fb_count    = 2;              // второй буфер + GRAB_LATEST = свежий кадр
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.fb_count    = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
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
    s->set_framesize(s, CAM_FRAME_SIZE);
    s->set_quality(s, CAM_QUALITY);
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
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

#if ENABLE_SERVOS
  servosInit();
#endif

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

#if ENABLE_SERVOS
  servoTick();
#endif

  // Failsafe: пульт отвалился - моторы стоп, голова замирает.
  static bool stopped = false;
  if (!g_showRunning && millis() - g_lastCmd > FAILSAFE_MS) {
    if (!stopped) {
      allStop();
#if ENABLE_SERVOS
      g_panDir = g_tiltDir = 0;
#endif
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
