/*
 * 18_car_control - главная прошивка: робот раздаёт Wi-Fi, телефон им управляет,
 * видео с камеры идёт в браузер.
 *
 * Сеть:  RobotCar  (без пароля)
 * Пульт: http://192.168.4.1
 * Видео: http://192.168.4.1:81/stream
 *
 * Tools -> Partition Scheme: Huge APP    |  Tools -> PSRAM: Enabled
 *
 * Управление: WASD - колёса, стрелки - голова. На телефоне два джойстика
 * по нижним углам: слева колёса, справа голова. Кнопка [ ] - на весь экран.
 * Ползунок - предел скорости моторов, 80..255.
 *
 * Колёса вывесить на первый запуск, антенна прикручена.
 */

#include <WiFi.h>
#include "esp_http_server.h"

// ------------------------------------------------------------- переключатели
#define ENABLE_CAMERA  1     // 0 = без камеры (отладка Wi-Fi и моторов)
#define ENABLE_MOTORS  1     // 0 = не трогать пины моторов вообще
#define ENABLE_SERVOS  1     // 0 = не трогать пины серво вообще

// РАЗВОДКА МОТОРОВ. Драйвер даёт две группы выходов: A (13/12) и B (15/14).
//   WIRING_SIDES 1 - группы это борта, повороты работают.
//   WIRING_SIDES 0 - группы это оси (перед/зад), поворотов нет физически.
// На нашем шасси 1: провода переставлены по схеме Keyestudio (оба правых
// мотора на OUT1/OUT2, оба левых на OUT3/OUT4), 19_wheel_sides подтвердил.
#define WIRING_SIDES 1
#define SWAP_SIDES   0       // кнопки влево-вправо наоборот -> 1
#define INVERT_A     0       // группа 13/12 едет не в ту сторону -> 1
#define INVERT_B     0       // группа 15/14 едет не в ту сторону -> 1

// ----------------------------------------------------------- настройка камеры
#define CAM_VFLIP    0       // картинка вверх ногами -> 1
#define CAM_HMIRROR  0       // картинка зеркальная -> 1

// Размер кадра. Меньше = меньше задержка. Это главный рычаг против лагов.
//   FRAMESIZE_QVGA   320x240 - норма
//   FRAMESIZE_HQVGA  240x176 - заметно шустрее
//   FRAMESIZE_QQVGA  160x120 - почти без задержки, но мелко
#define CAM_FRAME_SIZE FRAMESIZE_QVGA
#define CAM_QUALITY    16    // 10 = красиво и тяжело, 25 = мыло и легко
#define CAM_FRAME_GAP_MS 5   // пауза между кадрами: время Wi-Fi и пульту

// ------------------------------------------------------------ серво (голова)
// GPIO2 свободен. Наклон на GPIO3 (RX): так живы логи в Serial Monitor,
// а ввод в него нам не нужен. Каналы 14/15 - это группа low-speed, таймер 3:
// максимально далеко и от камеры (таймер 0), и от моторов (группа high-speed).
#define PIN_PAN   2
#define PIN_TILT  3
#define CH_PAN    14
#define CH_TILT   15
#define SERVO_FREQ 50
#define SERVO_RES  16

// Импульс для 0 и 180 градусов. 1000-2000 безопасно для любого серво.
#define PULSE_MIN_US 1000
#define PULSE_MAX_US 2000

// Пределы углов. Серво упирается и начинает выть - сузить эти числа.
#define PAN_MIN     60
#define PAN_CENTER  90
#define PAN_MAX    120
#define TILT_MIN    70
#define TILT_CENTER 90
#define TILT_MAX   110
#define SERVO_STEP_MS 15     // один градус за столько мс, пока держишь кнопку

#if ENABLE_CAMERA
  #include "esp_camera.h"
#endif

// ------------------------------------------------------------------ Wi-Fi
static const char *AP_SSID = "RobotCar";   // без пароля
static const int   AP_CHAN = 1;            // не подошёл - попробуй 6, потом 11

// --------------------------------------------------------------- пины моторов
// GPIO12 - strapping-пин. Ловишь циклический ребут - перенеси на GPIO2.
#define PIN_A_IN1   13
#define PIN_A_IN2   12
#define PIN_B_IN1   15
#define PIN_B_IN2   14
#define PIN_FLASH_LED 4      // белый светодиод-фара на плате AI-Thinker

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
// Камера занимает LEDC-таймер 0 под XCLK, моторам отдаём каналы 2..5
// (таймеры 1 и 2), серво - каналы 14/15 из другой группы.
#define PWM_FREQ  1000
#define PWM_RES   8          // 0..255
#define CH_A_IN1  2
#define CH_A_IN2  3
#define CH_B_IN1  4
#define CH_B_IN2  5

// В ядре 3.x API LEDC сменился - поддерживаем обе версии.
// setupF возвращает реально полученную частоту: 0 значит LEDC отказал.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  static inline uint32_t pwmSetup(int pin, int ch, uint32_t freq, uint8_t res) {
    (void)ch;
    return ledcAttach(pin, freq, res) ? freq : 0;
  }
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((pin), (duty))
#else
  static inline uint32_t pwmSetup(int pin, int ch, uint32_t freq, uint8_t res) {
    uint32_t got = (uint32_t)ledcSetup(ch, freq, res);
    if (got) ledcAttachPin(pin, ch);
    return got;
  }
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((ch), (duty))
#endif

// --------------------------------------------------------------- состояние
static int      g_speed   = 200;              // предел скорости, 0..255
static uint32_t g_lastCmd = 0;
static const uint32_t FAILSAFE_MS = 2000;

// Каждый новый клиент стрима вытесняет предыдущего: иначе после перезагрузки
// страницы два обработчика тянут кадры из одного буфера и начинаются рывки.
static volatile uint32_t g_streamGen = 0;

#if ENABLE_SERVOS
static volatile int g_panDir  = 0;            // -1, 0, +1 пока кнопка зажата
static volatile int g_tiltDir = 0;
static int  g_panNow  = PAN_CENTER;
static int  g_tiltNow = TILT_CENTER;
static bool g_servosOk = false;
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
#else
  (void)pinA; (void)chA; (void)pinB; (void)chB; (void)v;
#endif
}

// a - группа на пинах 13/12, b - группа на 15/14.
static void motors(int a, int b) {
#if INVERT_A
  a = -a;
#endif
#if INVERT_B
  b = -b;
#endif
  groupDrive(PIN_A_IN1, CH_A_IN1, PIN_A_IN2, CH_A_IN2, constrain(a, -255, 255));
  groupDrive(PIN_B_IN1, CH_B_IN1, PIN_B_IN2, CH_B_IN2, constrain(b, -255, 255));
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
  pwmSetup(PIN_A_IN1, CH_A_IN1, PWM_FREQ, PWM_RES);
  pwmSetup(PIN_A_IN2, CH_A_IN2, PWM_FREQ, PWM_RES);
  pwmSetup(PIN_B_IN1, CH_B_IN1, PWM_FREQ, PWM_RES);
  pwmSetup(PIN_B_IN2, CH_B_IN2, PWM_FREQ, PWM_RES);
  allStop();
  Serial.println("motors: init OK");
#else
  Serial.println("motors: DISABLED");
#endif
}

// ------------------------------------------------------------------- серво
#if ENABLE_SERVOS
static uint32_t angleToDuty(int deg) {
  deg = constrain(deg, 0, 180);
  uint32_t us = PULSE_MIN_US + (uint32_t)deg * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  return ((uint32_t)us << SERVO_RES) / 20000UL;   // 20000 мкс = период 50 Гц
}

static void servoWrite(int pin, int ch, int deg) {
  PWM_WRITE(pin, ch, angleToDuty(deg));
}

// Вызывается ПОСЛЕ инициализации камеры: esp_camera_init трогает LEDC, и
// если настроить серво до него, настройки могут быть перезаписаны.
static void servosInit() {
  uint32_t fPan  = pwmSetup(PIN_PAN,  CH_PAN,  SERVO_FREQ, SERVO_RES);
  uint32_t fTilt = pwmSetup(PIN_TILT, CH_TILT, SERVO_FREQ, SERVO_RES);
  g_servosOk = (fPan && fTilt);

  Serial.printf("servos: pan GPIO%d ch%d -> %lu Hz | tilt GPIO%d ch%d -> %lu Hz\n",
                PIN_PAN, CH_PAN, (unsigned long)fPan,
                PIN_TILT, CH_TILT, (unsigned long)fTilt);
  if (!g_servosOk) {
    Serial.println("servos: LEDC OTKAZAL - signala na pinah NET");
    return;
  }
  Serial.printf("servos: duty %d grad = %lu iz %lu\n", PAN_CENTER,
                (unsigned long)angleToDuty(PAN_CENTER), (1UL << SERVO_RES) - 1);

  // Короткий кивок на старте: видно сразу, идёт ли сигнал, без телефона.
  servoWrite(PIN_PAN,  CH_PAN,  PAN_CENTER);
  servoWrite(PIN_TILT, CH_TILT, TILT_CENTER);
  delay(400);
  servoWrite(PIN_PAN,  CH_PAN,  PAN_CENTER + 8);
  servoWrite(PIN_TILT, CH_TILT, TILT_CENTER + 8);
  delay(400);
  servoWrite(PIN_PAN,  CH_PAN,  PAN_CENTER);
  servoWrite(PIN_TILT, CH_TILT, TILT_CENTER);
  Serial.println("servos: init OK (dolzhen byt kivok)");
}

// Один градус в заданную сторону. Дёргается из loop() по таймеру, пока
// кнопка зажата - получается плавное непрерывное движение.
static void servoTick() {
  if (!g_servosOk) return;
  static uint32_t last = 0;
  if (millis() - last < SERVO_STEP_MS) return;
  last = millis();

  if (g_panDir) {
    int t = g_panNow + g_panDir;
    if (t >= PAN_MIN && t <= PAN_MAX) {
      g_panNow = t;
      servoWrite(PIN_PAN, CH_PAN, g_panNow);
    }
  }
  if (g_tiltDir) {
    int t = g_tiltNow + g_tiltDir;
    if (t >= TILT_MIN && t <= TILT_MAX) {
      g_tiltNow = t;
      servoWrite(PIN_TILT, CH_TILT, g_tiltNow);
    }
  }
}

static void headCenter() {
  g_panDir = g_tiltDir = 0;
  g_panNow  = PAN_CENTER;
  g_tiltNow = TILT_CENTER;
  if (!g_servosOk) return;
  servoWrite(PIN_PAN,  CH_PAN,  g_panNow);
  servoWrite(PIN_TILT, CH_TILT, g_tiltNow);
}
#endif  // ENABLE_SERVOS

// ---------------------------------------------------------------- команды
static void applyCommand(const char *cmd) {
  g_lastCmd = millis();
  const int s = g_speed;

  if      (!strcmp(cmd, "forward"))  sides( s,  s);
  else if (!strcmp(cmd, "backward")) sides(-s, -s);
  else if (!strcmp(cmd, "stop"))     allStop();
#if WIRING_SIDES
  else if (!strcmp(cmd, "left"))     sides(0, s);   // правый борт толкает
  else if (!strcmp(cmd, "right"))    sides(s, 0);   // левый борт толкает
#else
  else if (!strcmp(cmd, "left") || !strcmp(cmd, "right"))
    Serial.println("povorot nevozmozhen: WIRING_SIDES 0");
#endif
#if ENABLE_SERVOS
  else if (!strcmp(cmd, "pan_l"))    g_panDir  = -1;
  else if (!strcmp(cmd, "pan_r"))    g_panDir  = +1;
  else if (!strcmp(cmd, "pan_s"))    g_panDir  =  0;
  else if (!strcmp(cmd, "tilt_u"))   g_tiltDir = +1;
  else if (!strcmp(cmd, "tilt_d"))   g_tiltDir = -1;
  else if (!strcmp(cmd, "tilt_s"))   g_tiltDir =  0;
  else if (!strcmp(cmd, "head_c"))   headCenter();
#endif
  else if (!strcmp(cmd, "led_on"))   digitalWrite(PIN_FLASH_LED, HIGH);
  else if (!strcmp(cmd, "led_off"))  digitalWrite(PIN_FLASH_LED, LOW);
  else Serial.printf("unknown cmd: %s\n", cmd);
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
  #cam{position:fixed;inset:0;width:100%;height:100%;object-fit:contain;
       background:#000;z-index:0}
  #bar{position:fixed;top:0;left:0;right:0;z-index:2;display:flex;
       align-items:center;gap:8px;padding:8px 12px;
       padding-top:calc(8px + env(safe-area-inset-top));
       background:linear-gradient(#000c,#0000)}
  #bar button{height:34px;padding:0 12px;border:0;border-radius:8px;
              background:#ffffff26;color:#e6e8ee;font-size:13px}
  #bar button:active{background:#4a71f0}
  #sp{flex:1;max-width:130px}
  .cap{font-size:12px;color:#ffffffa6;white-space:nowrap}
  #st{margin-left:auto;font-size:12px;color:#ffffff8c;white-space:nowrap;
      overflow:hidden;text-overflow:ellipsis}
  .pad{position:fixed;bottom:calc(14px + env(safe-area-inset-bottom));z-index:2;
       display:grid;grid-template-columns:repeat(3,58px);
       grid-template-rows:repeat(3,52px);gap:6px}
  .pad.l{left:calc(14px + env(safe-area-inset-left))}
  .pad.r{right:calc(14px + env(safe-area-inset-right))}
  .pad button{border:0;border-radius:10px;background:#ffffff2e;color:#fff;
              font-size:20px}
  .pad button:active{background:#4a71f0}
  .pad .lab{grid-column:1/4;display:flex;align-items:center;
            justify-content:center;font-size:11px;color:#ffffff8c;
            letter-spacing:.06em}
  .pad.r button{background:#ffffff1f}
  @media (min-width:900px) and (pointer:fine){ .pad{opacity:.35} }
</style></head><body>

<img id="cam" alt="">

<div id="bar">
  <button id="fs">[ ] экран</button>
  <button id="led">фара</button>
  <button id="ctr">центр</button>
  <span class="cap">скорость</span>
  <input type="range" id="sp" min="80" max="255" value="200">
  <span class="cap" id="spv">200</span>
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

  // Не больше двух запросов в воздухе. Каждый запрос - отдельное TCP
  // соединение, а их у платы мало: если сыпать их пачками, видео встаёт.
  var inflight = 0, queued = null;
  function raw(q){
    if (inflight >= 2) { queued = q; return; }
    inflight++;
    var done = function(){
      inflight--;
      if (queued) { var n = queued; queued = null; raw(n); }
    };
    fetch('/action?' + q).then(done, function(){ st.textContent = 'нет связи'; done(); });
  }

  // Одна и та же команда два раза подряд платой не нужна.
  var last = '';
  function cmd(c){ if (c === last) return; last = c; raw('go=' + c); }

  function bindHold(b){
    var on  = b.dataset.hold;
    var off = b.dataset.off || 'stop';
    var down = function(e){ e.preventDefault(); cmd(on); };
    var up   = function(e){ e.preventDefault(); cmd(off); };
    b.addEventListener('touchstart', down, {passive:false});
    b.addEventListener('touchend',   up);
    b.addEventListener('touchcancel',up);
    b.addEventListener('mousedown',  down);
    b.addEventListener('mouseup',    up);
    b.addEventListener('mouseleave', up);
  }
  document.querySelectorAll('button[data-hold]').forEach(bindHold);
  document.querySelectorAll('button[data-tap]').forEach(function(b){
    b.addEventListener('click', function(e){ e.preventDefault(); cmd(b.dataset.tap); });
  });

  var KEY = {
    KeyW:['forward','stop'],       KeyS:['backward','stop'],
    KeyA:['left','stop'],          KeyD:['right','stop'],
    ArrowUp:['tilt_u','tilt_s'],   ArrowDown:['tilt_d','tilt_s'],
    ArrowLeft:['pan_l','pan_s'],   ArrowRight:['pan_r','pan_s']
  };
  var held = {};
  document.addEventListener('keydown', function(e){
    var k = KEY[e.code];
    if (!k) return;
    e.preventDefault();
    if (held[e.code]) return;            // не спамим на автоповторе
    held[e.code] = 1;
    cmd(k[0]);
  });
  document.addEventListener('keyup', function(e){
    var k = KEY[e.code];
    if (!k) return;
    e.preventDefault();
    held[e.code] = 0;
    cmd(k[1]);
  });
  window.addEventListener('blur', function(){
    held = {}; cmd('stop'); cmd('pan_s'); cmd('tilt_s');
  });

  document.getElementById('fs').onclick = function(){
    var d = document.documentElement;
    if (document.fullscreenElement) document.exitFullscreen();
    else if (d.requestFullscreen) d.requestFullscreen();
    else if (d.webkitRequestFullscreen) d.webkitRequestFullscreen();
  };

  var ledOn = false;
  document.getElementById('led').onclick = function(){
    ledOn = !ledOn; cmd(ledOn ? 'led_on' : 'led_off');
  };
  document.getElementById('ctr').onclick = function(){ cmd('head_c'); };

  // Цифру двигаем сразу, а на плату шлём только когда отпустил ползунок:
  // иначе одно перетаскивание = сотня запросов и видео захлёбывается.
  var sp = document.getElementById('sp'), spv = document.getElementById('spv');
  sp.oninput  = function(){ spv.textContent = sp.value; };
  sp.onchange = function(){ raw('speed=' + sp.value); };

  // Стрим иногда обрывается. Поднимаем заново, но не чаще раза в секунду.
  var cam = document.getElementById('cam');
  function startStream(){
    cam.src = 'http://' + location.hostname + ':81/stream?t=' + Date.now();
  }
  cam.onerror = function(){ setTimeout(startStream, 1000); };
  startStream();
</script></body></html>
)HTML";

static httpd_handle_t ctrl_httpd   = NULL;
static httpd_handle_t stream_httpd = NULL;

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t action_handler(httpd_req_t *req) {
  // Команды короткие, так что читаем в буфер на стеке - без malloc и без
  // шанса что-то забыть освободить на пути ошибки.
  char query[64];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad query");
    return ESP_FAIL;
  }

  char value[24];
  if (httpd_query_key_value(query, "go", value, sizeof(value)) == ESP_OK) {
    applyCommand(value);
  } else if (httpd_query_key_value(query, "speed", value, sizeof(value)) == ESP_OK) {
    g_speed = constrain(atoi(value), 0, 255);
    Serial.printf("speed = %d\n", g_speed);
  } else {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no cmd");
    return ESP_FAIL;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "ok", 2);
}

#if ENABLE_CAMERA
#define PART_BOUNDARY "frameboundary"
static const char *STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
// Граница и заголовок склеены: одна отправка на кадр вместо двух.
static const char *STREAM_PART =
  "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  const uint32_t myGen = ++g_streamGen;    // вытесняем предыдущего клиента

  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char head[96];
  while (g_streamGen == myGen) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("esp_camera_fb_get() failed");
      res = ESP_FAIL;
      break;
    }

    // pixel_format = JPEG, так что кадр уже сжат: перекодировать не надо
    // и промежуточных буферов, которые надо освобождать, тоже нет.
    int hlen = snprintf(head, sizeof(head), STREAM_PART, (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, head, hlen);
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }
    esp_camera_fb_return(fb);              // возвращаем всегда, до выхода

    if (res != ESP_OK) break;              // клиент отключился
    vTaskDelay(pdMS_TO_TICKS(CAM_FRAME_GAP_MS));
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
  c.grab_mode    = CAMERA_GRAB_LATEST;   // отдаём свежий кадр, а не очередь
  c.frame_size   = CAM_FRAME_SIZE;       // сразу нужный размер, без VGA
  c.jpeg_quality = CAM_QUALITY;

  if (psramFound()) {
    c.fb_count    = 2;
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
  // Пульт с приоритетом выше стрима: нажатия не ждут, пока уйдёт кадр.
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port      = 80;
  cfg.ctrl_port        = 32768;
  cfg.max_uri_handlers = 4;
  cfg.max_open_sockets = 4;
  cfg.lru_purge_enable = true;
  cfg.task_priority    = tskIDLE_PRIORITY + 6;

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
  // Стрим занимает рабочий поток целиком, поэтому он на отдельном порту,
  // с приоритетом пониже и стеком побольше.
  httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
  scfg.server_port      = 81;
  scfg.ctrl_port        = 32769;
  scfg.max_uri_handlers = 1;
  scfg.max_open_sockets = 2;
  scfg.lru_purge_enable = true;
  scfg.task_priority    = tskIDLE_PRIORITY + 3;
  scfg.stack_size       = 8192;

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

  motorsInit();          // моторы первыми, чтобы не дёргались на старте

#if ENABLE_CAMERA
  if (!cameraInit()) {
    Serial.println("prodolzhayu bez kamery - upravlenie budet rabotat");
  }
#endif

#if ENABLE_SERVOS
  servosInit();          // строго после камеры: она трогает LEDC
#endif

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(200);

  bool ok = WiFi.softAP(AP_SSID, NULL, AP_CHAN, 0, 4);   // NULL = без пароля
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);                                  // сон = рывки

  Serial.printf("softAP(\"%s\", OTKRYTAYA, ch %d): %s\n",
                AP_SSID, AP_CHAN, ok ? "OK" : "FAIL");
  Serial.print("AP IP:    ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("TX power: %d (78 = 19.5 dBm, max)\n", (int)WiFi.getTxPower());

  startServers();
  g_lastCmd = millis();
}

void loop() {
#if ENABLE_SERVOS
  servoTick();
#endif

  // Failsafe: пульт отвалился - моторы стоп, голова замирает.
  static bool stopped = false;
  if (millis() - g_lastCmd > FAILSAFE_MS) {
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
    Serial.printf("[%6lu s] clients: %u | heap: %u | min heap: %u | speed: %d\n",
                  millis() / 1000, WiFi.softAPgetStationNum(),
                  ESP.getFreeHeap(), ESP.getMinFreeHeap(), g_speed);
  }

  delay(5);   // серво шагает раз в 15 мс, так что loop должен быть бодрее
}
