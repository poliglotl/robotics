/*
 * 18_car_control - робот раздаёт Wi-Fi, телефон им управляет, видео в браузере.
 * Только необходимое: колёса, голова, свет, картинка, полный экран.
 *
 * Сеть:  RobotCar  (без пароля)
 * Пульт: http://192.168.4.1
 *
 * Tools -> Partition Scheme: Huge APP    |  Tools -> PSRAM: Enabled
 *
 * Управление: WASD - колёса, стрелки - голова. На телефоне два джойстика
 * по нижним углам: слева колёса, справа голова. Кнопка [ ] - полный экран.
 *
 * Видео не поток, а по одному кадру: браузер просит следующий только когда
 * отрисовал предыдущий. Так кадры не копятся в буфере и картинка не идёт
 * рывками "стоп-стоп-стоп-пачка кадров".
 */

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_wifi.h"           // ширина канала, список клиентов
#include "soc/soc.h"            // отключение детектора просадки питания
#include "soc/rtc_cntl_reg.h"

// --------------------------------------------------------------- разводка
// Драйвер даёт две группы выходов: A (13/12) и B (15/14).
#define WIRING_SIDES 1       // 0 = группы это оси, поворотов нет физически
#define SWAP_SIDES   0       // кнопки влево-вправо наоборот -> 1
#define INVERT_A     0       // группа 13/12 едет не в ту сторону -> 1
#define INVERT_B     0       // группа 15/14 едет не в ту сторону -> 1
#define SPEED      200       // скорость моторов, 0..255

// ------------------------------------------------------------------ Wi-Fi
// Вокруг под 90 сетей, и канал 1 - самый забитый из всех. На забитом
// канале каждый кадр отбивается и переотправляется: отсюда "стоп-стоп-
// пачка кадров". Поэтому канал выбираем сами, по замеру эфира.
#define AUTO_CHANNEL 1       // 0 = взять AP_CHAN как есть
#define AP_CHAN      1       // запасной канал, если AUTO_CHANNEL 0
#define WIDE_CHANNEL 1       // 1 = HT40, двойная ширина канала = вдвое быстрее

// ----------------------------------------------------------------- камера
#define CAM_VFLIP    0       // картинка вверх ногами -> 1
#define CAM_HMIRROR  0       // картинка зеркальная -> 1

// Размер и сжатие. Крутить это ИМЕЕТ СМЫСЛ ТОЛЬКО ПОСЛЕ того, как в логе
// видно нормальные кадр/сек: если эфир забит, мельчить кадр бесполезно.
//   FRAMESIZE_VGA    640x480 - как в заводской прошивке Keyestudio
//   FRAMESIZE_QVGA   320x240 - вдвое легче, сейчас стоит это
//   FRAMESIZE_QQVGA  160x120 - для гонок по коридору, смотреть не на что
// Качество: МЕНЬШЕ число = лучше картинка и тяжелее кадр. 10 - хорошо,
// 12-14 - компромисс, 18+ - мыло.
#define CAM_FRAME_SIZE FRAMESIZE_QVGA
#define CAM_QUALITY   12

// ------------------------------------------------------------------ серво
// Наклон на GPIO3 (RX): логи в Serial остаются живыми, ввод не нужен.
#define PIN_PAN   2
#define PIN_TILT  3
#define CH_PAN    14
#define CH_TILT   15
#define PULSE_MIN_US 1000    // безопасный диапазон импульса для любого серво
#define PULSE_MAX_US 2000
#define PAN_MIN     60
#define PAN_CENTER  90
#define PAN_MAX    120
#define TILT_MIN    70
#define TILT_CENTER 90
#define TILT_MAX   110
#define SERVO_STEP_MS 15     // один градус за столько мс, пока держишь кнопку

// ------------------------------------------------------------------- пины
#define PIN_A_IN1 13         // GPIO12 - strapping-пин: ребут по кругу ->
#define PIN_A_IN2 12         // перенести на GPIO16
#define PIN_B_IN1 15
#define PIN_B_IN2 14
#define PIN_LED    4         // белый светодиод-фара

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

// ----------------------------------------------------------------- ШИМ
// Камера держит LEDC-таймер 0, моторам каналы 2..5, серво 14/15.
#define CH_A_IN1 2
#define CH_A_IN2 3
#define CH_B_IN1 4
#define CH_B_IN2 5

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  static inline uint32_t pwmSetup(int pin, int ch, uint32_t f, uint8_t r) {
    (void)ch; return ledcAttach(pin, f, r) ? f : 0;
  }
  #define PWM_WRITE(pin, ch, duty) ledcWrite((pin), (duty))
#else
  static inline uint32_t pwmSetup(int pin, int ch, uint32_t f, uint8_t r) {
    uint32_t got = (uint32_t)ledcSetup(ch, f, r);
    if (got) ledcAttachPin(pin, ch);
    return got;
  }
  #define PWM_WRITE(pin, ch, duty) ledcWrite((ch), (duty))
#endif

// -------------------------------------------------------------- состояние
static uint32_t g_lastCmd = 0;
static volatile int g_panDir = 0, g_tiltDir = 0;
static int  g_panNow = PAN_CENTER, g_tiltNow = TILT_CENTER;
static bool g_servosOk = false;

// ----------------------------------------------------------------- моторы
static void half(int pinA, int chA, int pinB, int chB, int v) {
  PWM_WRITE(pinA, chA, v > 0 ?  v : 0);
  PWM_WRITE(pinB, chB, v < 0 ? -v : 0);
}

static void motors(int a, int b) {
#if INVERT_A
  a = -a;
#endif
#if INVERT_B
  b = -b;
#endif
  half(PIN_A_IN1, CH_A_IN1, PIN_A_IN2, CH_A_IN2, constrain(a, -255, 255));
  half(PIN_B_IN1, CH_B_IN1, PIN_B_IN2, CH_B_IN2, constrain(b, -255, 255));
}

static void sides(int l, int r) {
#if SWAP_SIDES
  motors(r, l);
#else
  motors(l, r);
#endif
}

// ------------------------------------------------------------------ серво
static uint32_t angleToDuty(int deg) {
  deg = constrain(deg, 0, 180);
  uint32_t us = PULSE_MIN_US + (uint32_t)deg * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  return (us * 65536UL) / 20000UL;     // 20000 мкс = период 50 Гц
}

static void servoWrite(int pin, int ch, int deg) {
  PWM_WRITE(pin, ch, angleToDuty(deg));
}

// Строго после камеры: esp_camera_init трогает LEDC.
static void servosInit() {
  uint32_t fp = pwmSetup(PIN_PAN,  CH_PAN,  50, 16);
  uint32_t ft = pwmSetup(PIN_TILT, CH_TILT, 50, 16);
  g_servosOk = (fp && ft);

  Serial.printf("servos: pan GPIO%d ch%d -> %lu Hz | tilt GPIO%d ch%d -> %lu Hz\n",
                PIN_PAN, CH_PAN, (unsigned long)fp,
                PIN_TILT, CH_TILT, (unsigned long)ft);
  if (!g_servosOk) {
    Serial.println("servos: LEDC OTKAZAL - signala na pinah NET");
    return;
  }
  Serial.printf("servos: duty %d grad = %lu iz 65535\n",
                PAN_CENTER, (unsigned long)angleToDuty(PAN_CENTER));

  // Кивок на старте: сразу видно, идёт ли сигнал, без телефона.
  servoWrite(PIN_PAN, CH_PAN, PAN_CENTER);
  servoWrite(PIN_TILT, CH_TILT, TILT_CENTER);
  delay(400);
  servoWrite(PIN_PAN, CH_PAN, PAN_CENTER + 8);
  servoWrite(PIN_TILT, CH_TILT, TILT_CENTER + 8);
  delay(400);
  servoWrite(PIN_PAN, CH_PAN, PAN_CENTER);
  servoWrite(PIN_TILT, CH_TILT, TILT_CENTER);
  Serial.println("servos: init OK (dolzhen byt kivok)");
}

static void servoTick() {
  if (!g_servosOk) return;
  static uint32_t last = 0;
  if (millis() - last < SERVO_STEP_MS) return;
  last = millis();

  if (g_panDir) {
    int t = g_panNow + g_panDir;
    if (t >= PAN_MIN && t <= PAN_MAX) servoWrite(PIN_PAN, CH_PAN, g_panNow = t);
  }
  if (g_tiltDir) {
    int t = g_tiltNow + g_tiltDir;
    if (t >= TILT_MIN && t <= TILT_MAX) servoWrite(PIN_TILT, CH_TILT, g_tiltNow = t);
  }
}

// ---------------------------------------------------------------- команды
// Коды короткие: меньше байт в запросе - меньше работы на каждое нажатие.
static void applyCommand(const char *c) {
  g_lastCmd = millis();
  switch (c[0]) {
    case 'f': sides( SPEED,  SPEED); break;      // вперёд
    case 'b': sides(-SPEED, -SPEED); break;      // назад
    case 's': sides(0, 0);           break;      // стоп
#if WIRING_SIDES
    case 'l': sides(0, SPEED);       break;      // влево
    case 'r': sides(SPEED, 0);       break;      // вправо
#endif
    case 'L': g_panDir  = -1;        break;      // голова влево
    case 'R': g_panDir  = +1;        break;
    case 'P': g_panDir  =  0;        break;      // отпустили
    case 'U': g_tiltDir = +1;        break;      // голова вверх
    case 'D': g_tiltDir = -1;        break;
    case 'T': g_tiltDir =  0;        break;
    case '1': digitalWrite(PIN_LED, HIGH); break;
    case '0': digitalWrite(PIN_LED, LOW);  break;
    default: break;
  }
}

// ----------------------------------------------------------------- веб UI
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no,viewport-fit=cover">
<title>Car</title>
<style>
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  html,body{margin:0;height:100%;overflow:hidden;background:#000;
            font:15px system-ui,sans-serif;user-select:none;
            -webkit-user-select:none;touch-action:none}
  #cam{position:fixed;inset:0;width:100%;height:100%;object-fit:contain}
  #bar{position:fixed;top:0;left:0;z-index:2;display:flex;gap:8px;padding:8px}
  #bar button{height:32px;padding:0 12px;border:0;border-radius:8px;
              background:#ffffff26;color:#fff;font-size:13px}
  .pad{position:fixed;bottom:calc(14px + env(safe-area-inset-bottom));z-index:2;
       display:grid;grid-template-columns:repeat(3,58px);
       grid-template-rows:repeat(3,52px);gap:6px}
  .pad.l{left:calc(14px + env(safe-area-inset-left))}
  .pad.r{right:calc(14px + env(safe-area-inset-right))}
  .pad button{border:0;border-radius:10px;background:#ffffff2e;color:#fff;
              font-size:20px}
  .pad button:active,#bar button:active{background:#4a71f0}
</style></head><body>

<img id="cam" alt="">

<div id="bar">
  <button id="fs">[ ]</button>
  <button id="led">фара</button>
</div>

<div class="pad l">
  <span></span><button data-h="f">&#9650;</button><span></span>
  <button data-h="l">&#9664;</button>
  <button data-c="s">&#9632;</button>
  <button data-h="r">&#9654;</button>
  <span></span><button data-h="b">&#9660;</button><span></span>
</div>

<div class="pad r">
  <span></span><button data-h="U" data-o="T">&#9650;</button><span></span>
  <button data-h="L" data-o="P">&#9664;</button>
  <span></span>
  <button data-h="R" data-o="P">&#9654;</button>
  <span></span><button data-h="D" data-o="T">&#9660;</button><span></span>
</div>

<script>
  // Команды: один короткий запрос, дубли не отправляем.
  var last = '';
  function cmd(c){
    if (c === last) return;
    last = c;
    fetch('/a?c=' + c).catch(function(){});
  }

  function bind(b){
    var on = b.dataset.h, off = b.dataset.o || 's';
    var d = function(e){ e.preventDefault(); cmd(on); };
    var u = function(e){ e.preventDefault(); cmd(off); };
    b.addEventListener('touchstart', d, {passive:false});
    b.addEventListener('touchend', u);
    b.addEventListener('touchcancel', u);
    b.addEventListener('mousedown', d);
    b.addEventListener('mouseup', u);
    b.addEventListener('mouseleave', u);
  }
  document.querySelectorAll('button[data-h]').forEach(bind);
  document.querySelectorAll('button[data-c]').forEach(function(b){
    b.addEventListener('click', function(e){ e.preventDefault(); cmd(b.dataset.c); });
  });

  var KEY = { KeyW:['f','s'], KeyS:['b','s'], KeyA:['l','s'], KeyD:['r','s'],
              ArrowUp:['U','T'], ArrowDown:['D','T'],
              ArrowLeft:['L','P'], ArrowRight:['R','P'] };
  var held = {};
  onkeydown = function(e){
    var k = KEY[e.code]; if (!k) return;
    e.preventDefault();
    if (held[e.code]) return;        // автоповтор клавиши игнорируем
    held[e.code] = 1; cmd(k[0]);
  };
  onkeyup = function(e){
    var k = KEY[e.code]; if (!k) return;
    e.preventDefault();
    held[e.code] = 0; cmd(k[1]);
  };
  onblur = function(){ held = {}; cmd('s'); cmd('P'); cmd('T'); };

  document.getElementById('fs').onclick = function(){
    var d = document.documentElement;
    if (document.fullscreenElement) document.exitFullscreen();
    else if (d.requestFullscreen) d.requestFullscreen();
    else if (d.webkitRequestFullscreen) d.webkitRequestFullscreen();
  };
  var on = false;
  document.getElementById('led').onclick = function(){
    on = !on; cmd(on ? '1' : '0');
  };

  // Кадр за кадром: следующий просим только когда предыдущий загрузился.
  // Очередь кадров не копится, поэтому видно всегда самое свежее.
  var cam = document.getElementById('cam'), n = 0;
  function frame(){
    var img = new Image();
    img.onload  = function(){ cam.src = img.src; frame(); };
    img.onerror = function(){ setTimeout(frame, 500); };
    img.src = '/jpg?' + (n++);
  }
  frame();
</script></body></html>
)HTML";

static httpd_handle_t server = NULL;

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char q[32], v[8];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
      httpd_query_key_value(q, "c", v, sizeof(v)) == ESP_OK) {
    applyCommand(v);
  }
  return httpd_resp_send(req, NULL, 0);     // пустой ответ, нечего парсить
}

static esp_err_t jpg_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  const size_t fb_len = fb->len;
  esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);                 // возвращаем всегда

  // Кадров в секунду и уровень сигнала телефона. Если fps низкий, а
  // сигнал слабый - виноват эфир. Если сигнал сильный, а fps низкий -
  // виновата плата, и надо уменьшать кадр.
  static uint32_t frames = 0, at = 0;
  if (++frames >= 50) {
    wifi_sta_list_t sta;
    int rssi = 0;
    if (esp_wifi_ap_get_sta_list(&sta) == ESP_OK && sta.num > 0) {
      rssi = sta.sta[0].rssi;
    }
    Serial.printf("video: %lu kadr/sek, %u bayt, signal %d dBm\n",
                  (unsigned long)(frames * 1000UL / (millis() - at)),
                  (unsigned)fb_len, rssi);
    frames = 0;
    at = millis();
  }
  return r;
}

// Считаем "шум" каждого канала: чужая сеть мешает своим каналом сильно,
// соседними - слабее. Вес по мощности, чтобы близкий роутер весил больше.
#if AUTO_CHANNEL
static int pickQuietestChannel() {
  int n = WiFi.scanNetworks(false, false, false, 120);
  if (n <= 0) {
    Serial.println("wifi: skanirovanie pusto, kanal 1");
    return 1;
  }

  long score[12] = {0};
  for (int i = 0; i < n; i++) {
    int ch = WiFi.channel(i);
    if (ch < 1 || ch > 11) continue;
    long w = WiFi.RSSI(i) + 100;          // -90 dBm -> 10, -30 dBm -> 70
    if (w < 1) w = 1;
    w = w * w;                            // ближний сосед мешает намного сильнее
    for (int d = -2; d <= 2; d++) {
      int c = ch + d;
      if (c < 1 || c > 11) continue;
      score[c] += (d == 0) ? w : w / 3;   // соседний канал мешает втрое слабее
    }
  }
  WiFi.scanDelete();

  int best = 1;
  for (int c = 1; c <= 11; c++) if (score[c] < score[best]) best = c;
  Serial.printf("wifi: setey %d, vybran kanal %d (shum %ld, na 1-m %ld)\n",
                n, best, score[best], score[1]);
  return best;
}
#endif

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
  // Один буфер и съёмка по запросу. С двумя буферами и GRAB_LATEST камера
  // снимает непрерывно и постоянно жуёт шину PSRAM - ту самую, через
  // которую идёт Wi-Fi. Нам кадр нужен ровно тогда, когда его просят.
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  c.frame_size   = CAM_FRAME_SIZE;
  c.jpeg_quality = CAM_QUALITY;
  c.fb_count     = 1;
  c.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("camera: init FAILED (shleyf, PSRAM, pitanie)");
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, CAM_FRAME_SIZE);
    s->set_quality(s, CAM_QUALITY);
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
  }
  Serial.println("camera: OK");
  return true;
}

void setup() {
  // Детектор просадки питания сбрасывал плату на рывках моторов: они
  // проседают 5 В сильнее, чем ему нравится. Заводская прошивка
  // Keyestudio глушит его первой же строкой - делаем так же.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== car ===");

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pwmSetup(PIN_A_IN1, CH_A_IN1, 1000, 8);
  pwmSetup(PIN_A_IN2, CH_A_IN2, 1000, 8);
  pwmSetup(PIN_B_IN1, CH_B_IN1, 1000, 8);
  pwmSetup(PIN_B_IN2, CH_B_IN2, 1000, 8);
  motors(0, 0);

  cameraInit();
  servosInit();

  WiFi.persistent(false);

  int chan = AP_CHAN;
#if AUTO_CHANNEL
  WiFi.mode(WIFI_STA);                      // сканировать можно только в STA
  WiFi.disconnect(true);
  delay(150);
  chan = pickQuietestChannel();
#endif

  WiFi.mode(WIFI_AP);
  delay(200);
  WiFi.softAP("RobotCar", NULL, chan, 0, 1);  // без пароля, один клиент
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);                       // сон Wi-Fi = рывки

#if WIDE_CHANNEL
  // HT40 - канал двойной ширины, вдвое больше пропускная способность.
  // Если в эфире тесно и стало хуже - поставить WIDE_CHANNEL 0.
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
#endif

  Serial.printf("wifi: kanal %d, %s\n", chan, WIDE_CHANNEL ? "HT40" : "HT20");
  Serial.print("pult: http://");
  Serial.println(WiFi.softAPIP());

  // Один сервер: страница, команды и кадры. Раньше стрим занимал свой
  // сервер целиком, теперь кадр - обычный короткий запрос.
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 3;
  cfg.max_open_sockets = 4;
  cfg.lru_purge_enable = true;
  cfg.stack_size       = 8192;

  httpd_uri_t page = { "/",    HTTP_GET, index_handler, NULL };
  httpd_uri_t act  = { "/a",   HTTP_GET, cmd_handler,   NULL };
  httpd_uri_t jpg  = { "/jpg", HTTP_GET, jpg_handler,   NULL };

  if (httpd_start(&server, &cfg) == ESP_OK) {
    httpd_register_uri_handler(server, &page);
    httpd_register_uri_handler(server, &act);
    httpd_register_uri_handler(server, &jpg);
  } else {
    Serial.println("server FAILED");
  }
  g_lastCmd = millis();
}

void loop() {
  servoTick();

  // Failsafe: пульт отвалился - всё стоп.
  static bool stopped = false;
  if (millis() - g_lastCmd > 2000) {
    if (!stopped) {
      motors(0, 0);
      g_panDir = g_tiltDir = 0;
      stopped = true;
    }
  } else {
    stopped = false;
  }

  delay(10);
}
