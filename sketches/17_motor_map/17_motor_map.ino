/*
 * 17_motor_map - какая группа колёс к какому пину.
 *
 * Фара мигает 1 раз -> крутится ГРУППА А (пины 13/12), 2 секунды
 * Фара мигает 2 раза -> крутится ГРУППА Б (пины 15/14), 2 секунды
 * И так по кругу. Колёса вывесить, чтобы робот не уехал.
 *
 * ЗАПИШИ: какие колёса крутятся в группе А, какие в группе Б,
 * и в какую сторону (вперёд или назад).
 *
 * Результат на нашем шасси: А = передние, Б = задние. Значит бортами
 * управлять нельзя и поворотов нет, пока провода не переставлены.
 */

#define PIN_A_IN1 13
#define PIN_A_IN2 12
#define PIN_B_IN1 15
#define PIN_B_IN2 14
#define PIN_FLASH  4

#define CH_A1 2
#define CH_A2 3
#define CH_B1 4
#define CH_B2 5

#define MOTOR_FREQ 1000
#define MOTOR_RES  8
#define TEST_SPEED 170

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch, freq, res) ledcAttach(pin, freq, res)
  #define PWM_WRITE(pin, ch, duty)      ledcWrite(pin, duty)
#else
  #define PWM_SETUP(pin, ch, freq, res) do { ledcSetup(ch, freq, res); \
                                             ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, duty)      ledcWrite(ch, duty)
#endif

static void allStop() {
  PWM_WRITE(PIN_A_IN1, CH_A1, 0);
  PWM_WRITE(PIN_A_IN2, CH_A2, 0);
  PWM_WRITE(PIN_B_IN1, CH_B1, 0);
  PWM_WRITE(PIN_B_IN2, CH_B2, 0);
}

static void blink(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_FLASH, HIGH);
    delay(150);
    digitalWrite(PIN_FLASH, LOW);
    delay(250);
  }
}

void setup() {
  pinMode(PIN_FLASH, OUTPUT);
  digitalWrite(PIN_FLASH, LOW);

  PWM_SETUP(PIN_A_IN1, CH_A1, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_A_IN2, CH_A2, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_B_IN1, CH_B1, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_B_IN2, CH_B2, MOTOR_FREQ, MOTOR_RES);
  allStop();
  delay(2000);
}

void loop() {
  // ГРУППА А, пины 13/12
  blink(1);
  PWM_WRITE(PIN_A_IN1, CH_A1, TEST_SPEED);
  PWM_WRITE(PIN_A_IN2, CH_A2, 0);
  delay(2000);
  allStop();
  delay(2000);

  // ГРУППА Б, пины 15/14
  blink(2);
  PWM_WRITE(PIN_B_IN1, CH_B1, TEST_SPEED);
  PWM_WRITE(PIN_B_IN2, CH_B2, 0);
  delay(2000);
  allStop();
  delay(2000);
}
