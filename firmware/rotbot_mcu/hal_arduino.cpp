#ifdef ARDUINO
#include "hal_arduino.h"

#if WHEELS_ENABLED
#include <HardwareTimer.h>

#include "PeripheralPins.h"
#include "analog.h"
#include "pinmap.h"
#endif

#if FLY_HAS_TACH
// 量「脈衝週期」而非固定窗口計數：2400rpm、1 脈衝/轉時週期 25ms，micros() 解析 4us → 0.02%；
// 100ms 窗口計數只有 40 個脈衝 → 量化誤差 2.5%，對 6m 的 v0 穩定度太粗。
static volatile uint32_t g_last_us[2] = {0, 0};
static volatile uint32_t g_period_us[2] = {0, 0};
static void isr_tach_l() {
  uint32_t t = micros();
  g_period_us[0] = t - g_last_us[0];
  g_last_us[0] = t;
}
static void isr_tach_r() {
  uint32_t t = micros();
  g_period_us[1] = t - g_last_us[1];
  g_last_us[1] = t;
}
#endif

void ArduinoHal::begin() {
  pinMode(PIN_YAW_STEP, OUTPUT);
  pinMode(PIN_YAW_DIR, OUTPUT);
  pinMode(PIN_PITCH_STEP, OUTPUT);
  pinMode(PIN_PITCH_DIR, OUTPUT);
  pinMode(PIN_STEPPER_EN, OUTPUT);
  pinMode(PIN_FLY_L, OUTPUT);
  pinMode(PIN_FLY_R, OUTPUT);
  pinMode(PIN_FEED_IN1, OUTPUT);
  pinMode(PIN_FEED_IN2, OUTPUT);
  pinMode(PIN_FEED_IN3, OUTPUT);
  pinMode(PIN_FEED_IN4, OUTPUT);
  writeFeed(false);
  pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
#if MAG_ENABLED
  pinMode(PIN_MAG_PWM, OUTPUT);
  digitalWrite(PIN_MAG_PWM, LOW);
  pinMode(PIN_MAG_SW, INPUT_PULLUP);
#endif
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, HIGH);         // 先失能，App::begin() 再致能
#if WHEELS_ENABLED
  analogWriteFrequency(WHEEL_PWM_HZ);          // D6 與 M2 RPWM 共用 TIM2，頻率要一致
  beginWheels();
#endif
  analogWrite(PIN_FLY_L, 0);
  analogWrite(PIN_FLY_R, 0);
#if FLY_HAS_TACH
  pinMode(PIN_TACH_L, INPUT_PULLUP);
  pinMode(PIN_TACH_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH_L), isr_tach_l, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH_R), isr_tach_r, FALLING);
#endif
  tach_t0_ = ::millis();
}

// 整步、兩相同時通電：相 k 通 IN(k+1)、IN(k+2) → 1100、0110、0011、1001。HIGH = ULN2003 導通 = 線圈通電
void ArduinoHal::writeFeed(bool on) {
  static const uint32_t pins[4] = {PIN_FEED_IN1, PIN_FEED_IN2, PIN_FEED_IN3, PIN_FEED_IN4};
  for (uint8_t i = 0; i < 4; ++i)
    digitalWrite(pins[i], on && (i == feed_phase_ || i == ((feed_phase_ + 1) & 3)) ? HIGH : LOW);
}

void ArduinoHal::step(uint8_t axis, bool forward) {
  if (axis == AXIS_FEED) {
    feed_phase_ = (uint8_t)((feed_phase_ + (forward ? 1 : 3)) & 3);
    writeFeed(true);
    return;
  }
  bool pitch = axis == AXIS_PITCH;
  uint8_t sp = pitch ? PIN_PITCH_STEP : PIN_YAW_STEP;
  uint8_t dp = pitch ? PIN_PITCH_DIR : PIN_YAW_DIR;
  bool level = forward ^ (pitch ? PITCH_DIR_INVERT : YAW_DIR_INVERT);
  if ((int8_t)forward != last_dir_[axis]) {
    digitalWrite(dp, level ? HIGH : LOW);
    last_dir_[axis] = (int8_t)forward;
    delayMicroseconds(DIR_SETUP_US);           // DIR 建立時間
  }
  digitalWrite(sp, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(sp, LOW);
}

int32_t ArduinoHal::flywheelRpm() {
#if FLY_HAS_TACH
  noInterrupts();
  uint32_t p0 = g_period_us[0], p1 = g_period_us[1], l0 = g_last_us[0], l1 = g_last_us[1];
  interrupts();
  uint32_t now = ::micros();
  // 超過 200ms 沒脈衝 → 視為停止（< 300rpm）
  float r0 = (p0 == 0 || now - l0 > 200000UL) ? 0.0f : 60.0e6f / ((float)p0 * FLY_TACH_PPR);
  float r1 = (p1 == 0 || now - l1 > 200000UL) ? 0.0f : 60.0e6f / ((float)p1 * FLY_TACH_PPR);
  rpm_ = (int32_t)((r0 + r1) * 0.5f + 0.5f);   // 兩輪平均（左右差太大代表打滑/卡住，可另外檢查）
  return rpm_;
#else
  return -1;
#endif
}

#if WHEELS_ENABLED
// PWM：pwm_start 依 PinName（含 _ALTn）選計時器與通道；之後只改比較值。
// 編碼器：計時器的 encoder mode（TI1 + TI2 四倍頻），16 位元計數，讀取時取差值累加。
static const PinName kRpwm[4] = WHEEL_RPWM_PINS;
static const PinName kLpwm[4] = WHEEL_LPWM_PINS;
static const PinName kEncA[4] = WHEEL_ENC_A_PINS;
static const PinName kEncB[4] = WHEEL_ENC_B_PINS;
static HardwareTimer* g_pwm_tim[8];
static uint32_t g_pwm_ch[8];
static TIM_HandleTypeDef* g_enc[4];

static HardwareTimer* timerOf(PinName p) {
  TIM_TypeDef* inst = (TIM_TypeDef*)pinmap_peripheral(p, PinMap_TIM);
  timerObj_t* obj = HardwareTimer_Handle[get_timer_index(inst)];
  return obj ? (HardwareTimer*)obj->__this : new HardwareTimer(inst);
}

void ArduinoHal::beginWheels() {
  pinMode(PIN_WHEEL_EN, OUTPUT);
  digitalWrite(PIN_WHEEL_EN, LOW);
  for (uint8_t k = 0; k < 8; ++k) {            // 偶數 = RPWM、奇數 = LPWM
    PinName p = (k & 1) ? kLpwm[k / 2] : kRpwm[k / 2];
    pwm_start(p, WHEEL_PWM_HZ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    g_pwm_tim[k] = timerOf(p);
    g_pwm_ch[k] = STM_PIN_CHANNEL(pinmap_function(p, PinMap_TIM));
  }
  for (uint8_t i = 0; i < 4; ++i) {
    TIM_HandleTypeDef* h = timerOf(kEncA[i])->getHandle();   // 建構時已開時脈
    pinmap_pinout(kEncA[i], PinMap_TIM);                       // AF + 上拉（霍爾輸出多為開集極）
    pinmap_pinout(kEncB[i], PinMap_TIM);
    h->Init.Prescaler = 0;
    h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.Period = 0xFFFF;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h->Init.RepetitionCounter = 0;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    TIM_Encoder_InitTypeDef c = {};
    c.EncoderMode = TIM_ENCODERMODE_TI12;
    c.IC1Polarity = TIM_ICPOLARITY_RISING;
    c.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    c.IC1Prescaler = TIM_ICPSC_DIV1;
    c.IC1Filter = 10;                          // 數位濾波，擋掉馬達 PWM 耦合的短脈衝
    c.IC2Polarity = TIM_ICPOLARITY_RISING;
    c.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    c.IC2Prescaler = TIM_ICPSC_DIV1;
    c.IC2Filter = 10;
    HAL_TIM_Encoder_Init(h, &c);
    HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL);
    g_enc[i] = h;
    enc_last_[i] = (uint16_t)__HAL_TIM_GET_COUNTER(h);
  }
}

void ArduinoHal::setWheel(uint8_t i, int16_t duty) {
  if ((WHEEL_INVERT >> i) & 1) duty = -duty;
  uint32_t v = (uint32_t)((duty < 0 ? -duty : duty) * 1023L / 1000);
  g_pwm_tim[2 * i]->setCaptureCompare(g_pwm_ch[2 * i], duty > 0 ? v : 0, RESOLUTION_10B_COMPARE_FORMAT);
  g_pwm_tim[2 * i + 1]->setCaptureCompare(g_pwm_ch[2 * i + 1], duty < 0 ? v : 0, RESOLUTION_10B_COMPARE_FORMAT);
}

int32_t ArduinoHal::wheelCount(uint8_t i) {
  uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(g_enc[i]);
  int16_t d = (int16_t)(raw - enc_last_[i]);
  enc_last_[i] = raw;
  bool inv = (((WHEEL_INVERT ^ WHEEL_ENC_INVERT) >> i) & 1) != 0;
  enc_acc_[i] += inv ? -d : d;
  return enc_acc_[i];
}

uint16_t ArduinoHal::wheelRaw(uint8_t i) { return (uint16_t)__HAL_TIM_GET_COUNTER(g_enc[i]); }

uint8_t ArduinoHal::wheelPins() {                // AF 模式下 IDR 仍反映腳位電位
  uint8_t m = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    if (digitalReadFast(kEncA[i])) m |= (uint8_t)(1 << (2 * i));
    if (digitalReadFast(kEncB[i])) m |= (uint8_t)(1 << (2 * i + 1));
  }
  return m;
}
#endif

float ArduinoHal::batteryVolts() {
#if VBAT_ENABLED
  uint32_t now = ::millis();
  if (vbat_ < 0 || now - vbat_t_ >= 20) {      // analogRead ~110us，每 20ms 讀一次 + 一階低通
    float v = analogRead(PIN_VBAT) * (VBAT_ADC_VREF / 1023.0f) * VBAT_DIVIDER;   // analogRead 預設 10 bit（Uno 與 STM32duino 相同）
    vbat_ = vbat_ < 0 ? v : vbat_ + 0.1f * (v - vbat_);
    vbat_t_ = now;
  }
  return vbat_;
#else
  return -1.0f;
#endif
}
#endif  // ARDUINO
