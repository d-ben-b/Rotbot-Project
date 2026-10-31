#ifdef ARDUINO
#include "hal_arduino.h"

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
  pinMode(PIN_FEED, OUTPUT);
  pinMode(PIN_FEED_HOME, INPUT_PULLUP);
  pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, HIGH);         // 先失能，App::begin() 再致能
  analogWrite(PIN_FLY_L, 0);
  analogWrite(PIN_FLY_R, 0);
  analogWrite(PIN_FEED, 0);
#if FLY_HAS_TACH
  pinMode(PIN_TACH_L, INPUT_PULLUP);
  pinMode(PIN_TACH_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH_L), isr_tach_l, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH_R), isr_tach_r, FALLING);
#endif
  tach_t0_ = ::millis();
}

void ArduinoHal::step(uint8_t axis, bool forward) {
  bool pitch = axis == AXIS_PITCH;
  uint8_t sp = pitch ? PIN_PITCH_STEP : PIN_YAW_STEP;
  uint8_t dp = pitch ? PIN_PITCH_DIR : PIN_YAW_DIR;
  bool level = forward ^ (pitch ? PITCH_DIR_INVERT : YAW_DIR_INVERT);
  if (forward != last_dir_[axis]) {
    digitalWrite(dp, level ? HIGH : LOW);
    last_dir_[axis] = forward;
    delayMicroseconds(2);                      // DIR 建立時間
  }
  digitalWrite(sp, HIGH);
  delayMicroseconds(2);                        // TMC2209 STEP 脈寬 >= 100 ns
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
