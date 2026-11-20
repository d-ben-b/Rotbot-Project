#include "app.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FLY_KP
#define FLY_KP 0.05f   // PWM / rpm（前饋負責大部分，PI 只修殘差；實機需依飛輪重新調）
#define FLY_KI 0.20f   // PWM / (rpm*s)
#endif

static long round_l(float x) { return (long)(x >= 0 ? x + 0.5f : x - 0.5f); }

// ══════════════════════════ StepAxis ══════════════════════════
void StepAxis::configure(uint8_t axis_id, float spd, long min_cdeg, long max_cdeg,
                         float max_dps, float acc_dps2) {
  id = axis_id;
  steps_per_deg = spd;
  min_steps = to_steps(min_cdeg);
  max_steps = to_steps(max_cdeg);
  max_speed = max_dps * spd;
  accel = acc_dps2 * spd;
  pos = target = 0;
  speed = jog_speed = frac = 0;
  jog = false;
}

long StepAxis::to_steps(long cdeg) const { return round_l((float)cdeg * steps_per_deg / 100.0f); }
long StepAxis::to_cdeg(long steps) const { return round_l((float)steps * 100.0f / steps_per_deg); }

bool StepAxis::moving() const {
  if (fabs(speed) > 0.5f) return true;
  return !jog && pos != target;
}

void StepAxis::update(Hal& hal, float dt) {
  float v_des;
  if (jog) {
    v_des = jog_speed;
  } else {
    long err = target - pos;
    if (err == 0 && fabs(speed) <= accel * dt * 2) {   // 已到位
      speed = 0;
      frac = 0;
      return;
    }
    float v = sqrt(2.0f * accel * (float)labs(err));   // 能在目標前停下的最大速度
    if (v > max_speed) v = max_speed;
    v_des = err > 0 ? v : (err < 0 ? -v : 0.0f);
  }
  float dv = accel * dt;                                 // 加速度限制
  if (v_des > speed + dv) speed += dv;
  else if (v_des < speed - dv) speed -= dv;
  else speed = v_des;

  frac += speed * dt;
  while (frac >= 1.0f) {
    if (pos >= max_steps || (!jog && pos >= target)) { frac = 0; speed = 0; break; }
    hal.step(id, true);
    ++pos;
    frac -= 1.0f;
  }
  while (frac <= -1.0f) {
    if (pos <= min_steps || (!jog && pos <= target)) { frac = 0; speed = 0; break; }
    hal.step(id, false);
    --pos;
    frac += 1.0f;
  }
}

// ══════════════════════════ App ══════════════════════════
void App::begin() {
  pitch_.configure(AXIS_PITCH, PITCH_STEPS_PER_DEG, PITCH_MIN_CDEG, PITCH_MAX_CDEG,
                   PITCH_MAX_DPS, PITCH_ACCEL_DPS2);
  yaw_.configure(AXIS_YAW, YAW_STEPS_PER_DEG, YAW_MIN_CDEG, YAW_MAX_CDEG,
                 YAW_MAX_DPS, YAW_ACCEL_DPS2);
  feed_.configure(AXIS_FEED, FEED_STEPS_PER_REV / 360.0f, 0, 0, 0, 0);
  feed_.max_steps = 0x7FFFFFFFL;               // 只往前轉，不設上限
  feed_.max_speed = FEED_STEP_HZ;
  feed_.accel = FEED_ACCEL;
  feed_rem_ = 0;
  homed_ = wdog_ = estop_ = was_ready_ = armed_ = false;
  fly_target_ = 0;
  fly_i_ = 0;
  fly_ok_ = false;
  firing_ = loading_ = false;
  hal_.setMagazine(false);
  shots_ = 0;
  last_err_ = 0;
  crc_errors_ = 0;
  evseq_ = 0;
  last_seq_ = 0;
  last_req_crc_ = 0;
  last_reply_[0] = '\0';
  hal_.setFlywheelPwm(0);
  hal_.enableFeeder(false);
  hal_.enableSteppers(true);
  hal_.enableWheels(false);                    // 收到第一個合法幀才致能
  for (uint8_t i = 0; i < 4; ++i) {
    wheel_target_[i] = wheel_out_[i] = wheel_rpm10_[i] = 0;
    wheel_duty_[i] = 0;
    hal_.setWheel(i, 0);
    wheel_count_[i] = hal_.wheelCount(i);
  }
  last_us_ = hal_.micros();
  last_rx_ms_ = still_since_ms_ = fly_changed_ms_ = fly_ok_since_ms_ = hal_.millis();
  drive_rx_ms_ = wheel_rpm_ms_ = last_rx_ms_;
  sendEvent("BOOT", FW_VERSION);
}

void App::onByte(char c) {
  if (reader_.feed(c)) handleLine(reader_.line());
}

void App::reply(const char* body) {
  char out[proto::MAX_LINE + 1];
  size_t n = proto::frame(out, sizeof(out), body);
  if (!n) return;
  hal_.write(out, n);
  memcpy(last_reply_, out, n + 1);           // 供重送去重
}

void App::sendAck(uint8_t seq, uint8_t code) {
  char body[24];
  if (code != proto::E_OK) last_err_ = code;
  snprintf(body, sizeof(body), code == proto::E_OK ? "ACK,%u,%u" : "NAK,%u,%u",
           (unsigned)seq, (unsigned)code);
  reply(body);
}

void App::sendEvent(const char* name, long arg) {
  char body[40], out[proto::MAX_LINE + 1];
  ++evseq_;
  snprintf(body, sizeof(body), "EV,%u,%s,%ld", (unsigned)evseq_, name, arg);
  size_t n = proto::frame(out, sizeof(out), body);
  if (n) hal_.write(out, n);
}

uint8_t App::flags() {
  uint8_t f = 0;
  if (ready()) f |= proto::F_READY;
  if (pitch_.moving() || yaw_.moving()) f |= proto::F_MOVING;
  if (fly_ok_) f |= proto::F_FLY_OK;
  if (firing_) f |= proto::F_FIRING;
  if (estop_) f |= proto::F_ESTOP;
  if (wdog_) f |= proto::F_WDOG;
  if (homed_) f |= proto::F_HOMED;
  if (pitch_.jog || yaw_.jog) f |= proto::F_JOGGING;
  return f;
}

bool App::ready() {
  return homed_ && !estop_ && !wdog_ && !firing_ && fly_ok_ && !pitch_.moving() &&
         !yaw_.moving() && (uint32_t)(hal_.millis() - still_since_ms_) >= SETTLE_MS;
}

void App::enterSafe() {
  fly_target_ = 0;
  fly_changed_ms_ = hal_.millis();
  pitch_.jog_speed = yaw_.jog_speed = 0;
  stopWheels();
}

void App::stopWheels() {
  for (uint8_t i = 0; i < 4; ++i) {
    wheel_target_[i] = wheel_out_[i] = 0;
    wheel_duty_[i] = 0;
    hal_.setWheel(i, 0);
  }
}

void App::handleLine(char* line) {
  char* f[proto::MAX_FIELDS];
  uint8_t n = 0;
  uint8_t fingerprint = proto::crc8(line, strlen(line));   // 去重用：整行內容指紋
  proto::ParseStatus ps = proto::split(line, f, &n, ALLOW_NO_CRC);
  if (ps == proto::PARSE_BAD_CRC) { ++crc_errors_; return; }   // SEQ 不可信 → 不回，讓 Orin 逾時重送
  if (ps == proto::PARSE_EMPTY || n < 2) return;
  long seq_l;
  if (!proto::to_long(f[1], &seq_l) || seq_l < 0 || seq_l > 255) return;
  uint8_t seq = (uint8_t)seq_l;
  uint32_t now = hal_.millis();
  last_rx_ms_ = now;
  if (!armed_ && !estop_) hal_.enableWheels(true);
  armed_ = true;
  wdog_ = false;                                  // 收到合法幀即解除看門狗安全狀態

  if (seq != 0 && seq == last_seq_ && fingerprint == last_req_crc_ && last_reply_[0]) {
    hal_.write(last_reply_, strlen(last_reply_)); // 重送：只回上次的回覆，不重做（FIRE 不連發）
    return;
  }
  last_seq_ = seq;
  last_req_crc_ = fingerprint;
  last_reply_[0] = '\0';

  const char* cmd = f[0];
  uint8_t nargs = n - 2;
  long a = 0, b = 0;
  bool args_ok = (nargs < 1 || proto::to_long(f[2], &a)) && (nargs < 2 || proto::to_long(f[3], &b));
  if (!args_ok) { sendAck(seq, proto::E_FORMAT); return; }
  char body[proto::MAX_LINE];

  if (!strcmp(cmd, "PING")) {
    if (nargs != 0) { sendAck(seq, proto::E_FORMAT); return; }
    snprintf(body, sizeof(body), "PONG,%u,%u,%u,%lu", (unsigned)seq, (unsigned)proto::VERSION,
             (unsigned)FW_VERSION, (unsigned long)now);
    reply(body);
  } else if (!strcmp(cmd, "STAT")) {
    if (nargs != 0) { sendAck(seq, proto::E_FORMAT); return; }
    float vbat = hal_.batteryVolts();
    snprintf(body, sizeof(body), "ST,%u,%u,%ld,%ld,%ld,%ld,%u,%u,%u,%ld", (unsigned)seq,
             (unsigned)flags(), pitchCdeg(), yawCdeg(), (long)hal_.flywheelRpm(), fly_target_,
             (unsigned)shots_, (unsigned)last_err_, (unsigned)crc_errors_,
             vbat > 0 ? (long)(vbat * 10.0f + 0.5f) : -1L);
    reply(body);
  } else if (!strcmp(cmd, "AIM")) {
    if (nargs != 2) { sendAck(seq, proto::E_FORMAT); return; }
    if (estop_) { sendAck(seq, proto::E_ESTOP); return; }
    if (firing_) { sendAck(seq, proto::E_BUSY); return; }
    if (!homed_) { sendAck(seq, proto::E_NOT_HOMED); return; }
    if (a < PITCH_MIN_CDEG || a > PITCH_MAX_CDEG || b < YAW_MIN_CDEG || b > YAW_MAX_CDEG) {
      sendAck(seq, proto::E_RANGE);
      return;
    }
    pitch_.jog = yaw_.jog = false;
    pitch_.target = pitch_.to_steps(a);
    yaw_.target = yaw_.to_steps(b);
    sendAck(seq, proto::E_OK);
  } else if (!strcmp(cmd, "FLY")) {
    if (nargs != 1) { sendAck(seq, proto::E_FORMAT); return; }
    if (estop_ && a != 0) { sendAck(seq, proto::E_ESTOP); return; }
    if (a < 0 || a > (long)FLY_MAX_RPM * 2) { sendAck(seq, proto::E_RANGE); return; }
    if (a != fly_target_) {
      fly_target_ = a;
      fly_changed_ms_ = fly_ok_since_ms_ = now;
      fly_ok_ = false;
    }
    sendAck(seq, proto::E_OK);
  } else if (!strcmp(cmd, "FIRE")) {
    if (nargs != 0) { sendAck(seq, proto::E_FORMAT); return; }
    if (estop_) { sendAck(seq, proto::E_ESTOP); return; }
    if (firing_) { sendAck(seq, proto::E_BUSY); return; }
    if (!ready()) { sendAck(seq, proto::E_NOT_READY); return; }
    firing_ = true;
    loading_ = feed_.pos == feed_.target;      // 上一圈被急停打斷時不上彈：推塊可能還在前面，先把那一圈轉完
    if (loading_) {
      load_ms_ = sw_ms_ = now;
      hal_.setMagazine(!hal_.ballReady());     // 球已經壓著開關就不轉 N20
    } else {
      startPush();
    }
    sendAck(seq, proto::E_OK);
  } else if (!strcmp(cmd, "JOG")) {
    if (nargs != 2) { sendAck(seq, proto::E_FORMAT); return; }
    if (estop_) { sendAck(seq, proto::E_ESTOP); return; }
    if (labs(a) > (long)(JOG_MAX_DPS * 100) || labs(b) > (long)(JOG_MAX_DPS * 100)) {
      sendAck(seq, proto::E_RANGE);
      return;
    }
    pitch_.jog = yaw_.jog = true;
    pitch_.jog_speed = (float)a / 100.0f * pitch_.steps_per_deg;
    yaw_.jog_speed = (float)b / 100.0f * yaw_.steps_per_deg;
    sendAck(seq, proto::E_OK);
  } else if (!strcmp(cmd, "DRIVE")) {           // DRIVE,seq,m1,m2,m3,m4（千分比，正 = 前進）
    if (nargs != 4) { sendAck(seq, proto::E_FORMAT); return; }
    long v[4];
    for (uint8_t i = 0; i < 4; ++i)
      if (!proto::to_long(f[2 + i], &v[i])) { sendAck(seq, proto::E_FORMAT); return; }
    for (uint8_t i = 0; i < 4; ++i)
      if (labs(v[i]) > 1000) { sendAck(seq, proto::E_RANGE); return; }
    if (estop_) { sendAck(seq, proto::E_ESTOP); return; }
    for (uint8_t i = 0; i < 4; ++i) wheel_target_[i] = (int16_t)(v[i] * WHEEL_MAX_PERMILLE / 1000);
    drive_rx_ms_ = now;
    sendAck(seq, proto::E_OK);
  } else if (!strcmp(cmd, "WHL")) {             // 四輪輸出軸轉速 × 10
    if (nargs != 0) { sendAck(seq, proto::E_FORMAT); return; }
    snprintf(body, sizeof(body), "WH,%u,%d,%d,%d,%d", (unsigned)seq, (int)wheel_rpm10_[0],
             (int)wheel_rpm10_[1], (int)wheel_rpm10_[2], (int)wheel_rpm10_[3]);
    reply(body);
  } else if (!strcmp(cmd, "ENC")) {             // 除錯：四輪編碼器原始計數 + A/B 腳位電位
    if (nargs != 0) { sendAck(seq, proto::E_FORMAT); return; }
    snprintf(body, sizeof(body), "ER,%u,%u,%u,%u,%u,%u", (unsigned)seq, (unsigned)hal_.wheelRaw(0),
             (unsigned)hal_.wheelRaw(1), (unsigned)hal_.wheelRaw(2), (unsigned)hal_.wheelRaw(3),
             (unsigned)hal_.wheelPins());
    reply(body);
  } else if (!strcmp(cmd, "STOP")) {
    if (nargs != 0) { sendAck(seq, proto::E_FORMAT); return; }
    pitch_.jog = yaw_.jog = true;                 // 速度模式 + 目標速度 0 → 減速停在原地
    pitch_.jog_speed = yaw_.jog_speed = 0;
    fly_target_ = 0;
    fly_changed_ms_ = now;
    stopWheels();
    sendAck(seq, proto::E_OK);
  } else if (!strcmp(cmd, "ZERO")) {
    if (nargs != 2) { sendAck(seq, proto::E_FORMAT); return; }
    if (pitch_.moving() || yaw_.moving() || firing_) { sendAck(seq, proto::E_BUSY); return; }
    if (a < PITCH_MIN_CDEG || a > PITCH_MAX_CDEG || b < YAW_MIN_CDEG || b > YAW_MAX_CDEG) {
      sendAck(seq, proto::E_RANGE);
      return;
    }
    pitch_.jog = yaw_.jog = false;
    pitch_.pos = pitch_.target = pitch_.to_steps(a);
    yaw_.pos = yaw_.target = yaw_.to_steps(b);
    pitch_.frac = yaw_.frac = 0;
    homed_ = true;
    sendAck(seq, proto::E_OK);
  } else {
    sendAck(seq, proto::E_UNKNOWN);
  }
}

void App::updateFlywheel(uint32_t now, float dt) {
  int32_t meas = hal_.flywheelRpm();
  float pwm = 0;
  if (fly_target_ <= 0) {
    fly_i_ = 0;
    fly_ok_ = false;
  } else {
    pwm = (float)fly_target_ * 255.0f / (float)FLY_MAX_RPM;      // 前饋
    float vbat = hal_.batteryVolts();
    if (vbat > 1.0f) pwm *= FLY_VNOM / vbat;                      // 電池電壓補償（電量下降時自動加大 PWM）
#if FLY_HAS_TACH
    if (meas >= 0) {                                               // PI 穩速（v0 穩定度關鍵）
      float e = (float)(fly_target_ - meas);
      if (fabs(e) < 0.05f * (float)fly_target_)                    // 只在 ±5% 內積分（防起轉 windup 過衝）
        fly_i_ += e * FLY_KI * dt;
      if (fly_i_ > 80) fly_i_ = 80;
      if (fly_i_ < -80) fly_i_ = -80;
      pwm += e * FLY_KP + fly_i_;
    }
    // 到速判定加遲滯：連續 FLY_STABLE_MS 在 ±TOL 內才算到速；之後超出 ±2×TOL 才取消（避免 READY 閃爍）
    long dev100 = meas >= 0 ? labs((long)meas - fly_target_) * 100 : 0x7FFFFFFL;
    bool in_tol = dev100 <= fly_target_ * FLY_TOL_PCT;
    if (fly_ok_) {
      fly_ok_ = dev100 <= fly_target_ * FLY_TOL_PCT * 2;
      if (!fly_ok_) fly_ok_since_ms_ = now;
    } else {
      if (!in_tol) fly_ok_since_ms_ = now;
      fly_ok_ = in_tol && (uint32_t)(now - fly_ok_since_ms_) >= FLY_STABLE_MS;
    }
#else
    (void)meas;
    (void)dt;
    fly_ok_ = (uint32_t)(now - fly_changed_ms_) >= FLY_SPINUP_MS;
#endif
  }
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  hal_.setFlywheelPwm((uint8_t)(pwm + 0.5f));
}

void App::startPush() {
  if (feed_.pos == feed_.target) {             // 上一圈被急停打斷時不加：這一發只把那一圈轉完，推塊回到最後面
    float want = FEED_STEPS_PER_REV + feed_rem_;
    long n = round_l(want);
    feed_rem_ = want - (float)n;               // 一圈不是整數步時，推塊不會越停越偏
    feed_.target += n;
  }
  hal_.enableFeeder(true);
}

void App::updateFeeder(uint32_t now, float dt) {
  if (!firing_) return;
  if (loading_) {
    if (!hal_.ballReady()) sw_ms_ = now;
    if ((uint32_t)(now - sw_ms_) >= MAG_DEBOUNCE_MS) {          // 球壓著開關 → N20 停、推彈
      hal_.setMagazine(false);
      loading_ = false;
      startPush();
    } else if ((uint32_t)(now - load_ms_) >= MAG_TIMEOUT_MS) {  // 沒球或卡住：不推彈，等下一個 FIRE
      hal_.setMagazine(false);
      loading_ = firing_ = false;
      sendEvent("FAULT", 1);
    }
    return;
  }
  feed_.update(hal_, dt);
  if (!feed_.moving()) {                       // 開迴路：卡住失步也偵測不到，只能數步數
    hal_.enableFeeder(false);
    firing_ = false;
    ++shots_;
    sendEvent("FIRED", shots_);
  }
}

void App::updateWheels(uint32_t now, float dt) {
  bool any = false;
  for (uint8_t i = 0; i < 4; ++i) any |= wheel_target_[i] != 0;
  if (any && (uint32_t)(now - drive_rx_ms_) > DRIVE_TIMEOUT_MS)   // deadman：Orin 沒有持續送 DRIVE
    for (uint8_t i = 0; i < 4; ++i) wheel_target_[i] = 0;
  float step = WHEEL_SLEW_PER_S * dt;
  for (uint8_t i = 0; i < 4; ++i) {
    float d = wheel_duty_[i], t = wheel_target_[i];
    d = t > d + step ? d + step : (t < d - step ? d - step : t);
    wheel_duty_[i] = d;
    int16_t out = (int16_t)(d >= 0 ? d + 0.5f : d - 0.5f);
    if (out != wheel_out_[i]) {
      wheel_out_[i] = out;
      hal_.setWheel(i, out);
    }
  }
  uint32_t el = now - wheel_rpm_ms_;
  if (el >= WHEEL_RPM_MS) {                     // 計數差 → 輸出軸 rpm × 10
    const float per_count = 600000.0f / ((float)WHEEL_ENC_PPR * 4.0f * (float)WHEEL_GEAR);
    for (uint8_t i = 0; i < 4; ++i) {
      int32_t c = hal_.wheelCount(i);
      float rpm10 = (float)(c - wheel_count_[i]) * per_count / (float)el;
      wheel_count_[i] = c;
      wheel_rpm10_[i] = (int16_t)(rpm10 >= 0 ? rpm10 + 0.5f : rpm10 - 0.5f);
    }
    wheel_rpm_ms_ = now;
  }
}

void App::update() {
  uint32_t now_us = hal_.micros();
  float dt = (float)(uint32_t)(now_us - last_us_) * 1e-6f;
  last_us_ = now_us;
  if (dt > 0.05f) dt = 0.05f;
  uint32_t now = hal_.millis();

  bool es = hal_.estopActive();
  if (es != estop_) {
    estop_ = es;
    if (es) {                                    // 急停：全停、步進失能、需重新 ZERO
      enterSafe();
      firing_ = loading_ = false;
      hal_.setMagazine(false);
      hal_.enableFeeder(false);
      feed_.speed = feed_.frac = 0;              // target 不動：下一發先把這一圈轉完
      hal_.enableSteppers(false);
      pitch_.speed = yaw_.speed = 0;
      pitch_.target = pitch_.pos;
      yaw_.target = yaw_.pos;
      pitch_.jog = yaw_.jog = false;
      homed_ = false;
      stopWheels();
      hal_.enableWheels(false);
    } else {
      hal_.enableSteppers(true);
      hal_.enableWheels(armed_);
    }
    sendEvent("ESTOP", es ? 1 : 0);
  }
  if (armed_ && !wdog_ && (uint32_t)(now - last_rx_ms_) > WDOG_MS) {
    wdog_ = true;                                // Orin 失聯：飛輪停、JOG 停（位置保持）
    enterSafe();
    sendEvent("WDOG", WDOG_MS);
  }
  if (!estop_) {
    pitch_.update(hal_, dt);
    yaw_.update(hal_, dt);
  }
  if (pitch_.moving() || yaw_.moving()) still_since_ms_ = now;
  updateFlywheel(now, dt);
  updateFeeder(now, dt);
  updateWheels(now, dt);
  bool r = ready();
  if (r && !was_ready_) sendEvent("READY", shots_);
  was_ready_ = r;
}
