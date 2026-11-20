// MCU 應用層：協定解析 + 轉塔/飛輪/推球狀態機 + 看門狗/急停。不含任何腳位操作（交給 Hal）。
#pragma once
#include <stdint.h>

#include "config.h"
#include "hal.h"
#include "proto.h"

// 單軸步進：位置模式（梯形速度、減速停在目標）或速度模式（JOG）。
struct StepAxis {
  uint8_t id;
  float steps_per_deg, max_speed, accel;   // steps/deg, steps/s, steps/s^2
  long min_steps, max_steps;
  long pos = 0, target = 0;                // steps
  float speed = 0, jog_speed = 0, frac = 0;
  bool jog = false;

  void configure(uint8_t axis_id, float spd, long min_cdeg, long max_cdeg, float max_dps, float acc_dps2);
  long to_steps(long cdeg) const;
  long to_cdeg(long steps) const;
  bool moving() const;
  void update(Hal& hal, float dt);
};

class App {
 public:
  explicit App(Hal& hal) : hal_(hal) {}
  void begin();                // 開機（或模擬器新連線）：重置狀態並送 EV BOOT
  void onByte(char c);         // 餵序列輸入
  void update();               // 每次 loop() 呼叫

  // 測試/模擬器觀察用
  long pitchCdeg() const { return pitch_.to_cdeg(pitch_.pos); }
  long yawCdeg() const { return yaw_.to_cdeg(yaw_.pos); }
  uint16_t shots() const { return shots_; }

 private:
  void handleLine(char* line);
  void reply(const char* body);
  void sendAck(uint8_t seq, uint8_t code);
  void sendEvent(const char* name, long arg);
  uint8_t flags();
  bool ready();
  void enterSafe();
  void updateFlywheel(uint32_t now, float dt);
  void updateFeeder(uint32_t now, float dt);
  void startPush();            // 推彈開始轉一圈（上一圈被急停打斷時只轉完那一圈）
  void updateWheels(uint32_t now, float dt);
  void stopWheels();           // 目標與輸出立刻歸 0（煞車）

  Hal& hal_;
  proto::LineReader reader_;
  StepAxis pitch_, yaw_;
  uint32_t last_us_ = 0, last_rx_ms_ = 0, still_since_ms_ = 0;
  bool homed_ = false, wdog_ = false, estop_ = false, was_ready_ = false;
  bool armed_ = false;         // 收到第一個合法幀後看門狗才開始計時
  // 飛輪
  long fly_target_ = 0;
  uint32_t fly_changed_ms_ = 0, fly_ok_since_ms_ = 0;
  float fly_i_ = 0;
  bool fly_ok_ = false;
  // 推球：第三個步進軸，只往前轉，FIRE 一次轉一圈
  StepAxis feed_;
  float feed_rem_ = 0;         // 一圈步數的小數部分累積到下一發
  bool firing_ = false;
  bool loading_ = false;       // FIRE 的第一段：彈倉 N20 把球推到微動開關，再多推 MAG_EXTRA_MS
  bool pressed_ = false;       // 上彈中已經壓到開關（之後只計時，不再看開關）
  uint32_t load_ms_ = 0, sw_ms_ = 0;   // 開始上彈的時間；開關最近一次沒被壓著的時間（去彈跳）
  uint16_t shots_ = 0;
  // 車輪
  int16_t wheel_target_[4] = {0, 0, 0, 0};   // 千分比
  float wheel_duty_[4] = {0, 0, 0, 0};       // 經過變化率限制的輸出
  int16_t wheel_out_[4] = {0, 0, 0, 0};      // 上次送給 Hal 的值
  int32_t wheel_count_[4] = {0, 0, 0, 0};
  int16_t wheel_rpm10_[4] = {0, 0, 0, 0};    // 輸出軸轉速 × 10
  uint32_t drive_rx_ms_ = 0, wheel_rpm_ms_ = 0;
  // 協定狀態
  uint8_t last_err_ = 0;
  uint16_t crc_errors_ = 0, evseq_ = 0;
  uint8_t last_seq_ = 0, last_req_crc_ = 0;
  char last_reply_[proto::MAX_LINE + 1];
};
