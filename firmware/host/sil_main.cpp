// 韌體在電腦上執行（Software-in-the-Loop）：同一份 app.cpp / proto.cpp，硬體換成模擬。
// 以 TCP 提供「序列埠」：Orin 端用 SerialMCU("socket://127.0.0.1:5760")，與接真 Arduino
// 走同一條 pyserial 程式路徑。每次有新連線就模擬 Uno 開埠被 DTR 重置：bootloader 期間（BOOT_DELAY_MS）
// 丟棄收到的資料，之後才 App::begin() → EV BOOT。
//
// 模擬控制（以 '!' 開頭、不經協定、不回覆）：
//   !ESTOP 1|0     按下/放開急停      !BATT 0.85   電池電壓係數（飛輪最高轉速 × 係數）
//   !JAM 1|0       推球卡彈           !RESET       重新開機
// 建置：python -m tools.mcu_sil --build（自動找 g++/clang++，沒有就用 pip 的 ziglang）
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <cmath>
#include <random>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
typedef SOCKET sock_t;
#define CLOSESOCK closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sock_t;
#define INVALID_SOCKET (-1)
#define CLOSESOCK close
#endif

#include "app.h"

class SimHal : public Hal {
 public:
  SimHal() : t0_(std::chrono::steady_clock::now()), rng_(12345) {}

  uint32_t millis() override { return (uint32_t)(elapsed_us() / 1000); }
  uint32_t micros() override { return (uint32_t)elapsed_us(); }
  void step(uint8_t, bool) override { ++steps_; }
  void enableSteppers(bool on) override { steppers_on_ = on; }
  void setFlywheelPwm(uint8_t pwm) override { fly_pwm_ = pwm; }
  int32_t flywheelRpm() override {
#if FLY_HAS_TACH
    std::normal_distribution<double> n(0.0, 0.004 * rpm_ + 1.0);   // 轉速計量測雜訊
    double v = rpm_ + n(rng_);
    return v < 0 ? 0 : (int32_t)v;
#else
    return -1;
#endif
  }
  void setFeeder(uint8_t pwm) override { feeder_on_ = pwm > 0; }
  bool feederHome() override { return phase_ < 0.12; }
  bool estopActive() override { return estop_; }
  void write(const char* s, size_t n) override { out.append(s, n); }
  float batteryVolts() override {
#if VBAT_ENABLED
    return (float)(12.0 * batt);                                    // 模擬 3S 電池（滿電 12.0V 起算）
#else
    return -1.0f;
#endif
  }

  void enableWheels(bool on) override { wheels_on_ = on; }
  void setWheel(uint8_t i, int16_t duty) override { wheel_duty_[i] = duty; }
  int32_t wheelCount(uint8_t i) override { return (int32_t)std::floor(wheel_counts_[i]); }
  uint16_t wheelRaw(uint8_t i) override { return (uint16_t)wheelCount(i); }
  uint8_t wheelPins() override {                                    // 正交訊號：計數每 +1 依序 00→10→11→01
    uint8_t m = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      int ph = ((wheelCount(i) % 4) + 4) % 4;
      if (ph == 1 || ph == 2) m |= (uint8_t)(1 << (2 * i));
      if (ph >= 2) m |= (uint8_t)(1 << (2 * i + 1));
    }
    return m;
  }

  // 物理推進（每圈呼叫）
  void tick() {
    double now = elapsed_us() * 1e-6;
    double dt = now - last_tick_;
    last_tick_ = now;
    if (dt <= 0) return;
    double target = fly_pwm_ / 255.0 * FLY_MAX_RPM * batt;          // 馬達穩態轉速
    rpm_ += (target - rpm_) * (1.0 - std::exp(-dt / 0.35));        // 一階延遲 τ=0.35 s
    if (feeder_on_ && !jam) phase_ = std::fmod(phase_ + dt / 0.35, 1.0);   // 0.35 s 轉一圈
    const double cpr = WHEEL_ENC_PPR * 4.0 * WHEEL_GEAR;            // 輸出軸每轉計數
    for (int i = 0; i < 4; ++i) {                                   // 車輪：輸出軸穩態 160 rpm（全速）、τ=0.15 s
      double w = wheels_on_ ? wheel_duty_[i] / 1000.0 * 160.0 * batt : 0.0;
      wheel_rpm_[i] += (w - wheel_rpm_[i]) * (1.0 - std::exp(-dt / 0.15));
      wheel_counts_[i] += wheel_rpm_[i] / 60.0 * cpr * dt;
    }
  }

  double wheel_rpm_[4] = {0, 0, 0, 0};

  std::string out;
  bool estop_ = false, jam = false;
  double batt = 1.0;
  unsigned long steps_ = 0;

 private:
  long long elapsed_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now() - t0_).count();
  }
  std::chrono::steady_clock::time_point t0_;
  std::mt19937 rng_;
  uint8_t fly_pwm_ = 0;
  bool steppers_on_ = false, feeder_on_ = false, wheels_on_ = false;
  double rpm_ = 0, phase_ = 0, last_tick_ = 0;
  int16_t wheel_duty_[4] = {0, 0, 0, 0};
  double wheel_counts_[4] = {0, 0, 0, 0};
};

static void handle_sim(SimHal& hal, App& app, const std::string& cmd) {
  char name[16] = {0};
  double v = 0;
  int n = sscanf(cmd.c_str(), "!%15s %lf", name, &v);
  if (n < 1) return;
  std::string s(name);
  if (s == "ESTOP") hal.estop_ = v != 0;
  else if (s == "JAM") hal.jam = v != 0;
  else if (s == "BATT" && n == 2) hal.batt = v;
  else if (s == "RESET") app.begin();
}

int main(int argc, char** argv) {
  int port = 5760;
  const char* host = "127.0.0.1";
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--host") && i + 1 < argc) host = argv[++i];
  }
#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { fprintf(stderr, "WSAStartup failed\n"); return 1; }
  timeBeginPeriod(1);                       // Windows 預設計時解析度 15.6 ms → 改 1 ms
#endif
  sock_t ls = socket(AF_INET, SOCK_STREAM, 0);
  int yes = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
  sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons((unsigned short)port);
  inet_pton(AF_INET, host, &addr.sin_addr);
  if (bind(ls, (sockaddr*)&addr, sizeof(addr)) != 0 || listen(ls, 1) != 0) {
    fprintf(stderr, "cannot listen on %s:%d\n", host, port);
    return 2;
  }
  socklen_t alen = sizeof(addr);                    // --port 0 → 系統配發，印出實際埠號
  getsockname(ls, (sockaddr*)&addr, &alen);
  port = ntohs(addr.sin_port);
  printf("SIL listening on %s:%d (proto v%u, fw %d, tach %d, home-switch %d, vbat %d)\n", host, port,
         (unsigned)proto::VERSION, FW_VERSION, FLY_HAS_TACH, FEED_HAS_HOME_SWITCH, VBAT_ENABLED);
  fflush(stdout);

  SimHal hal;
  App app(hal);
  app.begin();
  hal.out.clear();                          // 尚無連線：開機訊息丟棄
  sock_t cs = INVALID_SOCKET;
  std::string pending;                      // 組行（攔截 '!' 模擬指令）
  const uint32_t BOOT_DELAY_MS = 400;       // 模擬 bootloader（真 Uno 約 1.6 s；縮短以加快測試）
  bool booting = false;
  uint32_t boot_at = 0;

  for (;;) {
    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(ls, &rd);
    sock_t maxfd = ls;
    if (cs != INVALID_SOCKET) {
      FD_SET(cs, &rd);
      if (cs > maxfd) maxfd = cs;
    }
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    int r = select((int)maxfd + 1, &rd, nullptr, nullptr, &tv);
    if (r > 0 && FD_ISSET(ls, &rd)) {
      sock_t ns = accept(ls, nullptr, nullptr);
      if (ns != INVALID_SOCKET) {
        if (cs != INVALID_SOCKET) CLOSESOCK(cs);    // 一次只服務一個連線（同序列埠）
        cs = ns;
        setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, (const char*)&yes, sizeof(yes));
        pending.clear();
        hal.out.clear();
        booting = true;                             // 開埠重置：bootloader 期間不執行 App
        boot_at = hal.millis() + BOOT_DELAY_MS;
      }
    }
    if (booting && (int32_t)(hal.millis() - boot_at) >= 0) {
      booting = false;
      pending.clear();
      app.begin();                                  // → EV BOOT
    }
    if (r > 0 && cs != INVALID_SOCKET && FD_ISSET(cs, &rd)) {
      char buf[512];
      int n = recv(cs, buf, sizeof(buf), 0);
      if (n <= 0) {
        CLOSESOCK(cs);
        cs = INVALID_SOCKET;
      } else if (!booting) {                        // bootloader 期間收到的資料直接丟棄（同真 Uno）
        for (int i = 0; i < n; ++i) {
          char c = buf[i];
          if (c == '\n') {
            if (!pending.empty() && pending[0] == '!') {
              handle_sim(hal, app, pending);
            } else {
              for (char pc : pending) app.onByte(pc);
              app.onByte('\n');
            }
            pending.clear();
          } else if (pending.size() < 512) {
            pending.push_back(c);
          }
        }
      }
    }
    hal.tick();
    if (!booting) app.update();
    if (!hal.out.empty()) {
      if (cs != INVALID_SOCKET) send(cs, hal.out.data(), (int)hal.out.size(), 0);
      hal.out.clear();
    }
  }
}
