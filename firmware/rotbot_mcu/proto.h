// Orin <-> MCU 序列協定 v1（ASCII 一行一幀 + CRC-8）——與 rotbot/actuation/protocol.py 一致。
// 幀：<BODY>*<CRC>\n，BODY = TYPE,SEQ[,ARG...]，CRC = CRC-8/SMBUS(poly 0x07, init 0) 兩位大寫十六進位。
// 純 C++（不依賴 Arduino），Uno 與電腦上的韌體模擬器（firmware/host）共用。
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace proto {

const uint8_t VERSION = 1;
const uint8_t MAX_LINE = 64;     // 含 *CC\n
const uint8_t MAX_FIELDS = 8;

// NAK 錯誤碼
enum : uint8_t {
  E_OK = 0, E_FORMAT = 1, E_UNKNOWN = 2, E_RANGE = 3,
  E_NOT_READY = 4, E_ESTOP = 5, E_BUSY = 6, E_NOT_HOMED = 7
};

// STAT 旗標
enum : uint8_t {
  F_READY = 0x01, F_MOVING = 0x02, F_FLY_OK = 0x04, F_FIRING = 0x08,
  F_ESTOP = 0x10, F_WDOG = 0x20, F_HOMED = 0x40, F_JOGGING = 0x80
};

uint8_t crc8(const char* data, size_t len);

// 逐字元組行；完整一行（不含 \r\n）就緒時 feed() 回傳 true，line() 取得內容。
class LineReader {
 public:
  bool feed(char c);
  char* line() { return buf_; }
  uint16_t overflows() const { return overflows_; }

 private:
  char buf_[MAX_LINE + 1];
  uint8_t len_ = 0;
  bool overflow_ = false;
  uint16_t overflows_ = 0;
};

// 解析結果
enum ParseStatus : uint8_t { PARSE_OK = 0, PARSE_BAD_CRC = 1, PARSE_NO_CRC = 2, PARSE_EMPTY = 3 };

// 就地驗 CRC 並以 ',' 切欄位；n_fields 回傳欄位數。
// allow_no_crc=true 時沒有 '*' 的行（手打）也接受，回傳 PARSE_NO_CRC。
ParseStatus split(char* line, char* fields[], uint8_t* n_fields, bool allow_no_crc);

// 字串 → long；整串都是合法整數才回傳 true。
bool to_long(const char* s, long* out);

// body 加上 *CC\n 寫入 out（cap 含結尾 '\0'）；回傳總長度，失敗回傳 0。
size_t frame(char* out, size_t cap, const char* body);

}  // namespace proto
