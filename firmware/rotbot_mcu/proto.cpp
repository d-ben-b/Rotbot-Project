#include "proto.h"

#include <stdlib.h>
#include <string.h>

namespace proto {

uint8_t crc8(const char* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint8_t)data[i];
    for (uint8_t b = 0; b < 8; ++b) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

bool LineReader::feed(char c) {
  if (c == '\r') return false;
  if (c == '\n') {
    bool ready = !overflow_ && len_ > 0;
    if (overflow_) ++overflows_;
    buf_[len_] = '\0';
    len_ = 0;
    overflow_ = false;
    return ready;
  }
  if (len_ >= MAX_LINE) {          // 太長：丟掉整行，等下一個換行重新同步
    overflow_ = true;
    return false;
  }
  buf_[len_++] = c;
  return false;
}

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

ParseStatus split(char* line, char* fields[], uint8_t* n_fields, bool allow_no_crc) {
  *n_fields = 0;
  size_t len = strlen(line);
  if (len == 0) return PARSE_EMPTY;
  char* star = strrchr(line, '*');
  ParseStatus st = PARSE_OK;
  if (star) {
    int h = hexval(star[1]), l = hexval(star[2]);
    if (h < 0 || l < 0 || star[3] != '\0') return PARSE_BAD_CRC;
    if (crc8(line, (size_t)(star - line)) != (uint8_t)(h * 16 + l)) return PARSE_BAD_CRC;
    *star = '\0';
  } else if (allow_no_crc) {
    st = PARSE_NO_CRC;
  } else {
    return PARSE_BAD_CRC;
  }
  if (line[0] == '\0') return PARSE_EMPTY;
  char* p = line;
  while (*n_fields < MAX_FIELDS) {
    fields[(*n_fields)++] = p;
    char* comma = strchr(p, ',');
    if (!comma) break;
    *comma = '\0';
    p = comma + 1;
  }
  return st;
}

bool to_long(const char* s, long* out) {
  if (!s || !*s) return false;
  char* end = 0;
  long v = strtol(s, &end, 10);
  if (*end != '\0') return false;
  *out = v;
  return true;
}

static const char HEX_DIGITS[] = "0123456789ABCDEF";

size_t frame(char* out, size_t cap, const char* body) {
  size_t n = strlen(body);
  if (n + 5 > cap) return 0;        // body + '*' + 2 hex + '\n' + '\0'
  memcpy(out, body, n);
  uint8_t c = crc8(body, n);
  out[n] = '*';
  out[n + 1] = HEX_DIGITS[c >> 4];
  out[n + 2] = HEX_DIGITS[c & 0x0F];
  out[n + 3] = '\n';
  out[n + 4] = '\0';
  return n + 4;
}

}  // namespace proto
