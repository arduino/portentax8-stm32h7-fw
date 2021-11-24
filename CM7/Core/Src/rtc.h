#include <inttypes.h>

#ifndef RTC_H
#define RTC_H

struct rtc_time {
  uint8_t tm_sec;
  uint8_t tm_min;
  uint8_t tm_hour;
  uint8_t tm_mday;
  uint8_t tm_mon;
  uint8_t tm_year;
  uint8_t tm_wday;
};

void rtc_init();

void handle_rtc_operation(uint8_t opcode, struct rtc_time *tm);

#endif //RTC_H