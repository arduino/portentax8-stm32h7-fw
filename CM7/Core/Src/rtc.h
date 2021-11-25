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

//void handle_rtc_operation(uint8_t opcode, uint8_t *data, uint8_t size);

void rtc_set_date(uint8_t *data);

void rtc_get_date(uint8_t *data);

#endif //RTC_H