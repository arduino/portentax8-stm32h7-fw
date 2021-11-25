#include <inttypes.h>

#ifndef WATCHDOG_H
#define WATCHDOG_H

void watchdog_init(int prescaler);

void watchdog_refresh();

#endif //WATCHDOG_H