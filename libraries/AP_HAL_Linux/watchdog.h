#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
  setup the watchdog
 */
void linux_watchdog_init(void);

/*
  pat the dog, to prevent a reset. If not called for 1s
  after stm32_watchdog_init() then MCU will reset
 */
void linux_watchdog_pat(void);

void linux_watchdog_stop(void);
    
#ifdef __cplusplus
}
#endif
    
