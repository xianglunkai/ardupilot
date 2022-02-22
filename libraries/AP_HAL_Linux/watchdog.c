#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <linux/watchdog.h>
#include "watchdog.h"
#include <stdbool.h>

static bool watchdog_enabled = false;
static int fptr;
static int reset_time_seconds = 10;

/*
  setup the watchdog
 */
void linux_watchdog_init(void)
{
    // setup for 10s reset
    watchdog_enabled = true;
    fptr = open("/dev/watchdog",O_WRONLY);
    ioctl(fptr,WDIOC_SETTIMEOUT,&reset_time_seconds);

}

/*
  pat the dog, to prevent a reset. If not called for 1s
  after watchdog_init() then MCU will reset
 */
void linux_watchdog_pat(void)
{
  if(watchdog_enabled == true){
    ioctl(fptr,WDIOC_KEEPALIVE,NULL);
  }
}

void linux_watchdog_stop(void)
{
  int i_dis = WDIOS_DISABLECARD;
	ioctl(fptr,WDIOC_SETOPTIONS, &i_dis);
  close(fptr);
}
