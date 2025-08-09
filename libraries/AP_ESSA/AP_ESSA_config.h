
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_ESSA_ENABLED
#define AP_ESSA_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif
