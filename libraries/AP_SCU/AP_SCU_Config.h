#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SCU_ENABLED
#define AP_SCU_ENABLED 1
#endif

#ifndef AP_CANIO_ENABLE
#define AP_CANIO_ENABLE (HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

