#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SCU_ENABLED
#define AP_SCU_ENABLED 1
#define AP_CANIO_ENABLE (HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024)
#endif
