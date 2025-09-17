#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_CMD_MAX_PAYLOAD 6

typedef struct {
    uint8_t mode;                 // 0x00-0x03 per spec
    uint8_t channel;              // 0-3
    uint8_t payload[CONTROL_CMD_MAX_PAYLOAD];
    uint8_t length;               // number of valid bytes in payload
} control_cmd_t;

#ifdef __cplusplus
}
#endif
