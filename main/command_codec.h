#pragma once

#include <stddef.h>
#include "esp_err.h"

#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t control_cmd_parse(const uint8_t *buf, size_t len, size_t *consumed, control_cmd_t *out_cmd);

esp_err_t control_cmd_validate(const control_cmd_t *cmd);

size_t control_cmd_serialized_size(const control_cmd_t *cmd);

#ifdef __cplusplus
}
#endif

