#ifndef PAYLOAD_MAVLINK_BRIDGE_H
#define PAYLOAD_MAVLINK_BRIDGE_H

#include "payload_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PAYLOAD_MAVLINK_RESULT_ACCEPTED = 0,
    PAYLOAD_MAVLINK_RESULT_TEMPORARILY_REJECTED = 1,
    PAYLOAD_MAVLINK_RESULT_DENIED = 2,
    PAYLOAD_MAVLINK_RESULT_UNSUPPORTED = 3,
    PAYLOAD_MAVLINK_RESULT_FAILED = 4
} payload_mavlink_result_t;

typedef struct {
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
    uint32_t sequence;
} payload_mavlink_command_long_t;

payload_status_t payload_command_from_mavlink_long(
    const payload_mavlink_command_long_t *mav_cmd,
    payload_command_t *out_command);

payload_mavlink_result_t payload_status_to_mavlink_result(payload_status_t status);

#ifdef __cplusplus
}
#endif

#endif
