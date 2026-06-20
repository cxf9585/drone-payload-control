#ifndef PAYLOAD_CONTROL_H
#define PAYLOAD_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PAYLOAD_OK = 0,
    PAYLOAD_ERR_INVALID_ARG = -1,
    PAYLOAD_ERR_NOT_ARMED = -2,
    PAYLOAD_ERR_BUSY = -3,
    PAYLOAD_ERR_UNSUPPORTED = -4,
    PAYLOAD_ERR_HARDWARE = -5,
    PAYLOAD_ERR_LIMIT = -6
} payload_status_t;

typedef enum {
    PAYLOAD_STATE_DISARMED = 0,
    PAYLOAD_STATE_IDLE,
    PAYLOAD_STATE_BUSY,
    PAYLOAD_STATE_ERROR
} payload_state_t;

typedef enum {
    PAYLOAD_MODE_MANUAL = 0,
    PAYLOAD_MODE_MISSION,
    PAYLOAD_MODE_FAILSAFE
} payload_mode_t;

typedef enum {
    PAYLOAD_EVENT_ARMED = 0,
    PAYLOAD_EVENT_DISARMED,
    PAYLOAD_EVENT_COMMAND_ACCEPTED,
    PAYLOAD_EVENT_COMMAND_REJECTED,
    PAYLOAD_EVENT_HARDWARE_ERROR,
    PAYLOAD_EVENT_ESTOP
} payload_event_t;

typedef enum {
    PAYLOAD_CMD_ARM = 0,
    PAYLOAD_CMD_DISARM,
    PAYLOAD_CMD_SET_MODE,
    PAYLOAD_CMD_GIMBAL_SET_ANGLES,
    PAYLOAD_CMD_CAMERA_TRIGGER,
    PAYLOAD_CMD_CAMERA_RECORD_START,
    PAYLOAD_CMD_CAMERA_RECORD_STOP,
    PAYLOAD_CMD_GRIPPER_OPEN,
    PAYLOAD_CMD_GRIPPER_CLOSE,
    PAYLOAD_CMD_SPRAYER_SET_RATE,
    PAYLOAD_CMD_ESTOP
} payload_command_type_t;

typedef struct {
    float pitch_deg;
    float yaw_deg;
    float roll_deg;
} payload_gimbal_angles_t;

typedef struct {
    payload_command_type_t type;
    uint32_t sequence;
    union {
        payload_mode_t mode;
        payload_gimbal_angles_t gimbal;
        struct {
            uint16_t duration_ms;
        } camera_trigger;
        struct {
            uint16_t rate_permille;
        } sprayer;
    } data;
} payload_command_t;

typedef struct {
    float gimbal_pitch_min_deg;
    float gimbal_pitch_max_deg;
    float gimbal_yaw_min_deg;
    float gimbal_yaw_max_deg;
    uint16_t camera_trigger_ms_default;
    uint16_t sprayer_rate_max_permille;
    uint32_t command_timeout_ms;
} payload_config_t;

typedef struct {
    payload_state_t state;
    payload_mode_t mode;
    bool armed;
    uint32_t last_sequence;
    payload_status_t last_error;
} payload_status_report_t;

typedef struct {
    void *user;
    payload_status_t (*set_gimbal_angles)(void *user, payload_gimbal_angles_t angles);
    payload_status_t (*camera_trigger)(void *user, uint16_t duration_ms);
    payload_status_t (*camera_record)(void *user, bool enabled);
    payload_status_t (*gripper_set)(void *user, bool closed);
    payload_status_t (*sprayer_set_rate)(void *user, uint16_t rate_permille);
    payload_status_t (*all_outputs_safe)(void *user);
} payload_driver_t;

typedef void (*payload_event_cb_t)(
    void *user,
    payload_event_t event,
    payload_status_t status,
    const payload_status_report_t *report);

typedef struct {
    payload_config_t config;
    payload_driver_t driver;
    payload_event_cb_t event_cb;
    void *event_user;
    payload_status_report_t report;
} payload_controller_t;

payload_config_t payload_default_config(void);

payload_status_t payload_init(
    payload_controller_t *controller,
    const payload_config_t *config,
    const payload_driver_t *driver,
    payload_event_cb_t event_cb,
    void *event_user);

payload_status_t payload_handle_command(
    payload_controller_t *controller,
    const payload_command_t *command);

void payload_get_status(
    const payload_controller_t *controller,
    payload_status_report_t *out_report);

#ifdef __cplusplus
}
#endif

#endif
