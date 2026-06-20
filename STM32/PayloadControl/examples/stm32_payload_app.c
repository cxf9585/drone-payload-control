#include "payload_control.h"
#include "payload_mavlink_bridge.h"

typedef struct {
    int placeholder;
} board_payload_io_t;

static payload_status_t board_set_gimbal_angles(void *user, payload_gimbal_angles_t angles)
{
    (void)user;
    (void)angles;
    /* TODO: convert angles to PWM/CAN/UART command for the selected gimbal. */
    return PAYLOAD_OK;
}

static payload_status_t board_camera_trigger(void *user, uint16_t duration_ms)
{
    (void)user;
    (void)duration_ms;
    /* TODO: drive GPIO optocoupler, UART command, or camera SDK trigger. */
    return PAYLOAD_OK;
}

static payload_status_t board_camera_record(void *user, bool enabled)
{
    (void)user;
    (void)enabled;
    return PAYLOAD_OK;
}

static payload_status_t board_gripper_set(void *user, bool closed)
{
    (void)user;
    (void)closed;
    return PAYLOAD_OK;
}

static payload_status_t board_sprayer_set_rate(void *user, uint16_t rate_permille)
{
    (void)user;
    (void)rate_permille;
    return PAYLOAD_OK;
}

static payload_status_t board_all_outputs_safe(void *user)
{
    (void)user;
    /* TODO: stop PWM, stop pump, release camera trigger GPIO, and disable actuators. */
    return PAYLOAD_OK;
}

static void payload_event_logger(
    void *user,
    payload_event_t event,
    payload_status_t status,
    const payload_status_report_t *report)
{
    (void)user;
    (void)event;
    (void)status;
    (void)report;
    /* TODO: publish status by MAVLink STATUSTEXT, UART log, or flight stack telemetry. */
}

void payload_app_init(payload_controller_t *controller, board_payload_io_t *io)
{
    payload_driver_t driver = {
        .user = io,
        .set_gimbal_angles = board_set_gimbal_angles,
        .camera_trigger = board_camera_trigger,
        .camera_record = board_camera_record,
        .gripper_set = board_gripper_set,
        .sprayer_set_rate = board_sprayer_set_rate,
        .all_outputs_safe = board_all_outputs_safe,
    };

    payload_config_t config = payload_default_config();
    (void)payload_init(controller, &config, &driver, payload_event_logger, NULL);
}

payload_mavlink_result_t payload_app_handle_mavlink_command(
    payload_controller_t *controller,
    const payload_mavlink_command_long_t *mav_cmd)
{
    payload_command_t command;
    payload_status_t status = payload_command_from_mavlink_long(mav_cmd, &command);
    if (status == PAYLOAD_OK) {
        status = payload_handle_command(controller, &command);
    }
    return payload_status_to_mavlink_result(status);
}
