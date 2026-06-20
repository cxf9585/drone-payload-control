#include "payload_control.h"

#include <stddef.h>
#include <string.h>

static void emit_event(payload_controller_t *controller, payload_event_t event, payload_status_t status)
{
    if (controller->event_cb != NULL) {
        controller->event_cb(controller->event_user, event, status, &controller->report);
    }
}

static payload_status_t reject(payload_controller_t *controller, payload_status_t status)
{
    controller->report.last_error = status;
    emit_event(controller, PAYLOAD_EVENT_COMMAND_REJECTED, status);
    return status;
}

static bool supports(const payload_controller_t *controller, payload_command_type_t type)
{
    switch (type) {
    case PAYLOAD_CMD_GIMBAL_SET_ANGLES:
        return controller->driver.set_gimbal_angles != NULL;
    case PAYLOAD_CMD_CAMERA_TRIGGER:
        return controller->driver.camera_trigger != NULL;
    case PAYLOAD_CMD_CAMERA_RECORD_START:
    case PAYLOAD_CMD_CAMERA_RECORD_STOP:
        return controller->driver.camera_record != NULL;
    case PAYLOAD_CMD_GRIPPER_OPEN:
    case PAYLOAD_CMD_GRIPPER_CLOSE:
        return controller->driver.gripper_set != NULL;
    case PAYLOAD_CMD_SPRAYER_SET_RATE:
        return controller->driver.sprayer_set_rate != NULL;
    case PAYLOAD_CMD_ESTOP:
        return controller->driver.all_outputs_safe != NULL;
    default:
        return true;
    }
}

static payload_status_t require_armed(payload_controller_t *controller, payload_command_type_t type)
{
    if (type == PAYLOAD_CMD_ARM || type == PAYLOAD_CMD_DISARM || type == PAYLOAD_CMD_SET_MODE || type == PAYLOAD_CMD_ESTOP) {
        return PAYLOAD_OK;
    }

    if (!controller->report.armed) {
        return PAYLOAD_ERR_NOT_ARMED;
    }

    return PAYLOAD_OK;
}

static payload_status_t validate_limits(const payload_controller_t *controller, const payload_command_t *command)
{
    if (command->type == PAYLOAD_CMD_GIMBAL_SET_ANGLES) {
        const payload_gimbal_angles_t *gimbal = &command->data.gimbal;
        if (gimbal->pitch_deg < controller->config.gimbal_pitch_min_deg ||
            gimbal->pitch_deg > controller->config.gimbal_pitch_max_deg ||
            gimbal->yaw_deg < controller->config.gimbal_yaw_min_deg ||
            gimbal->yaw_deg > controller->config.gimbal_yaw_max_deg) {
            return PAYLOAD_ERR_LIMIT;
        }
    }

    if (command->type == PAYLOAD_CMD_SPRAYER_SET_RATE &&
        command->data.sprayer.rate_permille > controller->config.sprayer_rate_max_permille) {
        return PAYLOAD_ERR_LIMIT;
    }

    return PAYLOAD_OK;
}

static payload_status_t run_driver_command(payload_controller_t *controller, const payload_command_t *command)
{
    switch (command->type) {
    case PAYLOAD_CMD_GIMBAL_SET_ANGLES:
        return controller->driver.set_gimbal_angles(controller->driver.user, command->data.gimbal);
    case PAYLOAD_CMD_CAMERA_TRIGGER: {
        uint16_t duration = command->data.camera_trigger.duration_ms;
        if (duration == 0U) {
            duration = controller->config.camera_trigger_ms_default;
        }
        return controller->driver.camera_trigger(controller->driver.user, duration);
    }
    case PAYLOAD_CMD_CAMERA_RECORD_START:
        return controller->driver.camera_record(controller->driver.user, true);
    case PAYLOAD_CMD_CAMERA_RECORD_STOP:
        return controller->driver.camera_record(controller->driver.user, false);
    case PAYLOAD_CMD_GRIPPER_OPEN:
        return controller->driver.gripper_set(controller->driver.user, false);
    case PAYLOAD_CMD_GRIPPER_CLOSE:
        return controller->driver.gripper_set(controller->driver.user, true);
    case PAYLOAD_CMD_SPRAYER_SET_RATE:
        return controller->driver.sprayer_set_rate(controller->driver.user, command->data.sprayer.rate_permille);
    case PAYLOAD_CMD_ESTOP:
        return controller->driver.all_outputs_safe(controller->driver.user);
    default:
        return PAYLOAD_OK;
    }
}

payload_config_t payload_default_config(void)
{
    payload_config_t config;
    config.gimbal_pitch_min_deg = -90.0f;
    config.gimbal_pitch_max_deg = 30.0f;
    config.gimbal_yaw_min_deg = -180.0f;
    config.gimbal_yaw_max_deg = 180.0f;
    config.camera_trigger_ms_default = 200U;
    config.sprayer_rate_max_permille = 1000U;
    config.command_timeout_ms = 1000U;
    return config;
}

payload_status_t payload_init(
    payload_controller_t *controller,
    const payload_config_t *config,
    const payload_driver_t *driver,
    payload_event_cb_t event_cb,
    void *event_user)
{
    if (controller == NULL || driver == NULL) {
        return PAYLOAD_ERR_INVALID_ARG;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = (config != NULL) ? *config : payload_default_config();
    controller->driver = *driver;
    controller->event_cb = event_cb;
    controller->event_user = event_user;
    controller->report.state = PAYLOAD_STATE_DISARMED;
    controller->report.mode = PAYLOAD_MODE_MANUAL;
    controller->report.armed = false;
    controller->report.last_error = PAYLOAD_OK;
    return PAYLOAD_OK;
}

payload_status_t payload_handle_command(payload_controller_t *controller, const payload_command_t *command)
{
    payload_status_t status;

    if (controller == NULL || command == NULL) {
        return PAYLOAD_ERR_INVALID_ARG;
    }

    if (!supports(controller, command->type)) {
        return reject(controller, PAYLOAD_ERR_UNSUPPORTED);
    }

    status = require_armed(controller, command->type);
    if (status != PAYLOAD_OK) {
        return reject(controller, status);
    }

    status = validate_limits(controller, command);
    if (status != PAYLOAD_OK) {
        return reject(controller, status);
    }

    controller->report.last_sequence = command->sequence;

    switch (command->type) {
    case PAYLOAD_CMD_ARM:
        controller->report.armed = true;
        controller->report.state = PAYLOAD_STATE_IDLE;
        controller->report.last_error = PAYLOAD_OK;
        emit_event(controller, PAYLOAD_EVENT_ARMED, PAYLOAD_OK);
        return PAYLOAD_OK;
    case PAYLOAD_CMD_DISARM:
        if (controller->driver.all_outputs_safe != NULL) {
            status = controller->driver.all_outputs_safe(controller->driver.user);
            if (status != PAYLOAD_OK) {
                controller->report.state = PAYLOAD_STATE_ERROR;
                controller->report.last_error = status;
                emit_event(controller, PAYLOAD_EVENT_HARDWARE_ERROR, status);
                return status;
            }
        }
        controller->report.armed = false;
        controller->report.state = PAYLOAD_STATE_DISARMED;
        controller->report.last_error = PAYLOAD_OK;
        emit_event(controller, PAYLOAD_EVENT_DISARMED, PAYLOAD_OK);
        return PAYLOAD_OK;
    case PAYLOAD_CMD_SET_MODE:
        controller->report.mode = command->data.mode;
        controller->report.last_error = PAYLOAD_OK;
        emit_event(controller, PAYLOAD_EVENT_COMMAND_ACCEPTED, PAYLOAD_OK);
        return PAYLOAD_OK;
    case PAYLOAD_CMD_ESTOP:
        status = run_driver_command(controller, command);
        controller->report.armed = false;
        controller->report.state = (status == PAYLOAD_OK) ? PAYLOAD_STATE_DISARMED : PAYLOAD_STATE_ERROR;
        controller->report.last_error = status;
        emit_event(controller, PAYLOAD_EVENT_ESTOP, status);
        return status;
    default:
        controller->report.state = PAYLOAD_STATE_BUSY;
        status = run_driver_command(controller, command);
        controller->report.state = (status == PAYLOAD_OK) ? PAYLOAD_STATE_IDLE : PAYLOAD_STATE_ERROR;
        controller->report.last_error = status;
        emit_event(
            controller,
            (status == PAYLOAD_OK) ? PAYLOAD_EVENT_COMMAND_ACCEPTED : PAYLOAD_EVENT_HARDWARE_ERROR,
            status);
        return status;
    }
}

void payload_get_status(const payload_controller_t *controller, payload_status_report_t *out_report)
{
    if (controller == NULL || out_report == NULL) {
        return;
    }

    *out_report = controller->report;
}
