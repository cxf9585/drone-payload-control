#include "payload_mavlink_bridge.h"

#include <stddef.h>

enum {
    MAV_CMD_DO_SET_MODE = 176,
    MAV_CMD_DO_SET_SERVO = 183,
    MAV_CMD_DO_DIGICAM_CONTROL = 203,
    MAV_CMD_DO_MOUNT_CONTROL = 205,
    MAV_CMD_VIDEO_START_CAPTURE = 2500,
    MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
    MAV_CMD_COMPONENT_ARM_DISARM = 400,
    MAV_CMD_DO_GRIPPER = 211,
    MAV_CMD_USER_1 = 31000
};

payload_status_t payload_command_from_mavlink_long(
    const payload_mavlink_command_long_t *mav_cmd,
    payload_command_t *out_command)
{
    if (mav_cmd == NULL || out_command == NULL) {
        return PAYLOAD_ERR_INVALID_ARG;
    }

    out_command->sequence = mav_cmd->sequence;

    switch (mav_cmd->command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        out_command->type = (mav_cmd->param1 > 0.5f) ? PAYLOAD_CMD_ARM : PAYLOAD_CMD_DISARM;
        return PAYLOAD_OK;
    case MAV_CMD_DO_SET_MODE:
        out_command->type = PAYLOAD_CMD_SET_MODE;
        out_command->data.mode = (payload_mode_t)((uint32_t)mav_cmd->param1);
        return PAYLOAD_OK;
    case MAV_CMD_DO_MOUNT_CONTROL:
        out_command->type = PAYLOAD_CMD_GIMBAL_SET_ANGLES;
        out_command->data.gimbal.pitch_deg = mav_cmd->param1;
        out_command->data.gimbal.roll_deg = mav_cmd->param2;
        out_command->data.gimbal.yaw_deg = mav_cmd->param3;
        return PAYLOAD_OK;
    case MAV_CMD_DO_DIGICAM_CONTROL:
        out_command->type = PAYLOAD_CMD_CAMERA_TRIGGER;
        out_command->data.camera_trigger.duration_ms = (uint16_t)mav_cmd->param5;
        return PAYLOAD_OK;
    case MAV_CMD_VIDEO_START_CAPTURE:
        out_command->type = PAYLOAD_CMD_CAMERA_RECORD_START;
        return PAYLOAD_OK;
    case MAV_CMD_VIDEO_STOP_CAPTURE:
        out_command->type = PAYLOAD_CMD_CAMERA_RECORD_STOP;
        return PAYLOAD_OK;
    case MAV_CMD_DO_GRIPPER:
        out_command->type = (mav_cmd->param2 > 0.5f) ? PAYLOAD_CMD_GRIPPER_CLOSE : PAYLOAD_CMD_GRIPPER_OPEN;
        return PAYLOAD_OK;
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_USER_1:
        out_command->type = PAYLOAD_CMD_SPRAYER_SET_RATE;
        out_command->data.sprayer.rate_permille = (uint16_t)mav_cmd->param2;
        return PAYLOAD_OK;
    default:
        return PAYLOAD_ERR_UNSUPPORTED;
    }
}

payload_mavlink_result_t payload_status_to_mavlink_result(payload_status_t status)
{
    switch (status) {
    case PAYLOAD_OK:
        return PAYLOAD_MAVLINK_RESULT_ACCEPTED;
    case PAYLOAD_ERR_BUSY:
        return PAYLOAD_MAVLINK_RESULT_TEMPORARILY_REJECTED;
    case PAYLOAD_ERR_NOT_ARMED:
    case PAYLOAD_ERR_LIMIT:
        return PAYLOAD_MAVLINK_RESULT_DENIED;
    case PAYLOAD_ERR_UNSUPPORTED:
        return PAYLOAD_MAVLINK_RESULT_UNSUPPORTED;
    default:
        return PAYLOAD_MAVLINK_RESULT_FAILED;
    }
}
