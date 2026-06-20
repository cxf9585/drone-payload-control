#include "payload_control.h"
#include "payload_mavlink_bridge.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint16_t camera_trigger_ms;
    bool gripper_closed;
    uint16_t sprayer_rate;
    bool safe_called;
    payload_gimbal_angles_t last_gimbal;
} fake_hw_t;

static payload_status_t fake_gimbal(void *user, payload_gimbal_angles_t angles)
{
    ((fake_hw_t *)user)->last_gimbal = angles;
    return PAYLOAD_OK;
}

static payload_status_t fake_camera_trigger(void *user, uint16_t duration_ms)
{
    ((fake_hw_t *)user)->camera_trigger_ms = duration_ms;
    return PAYLOAD_OK;
}

static payload_status_t fake_camera_record(void *user, bool enabled)
{
    (void)user;
    (void)enabled;
    return PAYLOAD_OK;
}

static payload_status_t fake_gripper(void *user, bool closed)
{
    ((fake_hw_t *)user)->gripper_closed = closed;
    return PAYLOAD_OK;
}

static payload_status_t fake_sprayer(void *user, uint16_t rate_permille)
{
    ((fake_hw_t *)user)->sprayer_rate = rate_permille;
    return PAYLOAD_OK;
}

static payload_status_t fake_safe(void *user)
{
    ((fake_hw_t *)user)->safe_called = true;
    return PAYLOAD_OK;
}

static payload_controller_t make_controller(fake_hw_t *hw)
{
    payload_controller_t controller;
    payload_driver_t driver;

    memset(&driver, 0, sizeof(driver));
    driver.user = hw;
    driver.set_gimbal_angles = fake_gimbal;
    driver.camera_trigger = fake_camera_trigger;
    driver.camera_record = fake_camera_record;
    driver.gripper_set = fake_gripper;
    driver.sprayer_set_rate = fake_sprayer;
    driver.all_outputs_safe = fake_safe;

    assert(payload_init(&controller, NULL, &driver, NULL, NULL) == PAYLOAD_OK);
    return controller;
}

static void rejects_payload_commands_until_armed(void)
{
    fake_hw_t hw = {0};
    payload_controller_t controller = make_controller(&hw);
    payload_command_t command = {0};

    command.type = PAYLOAD_CMD_CAMERA_TRIGGER;
    assert(payload_handle_command(&controller, &command) == PAYLOAD_ERR_NOT_ARMED);
    assert(hw.camera_trigger_ms == 0U);
}

static void accepts_camera_trigger_after_arm(void)
{
    fake_hw_t hw = {0};
    payload_controller_t controller = make_controller(&hw);
    payload_command_t command = {0};

    command.type = PAYLOAD_CMD_ARM;
    assert(payload_handle_command(&controller, &command) == PAYLOAD_OK);

    command.type = PAYLOAD_CMD_CAMERA_TRIGGER;
    command.data.camera_trigger.duration_ms = 0U;
    assert(payload_handle_command(&controller, &command) == PAYLOAD_OK);
    assert(hw.camera_trigger_ms == 200U);
}

static void enforces_gimbal_limits(void)
{
    fake_hw_t hw = {0};
    payload_controller_t controller = make_controller(&hw);
    payload_command_t command = {0};

    command.type = PAYLOAD_CMD_ARM;
    assert(payload_handle_command(&controller, &command) == PAYLOAD_OK);

    command.type = PAYLOAD_CMD_GIMBAL_SET_ANGLES;
    command.data.gimbal.pitch_deg = -120.0f;
    command.data.gimbal.yaw_deg = 0.0f;
    assert(payload_handle_command(&controller, &command) == PAYLOAD_ERR_LIMIT);
}

static void estop_disarms_and_safes_outputs(void)
{
    fake_hw_t hw = {0};
    payload_controller_t controller = make_controller(&hw);
    payload_command_t command = {0};
    payload_status_report_t report;

    command.type = PAYLOAD_CMD_ARM;
    assert(payload_handle_command(&controller, &command) == PAYLOAD_OK);

    command.type = PAYLOAD_CMD_ESTOP;
    assert(payload_handle_command(&controller, &command) == PAYLOAD_OK);

    payload_get_status(&controller, &report);
    assert(hw.safe_called);
    assert(!report.armed);
    assert(report.state == PAYLOAD_STATE_DISARMED);
}

static void maps_mavlink_gimbal_command(void)
{
    payload_mavlink_command_long_t mav_cmd = {0};
    payload_command_t command = {0};

    mav_cmd.command = 205U;
    mav_cmd.param1 = -35.0f;
    mav_cmd.param2 = 1.5f;
    mav_cmd.param3 = 90.0f;
    mav_cmd.sequence = 42U;

    assert(payload_command_from_mavlink_long(&mav_cmd, &command) == PAYLOAD_OK);
    assert(command.type == PAYLOAD_CMD_GIMBAL_SET_ANGLES);
    assert(command.sequence == 42U);
    assert(command.data.gimbal.pitch_deg == -35.0f);
    assert(command.data.gimbal.roll_deg == 1.5f);
    assert(command.data.gimbal.yaw_deg == 90.0f);
}

static void maps_status_to_mavlink_result(void)
{
    assert(payload_status_to_mavlink_result(PAYLOAD_OK) == PAYLOAD_MAVLINK_RESULT_ACCEPTED);
    assert(payload_status_to_mavlink_result(PAYLOAD_ERR_NOT_ARMED) == PAYLOAD_MAVLINK_RESULT_DENIED);
    assert(payload_status_to_mavlink_result(PAYLOAD_ERR_UNSUPPORTED) == PAYLOAD_MAVLINK_RESULT_UNSUPPORTED);
    assert(payload_status_to_mavlink_result(PAYLOAD_ERR_HARDWARE) == PAYLOAD_MAVLINK_RESULT_FAILED);
}

int main(void)
{
    rejects_payload_commands_until_armed();
    puts("[PASS] rejects_payload_commands_until_armed");
    accepts_camera_trigger_after_arm();
    puts("[PASS] accepts_camera_trigger_after_arm");
    enforces_gimbal_limits();
    puts("[PASS] enforces_gimbal_limits");
    estop_disarms_and_safes_outputs();
    puts("[PASS] estop_disarms_and_safes_outputs");
    maps_mavlink_gimbal_command();
    puts("[PASS] maps_mavlink_gimbal_command");
    maps_status_to_mavlink_result();
    puts("[PASS] maps_status_to_mavlink_result");
    puts("Payload control tests passed.");
    return 0;
}
