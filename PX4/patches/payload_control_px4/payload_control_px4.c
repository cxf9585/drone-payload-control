/****************************************************************************
 *
 * Payload control command module for PX4.
 *
 ****************************************************************************/

#include "payload_control.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

__EXPORT int payload_control_px4_main(int argc, char *argv[]);

static payload_controller_t g_controller;
static bool g_initialized;

static payload_status_t px4_set_gimbal_angles(void *user, payload_gimbal_angles_t angles)
{
	(void)user;
	PX4_INFO("gimbal pitch=%.2f yaw=%.2f roll=%.2f",
		 (double)angles.pitch_deg,
		 (double)angles.yaw_deg,
		 (double)angles.roll_deg);
	return PAYLOAD_OK;
}

static payload_status_t px4_camera_trigger(void *user, uint16_t duration_ms)
{
	(void)user;
	PX4_INFO("camera trigger %u ms", (unsigned)duration_ms);
	return PAYLOAD_OK;
}

static payload_status_t px4_camera_record(void *user, bool enabled)
{
	(void)user;
	PX4_INFO("camera record %s", enabled ? "start" : "stop");
	return PAYLOAD_OK;
}

static payload_status_t px4_gripper_set(void *user, bool closed)
{
	(void)user;
	PX4_INFO("gripper %s", closed ? "close" : "open");
	return PAYLOAD_OK;
}

static payload_status_t px4_sprayer_set_rate(void *user, uint16_t rate_permille)
{
	(void)user;
	PX4_INFO("sprayer rate %u permille", (unsigned)rate_permille);
	return PAYLOAD_OK;
}

static payload_status_t px4_all_outputs_safe(void *user)
{
	(void)user;
	PX4_WARN("payload outputs safe");
	return PAYLOAD_OK;
}

static void px4_payload_event(
	void *user,
	payload_event_t event,
	payload_status_t status,
	const payload_status_report_t *report)
{
	(void)user;
	PX4_INFO("event=%d status=%d state=%d armed=%d",
		 (int)event,
		 (int)status,
		 (int)report->state,
		 (int)report->armed);
}

static void ensure_initialized(void)
{
	if (g_initialized) {
		return;
	}

	payload_driver_t driver = {
		.user = NULL,
		.set_gimbal_angles = px4_set_gimbal_angles,
		.camera_trigger = px4_camera_trigger,
		.camera_record = px4_camera_record,
		.gripper_set = px4_gripper_set,
		.sprayer_set_rate = px4_sprayer_set_rate,
		.all_outputs_safe = px4_all_outputs_safe,
	};

	payload_config_t config = payload_default_config();
	payload_status_t status = payload_init(&g_controller, &config, &driver, px4_payload_event, NULL);

	if (status == PAYLOAD_OK) {
		g_initialized = true;
	}
}

static int send_simple_command(payload_command_type_t type)
{
	payload_command_t command = {
		.type = type,
		.sequence = 0U,
	};

	payload_status_t status = payload_handle_command(&g_controller, &command);
	return status == PAYLOAD_OK ? 0 : -1;
}

static int selftest(void)
{
	int result = 0;

	result |= send_simple_command(PAYLOAD_CMD_ARM);

	payload_command_t trigger = {
		.type = PAYLOAD_CMD_CAMERA_TRIGGER,
		.sequence = 1U,
	};

	result |= payload_handle_command(&g_controller, &trigger) == PAYLOAD_OK ? 0 : -1;
	result |= send_simple_command(PAYLOAD_CMD_ESTOP);

	if (result == 0) {
		PX4_INFO("payload_control_px4 selftest passed");
	}

	return result;
}

static void print_status(void)
{
	payload_status_report_t report;
	payload_get_status(&g_controller, &report);
	PX4_INFO("state=%d mode=%d armed=%d last_sequence=%lu last_error=%d",
		 (int)report.state,
		 (int)report.mode,
		 (int)report.armed,
		 (unsigned long)report.last_sequence,
		 (int)report.last_error);
}

static void print_usage(void)
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
UAV payload control shell command for CUAV/PX4 firmware integration.

### Examples
payload_control_px4 status
payload_control_px4 arm
payload_control_px4 disarm
payload_control_px4 estop
payload_control_px4 selftest
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("payload_control_px4", "module");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("arm");
	PRINT_MODULE_USAGE_COMMAND("disarm");
	PRINT_MODULE_USAGE_COMMAND("estop");
	PRINT_MODULE_USAGE_COMMAND("selftest");
}

int payload_control_px4_main(int argc, char *argv[])
{
	ensure_initialized();

	if (!g_initialized) {
		PX4_ERR("init failed");
		return -1;
	}

	if (argc < 2) {
		print_usage();
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		print_status();
		return 0;
	}

	if (strcmp(argv[1], "arm") == 0) {
		return send_simple_command(PAYLOAD_CMD_ARM);
	}

	if (strcmp(argv[1], "disarm") == 0) {
		return send_simple_command(PAYLOAD_CMD_DISARM);
	}

	if (strcmp(argv[1], "estop") == 0) {
		return send_simple_command(PAYLOAD_CMD_ESTOP);
	}

	if (strcmp(argv[1], "selftest") == 0) {
		return selftest();
	}

	print_usage();
	return -1;
}
