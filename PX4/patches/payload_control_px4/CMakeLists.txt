px4_add_module(
	MODULE modules__payload_control_px4
	MAIN payload_control_px4
	SRCS
		payload_control_px4.c
		src/payload_control.c
	INCLUDES
		include
	)
