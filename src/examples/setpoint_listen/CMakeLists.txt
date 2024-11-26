px4_add_module(
	MODULE examples__setpoint_listen
	MAIN setpoint_listen
	STACK_MAIN 2000
	SRCS
		setpoint_listen.c
	DEPENDS
	)
