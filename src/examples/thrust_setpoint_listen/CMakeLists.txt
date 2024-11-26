px4_add_module(
	MODULE examples__thrust_setpoint_listen
	MAIN thrust_setpoint_listen
	STACK_MAIN 2000
	SRCS
		thrust_setpoint_listen.c
	DEPENDS
	)
