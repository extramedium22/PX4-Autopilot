px4_add_module(
	MODULE examples__motor_efficiency_publish
	MAIN motor_efficiency_publish
	STACK_MAIN 2000
	SRCS
		motor_efficiency_publish.c
	DEPENDS
	)
