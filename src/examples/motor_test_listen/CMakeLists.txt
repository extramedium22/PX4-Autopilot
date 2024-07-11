px4_add_module(
	MODULE examples__motor_test_listen
	MAIN motor_test_listen
	STACK_MAIN 2000
	SRCS
		motor_test_listen.c
	DEPENDS
	)
