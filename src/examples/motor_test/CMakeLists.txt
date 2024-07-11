px4_add_module(
	MODULE examples__motor_test
	MAIN motor_test
	STACK_MAIN 2000
	SRCS
		motor_test.c
	DEPENDS
	)
