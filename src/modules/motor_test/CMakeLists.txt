px4_add_module(
	MODULE modules__motor_test
	MAIN motor_test
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	STACK_MAIN
		3000
	SRCS
		motor_test.cpp
	DEPENDS
	)
