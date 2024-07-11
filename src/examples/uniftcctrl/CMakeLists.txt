px4_add_module(
	MODULE examples__uniftcctrl
	MAIN uniftcctrl
	STACK_MAIN 2000
	SRCS
		uniftcctrl.c
	DEPENDS
	)
