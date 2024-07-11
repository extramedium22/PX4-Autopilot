px4_add_module(
	MODULE examples__posatt_listen
	MAIN posatt_listen
	STACK_MAIN 2000
	SRCS
		posatt_listen.c
	DEPENDS
	)
