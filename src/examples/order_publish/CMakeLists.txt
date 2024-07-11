px4_add_module(
	MODULE examples__order_publish
	MAIN order_publish
	STACK_MAIN 2000
	SRCS
		order_publish.c
	DEPENDS
	)
