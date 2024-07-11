px4_add_module(
	MODULE examples__px4_first_move
	MAIN px4_first_move
	STACK_MAIN 2000
	SRCS
		px4_first_move.c
	DEPENDS
	)
