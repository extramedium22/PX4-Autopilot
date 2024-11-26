px4_add_module(
	MODULE examples__motor_eff_ctrl
	MAIN motor_eff_ctrl
	STACK_MAIN 2000
	SRCS
		motor_eff_ctrl.cpp
	DEPENDS
	)
