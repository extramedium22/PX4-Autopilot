#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <uORB/topics/trajectory_setpoint.h>

#include <px4_platform_common/log.h>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_outputs.h>

#include <px4_platform_common/module.h>

#include <drivers/drv_hrt.h>

__EXPORT int motor_test_main(int argc, char *argv[]);

int motor_test_main(int argc, char *argv[])
{
	PX4_INFO("Now begin motor testing");

	/*
	struct actuator_motors_s rotor_commands;

	//hrt_abstime _timestamp_sample;

	rotor_commands.timestamp = hrt_absolute_time();
	//rotor_commands.timestamp_sample = 1.0;
	//rotor_commands.reversible_flags = 1.0;

	rotor_commands.control[0] = 1.0;
	rotor_commands.control[1] = 1.0;
	rotor_commands.control[2] = 1.0;
	rotor_commands.control[3] = 1.0;
	rotor_commands.control[4] = 0.0;
	rotor_commands.control[5] = 0.0;
	rotor_commands.control[6] = 0.0;
	rotor_commands.control[7] = 0.0;
	rotor_commands.control[8] = 0.0;
	rotor_commands.control[9] = 0.0;
	rotor_commands.control[10] = 0.0;
	rotor_commands.control[11] = 0.0;

	orb_advert_t rotor_commands_pub = orb_advertise(ORB_ID(actuator_motors), &rotor_commands);

	struct actuator_servos_s servo_commands;
	servo_commands.timestamp = hrt_absolute_time();

	servo_commands.control[0] = 0.0;
	servo_commands.control[1] = 0.0;
	servo_commands.control[2] = 0.0;
	servo_commands.control[3] = 0.0;
	servo_commands.control[4] = 0.0;
	servo_commands.control[5] = 0.0;
	servo_commands.control[6] = 0.0;
	servo_commands.control[7] = 0.0;

	orb_advert_t servo_commands_pub = orb_advertise(ORB_ID(actuator_servos), &servo_commands);
	*/

	struct actuator_outputs_s actuator_outputs;
	actuator_outputs.noutputs = 16;
	actuator_outputs.output[0] = 1.0;
	actuator_outputs.output[1] = 1.0;
	actuator_outputs.output[2] = 1.0;
	actuator_outputs.output[3] = 1.0;
	actuator_outputs.output[4] = 1.0;
	actuator_outputs.output[5] = 1.0;
	actuator_outputs.output[6] = 1.0;
	actuator_outputs.output[7] = 1.0;
	actuator_outputs.output[8] = 1.0;
	actuator_outputs.output[9] = 1.0;
	actuator_outputs.output[10] = 1.0;
	actuator_outputs.output[11] = 1.0;
	actuator_outputs.output[12] = 1.0;
	actuator_outputs.output[13] = 1.0;
	actuator_outputs.output[14] = 1.0;
	actuator_outputs.output[15] = 1.0;


	//orb_advert_t actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs_sim), &actuator_outputs);
	orb_advert_t actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &actuator_outputs);

	while (true) {
		PX4_INFO("Motor Running!");
		//orb_publish(ORB_ID(actuator_motors), rotor_commands_pub, &rotor_commands);
		//orb_publish(ORB_ID(actuator_servos), servo_commands_pub, &servo_commands);
		//orb_publish(ORB_ID(actuator_outputs_sim), actuator_outputs_pub, &actuator_outputs);
		orb_publish(ORB_ID(actuator_outputs), actuator_outputs_pub, &actuator_outputs);
		px4_usleep(20000);
	}

	PX4_INFO("Bye!");
	return 0;
}
