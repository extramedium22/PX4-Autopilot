#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <sched.h>

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
#include <uORB/topics/actuator_test.h>

#include <px4_platform_common/module.h>

#include <drivers/drv_hrt.h>

static int daemon_task;

extern "C" __EXPORT int motor_action(int argc, char *argv[]);

extern "C" __EXPORT int motor_test_main(int argc, char *argv[]);
int motor_test_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: motor_test {start}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		daemon_task = px4_task_spawn_cmd("motor_test",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT + 40,
						 PX4_STACK_ADJUSTED(3250),
						 motor_action,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}


	PX4_WARN("usage: motor_test {start}\n");
	return 1;
}


int motor_action(int argc, char *argv[])
{
	//PX4_INFO("Now begin motor testing");

	
	struct actuator_motors_s rotor_commands;

	//hrt_abstime _timestamp_sample{0};

	rotor_commands.timestamp = hrt_absolute_time();
	rotor_commands.timestamp_sample = hrt_absolute_time();
	rotor_commands.reversible_flags = 0;

	rotor_commands.control[0] = 0.0;
	rotor_commands.control[1] = 0.5;
	rotor_commands.control[2] = 0.0;
	rotor_commands.control[3] = 0.0;
	rotor_commands.control[4] = 0.0;
	rotor_commands.control[5] = 0.0;
	rotor_commands.control[6] = 0.0;
	rotor_commands.control[7] = 0.0;
	rotor_commands.control[8] = 0.0;
	rotor_commands.control[9] = 0.0;
	rotor_commands.control[10] = 0.0;
	rotor_commands.control[11] = 0.0;

	orb_advert_t rotor_commands_pub = orb_advertise(ORB_ID(actuator_motors), &rotor_commands);
	



	/*
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


	/*
	struct actuator_outputs_s actuator_outputs_commands;
	actuator_outputs_commands.noutputs = 16;
	actuator_outputs_commands.output[0] = 1500.0;
	actuator_outputs_commands.output[1] = 0.0;
	actuator_outputs_commands.output[2] = 0.0;
	actuator_outputs_commands.output[3] = 0.0;
	actuator_outputs_commands.output[4] = 0.0;
	actuator_outputs_commands.output[5] = 0.0;
	actuator_outputs_commands.output[6] = 0.0;
	actuator_outputs_commands.output[7] = 0.0;
	actuator_outputs_commands.output[8] = 0.0;
	actuator_outputs_commands.output[9] = 0.0;
	actuator_outputs_commands.output[10] = 0.0;
	actuator_outputs_commands.output[11] = 0.0;
	actuator_outputs_commands.output[12] = 0.0;
	actuator_outputs_commands.output[13] = 0.0;
	actuator_outputs_commands.output[14] = 0.0;
	actuator_outputs_commands.output[15] = 0.0;
	*/

	//orb_advert_t actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs_sim), &actuator_outputs);
	//orb_advert_t actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &actuator_outputs);



	/*
	struct actuator_test_s test_commands;

	test_commands.value = 0.5;

	orb_advert_t test_commands_pub = orb_advertise(ORB_ID(actuator_test), &test_commands);
	*/

	while (true) {
		//PX4_INFO("Motor Running!");
		orb_publish(ORB_ID(actuator_motors), rotor_commands_pub, &rotor_commands);
		//orb_publish(ORB_ID(actuator_servos), servo_commands_pub, &servo_commands);
		//orb_publish(ORB_ID(actuator_outputs_sim), actuator_outputs_pub, &actuator_outputs);
		//orb_publish(ORB_ID(actuator_outputs), actuator_outputs_pub, &actuator_outputs_commands);
		//orb_publish(ORB_ID(actuator_test), test_commands_pub, &test_commands);
		px4_usleep(1000);
	}

	//PX4_INFO("Bye!");
	return 0;
}
