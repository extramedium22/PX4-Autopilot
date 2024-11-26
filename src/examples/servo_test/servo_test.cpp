#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

#include <uORB/topics/vehicle_local_position_target.h>

#include <uORB/topics/trajectory_setpoint.h>

#include <uORB/topics/actuator_motors_efficiency.h>

#include <px4_platform_common/log.h>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>

#include <px4_platform_common/module.h>

#include <drivers/drv_hrt.h>

static int daemon_task;

//using namespace px4;

extern "C" __EXPORT int servo_action(int argc, char *argv[]);

extern "C" __EXPORT int servo_test_main(int argc, char *argv[]);

int servo_test_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: servo_test {start}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		daemon_task = px4_task_spawn_cmd("servo_test",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 servo_action,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}


	PX4_WARN("usage: servo_test {start}\n");
	return 1;
}


int servo_action(int argc, char *argv[])
{

	PX4_INFO("Servo Action!");

	struct actuator_servos_s servo_commands;

	orb_advert_t servo_commands_pub = orb_advertise(ORB_ID(actuator_servos), &servo_commands);

	servo_commands.control[0] = -1.0;
	servo_commands.control[1] = -1.0;
	servo_commands.control[2] = -1.0;
	servo_commands.control[3] = -1.0;
	servo_commands.control[4] = 0.0;
	servo_commands.control[5] = 0.0;
	servo_commands.control[6] = 0.0;
	servo_commands.control[7] = 0.0;

	while (true) {
		//PX4_INFO("Servo action!");
		orb_publish(ORB_ID(actuator_servos), servo_commands_pub, &servo_commands);
		px4_usleep(10000);
	}

	PX4_INFO("exiting");

	return 0;
}
