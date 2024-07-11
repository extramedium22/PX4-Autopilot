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

#include <uORB/topics/vehicle_local_position_target.h>

#include <uORB/topics/trajectory_setpoint.h>

#include <px4_platform_common/log.h>


#include <px4_platform_common/module.h>

#include <drivers/drv_hrt.h>

__EXPORT int order_publish_main(int argc, char *argv[]);

int order_publish_main(int argc, char *argv[])
{
	PX4_INFO("Now begin order publish");


	struct vehicle_local_position_target_s commands;

	//hrt_abstime _timestamp_sample;

	commands.timestamp = hrt_absolute_time();
	//rotor_commands.timestamp_sample = 1.0;
	//rotor_commands.reversible_flags = 1.0;

	commands.x = atof(argv[1]);
	commands.y = atof(argv[2]);
	commands.z = atof(argv[3]);
	//commands.vx = 0.0;
	//commands.vy = 0.0;
	//commands.vz = 0.0;
	//commands.acceleration[0] = 0.0;
	//commands.acceleration[1] = 0.0;
	//commands.acceleration[2] = 0.0;
	//commands.thrust[0] = 0.0;
	//commands.thrust[1] = 0.0;
	//commands.thrust[2] = 0.0;
	//commands.yaw = 0.0;
	//commands.yawspeed = 0.0;

	orb_advert_t commands_pub = orb_advertise(ORB_ID(vehicle_local_position_target), &commands);
	PX4_INFO("Heading to: %.2f, %.2f, %.2f",(double)commands.x, (double)commands.y, (double)commands.z);
	//int i = 0;
	while (true) {
		//PX4_INFO("Heading to: %.2f, %.2f, %.2f",(double)commands.x, (double)commands.y, (double)commands.z);
		orb_publish(ORB_ID(vehicle_local_position_target), commands_pub, &commands);
		px4_usleep(100);
		//i++;
	}

	PX4_INFO("Bye!");
	return 0;
}
