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

__EXPORT int px4_first_move_main(int argc, char *argv[]);

int px4_first_move_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");
	printf("and Hello World!\n");


	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));

	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
	};

	struct trajectory_setpoint_s next_pos;

	//memset(&next_pos, 0,sizeof(next_pos));

	next_pos.position[0] = 10;
	next_pos.position[1] = 10;
	next_pos.position[2] = 10;

	next_pos.velocity[0] = NAN;
	next_pos.velocity[1] = NAN;
	next_pos.velocity[2] = NAN;

	next_pos.acceleration[0] = NAN;
	next_pos.acceleration[1] = NAN;
	next_pos.acceleration[2] = NAN;

	next_pos.jerk[0] = NAN;
	next_pos.jerk[1] = NAN;
	next_pos.jerk[2] = NAN;

	next_pos.yaw = NAN;
	next_pos.yawspeed = NAN;

	orb_advert_t next_pos_pub = orb_advertise(ORB_ID(trajectory_setpoint), &next_pos);

	while (true) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		if (fds[0].revents & POLLIN) {
			/* obtained data for the first file descriptor */
			struct vehicle_local_position_s raw;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(vehicle_local_position), sensor_sub_fd, &raw);
			PX4_INFO("Position:\t%8.4f\t%8.4f\t%8.4f",
					(double)raw.x,
					(double)raw.y,
					(double)raw.z);
		}
		printf("%d",poll_ret);
		orb_publish(ORB_ID(trajectory_setpoint), next_pos_pub, &next_pos);
	}

	PX4_INFO("Bye!");
	return 0;
}
