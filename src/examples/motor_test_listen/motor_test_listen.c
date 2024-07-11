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

__EXPORT int motor_test_listen_main(int argc, char *argv[]);

int motor_test_listen_main(int argc, char *argv[])
{
	PX4_INFO("Listen to motor control commands!");

	int sensor_sub_fd = orb_subscribe(ORB_ID(actuator_motors));

	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
	};

	while (true) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);
		if (poll_ret > 0)
		{
			if (fds[0].revents & POLLIN) {
			/* obtained data for the first file descriptor */
			struct actuator_motors_s motor_listen;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(actuator_motors), sensor_sub_fd, &motor_listen);
			PX4_INFO("motor commands:%8.4f%8.4f%8.4f%8.4f",
					(double)motor_listen.control[0],
					(double)motor_listen.control[1],
					(double)motor_listen.control[2],
					(double)motor_listen.control[3]);
			}
		}
		
		//printf("%d",poll_ret);
	}

	PX4_INFO("Bye!");
	return 0;
}
