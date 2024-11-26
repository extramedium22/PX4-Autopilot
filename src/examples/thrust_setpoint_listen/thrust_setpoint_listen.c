#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_target.h>
#include <uORB/topics/trajectory_setpoint.h>

#include <uORB/topics/vehicle_thrust_setpoint.h>




__EXPORT int thrust_setpoint_listen_main(void);



int thrust_setpoint_listen_main(void)
{

	PX4_INFO("Hello Sky!");

	
	int thrust_setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_thrust_setpoint));


	/* limit the update rate to 5 Hz */
	orb_set_interval(thrust_setpoint_sub_fd, 5);

	px4_pollfd_struct_t fds[] = {
		{ .fd = thrust_setpoint_sub_fd,   .events = POLLIN },
	};

	struct vehicle_thrust_setpoint_s thrust_setpoint;

	int error_counter = 0;
	//int i = 0;

	while (true) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				PX4_INFO("Receiving data");

				orb_copy(ORB_ID(vehicle_thrust_setpoint), thrust_setpoint_sub_fd, &thrust_setpoint);
			}
			
		}
		printf("Thrust setpoint listen timestamp: %I64u\n", thrust_setpoint.timestamp);
		printf("Thrust setpoint listen timestamp sample: %I64u\n", thrust_setpoint.timestamp_sample);
		PX4_INFO("vehicle thrust setpoint along xyz: %.4f, %.4f, %.4f", (double)thrust_setpoint.xyz[0], (double)thrust_setpoint.xyz[1], (double)thrust_setpoint.xyz[2]);
		//i++;
	}

	PX4_INFO("exiting");

	return 0;
}
