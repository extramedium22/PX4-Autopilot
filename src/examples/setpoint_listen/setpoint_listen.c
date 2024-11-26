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
#include <uORB/topics/vehicle_local_position_setpoint.h>



__EXPORT int setpoint_listen_main(void);



int setpoint_listen_main(void)
{

	PX4_INFO("Hello Sky!");

	
	int setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int trajectory_setpoint_sub_fd = orb_subscribe(ORB_ID(trajectory_setpoint));


	/* limit the update rate to 5 Hz */
	orb_set_interval(setpoint_sub_fd, 200);

	px4_pollfd_struct_t fds[] = {
		{ .fd = setpoint_sub_fd,   .events = POLLIN }
	};

	struct vehicle_local_position_setpoint_s set_point;
	struct trajectory_setpoint_s received_trajectory_setpoint;

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
			/*
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_local_position), pos_sub_fd, &current_pos);
				PX4_INFO("Receive position data");
				current_status.px = current_pos.x;
				current_status.py = current_pos.y;
				current_status.pz = current_pos.z;
			}
			*/
			/*
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude), att_sub_fd, &current_att);
				q0 = current_att.q[0];
				q1 = current_att.q[1];
				q2 = current_att.q[2];
				q3 = current_att.q[3];
				current_status.phi = atan2(2 * (q0 *q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
				current_status.theta = asin(2 * (q0 * q2 - q1 * q3));
				current_status.psi = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
			}
			*/
			/*
			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_angular_velocity), attvel_sub_fd, &current_attvel);
				PX4_INFO("Receive angular velocity data");
				current_status.p = current_attvel.xyz[0];
				current_status.q = current_attvel.xyz[1];
				current_status.r = current_attvel.xyz[2];
			}
			*/

			if (fds[0].revents & POLLIN) {
				PX4_INFO("Receiving data");
				orb_copy(ORB_ID(vehicle_local_position_setpoint), setpoint_sub_fd, &set_point);
				orb_copy(ORB_ID(trajectory_setpoint), trajectory_setpoint_sub_fd, &received_trajectory_setpoint);
				PX4_INFO("setpoint = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
						(double)set_point.x, (double)set_point.y, (double)set_point.z,
						(double)set_point.vx, (double)set_point.vy, (double)set_point.vz,
						(double)set_point.yaw, (double)set_point.yawspeed
						);
				PX4_INFO("setpoint_track = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
						(double)received_trajectory_setpoint.position[0], (double)received_trajectory_setpoint.position[1], (double)received_trajectory_setpoint.position[2],
						(double)received_trajectory_setpoint.velocity[0], (double)received_trajectory_setpoint.velocity[1], (double)received_trajectory_setpoint.velocity[2]
						);
			}
			
		}
		//i++;
	}

	PX4_INFO("exiting");

	return 0;
}
