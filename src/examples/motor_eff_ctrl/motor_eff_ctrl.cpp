#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <sched.h>

#include <uORB/uORB.h>

#include <uORB/topics/rc_channels.h>
#include <uORB/topics/actuator_motors_efficiency.h>

static int daemon_task;

extern "C" __EXPORT int motor_efficiency_control(int argc, char *argv[]);

extern "C" __EXPORT int motor_eff_ctrl_main(int argc, char *argv[]);
int motor_eff_ctrl_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: motor_eff_ctrl {start}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		daemon_task = px4_task_spawn_cmd("motor_eff_ctrl",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT + 40,
						 PX4_STACK_ADJUSTED(3250),
						 motor_efficiency_control,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}


	PX4_WARN("usage: motor_eff_ctrl {start}\n");
	return 1;
}

int motor_efficiency_control(int argc, char *argv[])
{

	PX4_INFO("Hello Sky!");

	
	int rc_channels_sub_fd = orb_subscribe(ORB_ID(rc_channels));


	/* limit the update rate to 5 Hz */
	orb_set_interval(rc_channels_sub_fd, 5);

	px4_pollfd_struct_t fds[] = {
		{ .fd = rc_channels_sub_fd,   .events = POLLIN },
	};

	struct rc_channels_s rc_channel_signal;

	struct actuator_motors_efficiency_s motors_efficiency;
	orb_advert_t actuator_motors_efficiency_pub = orb_advertise(ORB_ID(actuator_motors_efficiency), &motors_efficiency);
	
	motors_efficiency.efficiency[0] = 1.0;
	motors_efficiency.efficiency[1] = 1.0;
	motors_efficiency.efficiency[2] = 1.0;
	motors_efficiency.efficiency[3] = 1.0;
	motors_efficiency.efficiency[4] = 1.0;
	motors_efficiency.efficiency[5] = 1.0;
	motors_efficiency.efficiency[6] = 1.0;
	motors_efficiency.efficiency[7] = 1.0;
	motors_efficiency.efficiency[8] = 1.0;
	motors_efficiency.efficiency[9] = 1.0;
	motors_efficiency.efficiency[10] = 1.0;
	motors_efficiency.efficiency[11] = 1.0;

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

				orb_copy(ORB_ID(rc_channels), rc_channels_sub_fd, &rc_channel_signal);
			}
			
		}

		if (rc_channel_signal.channels[5] > (float)0.75)
		{
			motors_efficiency.efficiency[0] = 1.0;
			motors_efficiency.efficiency[1] = 1.0;
			motors_efficiency.efficiency[2] = 1.0;
			motors_efficiency.efficiency[3] = 1.0;
		}
		else if (rc_channel_signal.channels[5] < (float)-0.75)
		{
			motors_efficiency.efficiency[0] = 0.0;
			motors_efficiency.efficiency[1] = 0.0;
			motors_efficiency.efficiency[2] = 0.0;
			motors_efficiency.efficiency[3] = 0.0;
		}
		else
		{
			motors_efficiency.efficiency[0] = 1.0;
			motors_efficiency.efficiency[1] = 1.0;
			motors_efficiency.efficiency[2] = 1.0;
			motors_efficiency.efficiency[3] = 0.0;
		}
		

		orb_publish(ORB_ID(actuator_motors_efficiency), actuator_motors_efficiency_pub, &motors_efficiency);
		//PX4_INFO("vehicle thrust setpoint along xyz: %.4f, %.4f, %.4f", (double)thrust_setpoint.xyz[0], (double)thrust_setpoint.xyz[1], (double)thrust_setpoint.xyz[2]);
		//i++;
	}

	PX4_INFO("exiting");

	return 0;
}
