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
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

#include <uORB/topics/vehicle_local_position_target.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <uORB/topics/trajectory_setpoint.h>

#include <uORB/topics/actuator_motors_efficiency.h>

#include <px4_platform_common/log.h>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>

#include <px4_platform_common/module.h>

#include <uORB/topics/control_allocator_status.h>

#include <uORB/topics/vehicle_thrust_setpoint.h>

#include <uORB/topics/rc_channels.h>

#include <drivers/drv_hrt.h>

struct ud_vec {
	float f;
	float taop;
	float taoq;
	float taor;
};

struct T_vec {
	float T1;
	float T2;
	float T3;
	float T4;
};

struct pos_vec {
	float x;
	float y;
	float z;
};

struct status {
	float px;
	float py;
	float pz;
	float vx;
	float vy;
	float vz;
	float ax;
	float ay;
	float az;
	float phi;
	float theta;
	float psi;
	float p;
	float q;
	float r;
	float dotp;
	float dotq;
	float dotr;
};

static int daemon_task;

extern "C" __EXPORT int quadcopter_backstepping_ctrl(int argc, char *argv[]);

//__EXPORT struct T_vec control_allocation(struct ud_vec ud);
extern "C" __EXPORT struct T_vec control_allocation(struct ud_vec uc);

extern "C" __EXPORT struct ud_vec desired_control_law(struct status current_status, struct pos_vec target_pos, struct pos_vec target_vel, float target_yaw, float target_yawspeed);

//__EXPORT struct ud_vec u_filter(struct ud_vec u_in, struct ud_vec u_last);

//__EXPORT struct pos_vec get_target(struct pos_vec hold_pos, struct vehicle_local_position_setpoint_s set_point, struct pos_vec takeoff_pos, int tookoff);
//__EXPORT int check_tookoff(struct status current_status, struct pos_vec takeoff_pos, int tookoff);
//__EXPORT struct pos_vec get_hold_pos(struct pos_vec hold_pos, struct vehicle_local_position_setpoint_s set_point);


extern "C" __EXPORT int quadcopterbsctrl_main(int argc, char *argv[]);
int quadcopterbsctrl_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: quadcopterbsctrl {start}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		daemon_task = px4_task_spawn_cmd("quadcopterbsctrl",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT + 40,
						 PX4_STACK_ADJUSTED(3250),
						 quadcopter_backstepping_ctrl,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}


	PX4_WARN("usage: quadcopterbsctrl {start}\n");
	return 1;
}

int quadcopter_backstepping_ctrl(int argc, char *argv[])
{
	struct status current_status = {
		.px = 0,
		.py = 0,
		.pz = 0,
		.vx = 0,
		.vy = 0,
		.vz = 0,
		.ax = 0,
		.ay = 0,
		.az = 0,
		.phi = 0,
		.theta = 0,
		.psi = 0,
		.p = 0,
		.q = 0,
		.r = 0,
		.dotp = 0,
		.dotq = 0,
		.dotr = 0
	};


	struct pos_vec target_pos = {
		.x = 0,
		.y = 0,
		.z = -2
	};

	struct pos_vec target_vel = {
		.x = 0,
		.y = 0,
		.z = 0
	};

	float target_yaw = 0;
	float target_yawspeed = 0;

	struct ud_vec uc = {
		.f = 0,
		.taop = 0,
		.taoq = 0,
		.taor = 0
	};
	/*
	struct ud_vec u_last = {
		.f = 0,
		.taop = 0,
		.taoq = 0,
		.taor = 0
	};
	*/
	struct T_vec T_desired = {
		.T1 = 0,
		.T2 = 0,
		.T3 = 0,
		.T4 = 0,
	};

	/*
	struct pos_vec hold_pos = {
		.x = 0,
		.y = 0,
		.z = -10
	};
	*/

	//float b = 0.159;
	//float c = 0.06;


	//float m = 1.5;
	//float g = 9.81;

	//PX4_INFO("Hello Sky!");

	
	int pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	int att_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	int attvel_sub_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	//int target_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position_target));
	//int setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int track_setpoint_sub_fd = orb_subscribe(ORB_ID(trajectory_setpoint));

	int rc_channels_sub_fd = orb_subscribe(ORB_ID(rc_channels));

	int thrust_setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_thrust_setpoint));

	

	float q0 = 1;
	float q1 = 0;
	float q2 = 0;
	float q3 = 0;

	//param for x250
	float mc = 0.0000013853;
	float maxRotVelocity = 2791.0;

	/* limit the update rate to 5 Hz */
	//orb_set_interval(pos_sub_fd, 5);
	//orb_set_interval(att_sub_fd, 2.5);
	//orb_set_interval(att_sub_fd, 500);
	//orb_set_interval(attvel_sub_fd, 200);

	px4_pollfd_struct_t fds[] = {
		//{ .fd = pos_sub_fd,   .events = POLLIN },
		{ .fd = att_sub_fd,   .events = POLLIN },
		//{ .fd = attvel_sub_fd,   .events = POLLIN },
		//{ .fd = target_sub_fd,   .events = POLLIN },
	};

	struct vehicle_local_position_s current_pos;
	struct vehicle_attitude_s current_att;
	struct vehicle_angular_velocity_s current_attvel;
	//struct vehicle_local_position_target_s received_order;
	//struct vehicle_local_position_setpoint_s set_point;
	struct trajectory_setpoint_s track_setpoint;

	struct vehicle_thrust_setpoint_s thrust_setpoint;

	struct rc_channels_s rc_channel_signal;

	struct actuator_motors_s rotor_commands;
	//struct actuator_servos_s servo_commands;
	//struct control_allocator_status_s control_status;

	int error_counter = 0;
	//int i = 0;

	orb_advert_t rotor_commands_pub = orb_advertise(ORB_ID(actuator_motors), &rotor_commands);
	//orb_advert_t servo_commands_pub = orb_advertise(ORB_ID(actuator_servos), &servo_commands);
	//orb_advert_t control_status_pub = orb_advertise(ORB_ID(control_allocator_status), &control_status);


	//control_status.torque_setpoint_achieved = 1;
	//control_status.thrust_setpoint_achieved = 1;

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
				//PX4_INFO("Receiving data");
				orb_copy(ORB_ID(vehicle_attitude), att_sub_fd, &current_att);
				q0 = current_att.q[0];
				q1 = current_att.q[1];
				q2 = current_att.q[2];
				q3 = current_att.q[3];
				current_status.phi = atan2(2 * (q0 *q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
				current_status.theta = asin(2 * (q0 * q2 - q1 * q3));
				current_status.psi = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
				orb_copy(ORB_ID(vehicle_angular_velocity), attvel_sub_fd, &current_attvel);
				current_status.p = current_attvel.xyz[0];
				current_status.q = current_attvel.xyz[1];
				current_status.r = current_attvel.xyz[2];
				current_status.dotp = current_attvel.xyz_derivative[0];
				current_status.dotq = current_attvel.xyz_derivative[1];
				current_status.dotr = current_attvel.xyz_derivative[2];
				orb_copy(ORB_ID(vehicle_local_position), pos_sub_fd, &current_pos);
				current_status.px = current_pos.x;
				current_status.py = current_pos.y;
				current_status.pz = current_pos.z;
				current_status.vx = current_pos.vx;
				current_status.vy = current_pos.vy;
				current_status.vz = current_pos.vz;
				current_status.ax = current_pos.ax;
				current_status.ay = current_pos.ay;
				current_status.az = current_pos.az;
				//PX4_INFO("current_pos = %.4f, %.4f, %.4f", (double)current_pos.x, (double)current_pos.y, (double)current_pos.z);
				//PX4_INFO("current_att = %.4f, %.4f, %.4f", (double)current_status.p, (double)current_status.q, (double)current_status.r);
				/*
				orb_copy(ORB_ID(vehicle_local_position_target), target_sub_fd, &received_order);
				target.x = received_order.x;
				target.y = received_order.y;
				target.z = received_order.z;
				//target.x = 100.0;
				//target.y = 100.0;
				//target.z = -2.0;
				*/
				//orb_copy(ORB_ID(vehicle_local_position_setpoint), setpoint_sub_fd, &set_point);
				orb_copy(ORB_ID(trajectory_setpoint), track_setpoint_sub_fd, &track_setpoint);
				
				/*
				PX4_INFO("setpoint = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
						(double)set_point.x, (double)set_point.y, (double)set_point.z,
						(double)set_point.vx, (double)set_point.vy, (double)set_point.vz
						);
				*/
				/*
				PX4_INFO("setpoint = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
						(double)track_setpoint.position[0], (double)track_setpoint.position[1], (double)track_setpoint.position[2],
						(double)track_setpoint.velocity[0], (double)track_setpoint.velocity[1], (double)track_setpoint.velocity[2]
						);
				*/
				
				//tookoff = check_tookoff(current_status, takeoff_pos, tookoff);
				//tookoff = 1;
				//PX4_INFO("If took off: %d", tookoff);
				//hold_pos = get_hold_pos(hold_pos, set_point);
				//target = get_target(hold_pos, set_point, takeoff_pos, tookoff);


				//PX4_INFO("target = %.4f, %.4f, %.4f", (double)target.x, (double)target.y, (double)target.z);

				//hear from FCU
				/*
				target_pos.x = set_point.x;
				target_pos.y = set_point.y;
				target_pos.z = set_point.z;
				target_vel.x = set_point.vx;
				target_vel.y = set_point.vy;
				target_vel.z = set_point.vz;
				*/
				//target_pos.x = track_setpoint.position[0];
				//target_pos.y = track_setpoint.position[1];
				target_pos.z = track_setpoint.position[2];
				target_vel.x = track_setpoint.velocity[0];
				target_vel.y = track_setpoint.velocity[1];
				target_vel.z = track_setpoint.velocity[2];
				target_yaw = track_setpoint.yaw;
				target_yawspeed = track_setpoint.yawspeed;

				//for testing
				target_pos.x = 0;
				target_pos.y = 0;
				//target_pos.z = -30; 
				//target_vel.x = 0;
				//target_vel.y = 0;
				//target_vel.z = 0;
				//target_yaw = 0;
				//target_yawspeed = 0;

				uc = desired_control_law(current_status, target_pos, target_vel, target_yaw, target_yawspeed);
				/*
				uc = u_filter(uc, u_last);
				u_last.f = uc.f;
				u_last.taop = uc.taop;
				u_last.taoq = uc.taoq;
				u_last.taor = uc.taor;
				*/

				//T_desired = control_allocation(uc);
				T_desired = control_allocation(uc);

				orb_copy(ORB_ID(vehicle_thrust_setpoint), thrust_setpoint_sub_fd, &thrust_setpoint);
				
				/*
				//hrt_abstime _timestamp_sample;

				servo_commands.timestamp = hrt_absolute_time();
				servo_commands.timestamp_sample = thrust_setpoint.timestamp_sample;
				
				servo_commands.control[0] = 0.0;
				servo_commands.control[1] = 0.0;
				servo_commands.control[2] = 0.0;
				servo_commands.control[3] = 0.0;
				servo_commands.control[4] = 0.0;
				servo_commands.control[5] = 0.0;
				servo_commands.control[6] = 0.0;
				servo_commands.control[7] = 0.0;

				orb_publish(ORB_ID(actuator_servos), servo_commands_pub, &servo_commands);
				*/
			
				/*
				rotor_commands.control[0] = ((float)0.001 * ((float)sqrt(T_desired.T1/mc)));
				rotor_commands.control[1] = ((float)0.001 * ((float)sqrt(T_desired.T2/mc)));
				rotor_commands.control[2] = ((float)0.001 * ((float)sqrt(T_desired.T3/mc)));
				rotor_commands.control[3] = ((float)0.001 * ((float)sqrt(T_desired.T3/mc)));
				rotor_commands.control[4] = ((float)0.001 * ((float)sqrt(T_desired.T4/mc)));
				rotor_commands.control[5] = ((float)0.001 * ((float)sqrt(T_desired.T4/mc)));
				rotor_commands.control[6] = 0.0;
				rotor_commands.control[7] = 0.0;
				rotor_commands.control[8] = 0.0;
				rotor_commands.control[9] = 0.0;
				rotor_commands.control[10] = 0.0;
				rotor_commands.control[11] = 0.0;
				*/

				

				rotor_commands.timestamp = hrt_absolute_time();
				rotor_commands.timestamp_sample = thrust_setpoint.timestamp_sample;
				
				rotor_commands.reversible_flags = 0;

				//rotor_commands.NUM_CONTROLS = 12;

				rotor_commands.control[0] = ((float)sqrt(T_desired.T1/mc)) / maxRotVelocity;
				rotor_commands.control[1] = ((float)sqrt(T_desired.T2/mc)) / maxRotVelocity;
				rotor_commands.control[2] = ((float)sqrt(T_desired.T3/mc)) / maxRotVelocity;
				rotor_commands.control[3] = ((float)sqrt(T_desired.T4/mc)) / maxRotVelocity;
				rotor_commands.control[4] = 0.0;
				rotor_commands.control[5] = 0.0;
				rotor_commands.control[6] = 0.0;
				rotor_commands.control[7] = 0.0;
				rotor_commands.control[8] = 0.0;
				rotor_commands.control[9] = 0.0;
				rotor_commands.control[10] = 0.0;
				rotor_commands.control[11] = 0.0;

				//PX4_INFO("motor command = %.4f, %.4f, %.4f, %.4f.", (double)rotor_commands.control[0], (double)rotor_commands.control[1], (double)rotor_commands.control[2], (double)rotor_commands.control[3]);


				/*
				//for testing
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
				*/

				//printf("Actuator motors reversible_flags: %I16u\n", rotor_commands.reversible_flags);
				//printf("Actuator motors timestamp: %I64u\n", rotor_commands.timestamp);
				//printf("Actuator motors timestamp_sample: %I64u\n", rotor_commands.timestamp_sample);
				orb_copy(ORB_ID(rc_channels), rc_channels_sub_fd, &rc_channel_signal);
				
				/*
				if ((rc_channel_signal.channels[5] > (float)-0.75) && (rc_channel_signal.channels[5] < (float)0.75))
				{
					rotor_commands.control[0] = -1.0;
				}
				*/

				if (rc_channel_signal.channels[5] < (float)-0.75)
				{
					rotor_commands.control[0] = -1.0;
					rotor_commands.control[1] = -1.0;
					rotor_commands.control[2] = -1.0;
					rotor_commands.control[3] = -1.0;
				}
				//PX4_INFO("Motor Running!");
				orb_publish(ORB_ID(actuator_motors), rotor_commands_pub, &rotor_commands);
				//orb_publish(ORB_ID(control_allocator_status), control_status_pub, &control_status);
				
			}
			
		}
		//PX4_INFO("current pos: %.2f, %.2f, %.2f", (double)current_status.px, (double)current_status.py, (double)current_status.pz);		
		//PX4_INFO("current att: %.2f, %.2f, %.2f", (double)current_status.phi, (double)current_status.theta, (double)current_status.psi);
		//PX4_INFO("att in q: %.4f, %.4f, %.4f, %.4f", (double)current_att.q[0], (double)current_att.q[1], (double)current_att.q[2], (double)current_att.q[3]);
		//PX4_INFO("angular velocity: %.4f, %.4f, %.4f", (double)current_status.p, (double)current_status.q, (double)current_status.r);
		//PX4_INFO("heading to: %.2f, %.2f, %.2f", (double)target.x, (double)target.y, (double)target.z);
		//i++;
	}

	//PX4_INFO("exiting");

	return 0;
}

struct ud_vec desired_control_law(struct status current_status, struct pos_vec target_pos, struct pos_vec target_vel, float target_yaw, float target_yawspeed)
{
	struct ud_vec ud;
	//struct ud_vec d;
	//struct ud_vec uc;

	float g = 9.81;

	//param for x450
	//float m = 1.5;
	//float Jx = 0.029125;
	//float Jy = 0.029125;
	//float Jz = 0.055225;

	//param for x250
	float m = 0.72;
	float Jx = 0.0056;
	float Jy = 0.0056;
	float Jz = 0.0104;


	float kp = 1.0;
	float kv = 1.0;
	float kvz = 5;
	//float a = 0.15;
	float a = 3;
	//float kn3 = 3;
	float kn3 = 8;
	//float kw = 14;
	float kw = 20;
	float kr = 1.0;
	//kr = kw * (float)0.3;
	//float epsilon = 0.06;
	//float krotor = 2.0;
	//float krotor = 5.0;
	//float krotorz = 0.1;
	float kcp = 0.6;
	//float kpsi = 0.1;
	float kvzd = 1;
	
	float vxd = 0;
	float vyd = 0;
	float vzd = 0;
	float axd = 0;
	float ayd = 0;
	float azd = 0;
	float anorm = 1;
	float agnorm = 1;
	float n3xd = 0;
	float n3yd = 0;
	//float n3zd = 1;
	float pd = 0;
	float qd = 0;
	float rd = 0;

	float sn3x = 0;
	float sn3y = 0;
	//float sn3z = 0;
	float sp = 0;
	float sq = 0;
	//float sr = 0;

	float A = 0;
	float B = 0;
	float C = 0;
	float D = 0;
	float n3x = 0;
	float n3y = 0;
	float n3z = 1;

	float dotA = 0;
	float dotB = 0;
	float dotC = 0;
	float dotD = 0;

	/*
	vxd = kp * (target.x - current_status.px);
	vyd = kp * (target.y - current_status.py);
	vzd = kp * (target.z - current_status.pz);
	*/


	if (isnan(target_pos.x))
	{
		if (isnan(target_vel.x))
		{
			vxd = 0.0;
		}
		else
		{
			vxd = target_vel.x;
		}
	}
	else
	{
		vxd = kp * (target_pos.x - current_status.px);
	}

	if (isnan(target_pos.y))
	{
		if (isnan(target_vel.y))
		{
			vyd = 0.0;
		}
		else
		{
			vyd = target_vel.y;
		}
	}
	else
	{
		vyd = kp * (target_pos.y - current_status.py);
	}

	if (isnan(target_pos.z))
	{
		if (isnan(target_vel.z))
		{
			vzd = 0;
		}
		else
		{
			vzd = target_vel.z * kvzd;
		}
	}
	else
	{
		vzd = kp * (target_pos.z - current_status.pz);
	}
	//PX4_INFO("target = %.4f, %.4f, %.4f", (double)target_pos.x, (double)target_pos.y, (double)target_pos.z);
	//PX4_INFO("vd = %.4f, %.4f, %.4f", (double)vxd, (double)vyd, (double)vzd);

	axd = kv * (vxd - current_status.vx);
	ayd = kv * (vyd - current_status.vy);
	azd = kvz * (vzd - current_status.vz);

	//PX4_INFO("ad = %.4f, %.4f, %.4f", (double)axd, (double)ayd, (double)azd);

	anorm = (float)sqrt(axd * axd + ayd * ayd + azd * azd);
	if (anorm > a)
	{
		axd = (a/anorm) * axd;
		ayd = (a/anorm) * ayd;
		azd = (a/anorm) * azd;
	}
	agnorm = (float)sqrt(axd * axd + ayd * ayd + ((azd - g) * (azd - g)));
	n3xd = -1 * axd / agnorm;
	n3yd = -1 * ayd / agnorm;
	//n3zd = -1 * (azd - g) / agnorm;

	//PX4_INFO("n3d = %.4f, %.4f", (double)n3xd, (double)n3yd);

	A = (float)cos(current_status.psi) * (float)cos(current_status.theta);
	B = (float)sin(current_status.psi) * (float)cos(current_status.theta);
	C = (float)cos(current_status.psi) * (float)sin(current_status.theta) * (float)sin(current_status.phi) - (float)sin(current_status.psi) * (float)cos(current_status.phi);
	D = (float)sin(current_status.psi) * (float)sin(current_status.theta) * (float)sin(current_status.phi) + (float)cos(current_status.psi) * (float)cos(current_status.phi);
	n3x = (float)cos(current_status.psi) * (float)sin(current_status.theta) * (float)cos(current_status.phi) + (float)sin(current_status.psi) * (float)sin(current_status.phi);
	n3y = (float)sin(current_status.psi) * (float)sin(current_status.theta) * (float)cos(current_status.phi) - (float)cos(current_status.psi) * (float)sin(current_status.phi);
	n3z = (float)cos(current_status.theta) * (float)cos(current_status.phi);
	dotA = C * current_status.r - current_status.q * n3x;
	dotB = D * current_status.r - current_status.q * n3y;
	dotC = -1 * A * current_status.r + current_status.p * n3x;
	dotD = -1 * B * current_status.r + current_status.p * n3y;

	sn3x = n3x - n3xd;
	sn3y = n3y - n3yd;
	//sn3z = n3z - n3zd;

	//PX4_INFO("sn3 = %.4f, %.4f", (double)sn3x, (double)sn3y);

	pd = kn3 * (-1 * B * sn3x + A * sn3y);
	qd = kn3 * (-1 * D * sn3x + C * sn3y);
	sp = current_status.p - pd;
	sq = current_status.q - qd;

	//PX4_INFO("omega_d = %.4f, %.4f", (double)pd, (double)qd);
	//PX4_INFO("s2 = %.4f, %.4f", (double)sp, (double)sq);

	//ud.taop = Jx * (-1 * kw * sp + current_status.q * current_status.r * (Jz - Jy) / Jx - sn3x * (-1 * C + kn3 * dotB) - sn3y * (-1 * D - kn3 * dotA) - n3z * kn3 * current_status.p - kcp * (Jz - Jy) * current_status.r * sq / Jx);
	//ud.taoq = Jy * (-1 * kw * sq + current_status.p * current_status.r * (Jx - Jz) / Jy - sn3x * (A + kn3 * dotD) - sn3y * (B - kn3 * dotC) - n3z * kn3 * current_status.q - kcp * (Jx -Jz) * current_status.r * sp / Jy);

	ud.taop = Jx * (-1 * kw * sp + current_status.q * current_status.r * (Jz - Jy) / Jx - sn3x * (-1 * C + kn3 * dotB - n3z * kn3 * kn3 * B) - sn3y * (-1 * D - kn3 * dotA + n3z * kn3 * kn3 * A) - n3z * kn3 * sp - kcp * (Jz - Jy) * current_status.r * sq / Jx);
	ud.taoq = Jy * (-1 * kw * sq + current_status.p * current_status.r * (Jx - Jz) / Jy - sn3x * (A + kn3 * dotD - n3z * kn3 * kn3 * D) - sn3y * (B - kn3 * dotC + n3z * kn3 * kn3 * C) - n3z * kn3 * sq - kcp * (Jx -Jz) * current_status.r * sp / Jy);

	/*
	if (isnan(target_yaw))
	{
		if (isnan(target_yawspeed))
		{
			rd = 0.0;
		}
		else
		{
			rd = target_yawspeed;
		}
	}
	else
	{
		rd = kpsi * (target_yaw - current_status.psi);
	}
	*/

	//PX4_INFO("target yaw: %.4f, yawspeed: %.4f, rd: %.4f", (double)target_yaw, (double)target_yawspeed, (double)rd);
	rd = 0;
	ud.taor = -1 * Jz * kr * (current_status.r - rd);

	ud.f = m * (kvz * (current_status.vz - vzd) + g) / n3z;

	float uf_max = 40;
	if (ud.f > uf_max)
	{
		ud.f = uf_max;
	}

	//PX4_INFO("ud: %.4f, %.4f, %.4f, %.4f",(double)ud.f, (double)ud.taop, (double)ud.taoq, (double)ud.taor);

	/*
	d.taop = b * (-T_k1.T1 + T_k1.T2 + T_k1.T3 - T_k1.T4) - Jx * current_status.dotp - (Jz - Jy) * current_status.q * current_status.r;
	d.taoq = b * (T_k1.T1 - T_k1.T2 + T_k1.T3 - T_k1.T4) - Jy * current_status.dotq - (Jx - Jz) * current_status.p * current_status.r;
	d.taor = c * (T_k1.T1 + T_k1.T2 - T_k1.T3 - T_k1.T4) - Jz * current_status.dotr - (Jy - Jx) * current_status.p * current_status.q;
	d.f = (T_k1.T1 + T_k1.T2 + T_k1.T3 + T_k1.T4) + (current_status.az - g) * m / n3z;
	PX4_INFO("d: %.4f, %.4f, %.4f, %.4f",(double)d.f, (double)d.taop, (double)d.taoq, (double)d.taor);

	uc.taop = krotor * (ud.taop + d.taop - b * (-T_k1.T1 + T_k1.T2 + T_k1.T3 - T_k1.T4));
	uc.taoq = krotor * (ud.taoq + d.taoq - b * (T_k1.T1 - T_k1.T2 + T_k1.T3 - T_k1.T4));
	uc.taor = krotorz * (ud.taor + d.taor - c * (T_k1.T1 + T_k1.T2 - T_k1.T3 - T_k1.T4));
	uc.f = ud.f + d.f;
	PX4_INFO("uc: %.4f, %.4f, %.4f, %.4f",(double)uc.f, (double)uc.taop, (double)uc.taoq, (double)uc.taor);
	*/

	//return uc;
	return ud;
}

/*
struct T_vec control_allocation(struct ud_vec uc)
{
	struct T_vec T_desired;
	float b = 0.0884;
	float c = 0.06;
	T_desired.T1 = (float)0.25*(uc.f-(uc.taop/b)+(uc.taoq/b)+(uc.taor/c));
	T_desired.T2 = (float)0.25*(uc.f+(uc.taop/b)-(uc.taoq/b)+(uc.taor/c));
	T_desired.T3 = (float)0.25*(uc.f+(uc.taop/b)+(uc.taoq/b)-(uc.taor/c));
	T_desired.T4 = (float)0.25*(uc.f-(uc.taop/b)-(uc.taoq/b)-(uc.taor/c));
	return T_desired;
}
*/


struct T_vec control_allocation(struct ud_vec uc)
{
	float T_max = 10.791;
	float T_min = 0;

	float T[4] = {0, 0, 0, 0};

	struct T_vec T_out = {
		.T1 = 0,
		.T2 = 0,
		.T3 = 0,
		.T4 = 0
	};

	//param for x450
	//float b = 0.159;
	//float c = 0.06;
	//float b_re = 6.289308176;
	//float c_re = 16.666666667;

	//param for x250
	//float b = 0.0884;
	//float c = 0.02;
	float b_re = 2.8289;
	float c_re = 12.5;

	T[0] = (uc.f - b_re * uc.taop + b_re * uc.taoq + c_re * uc.taor) * (float)0.25;
	T[1] = (uc.f + b_re * uc.taop - b_re * uc.taoq + c_re * uc.taor) * (float)0.25;
	T[2] = (uc.f + b_re * uc.taop + b_re * uc.taoq - c_re * uc.taor) * (float)0.25;
	T[3] = (uc.f - b_re * uc.taop - b_re * uc.taoq - c_re * uc.taor) * (float)0.25;


	for (int j = 0; j < 4; j++)
	{
		if (T[j] > T_max)
		{
			T[j] = T_max;
		}
		else if (T[j] < T_min)
		{
			T[j] = T_min;
		}
		
	}
	
	T_out.T1 = T[0];
	T_out.T2 = T[1];
	T_out.T3 = T[2];
	T_out.T4 = T[3];

	//for testing
	//T_out.T1 = 1;
	//T_out.T2 = 1;
	//T_out.T3 = 1;
	//T_out.T4 = 1;

	//PX4_INFO("Thrust: %.4f, %.4f, %.4f, %.4f", (double)T_out.T1, (double)T_out.T2, (double)T_out.T3, (double)T_out.T4);

	return T_out;
}

/*
struct pos_vec get_target(struct pos_vec hold_pos, struct vehicle_local_position_setpoint_s set_point, struct pos_vec takeoff_pos, int tookoff)
{
	struct pos_vec target = {
		.x = 0,
		.y = 0,
		.z = 0
	};

	if (tookoff == 0)
	{
		target.x = takeoff_pos.x;
		target.y = takeoff_pos.y;
		target.z = takeoff_pos.z;
	}
	else
	{
		if (isnan(set_point.x))
		{
			target.x = hold_pos.x;
		}
		else
		{
			target.x = set_point.x;
		}

		if (isnan(set_point.y))
		{
			target.y = hold_pos.y;
		}
		else
		{
			target.y = set_point.y;
		}

		if (isnan(set_point.z))
		{
			target.z = hold_pos.z;
		}
		else
		{
			target.z = set_point.z;
		}
	}
	return target;
}

int check_tookoff(struct status current_status, struct pos_vec takeoff_pos, int tookoff)
{
	//float distance = 10;
	
	//distance = sqrt(((current_status.px - takeoff_pos.x) * (current_status.px - takeoff_pos.x)) + ((current_status.py - takeoff_pos.y) * (current_status.py - takeoff_pos.y)) + ((current_status.pz - takeoff_pos.z) * (current_status.pz - takeoff_pos.z)));

	float alt = 0;

	alt = current_status.pz;

	if (tookoff == 1)
	{
		return 1;
	}
	else if (alt < (float)-10.0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

struct pos_vec get_hold_pos(struct pos_vec hold_pos, struct vehicle_local_position_setpoint_s set_point)
{
	struct pos_vec new_hold_pos = {
		.x = 0,
		.y = 0,
		.z = 0
	};
	if (isnan(set_point.x))
	{
		new_hold_pos.x = hold_pos.x;
	}
	else
	{
		new_hold_pos.x = set_point.x;
	}
	if (isnan(set_point.y))
	{
		new_hold_pos.y = hold_pos.y;
	}
	else
	{
		new_hold_pos.y = set_point.y;
	}
	if (isnan(set_point.z))
	{
		new_hold_pos.z = hold_pos.z;
	}
	else
	{
		new_hold_pos.z = set_point.z;
	}
	return new_hold_pos;
}
*/
