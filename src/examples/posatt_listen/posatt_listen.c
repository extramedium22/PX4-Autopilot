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
#include <uORB/topics/actuator_motors.h>




struct status{
	float px;
	float py;
	float pz;
	float vx;
	float vy;
	float vz;
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

struct target_vec{
	float x;
	float y;
	float z;
	float yaw;
};

struct u_vec{
	float f;
	float taup;
	float tauq;
	float taur;
};

struct T_vec{
	float T1;
	float T2;
	float T3;
	float T4;
};

__EXPORT int posatt_listen_main(void);



int posatt_listen_main(void)
{
	struct status current_status = {
		.px = 0,
		.py = 0,
		.pz = 0,
		.vx = 0,
		.vy = 0,
		.vz = 0,
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

	struct target_vec target = {
		.x = 0,
		.y = 0,
		.z = 0,
		.yaw = 0
	};

	struct target_vec target_error = {
		.x = 0,
		.y = 0,
		.z = 0,
		.yaw = 0
	};

	struct u_vec u_real = {
		.f = 0,
		.taup = 0,
		.tauq = 0,
		.taur = 0
	};

	struct T_vec T_omega = {
		.T1 = 0,
		.T2 = 0,
		.T3 = 0,
		.T4 = 0
	};

	struct T_vec T_real = {
		.T1 = 0,
		.T2 = 0,
		.T3 = 0,
		.T4 = 0
	};

	PX4_INFO("Hello Sky!");

	
	int pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	int att_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	int attvel_sub_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	//int target_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position_target));
	int track_setpoint_sub_fd = orb_subscribe(ORB_ID(trajectory_setpoint));

	int actuator_motors_sub_fd = orb_subscribe(ORB_ID(actuator_motors));

	float q0 = 1;
	float q1 = 0;
	float q2 = 0;
	float q3 = 0;

	//param for x250
	float b = 0.0884;
	float c = 0.02;
	float rotor_max_speed = 2791;
	float motor_constant = 0.00000139;

	/* limit the update rate to 5 Hz */
	//orb_set_interval(pos_sub_fd, 5);
	orb_set_interval(att_sub_fd, 200);
	//orb_set_interval(attvel_sub_fd, 200);

	px4_pollfd_struct_t fds[] = {
		{ .fd = pos_sub_fd,   .events = POLLIN },
		//{ .fd = att_sub_fd,   .events = POLLIN },
		//{ .fd = attvel_sub_fd,   .events = POLLIN },
		//{ .fd = target_sub_fd,   .events = POLLIN },
	};

	struct vehicle_local_position_s current_pos;
	struct vehicle_attitude_s current_att;
	struct vehicle_angular_velocity_s current_attvel;
	//struct vehicle_local_position_target_s received_order;
	struct trajectory_setpoint_s track_setpoint;

	struct actuator_motors_s motor_commands;

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
				orb_copy(ORB_ID(vehicle_local_position), pos_sub_fd, &current_pos);
				current_status.px = current_pos.x;
				current_status.py = current_pos.y;
				current_status.pz = current_pos.z;
				current_status.vx = current_pos.vx;
				current_status.vy = current_pos.vy;
				current_status.vz = current_pos.vz;
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
				/*
				orb_copy(ORB_ID(vehicle_local_position_target), target_sub_fd, &received_order);
				target.x = received_order.x;
				target.y = received_order.y;
				target.z = received_order.z;
				*/
				orb_copy(ORB_ID(trajectory_setpoint), track_setpoint_sub_fd, &track_setpoint);
				target.x = track_setpoint.position[0];
				target.y = track_setpoint.position[1];
				target.z = track_setpoint.position[2];
				target.yaw = track_setpoint.yaw;
				target_error.x = target.x - current_status.px;
				target_error.y = target.y - current_status.py;
				target_error.z = target.z - current_status.pz;
				target_error.yaw = target.yaw - current_status.psi;
				orb_copy(ORB_ID(actuator_motors), actuator_motors_sub_fd, &motor_commands);
				T_omega.T1 = motor_commands.control[0] * rotor_max_speed;
				T_omega.T2 = motor_commands.control[1] * rotor_max_speed;
				T_omega.T3 = motor_commands.control[2] * rotor_max_speed;
				T_omega.T4 = motor_commands.control[3] * rotor_max_speed;
				T_real.T1 = T_omega.T1 * T_omega.T1 * motor_constant;
				T_real.T2 = T_omega.T2 * T_omega.T2 * motor_constant;
				T_real.T3 = T_omega.T3 * T_omega.T3 * motor_constant;
				T_real.T4 = T_omega.T4 * T_omega.T4 * motor_constant;
				u_real.f = T_real.T1 + T_real.T2 + T_real.T3 + T_real.T4;
				u_real.taup = (-1 * T_real.T1 + T_real.T2 + T_real.T3 - T_real.T4) * b;
				u_real.tauq = (T_real.T1 - T_real.T2 + T_real.T3 - T_real.T4) * b;
				u_real.taur = (T_real.T1 + T_real.T2 - T_real.T3 - T_real.T4) * c;
			}
			
		}
		PX4_INFO("current pos: %.2f, %.2f, %.2f", (double)current_status.px, (double)current_status.py, (double)current_status.pz);
		//PX4_INFO("current vel: %6.4f, %6.4f, %6.4f", (double)current_status.vx, (double)current_status.vy, (double)current_status.vz);	
		PX4_INFO("current att: %.2f, %.2f, %.2f", (double)current_status.phi, (double)current_status.theta, (double)current_status.psi);
		//PX4_INFO("att in q: %.4f, %.4f, %.4f, %.4f", (double)current_att.q[0], (double)current_att.q[1], (double)current_att.q[2], (double)current_att.q[3]);
		//PX4_INFO("angular velocity: %6.4f, %6.4f, %6.4f", (double)current_status.p, (double)current_status.q, (double)current_status.r);
		PX4_INFO("target: %.2f, %.2f, %.2f, %.2f", (double)target.x, (double)target.y, (double)target.z, (double)target.yaw);
		PX4_INFO("target error: %.2f, %.2f, %.2f, %.2f", (double)target_error.x, (double)target_error.y, (double)target_error.z, (double)target_error.yaw);
		PX4_INFO("Resultant force and moment: %.4f, %.4f, %.4f, %.4f", (double)u_real.f, (double)u_real.taup, (double)u_real.tauq, (double)u_real.taur);
		//i++;
	}

	PX4_INFO("exiting");

	return 0;
}
