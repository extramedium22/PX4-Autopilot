#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
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

__EXPORT int uniftcctrl_main(int argc, char *argv[]);

//__EXPORT struct T_vec control_allocation(struct ud_vec ud);
__EXPORT struct T_vec control_allocation(float H[4][4], float WM[4][4], struct ud_vec uc, struct T_vec T0);

__EXPORT struct ud_vec desired_control_law(struct status current_status, struct pos_vec target, struct T_vec T_k1);

//__EXPORT struct ud_vec u_filter(struct ud_vec u_in, struct ud_vec u_last);




int uniftcctrl_main(int argc, char *argv[])
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

	struct pos_vec target = {
		.x = 0,
		.y = 0,
		.z = -2
	};

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
		.T4 = 0
	};

	struct T_vec T_eff = {
		.T1 = 1,
		.T2 = 1,
		.T3 = 1,
		.T4 = 1
	};

	float b = 0.0884;
	float c = 0.06;
	float H[4][4] = {{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1}};
	float W[4] = {1,20,20,0.1};
	float WM[4][4] = {{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1}};

	H[0][0] = W[0] + b * b * W[1] + b * b * W[2] + c * c * W[3];
	H[0][1] = W[0] - b * b * W[1] - b * b * W[2] + c * c * W[3];
	H[0][2] = W[0] - b * b * W[1] + b * b * W[2] - c * c * W[3];
	H[0][3] = W[0] + b * b * W[1] - b * b * W[2] - c * c * W[3];
	H[1][0] = W[0] - b * b * W[1] - b * b * W[2] + c * c * W[3];
	H[1][1] = W[0] + b * b * W[1] + b * b * W[2] + c * c * W[3];
	H[1][2] = W[0] + b * b * W[1] - b * b * W[2] - c * c * W[3];
	H[1][3] = W[0] - b * b * W[1] + b * b * W[2] - c * c * W[3];
	H[2][0] = W[0] - b * b * W[1] + b * b * W[2] - c * c * W[3];
	H[2][1] = W[0] + b * b * W[1] - b * b * W[2] - c * c * W[3];
	H[2][2] = W[0] + b * b * W[1] + b * b * W[2] + c * c * W[3];
	H[2][3] = W[0] - b * b * W[1] - b * b * W[2] + c * c * W[3];
	H[3][0] = W[0] + b * b * W[1] - b * b * W[2] - c * c * W[3];
	H[3][1] = W[0] - b * b * W[1] + b * b * W[2] - c * c * W[3];
	H[3][2] = W[0] - b * b * W[1] - b * b * W[2] + c * c * W[3];
	H[3][3] = W[0] + b * b * W[1] + b * b * W[2] + c * c * W[3];

	WM[0][0] = W[0];
	WM[0][1] = W[0];
	WM[0][2] = W[0];
	WM[0][3] = W[0];
	WM[1][0] = -1 * b * W[1];
	WM[1][1] = b * W[1];
	WM[1][2] = b * W[1];
	WM[1][3] = -1 * b * W[1];
	WM[2][0] = b * W[2];
	WM[2][1] = -1 * b * W[2];
	WM[2][2] = b * W[2];
	WM[2][3] = -1 * b * W[2];
	WM[3][0] = c * W[3];
	WM[3][1] = c * W[3];
	WM[3][2] = -1 * c * W[3];
	WM[3][3] = -1 * c * W[3];


	float m = 0.72;
	float g = 9.8;
	struct T_vec T0 = {
		.T1 = 0,
		.T2 = 0,
		.T3 = 0,
		.T4 = 0
	};
	T0.T1 = (float)0.25 * m * g;
	T0.T2 = (float)0.25 * m * g;
	T0.T3 = (float)0.25 * m * g;
	T0.T4 = (float)0.25 * m * g;

	PX4_INFO("Hello Sky!");

	
	int pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	int att_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	int attvel_sub_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	int target_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position_target));
	int motor_eff_sub_fd = orb_subscribe(ORB_ID(actuator_motors_efficiency));

	

	float q0 = 1;
	float q1 = 0;
	float q2 = 0;
	float q3 = 0;

	float mc = 0.0000085;

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
	struct vehicle_local_position_target_s received_order;
	struct actuator_motors_efficiency_s motor_eff;

	struct actuator_motors_s rotor_commands;

	int error_counter = 0;
	//int i = 0;

	orb_advert_t rotor_commands_pub = orb_advertise(ORB_ID(actuator_motors), &rotor_commands);

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
				orb_copy(ORB_ID(vehicle_local_position_target), target_sub_fd, &received_order);
				target.x = received_order.x;
				target.y = received_order.y;
				target.z = received_order.z;
				//target.x = 100.0;
				//target.y = 100.0;
				//target.z = -2.0;
				orb_copy(ORB_ID(actuator_motors_efficiency), motor_eff_sub_fd, &motor_eff);
				T_eff.T1 = motor_eff.efficiency[0];
				T_eff.T2 = motor_eff.efficiency[1];
				T_eff.T3 = motor_eff.efficiency[2];
				T_eff.T4 = motor_eff.efficiency[3];

				uc = desired_control_law(current_status,target,T_desired);
				/*
				uc = u_filter(uc, u_last);
				u_last.f = uc.f;
				u_last.taop = uc.taop;
				u_last.taoq = uc.taoq;
				u_last.taor = uc.taor;
				*/

				//T_desired = control_allocation(uc);
				T_desired = control_allocation(H, WM, uc, T0);

				T0.T1 = T_desired.T1;
				T0.T2 = T_desired.T2;
				T0.T3 = T_desired.T3;
				T0.T4 = T_desired.T4;


				//hrt_abstime _timestamp_sample;

				//rotor_commands.timestamp = hrt_absolute_time();
				//rotor_commands.timestamp_sample = 1.0;
				//rotor_commands.reversible_flags = 1.0;

				rotor_commands.control[0] = ((float)0.001 * ((float)sqrt(T_desired.T1/mc))) * (float)T_eff.T1;
				rotor_commands.control[1] = ((float)0.001 * ((float)sqrt(T_desired.T2/mc))) * (float)T_eff.T2;
				rotor_commands.control[2] = ((float)0.001 * ((float)sqrt(T_desired.T3/mc))) * (float)T_eff.T3;
				rotor_commands.control[3] = ((float)0.001 * ((float)sqrt(T_desired.T4/mc))) * (float)T_eff.T4;
				rotor_commands.control[4] = 0.0;
				rotor_commands.control[5] = 0.0;
				rotor_commands.control[6] = 0.0;
				rotor_commands.control[7] = 0.0;
				rotor_commands.control[8] = 0.0;
				rotor_commands.control[9] = 0.0;
				rotor_commands.control[10] = 0.0;
				rotor_commands.control[11] = 0.0;

				

				//PX4_INFO("Motor Running!");
				orb_publish(ORB_ID(actuator_motors), rotor_commands_pub, &rotor_commands);
			}
			
		}
		//PX4_INFO("current pos: %.2f, %.2f, %.2f", (double)current_status.px, (double)current_status.py, (double)current_status.pz);		
		//PX4_INFO("current att: %.2f, %.2f, %.2f", (double)current_status.phi, (double)current_status.theta, (double)current_status.psi);
		//PX4_INFO("att in q: %.4f, %.4f, %.4f, %.4f", (double)current_att.q[0], (double)current_att.q[1], (double)current_att.q[2], (double)current_att.q[3]);
		//PX4_INFO("angular velocity: %.4f, %.4f, %.4f", (double)current_status.p, (double)current_status.q, (double)current_status.r);
		//PX4_INFO("heading to: %.2f, %.2f, %.2f", (double)target.x, (double)target.y, (double)target.z);
		//i++;
	}

	PX4_INFO("exiting");

	return 0;
}

struct ud_vec desired_control_law(struct status current_status, struct pos_vec target, struct T_vec T_k1)
{
	struct ud_vec ud;
	struct ud_vec d;
	struct ud_vec uc;

	float m = 0.72;
	float g = 9.8;
	float Jx = 0.0056;
	float Jy = 0.0056;
	float Jz = 0.0104;

	float b = 0.0884;
	float c = 0.06;

	float kp = 0.3;
	float kv = 0.1;
	float kvz = 1;
	//float a = 0.15;
	float a = 1;
	//float kn3 = 3;
	float kn3 = 1;
	//float kw = 14;
	float kw = 6;
	float kr;
	kr = kw * (float)0.3;
	//float epsilon = 0.06;
	float krotor = 5.0;
	float krotorz = 0.1;
	float kcp = 0.6;
	
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

	vxd = kp * (target.x - current_status.px);
	vyd = kp * (target.y - current_status.py);
	vzd = kp * (target.z - current_status.pz);

	PX4_INFO("vd = %.4f, %.4f, %.4f", (double)vxd, (double)vyd, (double)vzd);

	axd = kv * (vxd - current_status.vx);
	ayd = kv * (vyd - current_status.vy);
	azd = kvz * (vzd - current_status.vz);

	PX4_INFO("ad = %.4f, %.4f, %.4f", (double)axd, (double)ayd, (double)azd);

	anorm = sqrt(axd * axd + ayd * ayd + azd * azd);
	if (anorm > a)
	{
		axd = (a/anorm) * axd;
		ayd = (a/anorm) * ayd;
		azd = (a/anorm) * azd;
	}
	agnorm = sqrt(axd * axd + ayd * ayd + ((azd - g) * (azd - g)));
	n3xd = -1 * axd / agnorm;
	n3yd = -1 * ayd / agnorm;
	//n3zd = -1 * (azd - g) / agnorm;

	PX4_INFO("n3d = %.4f, %.4f", (double)n3xd, (double)n3yd);

	A = cos(current_status.psi) * cos(current_status.theta);
	B = sin(current_status.psi) * cos(current_status.theta);
	C = cos(current_status.psi) * sin(current_status.theta) * sin(current_status.phi) - sin(current_status.psi) * cos(current_status.phi);
	D = sin(current_status.psi) * sin(current_status.theta) * sin(current_status.phi) + cos(current_status.psi) * cos(current_status.phi);
	n3x = cos(current_status.psi) * sin(current_status.theta) * cos(current_status.phi) + sin(current_status.psi) * sin(current_status.phi);
	n3y = sin(current_status.psi) * sin(current_status.theta) * cos(current_status.phi) - cos(current_status.psi) * sin(current_status.phi);
	n3z = cos(current_status.theta) * cos(current_status.phi);
	dotA = C * current_status.r - current_status.q * n3x;
	dotB = D * current_status.r - current_status.q * n3y;
	dotC = -1 * A * current_status.r + current_status.p * n3x;
	dotD = -1 * B * current_status.r + current_status.p * n3y;

	sn3x = n3x - n3xd;
	sn3y = n3y - n3yd;
	//sn3z = n3z - n3zd;

	PX4_INFO("sn3 = %.4f, %.4f", (double)sn3x, (double)sn3y);

	pd = kn3 * (-1 * B * sn3x + A * sn3y);
	qd = kn3 * (-1 * D * sn3x + C * sn3y);
	sp = current_status.p - pd;
	sq = current_status.q - qd;

	PX4_INFO("omega_d = %.4f, %.4f", (double)pd, (double)qd);
	PX4_INFO("s2 = %.4f, %.4f", (double)sp, (double)sq);

	//ud.taop = Jx * (-1 * kw * sp + current_status.q * current_status.r * (Jz - Jy) / Jx - sn3x * (-1 * C + kn3 * dotB) - sn3y * (-1 * D - kn3 * dotA) - n3z * kn3 * current_status.p - kcp * (Jz - Jy) * current_status.r * sq / Jx);
	//ud.taoq = Jy * (-1 * kw * sq + current_status.p * current_status.r * (Jx - Jz) / Jy - sn3x * (A + kn3 * dotD) - sn3y * (B - kn3 * dotC) - n3z * kn3 * current_status.q - kcp * (Jx -Jz) * current_status.r * sp / Jy);

	ud.taop = Jx * (-1 * kw * sp + current_status.q * current_status.r * (Jz - Jy) / Jx - sn3x * (-1 * C + kn3 * dotB - n3z * kn3 * kn3 * B) - sn3y * (-1 * D - kn3 * dotA + n3z * kn3 * kn3 * A) - n3z * kn3 * sp - kcp * (Jz - Jy) * current_status.r * sq / Jx);
	ud.taoq = Jy * (-1 * kw * sq + current_status.p * current_status.r * (Jx - Jz) / Jy - sn3x * (A + kn3 * dotD - n3z * kn3 * kn3 * D) - sn3y * (B - kn3 * dotC + n3z * kn3 * kn3 * C) - n3z * kn3 * sq - kcp * (Jx -Jz) * current_status.r * sp / Jy);

	rd = 0;
	ud.taor = -1 * Jz * kr * (current_status.r - rd);

	ud.f = m * (kvz * (current_status.vz - vzd) + g) / n3z;
	PX4_INFO("ud: %.4f, %.4f, %.4f, %.4f",(double)ud.f, (double)ud.taop, (double)ud.taoq, (double)ud.taor);

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

	return uc;
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


struct T_vec control_allocation(float H[4][4], float WM[4][4], struct ud_vec uc, struct T_vec T0)
{
	float T_max = 8.5;
	float T_min = 0;

	float f[4] = {1, 1, 1, 1};
	f[0] = -1 * (uc.f * WM[0][0] + uc.taop * WM[1][0] + uc.taoq * WM[2][0] + uc.taor * WM[3][0]);
	f[1] = -1 * (uc.f * WM[0][1] + uc.taop * WM[1][1] + uc.taoq * WM[2][1] + uc.taor * WM[3][1]);
	f[2] = -1 * (uc.f * WM[0][2] + uc.taop * WM[1][2] + uc.taoq * WM[2][2] + uc.taor * WM[3][2]);
	f[3] = -1 * (uc.f * WM[0][3] + uc.taop * WM[1][3] + uc.taoq * WM[2][3] + uc.taor * WM[3][3]);


	float alpha = 0.01;
	float x[4] = {T0.T1, T0.T2, T0.T3, T0.T4};
	float p[4] = {0, 0, 0, 0};
	struct T_vec T_out = {
		.T1 = T0.T1,
		.T2 = T0.T2,
		.T3 = T0.T3,
		.T4 = T0.T4
	};

	for (int i = 1; i <= 1000; i++)
	{
		p[0] = -1 * (H[0][0] * x[0] + H[0][1] * x[1] + H[0][2] * x[2] +H[0][3] * x[3] + f[0]);
		p[1] = -1 * (H[1][0] * x[0] + H[1][1] * x[1] + H[1][2] * x[2] +H[1][3] * x[3] + f[1]);
		p[2] = -1 * (H[2][0] * x[0] + H[2][1] * x[1] + H[2][2] * x[2] +H[2][3] * x[3] + f[2]);
		p[3] = -1 * (H[3][0] * x[0] + H[3][1] * x[1] + H[3][2] * x[2] +H[3][3] * x[3] + f[3]);

		for (int j = 0; j < 4; j++)
		{
			x[j] = x[j] + alpha * p[j];
			if (x[j] > T_max)
			{
				x[j] = T_max;
			}
			else if (x[j] < T_min)
			{
				x[j] = T_min;
			}
			
		}
		//x[0] = x[0] + alpha * p[0];
		//x[1] = x[1] + alpha * p[1];
		//x[2] = x[2] + alpha * p[2];
		//x[3] = x[3] + alpha * p[3];
	}

	T_out.T1 = x[0];
	T_out.T2 = x[1];
	T_out.T3 = x[2];
	T_out.T4 = x[3];

	return T_out;
}

/*
struct ud_vec u_filter(struct ud_vec u_in, struct ud_vec u_last)
{
	struct ud_vec u_out;
	float epsilon = 0.6;
	u_out.f = u_last.f + epsilon * (u_in.f - u_last.f);
	u_out.taop = u_last.taop + epsilon * (u_in.taop - u_last.taop);
	u_out.taoq = u_last.taoq + epsilon * (u_in.taoq - u_last.taoq);
	u_out.taor = u_last.taor + epsilon * (u_in.taor - u_last.taor);
	return u_out;
}
*/
