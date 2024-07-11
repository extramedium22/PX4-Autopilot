#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>



#include <px4_platform_common/log.h>

#include <uORB/topics/actuator_motors_efficiency.h>


#include <px4_platform_common/module.h>

#include <drivers/drv_hrt.h>

__EXPORT int motor_efficiency_publish_main(int argc, char *argv[]);

int motor_efficiency_publish_main(int argc, char *argv[])
{
	PX4_INFO("Warning! Motor failure!");
	PX4_INFO("Motor efficiency: %s, %s, %s, %s", argv[1], argv[2], argv[3], argv[4]);

	struct actuator_motors_efficiency_s motors_efficiency;
	motors_efficiency.efficiency[0] = atof(argv[1]);
	motors_efficiency.efficiency[1] = atof(argv[2]);
	motors_efficiency.efficiency[2] = atof(argv[3]);
	motors_efficiency.efficiency[3] = atof(argv[4]);
	motors_efficiency.efficiency[4] = 1.0;
	motors_efficiency.efficiency[5] = 1.0;
	motors_efficiency.efficiency[6] = 1.0;
	motors_efficiency.efficiency[7] = 1.0;
	motors_efficiency.efficiency[8] = 1.0;
	motors_efficiency.efficiency[9] = 1.0;
	motors_efficiency.efficiency[10] = 1.0;
	motors_efficiency.efficiency[11] = 1.0;


	//orb_advert_t actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs_sim), &actuator_outputs);
	orb_advert_t actuator_motors_efficiency_pub = orb_advertise(ORB_ID(actuator_motors_efficiency), &motors_efficiency);

	while (true) {
		//PX4_INFO("Motor Running!");
		orb_publish(ORB_ID(actuator_motors_efficiency), actuator_motors_efficiency_pub, &motors_efficiency);
		px4_usleep(20000);
	}

	PX4_INFO("Bye!");
	return 0;
}
