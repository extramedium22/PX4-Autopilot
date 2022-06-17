/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4flow.cpp
 * @author Dominik Honegger
 * @author Ban Siesta <bansiesta@gmail.com>
 *
 * Driver for the PX4FLOW module connected via I2C.
 */

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_optical_flow.h>

/* Configuration Constants */
#define I2C_FLOW_ADDRESS_DEFAULT    0x42	///< 7-bit address. 8-bit address is 0x84, range 0x42 - 0x49
#define I2C_FLOW_ADDRESS_MIN        0x42	///< 7-bit address.
#define I2C_FLOW_ADDRESS_MAX        0x49	///< 7-bit address.

/* PX4FLOW Registers addresses */
#define PX4FLOW_REG			0x16	///< Measure Register 22

#define PX4FLOW_CONVERSION_INTERVAL_DEFAULT 100000	///< in microseconds! = 10Hz
#define PX4FLOW_CONVERSION_INTERVAL_MIN      10000	///< in microseconds! = 100 Hz
#define PX4FLOW_CONVERSION_INTERVAL_MAX    1000000	///< in microseconds! = 1 Hz

#define PX4FLOW_I2C_MAX_BUS_SPEED	400000	///< 400 KHz maximum speed

#define PX4FLOW_MAX_DISTANCE 5.0f
#define PX4FLOW_MIN_DISTANCE 0.3f

#include "i2c_frame.h"

class PX4FLOW: public device::I2C, public I2CSPIDriver<PX4FLOW>
{
public:
	PX4FLOW(const I2CSPIDriverConfig &config);
	virtual ~PX4FLOW();

	static void print_usage();

	int init() override;

	void print_status();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void				RunImpl();
protected:
	int			probe() override;

private:

	uint8_t _sonar_rotation;
	bool				_sensor_ok{false};
	bool				_collect_phase{false};

	uORB::PublicationMulti<distance_sensor_s> _distance_sensor_topic{ORB_ID(distance_sensor)};
	uORB::PublicationMulti<sensor_optical_flow_s> _sensor_optical_flow_pub{ORB_ID(sensor_optical_flow)};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	enum Rotation       _sensor_rotation;
	float 				_sensor_min_range{0.0f};
	float 				_sensor_max_range{0.0f};
	float 				_sensor_max_flow_rate{0.0f};

	i2c_frame _frame;
	i2c_integral_frame _frame_integral;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int					probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void				start();

	int					measure();
	int					collect();

};

extern "C" __EXPORT int px4flow_main(int argc, char *argv[]);

PX4FLOW::PX4FLOW(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_sonar_rotation(config.rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_sensor_rotation(Rotation::ROTATION_NONE)
{
}

PX4FLOW::~PX4FLOW()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
PX4FLOW::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	start();

	return ret;
}

int
PX4FLOW::probe()
{
	uint8_t val[I2C_FRAME_SIZE] {};

	// to be sure this is not a ll40ls Lidar (which can also be on
	// 0x42) we check if a I2C_FRAME_SIZE byte transfer works from address
	// 0. The ll40ls gives an error for that, whereas the flow
	// happily returns some data
	if (transfer(nullptr, 0, &val[0], 22) != OK) {
		return -EIO;
	}

	// that worked, so start a measurement cycle
	return measure();
}

int
PX4FLOW::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = PX4FLOW_REG;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

int
PX4FLOW::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	uint8_t val[I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE] = { };

	perf_begin(_sample_perf);

	if (PX4FLOW_REG == 0x00) {
		ret = transfer(nullptr, 0, &val[0], I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE);
	}

	if (PX4FLOW_REG == 0x16) {
		ret = transfer(nullptr, 0, &val[0], I2C_INTEGRAL_FRAME_SIZE);
	}

	if (ret < 0) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	if (PX4FLOW_REG == 0) {
		memcpy(&_frame, val, I2C_FRAME_SIZE);
		memcpy(&_frame_integral, &(val[I2C_FRAME_SIZE]), I2C_INTEGRAL_FRAME_SIZE);
	}

	if (PX4FLOW_REG == 0x16) {
		memcpy(&_frame_integral, val, I2C_INTEGRAL_FRAME_SIZE);
	}


	DeviceId device_id;
	device_id.devid = get_device_id();
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_PX4FLOW;
	device_id.devid_s.address = get_i2c_address();

	sensor_optical_flow_s report{};

	report.timestamp_sample = hrt_absolute_time();
	report.device_id = device_id.devid;

	report.pixel_flow[0] = static_cast<float>(_frame_integral.pixel_flow_x_integral) / 10000.f; //convert to radians
	report.pixel_flow[1] = static_cast<float>(_frame_integral.pixel_flow_y_integral) / 10000.f; //convert to radians

	// report.ground_distance_m = static_cast<float>(_frame_integral.ground_distance) / 1000.f; //convert to meters

	report.integration_timespan_us = _frame_integral.integration_timespan; // microseconds

	report.quality = _frame_integral.qual; // 0:bad ; 255 max quality

	report.delta_angle_available = true;
	report.delta_angle[0] = static_cast<float>(_frame_integral.gyro_x_rate_integral) / 10000.0f; // convert to radians
	report.delta_angle[1] = static_cast<float>(_frame_integral.gyro_y_rate_integral) / 10000.0f; // convert to radians
	report.delta_angle[2] = static_cast<float>(_frame_integral.gyro_z_rate_integral) / 10000.0f; // convert to radians

	/* rotate measurements in yaw from sensor frame to body frame according to parameter SENS_FLOW_ROT */
	float zeroval = 0.0f;

	rotate_3f(_sensor_rotation, report.pixel_flow[0], report.pixel_flow[1], zeroval);
	rotate_3f(_sensor_rotation, report.delta_angle[0], report.delta_angle[1], report.delta_angle[2]);

	report.max_flow_rate = 2.5f;
	report.min_ground_distance = 0.7f;
	report.max_ground_distance = 3.f;

	report.timestamp = hrt_absolute_time();
	_sensor_optical_flow_pub.publish(report);

	/* publish to the distance_sensor topic as well */
	if (_distance_sensor_topic.get_instance() == 0) {
		distance_sensor_s distance_report{};
		distance_report.device_id = device_id.devid;
		distance_report.min_distance = PX4FLOW_MIN_DISTANCE;
		distance_report.max_distance = PX4FLOW_MAX_DISTANCE;
		distance_report.current_distance = static_cast<float>(_frame_integral.ground_distance) / 1000.f; // convert to meters
		distance_report.variance = 0.0f;
		distance_report.signal_quality = -1;
		distance_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		distance_report.orientation = _sonar_rotation;
		distance_report.timestamp = hrt_absolute_time();

		_distance_sensor_topic.publish(distance_report);
	}

	perf_end(_sample_perf);

	return PX4_OK;
}

void
PX4FLOW::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
PX4FLOW::RunImpl()
{
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* perform collection */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* restart the measurement state machine */
		start();
		return;
	}

	ScheduleDelayed(PX4FLOW_CONVERSION_INTERVAL_DEFAULT);
}

void
PX4FLOW::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
PX4FLOW::print_usage()
{
	PRINT_MODULE_USAGE_NAME("px4flow", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x42);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 35, "Rotation (default=downwards)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
px4flow_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = PX4FLOW;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = PX4FLOW_I2C_MAX_BUS_SPEED;
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.i2c_address = I2C_FLOW_ADDRESS_DEFAULT;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_FLOW_DEVTYPE_PX4FLOW);

	if (!strcmp(verb, "start")) {
		// px4flow can require more time to fully start and be accessible
		static constexpr uint64_t STARTUP_MIN_TIME_US = 6'000'000;
		const hrt_abstime time_now_us = hrt_absolute_time();

		if (time_now_us < STARTUP_MIN_TIME_US) {
			px4_usleep(STARTUP_MIN_TIME_US - time_now_us);
		}

		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
