/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file hpm_tsns.cpp
 *
 * Driver for the HPM6E80 on-die temperature sensor (TSNS).
 *
 * The block sits at HPM_TSNS_BASE and needs its AHB clock (clock_tsns) to be
 * enabled; the temperature is then available in TSNS_Type.T scaled by 1/256
 * degrees Celsius.  The die temperature is published through debug_key_value:
 * the logger records that topic and MAVLink forwards it to the ground station
 * as NAMED_VALUE_FLOAT with the name "SOC_TEMP".  TSNS also tracks the minimum
 * and maximum sampled temperature; both are reported by "status".
 */

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>

#include "hpm_clock_drv.h"
#include "hpm_otp_table.h"
#include "hpm_soc.h"
#include "hpm_tsns_drv.h"

namespace
{
constexpr const char kDebugKey[] = "SOC_TEMP";
constexpr uint32_t kIntervalUs = 1000000;   /* one sample per second */
constexpr float kTempScale = 256.0f;        /* TSNS_TEMP_SCALE */

/* Factory calibration of the sensor lives in OTP word 21: a 12-bit base offset
 * in bits [11:0], a 12-bit slope in bits [23:12] and one enable bit for each.
 * When both enable bits are set the TSNS hardware applies the correction to the
 * value it reports in TSNS_Type.T itself, so the driver only reads T - the point
 * of checking the word is to know whether that value can be trusted.
 */

struct TsnsCalibration {
	uint16_t base;
	uint16_t slope;
	bool base_enabled;
	bool slope_enabled;
	bool factory_calibrated() const { return base_enabled && slope_enabled; }
};

TsnsCalibration read_calibration()
{
	const uint32_t word = HPM_OTP->SHADOW[OTP_TABLE_TSNS_BASE_OFFSET];

	TsnsCalibration calibration{};
	calibration.base = OTP_TABLE_TSNS_BASE_GET(word);
	calibration.slope = OTP_TABLE_TSNS_SLOPE_GET(word);
	calibration.base_enabled = OTP_TABLE_TSNS_BASE_EN_GET(word);
	calibration.slope_enabled = OTP_TABLE_TSNS_SLOPE_EN_GET(word);

	return calibration;
}

bool read_temperature(float &temperature)
{
	/* The SDK's tsns_get_current_temp() spins until the valid bit is set, which
	 * would hang a caller for good if the sensor stopped sampling; poll instead. */
	if (!tsns_temperature_is_valid(HPM_TSNS)) {
		return false;
	}

	temperature = (float)((int32_t)TSNS_T_T_GET(HPM_TSNS->T)) / kTempScale;
	return true;
}
}

class HpmTsns : public ModuleBase<HpmTsns>, public px4::ScheduledWorkItem
{
public:
	HpmTsns() :
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{}

	~HpmTsns() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();
	int print_status() override;

private:
	void Run() override;

	uORB::Publication<debug_key_value_s> _debug_key_value_pub{ORB_ID(debug_key_value)};

	TsnsCalibration _calibration{};
};

int HpmTsns::init()
{
	clock_add_to_group(clock_tsns, 0);
	clock_enable(clock_tsns);

	tsns_enable(HPM_TSNS);

	/* The block powers up in trigger mode, where a conversion only happens when
	 * asked for; continuous mode lets it sample on its own. */
	tsns_enable_continuous_mode(HPM_TSNS);

	_calibration = read_calibration();

	if (_calibration.factory_calibrated()) {
		PX4_INFO("TSNS: factory calibration from OTP (base 0x%03x, slope 0x%03x), applied by the sensor",
			 	 _calibration.base, _calibration.slope);

	} else {
		PX4_WARN("TSNS: OTP calibration not enabled (base 0x%03x/%d, slope 0x%03x/%d); reading is uncalibrated",
					 _calibration.base, (int)_calibration.base_enabled,
					 _calibration.slope, (int)_calibration.slope_enabled);
	}

	ScheduleOnInterval(kIntervalUs, kIntervalUs);

	return PX4_OK;
}

void HpmTsns::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	/* the first conversions take a moment; nothing to publish until valid */
	float temperature;

	if (!read_temperature(temperature)) {
		return;
	}

	debug_key_value_s report{};
	report.timestamp = hrt_absolute_time();
	memcpy(report.key, kDebugKey, sizeof(kDebugKey));
	report.value = temperature;

	_debug_key_value_pub.publish(report);
}

int HpmTsns::print_status()
{
	const float min = tsns_get_min_temp_raw(HPM_TSNS) / kTempScale;
	const float max = tsns_get_max_temp_raw(HPM_TSNS) / kTempScale;
	float current;

	if (read_temperature(current)) {
		PX4_INFO("TSNS: now %.2f degC, min %.2f, max %.2f, OTP cal %s (base 0x%03x slope 0x%03x)",
			  (double)current, (double)min, (double)max,
			  _calibration.factory_calibrated() ? "enabled" : "off",
			  _calibration.base, _calibration.slope);

	} else {
		PX4_WARN("TSNS: temperature not valid yet (min %.2f, max %.2f)", (double)min, (double)max);
	}

	return PX4_OK;
}

int HpmTsns::task_spawn(int argc, char *argv[])
{
	HpmTsns *instance = new HpmTsns();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -ENOMEM;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init() != PX4_OK) {
		instance->ScheduleClear();
		delete instance;
		_object.store(nullptr);
		_task_id = -1;

		return PX4_ERROR;
	}

	return PX4_OK;
}

int HpmTsns::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HpmTsns::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the HPM6E80 on-die temperature sensor (TSNS).

The die temperature is published once per second through debug_key_value with
the key "SOC_TEMP"; the logger records it and MAVLink streams it to the ground
station as NAMED_VALUE_FLOAT.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hpm_tsns", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int hpm_tsns_main(int argc, char *argv[]);

int hpm_tsns_main(int argc, char *argv[])
{
	return HpmTsns::main(argc, argv);
}
