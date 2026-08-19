/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#include "avs.h"
#include "rail_measure.h"
#include "regulator.h"

#include <tenstorrent/smc_msg.h>
#include <zephyr/drivers/misc/bh_fwtable.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(rail_measure);

static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));

/* The SerDes VRs are read over PMBus. READ_VOUT/READ_IOUT decoding lives in regulator.c. */
static bool read_serdes_vdd(float *voltage_mv, float *current_a)
{
	*voltage_mv = GetSerdesRailVoltage(SERDES_VDD_ADDR);
	*current_a = GetSerdesRailCurrent(SERDES_VDD_ADDR);
	return true;
}

static bool read_serdes_vddl(float *voltage_mv, float *current_a)
{
	*voltage_mv = GetSerdesRailVoltage(SERDES_VDDL_ADDR);
	*current_a = GetSerdesRailCurrent(SERDES_VDDL_ADDR);
	return true;
}

static bool read_serdes_vddh(float *voltage_mv, float *current_a)
{
	*voltage_mv = GetSerdesRailVoltage(SERDES_VDDH_ADDR);
	*current_a = GetSerdesRailCurrent(SERDES_VDDH_ADDR);
	return true;
}

/* VCOREM needs no PMBus decoding: the MAX20816 reports VOUT at a flat 0.5 mV/LSB, and the
 * AVS bus reports per-rail current directly in amps.
 */
static bool read_vcorem(float *voltage_mv, float *current_a)
{
	if (AVSReadCurrent(AVS_VCOREM_RAIL, current_a) != AVSOk) {
		return false;
	}

	*voltage_mv = get_vcorem();
	return true;
}

struct rail_desc {
	const char *name;
	bool (*read)(float *voltage_mv, float *current_a);
};

/* clang-format off */
static const struct rail_desc rails[TT_CHAR_RAIL_COUNT] = {
	[TT_CHAR_RAIL_SERDES_VDD]  = { .name = "serdes_vdd",  .read = read_serdes_vdd, },
	[TT_CHAR_RAIL_SERDES_VDDL] = { .name = "serdes_vddl", .read = read_serdes_vddl, },
	[TT_CHAR_RAIL_SERDES_VDDH] = { .name = "serdes_vddh", .read = read_serdes_vddh, },
	[TT_CHAR_RAIL_VCOREM]      = { .name = "vcorem",      .read = read_vcorem, },
};
/* clang-format on */

struct rail_sample {
	float voltage_mv;
	float current_a;
	bool valid;
};

static struct rail_sample samples[TT_CHAR_RAIL_COUNT];

/* Bitmask of TT_CHAR_RAIL_* that the host has asked for. Written by the message handler
 * thread, read by the DVFS work handler.
 */
static atomic_t enabled_rails;

/* Rail to start the round-robin scan from on the next tick. Only touched by the DVFS work
 * handler.
 */
static uint8_t next_rail;

/* The left chip of a p300 has no SerDes VDD regulator of its own; see
 * p300_left_regulators_config in regulator_config.c.
 */
static bool rail_present(uint8_t rail)
{
	return !(rail == TT_CHAR_RAIL_SERDES_VDD &&
		 tt_bh_fwtable_get_pcb_type(fwtable_dev) == PcbTypeP300 &&
		 tt_bh_fwtable_is_p300_left_chip());
}

uint8_t RailMeasureSetEnabled(uint8_t rail, uint8_t enable)
{
	if (rail >= TT_CHAR_RAIL_COUNT || enable > 1) {
		return 1;
	}

	if (!rail_present(rail)) {
		LOG_WRN("rail %s is not present on this board", rails[rail].name);
		return 1;
	}

	if (enable) {
		atomic_or(&enabled_rails, BIT(rail));
	} else {
		atomic_and(&enabled_rails, ~BIT(rail));
		samples[rail].valid = false;
	}

	LOG_INF("rail %s measurement %s", rails[rail].name, enable ? "enabled" : "disabled");
	return 0;
}

void RailMeasureUpdate(void)
{
	uint32_t mask = (uint32_t)atomic_get(&enabled_rails);

	if (mask == 0) {
		return;
	}

	for (uint8_t i = 0; i < TT_CHAR_RAIL_COUNT; i++) {
		uint8_t rail = (next_rail + i) % TT_CHAR_RAIL_COUNT;

		if ((mask & BIT(rail)) == 0) {
			continue;
		}

		next_rail = (rail + 1) % TT_CHAR_RAIL_COUNT;

#ifndef CONFIG_TT_BH_ARC_EMUL
		float voltage_mv;
		float current_a;

		if (!rails[rail].read(&voltage_mv, &current_a)) {
			return;
		}

		samples[rail].voltage_mv = voltage_mv;
		samples[rail].current_a = current_a;
#endif
		samples[rail].valid = true;
		return;
	}
}

bool RailMeasureGet(uint8_t rail, float *voltage_mv, float *current_a)
{
	if (rail >= TT_CHAR_RAIL_COUNT || !samples[rail].valid) {
		return false;
	}

	*voltage_mv = samples[rail].voltage_mv;
	*current_a = samples[rail].current_a;
	return true;
}
