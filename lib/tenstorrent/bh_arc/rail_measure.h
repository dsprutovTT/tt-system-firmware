/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RAIL_MEASURE_H
#define RAIL_MEASURE_H

#include <stdbool.h>
#include <stdint.h>

#include <tenstorrent/smc_msg.h>

/**
 * @brief Add or remove a rail from the DVFS measurement loop
 *
 * Backs @ref TT_SUB_MSG_SET_RAIL_MEASUREMENT. Only updates the enabled set; the rail is
 * sampled by the DVFS loop, not by this call.
 *
 * @param rail The rail to update, one of @ref char_rail_id
 * @param enable 0 to stop measuring the rail, 1 to start
 * @return 0 on success, 1 if the rail is unknown, not present on this board, or if
 *         @p enable is not 0 or 1
 */
uint8_t RailMeasureSetEnabled(uint8_t rail, uint8_t enable);

/**
 * @brief Sample the next enabled rail
 *
 * Called once per DVFS tick. Enabled rails are visited round-robin, one per tick, so the
 * PMBus cost of a tick does not grow with the number of enabled rails. Does nothing if no
 * rail is enabled.
 */
void RailMeasureUpdate(void);

/**
 * @brief Retrieve the most recent sample for a rail
 *
 * @param rail The rail to read, one of @ref char_rail_id
 * @param voltage_mv Filled with the rail voltage in mV
 * @param current_a Filled with the rail current in A
 * @return true if the rail is enabled and has been sampled at least once, false otherwise
 *         (in which case the outputs are left untouched)
 */
bool RailMeasureGet(uint8_t rail, float *voltage_mv, float *current_a);

/**
 * @brief Retrieve a dual-regulator sample (east + west)
 *
 * Only valid for rails that publish two sides (currently GDDR VDDA).
 *
 * @return true if the rail is enabled, valid, and dual; false otherwise
 */
bool RailMeasureGetDual(uint8_t rail, float *voltage_mv, float *current_a, float *voltage_mv_2,
			float *current_a_2);

/**
 * @brief Retrieve PMBus READ_POUT power for a rail (W)
 *
 * Currently populated for @ref TT_CHAR_RAIL_VCOREM only.
 */
bool RailMeasureGetPower(uint8_t rail, float *power_pout_w);

#endif
