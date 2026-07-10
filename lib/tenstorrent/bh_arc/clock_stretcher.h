/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TENSTORRENT_BH_ARC_CLOCK_STRETCHER_H_
#define TENSTORRENT_BH_ARC_CLOCK_STRETCHER_H_

#include <stdint.h>

/**
 * @brief Re-point the FLL droop detector's nominal RO count to a VCORE operating
 *        point using the calibrated fit, and (re-)enable the detector.
 *
 * Recomputes dd_fw_nominal_vdd from the stored RO-count-vs-VCORE fit and writes it;
 * the detector consumes the value immediately, so no disable/re-enable is needed and
 * the detector is never disarmed. Intended to be called from the DVFS path on each
 * VCORE change so the detector tracks the operating voltage without re-running the
 * boot-time characterisation sweep.
 *
 * Self-guarding: a no-op unless the clock stretcher was calibrated (i.e.
 * CONFIG_TT_BH_ARC_CLOCK_STRETCHER is enabled and clock_stretcher_init() produced a
 * valid fit). It is therefore safe to call unconditionally from the generic
 * voltage-change path -- before calibration it returns without touching the FLL,
 * rather than arming the detector against a zero nominal.
 *
 * @param vcore_mv Target core voltage in millivolts.
 */
void clock_stretcher_repoint(uint32_t vcore_mv);

#endif /* TENSTORRENT_BH_ARC_CLOCK_STRETCHER_H_ */
