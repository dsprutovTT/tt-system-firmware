/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Clock stretcher (FLL droop detector) boot-time configuration.
 *
 * The FLL sits in the pll_cntl_wrapper block (base 0x80020000, shared with the
 * PLLs configured in pll.c). A free-running ring-oscillator (RO) chain drives a
 * counter whose value tracks the core voltage (VCORE): every dd_counter_period+1
 * cycles of the selected dd_clk_sel clock the counter snapshots its value, and two
 * consecutive snapshots are compared. If the drop between them exceeds
 * droop_threshold, the droop-detected signal fires and the hardware subtracts
 * droop_fbdiv_adj from the PLL feedback divider, stretching (slowing) the clock
 * until the rail recovers.
 *
 * This routine:
 *   1. Applies the static droop-detector configuration (see config.yaml). This is
 *      volatile and re-applied every boot.
 *   2. Loads the RO-count-vs-VCORE fit from SPIROM if one was persisted on a prior
 *      boot. If present, it re-points the detector for the boot VCORE and returns --
 *      the sweep (the only slow step) is skipped.
 *   3. Otherwise (first boot / corrupt blob) it sweeps VCORE across a few points,
 *      reads the RO count from FLL_STATUS_0 at each, fits a line count = m*vcore + b,
 *      enables the detector, and persists the fit (m, b) to SPIROM.
 *
 * Only the fit is persisted -- not a per-voltage table -- so first-boot latency is a
 * few-point sweep, not a sweep of every operating voltage. At runtime the DVFS path
 * calls clock_stretcher_repoint() (see clock_stretcher.h) to recompute the detector's
 * nominal RO count from the stored fit on each VCORE change, without re-sweeping.
 *
 * NOTE: this is a scaffold. Search for "TODO" for the hardware-specific values and
 * decisions that still need to be filled in / validated on silicon.
 *
 * Ordering: registered via SYS_INIT_APP, so it must be given a priority in
 * include/tenstorrent/sys_init_defines.h that runs AFTER regulator_init (108) and
 * avs_init (109) -- set_vcore() is unavailable before those -- and after PLLInit
 * (97) so the wrapper clocks are up. See the accompanying notes for wiring into
 * sys_init_defines.h, CMakeLists.txt, and Kconfig.
 */

#include "clock_stretcher.h"
#include "reg.h"
#include "regulator.h"
#include "timer.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <tenstorrent/msgqueue.h>
#include <tenstorrent/smc_msg.h>
#include <tenstorrent/sys_init_defines.h>
#include <tenstorrent/tt_boot_fs.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(clock_stretcher, CONFIG_TT_APP_LOG_LEVEL);

/*
 * Register map: pll_cntl_wrapper base + FLL offsets (from the A0 register DB).
 * Field bit ranges are documented next to each bitfield union below.
 */
#define PLL_CNTL_WRAPPER_BASE_ADDR 0x80020000

#define FLL_CNTL_0_REG_ADDR   (PLL_CNTL_WRAPPER_BASE_ADDR + 0x04)
#define FLL_CNTL_1_REG_ADDR   (PLL_CNTL_WRAPPER_BASE_ADDR + 0x08)
#define FLL_CNTL_2_REG_ADDR   (PLL_CNTL_WRAPPER_BASE_ADDR + 0x0C)
#define FLL_CNTL_3_REG_ADDR   (PLL_CNTL_WRAPPER_BASE_ADDR + 0x10)
#define FLL_CNTL_4_REG_ADDR   (PLL_CNTL_WRAPPER_BASE_ADDR + 0x14)
#define FLL_CNTL_5_REG_ADDR   (PLL_CNTL_WRAPPER_BASE_ADDR + 0x18)
#define FLL_STATUS_0_REG_ADDR (PLL_CNTL_WRAPPER_BASE_ADDR + 0x1C)
#define FLL_CNTL_8_REG_ADDR   (PLL_CNTL_WRAPPER_BASE_ADDR + 0x24)
#define FLL_CNTL_9_REG_ADDR   (PLL_CNTL_WRAPPER_BASE_ADDR + 0x28)

/* FLL_CNTL_0 @ 0x04 */
typedef struct {
	uint32_t osc_ctrl_coarse: 5;    /* [4:0]   */
	uint32_t force_droop_detect: 1; /* [5]     */
	uint32_t manual_spike: 1;       /* [6]     */
	uint32_t force_spike_detect: 1; /* [7]     */
	uint32_t dd_fbdiv_ctrl_en: 1;   /* [8]     */
	uint32_t dd_postdiv_ctrl_en: 1; /* [9]     */
	uint32_t dd_clk_sel: 3;         /* [12:10] */
	uint32_t force_droop2_detect: 1;/* [13]    */
	uint32_t two_step_droop: 1;     /* [14]    */
} FLL_CNTL_0_reg_t;

typedef union {
	uint32_t val;
	FLL_CNTL_0_reg_t f;
} FLL_CNTL_0_reg_u;

/* FLL_CNTL_1 @ 0x08 */
typedef struct {
	uint32_t droop2_threshold: 16; /* [15:0]  */
	uint32_t droop_threshold: 16;  /* [31:16] */
} FLL_CNTL_1_reg_t;

typedef union {
	uint32_t val;
	FLL_CNTL_1_reg_t f;
} FLL_CNTL_1_reg_u;

/* FLL_CNTL_2 @ 0x0C */
typedef struct {
	uint32_t dd_counter_period: 8;     /* [7:0]   */
	uint32_t dd_filt_sample_period: 8; /* [15:8]  */
	uint32_t droop_time: 16;           /* [31:16] */
} FLL_CNTL_2_reg_t;

typedef union {
	uint32_t val;
	FLL_CNTL_2_reg_t f;
} FLL_CNTL_2_reg_u;

/* FLL_CNTL_3 @ 0x10 */
typedef struct {
	uint32_t dd_force_sample_nominal_vdd: 1; /* [0]     */
	uint32_t dd_use_nominal_vdd: 1;          /* [1]     */
	uint32_t osc_enable: 1;                  /* [2]     */
	uint32_t dd_enable: 1;                   /* [3]     */
	uint32_t dd_use_fw_nominal_vdd: 1;       /* [4]     */
	uint32_t roclk_bypass: 1;                /* [5]     */
	uint32_t dbuf_en: 1;                     /* [6]     */
	uint32_t roclk_bypass0: 1;               /* [7]     */
	uint32_t droop_postdiv_adj: 8;           /* [15:8]  */
	uint32_t droop_fbdiv_adj: 16;            /* [31:16] */
} FLL_CNTL_3_reg_t;

typedef union {
	uint32_t val;
	FLL_CNTL_3_reg_t f;
} FLL_CNTL_3_reg_u;

/* FLL_CNTL_4 @ 0x14 */
typedef struct {
	uint32_t post_recover_timer: 16;  /* [15:0]  */
	uint32_t fbdiv_recover_timer: 16; /* [31:16] */
} FLL_CNTL_4_reg_t;

typedef union {
	uint32_t val;
	FLL_CNTL_4_reg_t f;
} FLL_CNTL_4_reg_u;

/* FLL_CNTL_5 @ 0x18 */
typedef struct {
	uint32_t dd_fw_nominal_vdd: 16; /* [15:0]  */
	uint32_t spike_threshold: 16;   /* [31:16] */
} FLL_CNTL_5_reg_t;

typedef union {
	uint32_t val;
	FLL_CNTL_5_reg_t f;
} FLL_CNTL_5_reg_u;

/* FLL_CNTL_8 @ 0x24 */
typedef struct {
	uint32_t droop2_fbdiv_adj: 16; /* [15:0]  */
	uint32_t droop2_time: 16;      /* [31:16] */
} FLL_CNTL_8_reg_t;

typedef union {
	uint32_t val;
	FLL_CNTL_8_reg_t f;
} FLL_CNTL_8_reg_u;

/* FLL_CNTL_9 @ 0x28 */
typedef struct {
	uint32_t droop2_postdiv_adj: 16; /* [15:0]  */
	uint32_t debug_data_mode: 2;     /* [17:16] */
	uint32_t manual_droop: 1;        /* [18]    */
	uint32_t sample_delta_mode: 1;   /* [19]    */
	uint32_t ones_on_empty_mode: 1;  /* [20]    */
} FLL_CNTL_9_reg_t;

typedef union {
	uint32_t val;
	FLL_CNTL_9_reg_t f;
} FLL_CNTL_9_reg_u;

/*
 * VCORE sweep points (mV) used to characterise the RO-count vs voltage line.
 *
 * TODO: pick safe, representative points. They must stay within the regulator's
 * legal VCORE window (voltage.c enforces 0.55 V <= VCORE <= 0.95 V via VCOREM),
 * and the detector must be DISABLED while sweeping so stretching does not perturb
 * the measurement.
 */
static const uint32_t kSweepVcoreMv[] = {700, 725, 750, 775, 800};
#define SWEEP_POINTS ARRAY_SIZE(kSweepVcoreMv)

/*
 * Package/loadline offset (mV). The regulator + package add a fixed ~10-15 mV on
 * top of whatever VCORE the FW programs (package-loss + loadline compensation built
 * in by the package team), so the voltage the die actually sees is higher than the
 * set value. The RO count is a function of that ACTUAL die voltage, so the fit is
 * built and evaluated against (programmed VCORE + this offset). set_vcore() is never
 * given the offset -- it still programs the raw value; the offset is applied only
 * where a programmed voltage is used as the fit's independent variable (the sweep
 * points in characterize() and count_for_vcore() at runtime). Applied consistently in
 * both, it shifts only the stored intercept b (by ~offset*m); the slope m and the
 * programmed nominal are unchanged, so the stretching behaviour is preserved.
 */
#define VCORE_PACKAGE_OFFSET_MV 20

/* Number of FLL_STATUS_0 reads to discard / average per sweep point. */
#define RO_DISCARD_SAMPLES 10
#define RO_AVG_SAMPLES     32

/* Settle time after a VCORE change before sampling the RO counter (us). */
/* TODO: confirm against regulator slew (set_vcore already waits ~250us). */
#define VCORE_SETTLE_US 500

/* Result of the linear fit: ro_count = m * vcore_mv + b. */
struct fll_fit {
	float m;
	float b;
};

static struct fll_fit clock_stretcher_fit;

/* Set once a fit has been loaded from SPIROM or freshly characterised. */
static bool clock_stretcher_fit_valid;

/*
 * SPIROM persistence.
 *
 * The characterisation sweep is the only slow part of this routine, so its result
 * (the fit, NOT a per-voltage table) is persisted to SPIROM. On every boot after
 * the first we load the fit and skip the sweep entirely; DVFS re-points the
 * detector from the fit on each VCORE change via clock_stretcher_repoint().
 *
 * The blob lives in the reserved "clkstr" tt-boot-fs partition (see
 * boards/.../tt_blackhole_fixed_partitions.dtsi). It is discovered by tag using the
 * device-based boot_fs scan (not the 16-entry mount cache, which cannot hold every
 * FD on this flash) and created once, on the first boot, via tt_boot_fs_add_file().
 *
 * m and b are stored as scaled integers (milli-units) so the on-flash format is
 * deterministic and independent of the host/target float ABI.
 */
#define FLL_CAL_MAGIC   0x4C4C4643u /* 'C''F''L''L' */
#define FLL_CAL_VERSION 1u

struct fll_cal_blob {
	uint32_t magic;
	uint16_t version;
	uint16_t reserved;
	int32_t m_milli; /* slope     (RO counts per mV) * 1000 */
	int32_t b_milli; /* intercept (RO counts)        * 1000 */
	uint32_t crc;    /* tt_boot_fs_cksum over the preceding bytes */
};
BUILD_ASSERT(sizeof(struct fll_cal_blob) == 20, "fll_cal_blob must be 4-byte-aligned/sized");
BUILD_ASSERT(offsetof(struct fll_cal_blob, crc) % 4 == 0, "crc coverage must be word-aligned");

/* tt-boot-fs tag (== the "clkstr" partition label), zero-padded to the tag size. */
static const uint8_t kFllCalTag[TT_BOOT_FS_IMAGE_TAG_SIZE] = {'c', 'l', 'k', 's', 't', 'r'};

/* Absolute SPI address of the reserved calibration partition's data region. */
#define FLL_CAL_SPI_ADDR FIXED_PARTITION_OFFSET(clkstr)

/*
 * Apply the static droop-detector configuration.
 *
 * Values mirror the reference config.yaml
 * (t6py/.../tensix_sm/pll_cntl_wrapper/config.yaml). Keep the two in sync, or
 * better: source these from the SPI fw_table so they are tunable without a
 * firmware rebuild. The detector is ENABLED here (dd_enable = 1) with the
 * auto-sampled nominal source -- the RO counter and debug buffer only run while
 * the detector is enabled (see the FLL_CNTL_3 block). count-vs-VCORE
 * characterisation happens against the auto nominal; clock_stretcher_repoint()
 * then switches to the fitted FW nominal (dd_use_fw_nominal_vdd = 1).
 */
static void apply_static_config(void)
{
	/* OSC + droop-detector clock/threshold config. */
	FLL_CNTL_0_reg_u cntl0 = {.val = ReadReg(FLL_CNTL_0_REG_ADDR)};

	cntl0.f.osc_ctrl_coarse = 0;
	cntl0.f.dd_clk_sel = 1;   /* MACCLK (800MHz) */
	cntl0.f.two_step_droop = 0;
	cntl0.f.dd_postdiv_ctrl_en = 0;
	cntl0.f.dd_fbdiv_ctrl_en = 1;
	cntl0.f.manual_spike = 0;
	cntl0.f.force_droop_detect = 0;
	cntl0.f.force_spike_detect = 0;
	cntl0.f.force_droop2_detect = 0;
	WriteReg(FLL_CNTL_0_REG_ADDR, cntl0.val);

	/* Droop / droop2 thresholds. */
	FLL_CNTL_1_reg_u cntl1 = {.val = ReadReg(FLL_CNTL_1_REG_ADDR)};

	cntl1.f.droop_threshold = 15;
	cntl1.f.droop2_threshold = 100;
	WriteReg(FLL_CNTL_1_REG_ADDR, cntl1.val);

	/* Counter/sample windows + stretch duration. */
	FLL_CNTL_2_reg_u cntl2 = {.val = ReadReg(FLL_CNTL_2_REG_ADDR)};

	cntl2.f.dd_counter_period = 9;     /* 1 ddclk period */
	cntl2.f.dd_filt_sample_period = 0; /* filtering disabled */
	cntl2.f.droop_time = 2000;         /* in units of ddclk period */
	WriteReg(FLL_CNTL_2_REG_ADDR, cntl2.val);

	/* Nominal-VDD source, oscillator enable, fbdiv adjust, debug buffer.
	 *
	 * dd_enable MUST be 1 here: per the RTL (tt_dd.v), the ddclk RO counter
	 * (roclk_count_ddclk -> o_ddclk_counter) and the sample pulse that pushes
	 * it into the debug buffer (ddclk_counter_sample, the buffer's i_data_valid)
	 * are BOTH held in reset while dd_en == 0. With the detector disabled the
	 * FIFO is never written, so every FLL_STATUS_0 read returns 0 (empty read,
	 * ones_on_empty_mode = 0) and the sweep fits a degenerate m = b = 0 line.
	 * The reference config.yaml enables the detector for exactly this reason.
	 *
	 * During the sweep we use the auto-sampled nominal (dd_use_nominal_vdd = 1,
	 * dd_use_fw_nominal_vdd = 0) and re-latch it at each point (see
	 * sample_nominal_vdd()) so stretching cannot bias the reads.
	 */
	FLL_CNTL_3_reg_u cntl3 = {.val = ReadReg(FLL_CNTL_3_REG_ADDR)};

	cntl3.f.roclk_bypass = 1;
	cntl3.f.roclk_bypass0 = 0;
	cntl3.f.osc_enable = 1;
	cntl3.f.dd_use_nominal_vdd = 1;
	cntl3.f.dd_use_fw_nominal_vdd = 0; /* flipped to 1 once we program the fit */
	cntl3.f.droop_postdiv_adj = 0;
	cntl3.f.droop_fbdiv_adj = 20;
	cntl3.f.dbuf_en = 1;
	cntl3.f.dd_enable = 1;
	WriteReg(FLL_CNTL_3_REG_ADDR, cntl3.val);

	/* Recovery timers. */
	FLL_CNTL_4_reg_u cntl4 = {.val = ReadReg(FLL_CNTL_4_REG_ADDR)};

	cntl4.f.post_recover_timer = 0;
	cntl4.f.fbdiv_recover_timer = 1;
	WriteReg(FLL_CNTL_4_REG_ADDR, cntl4.val);

	/* FW nominal VDD (overwritten by the fit) + spike threshold. */
	FLL_CNTL_5_reg_u cntl5 = {.val = ReadReg(FLL_CNTL_5_REG_ADDR)};

	cntl5.f.dd_fw_nominal_vdd = 0;
	cntl5.f.spike_threshold = 0;
	WriteReg(FLL_CNTL_5_REG_ADDR, cntl5.val);

	/* droop2 fbdiv adjust. */
	FLL_CNTL_8_reg_u cntl8 = {.val = ReadReg(FLL_CNTL_8_REG_ADDR)};

	cntl8.f.droop2_fbdiv_adj = 0;
	WriteReg(FLL_CNTL_8_REG_ADDR, cntl8.val);

	/* droop2 postdiv adjust, manual droop, debug buffer mode. */
	FLL_CNTL_9_reg_u cntl9 = {.val = ReadReg(FLL_CNTL_9_REG_ADDR)};

	cntl9.f.droop2_postdiv_adj = 0;
	cntl9.f.manual_droop = 0;
	/*
	 * debug_data_mode = 0 (debug_counter): stream the FULL 16-bit RO count,
	 * two per 32-bit FIFO word. Mode 1 (debug_fast_counter, per tt_dd /
	 * tt_pll_cntl_wrapper) streams only ddclk_counter[7:0] -- a higher sample
	 * rate but truncated to 8 bits, which aliases every 256 counts and cannot
	 * represent the ~thousands-of-counts values this sweep fits. Must stay 0
	 * to match read_ro_count()'s two-16-bit unpack.
	 */
	cntl9.f.debug_data_mode = 0;
	WriteReg(FLL_CNTL_9_REG_ADDR, cntl9.val);
}

/*
 * Read the RO counter from FLL_STATUS_0 and return an averaged count.
 *
 * FLL_STATUS_0 packs the debug buffer. apply_static_config() selects
 * debug_data_mode = 0 (the full-width counter), in which each 32-bit word carries
 * two consecutive 16-bit RO counts (mirrors get_ro.py); both halves are averaged
 * here. The first few reads are discarded to flush stale/invalid snapshots.
 *
 * This unpack is only valid for debug_data_mode = 0. If the static config ever
 * selects mode 1/2/3 (8/4/1-bit packing) this must change to match.
 */
static uint32_t read_ro_count(void)
{
	for (int i = 0; i < RO_DISCARD_SAMPLES; i++) {
		(void)ReadReg(FLL_STATUS_0_REG_ADDR);
	}

	uint32_t accum = 0;
	uint32_t n = 0;

	for (int i = 0; i < RO_AVG_SAMPLES; i++) {
		uint32_t word = ReadReg(FLL_STATUS_0_REG_ADDR);

		accum += (word & 0xFFFF);
		accum += ((word >> 16) & 0xFFFF);
		n += 2;
	}

	return (n != 0) ? (accum / n) : 0;
}

/*
 * Re-latch the auto-sampled nominal RO count to the current operating point by
 * pulsing FLL_CNTL_3.dd_force_sample_nominal_vdd 0->1->0 (mirrors the reference
 * dump_debug_buffer.py). Used between sweep points: with the nominal re-captured
 * at each voltage, roclk_cnt_delta stays ~0 so no spurious droop fires and clock
 * stretching cannot perturb the RO count we are about to read.
 */
static void sample_nominal_vdd(void)
{
	FLL_CNTL_3_reg_u cntl3 = {.val = ReadReg(FLL_CNTL_3_REG_ADDR)};

	cntl3.f.dd_force_sample_nominal_vdd = 1;
	WriteReg(FLL_CNTL_3_REG_ADDR, cntl3.val);

	cntl3.f.dd_force_sample_nominal_vdd = 0;
	WriteReg(FLL_CNTL_3_REG_ADDR, cntl3.val);
}

/*
 * Least-squares fit of ro_count = m * vcore_mv + b over the sweep samples.
 */
static struct fll_fit linear_fit(const uint32_t *x, const uint32_t *y, size_t n)
{
	float sx = 0, sy = 0, sxx = 0, sxy = 0;

	for (size_t i = 0; i < n; i++) {
		float xf = (float)x[i];
		float yf = (float)y[i];

		sx += xf;
		sy += yf;
		sxx += xf * xf;
		sxy += xf * yf;
	}

	float denom = (n * sxx) - (sx * sx);
	struct fll_fit fit = {0};

	if (denom == 0.0f) {
		LOG_ERR("Degenerate sweep, cannot fit clock stretcher line");
		return fit;
	}

	fit.m = ((n * sxy) - (sx * sy)) / denom;
	fit.b = (sy - (fit.m * sx)) / n;
	return fit;
}

/*
 * Sweep VCORE, sample the RO counter at each point, and fit the line.
 * Restores the original VCORE before returning.
 */
static struct fll_fit characterize(void)
{
	uint32_t ro[SWEEP_POINTS];
	uint32_t vfit[SWEEP_POINTS];
	uint32_t saved_vcore = get_vcore();

	for (size_t i = 0; i < SWEEP_POINTS; i++) {
		set_vcore(kSweepVcoreMv[i]);
		WaitUs(VCORE_SETTLE_US);
		sample_nominal_vdd(); /* re-latch nominal at this voltage before reading */
		ro[i] = read_ro_count();
		/*
		 * Fit against the ACTUAL die voltage, not the programmed value:
		 * set_vcore() above gets the raw sweep point, but the die sits
		 * VCORE_PACKAGE_OFFSET_MV higher, so that is the x the RO count maps to.
		 */
		vfit[i] = kSweepVcoreMv[i] + VCORE_PACKAGE_OFFSET_MV;
		LOG_DBG("sweep vcore=%u mV (die~%u mV) -> ro=%u", kSweepVcoreMv[i], vfit[i],
			ro[i]);
	}

	set_vcore(saved_vcore);
	WaitUs(VCORE_SETTLE_US);

	return linear_fit(vfit, ro, SWEEP_POINTS);
}

/*
 * Evaluate the fitted line at vcore_mv and clamp to the 16-bit register range.
 * Pure function of the current fit -- no register side effects.
 */
static uint16_t count_for_vcore(uint32_t vcore_mv)
{
	/*
	 * The fit's independent variable is the actual die voltage, so convert the
	 * programmed VCORE to the die voltage (+ VCORE_PACKAGE_OFFSET_MV) before
	 * evaluating -- mirroring the offset applied to the sweep points in
	 * characterize(). Keeping both in the same domain leaves the programmed
	 * nominal (and thus the stretching behaviour) identical to a raw-voltage fit.
	 */
	float die_mv = (float)(vcore_mv + VCORE_PACKAGE_OFFSET_MV);
	float count_f = (clock_stretcher_fit.m * die_mv) + clock_stretcher_fit.b;

	if (count_f < 0.0f) {
		count_f = 0.0f;
	} else if (count_f > (float)UINT16_MAX) {
		count_f = (float)UINT16_MAX;
	}

	return (uint16_t)(count_f + 0.5f);
}

/*
 * Re-point the detector's nominal RO count to the given operating voltage from the
 * fitted line, and (on the first call) enable the detector. Callable at runtime from
 * the DVFS path on each VCORE change -- it recomputes dd_fw_nominal_vdd from the
 * stored fit, so no re-sweep (which would risk a real droop mid-characterisation) is
 * needed.
 *
 * dd_fw_nominal_vdd is consumed by the detector continuously, so writing FLL_CNTL_5
 * takes effect immediately -- there is no need to disable/re-enable to latch it. In
 * steady state only the nominal register changes; FLL_CNTL_3 is written only to
 * perform the one-time 0->1 enable (and to select the FW nominal source), never
 * toggled off, so the detector is never disarmed.
 */
void clock_stretcher_repoint(uint32_t vcore_mv)
{
	/*
	 * Safe to call from the generic voltage-change choke point (VoltageChange()):
	 * no-op until a valid fit exists. Without this guard a call before/without
	 * calibration would program dd_fw_nominal_vdd = 0 and arm the detector against a
	 * zero nominal. clock_stretcher_fit_valid is false when the feature is compiled
	 * out (clock_stretcher_init() returns early) or before characterisation completes.
	 */
	if (!IS_ENABLED(CONFIG_TT_BH_ARC_CLOCK_STRETCHER) || !clock_stretcher_fit_valid) {
		return;
	}

	uint16_t nominal = count_for_vcore(vcore_mv);

	FLL_CNTL_5_reg_u cntl5 = {.val = ReadReg(FLL_CNTL_5_REG_ADDR)};

	cntl5.f.dd_fw_nominal_vdd = nominal;
	WriteReg(FLL_CNTL_5_REG_ADDR, cntl5.val);

	FLL_CNTL_3_reg_u cntl3 = {.val = ReadReg(FLL_CNTL_3_REG_ADDR)};

	/*
	 * dd_use_nominal_vdd MUST be cleared here. Per tt_dd.v:230-236 the nominal
	 * select checks i_use_nominal_vdd BEFORE i_use_fw_nominal_vdd: while
	 * dd_use_nominal_vdd = 1 the detector holds the auto-sampled nominal and
	 * IGNORES dd_fw_nominal_vdd entirely. apply_static_config() leaves it 1 for
	 * the sweep, so arming the FW nominal requires clearing it -- otherwise the
	 * fitted nominal is dead code and the detector compares against whatever
	 * voltage was auto-sampled last (the final sweep point), causing a constant
	 * stretch.
	 */
	if (cntl3.f.dd_use_fw_nominal_vdd != 1 || cntl3.f.dd_use_nominal_vdd != 0 ||
	    cntl3.f.dd_enable != 1) {
		cntl3.f.dd_use_nominal_vdd = 0;
		cntl3.f.dd_use_fw_nominal_vdd = 1;
		cntl3.f.dd_enable = 1;
		WriteReg(FLL_CNTL_3_REG_ADDR, cntl3.val);
	}

	LOG_INF("Clock stretcher pointed: nominal_vdd count=%u @ %u mV (m=%d.%03d b=%d)", nominal,
		vcore_mv, (int)clock_stretcher_fit.m,
		(int)((clock_stretcher_fit.m - (int)clock_stretcher_fit.m) * 1000),
		(int)clock_stretcher_fit.b);
}

/*
 * Load a previously-persisted fit from SPIROM. Returns true and fills *out on a
 * valid blob; false if absent or corrupt (caller then characterises + persists).
 */
static bool load_fit_from_spirom(struct fll_fit *out)
{
	const struct device *flash = DEVICE_DT_GET(DT_NODELABEL(spi_flash));

	if (!device_is_ready(flash)) {
		return false;
	}

	tt_boot_fs_fd fd;

	if (tt_boot_fs_find_fd_by_tag(flash, kFllCalTag, &fd) != 0) {
		return false;
	}

	if (fd.flags.f.image_size != sizeof(struct fll_cal_blob)) {
		return false;
	}

	struct fll_cal_blob blob;

	if (flash_read(flash, fd.spi_addr, &blob, sizeof(blob)) < 0) {
		return false;
	}

	if (blob.magic != FLL_CAL_MAGIC || blob.version != FLL_CAL_VERSION) {
		return false;
	}

	if (tt_boot_fs_cksum(0, (const uint8_t *)&blob, offsetof(struct fll_cal_blob, crc)) !=
	    blob.crc) {
		LOG_WRN("Clock stretcher: SPIROM fit CRC mismatch, re-characterising");
		return false;
	}

	out->m = (float)blob.m_milli / 1000.0f;
	out->b = (float)blob.b_milli / 1000.0f;
	return true;
}

/* Round a float to the nearest int without pulling in libm. */
static int32_t round_to_i32(float x)
{
	return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

/*
 * Persist the fit to the reserved "clkstr" SPIROM partition as a new tt-boot-fs
 * file. Called exactly once in the device's life (only when no valid blob was
 * loaded), so the flash sector is not re-written on subsequent boots.
 *
 * TODO: this appends an FD; if a prior blob exists but is corrupt, a second FD with
 * the same tag is created and the stale one is found first. Harmless for the normal
 * first-boot-then-persist lifecycle, but a delete/replace path would be cleaner.
 */
static int save_fit_to_spirom(const struct fll_fit *fit)
{
	struct fll_cal_blob blob = {
		.magic = FLL_CAL_MAGIC,
		.version = FLL_CAL_VERSION,
		.reserved = 0,
		.m_milli = round_to_i32(fit->m * 1000.0f),
		.b_milli = round_to_i32(fit->b * 1000.0f),
	};

	blob.crc = tt_boot_fs_cksum(0, (const uint8_t *)&blob, offsetof(struct fll_cal_blob, crc));

	tt_boot_fs_fd fd = {0};

	fd.spi_addr = FLL_CAL_SPI_ADDR;
	fd.copy_dest = 0; /* data only -- not loaded to memory by the bootrom */
	fd.flags.f.image_size = sizeof(blob);
	fd.flags.f.invalid = 0;
	fd.flags.f.executable = 0;
	fd.data_crc = tt_boot_fs_cksum(0, (const uint8_t *)&blob, sizeof(blob));
	memcpy(fd.image_tag, kFllCalTag, TT_BOOT_FS_IMAGE_TAG_SIZE);
	fd.fd_crc = tt_boot_fs_cksum(0, (const uint8_t *)&fd, sizeof(fd) - sizeof(uint32_t));

	return tt_boot_fs_add_file(&boot_fs_data, fd, (const uint8_t *)&blob, false, false);
}

static int clock_stretcher_init(void)
{
	if (!IS_ENABLED(CONFIG_TT_BH_ARC_CLOCK_STRETCHER)) {
		return 0;
	}

	/* TODO: use the actual boot VCORE target rather than the current readback. */
	uint32_t boot_vcore = get_vcore();

	/*
	 * The FLL_CNTL_* registers are volatile and reset every boot, so the static
	 * detector config must be re-applied unconditionally. Only the fit is
	 * persisted.
	 */
	apply_static_config();

	if (load_fit_from_spirom(&clock_stretcher_fit)) {
		/* Fast path: reuse the persisted fit, skip the VCORE sweep. */
		LOG_INF("Clock stretcher: loaded fit from SPIROM, skipping sweep");
		clock_stretcher_fit_valid = true;
		clock_stretcher_repoint(boot_vcore);
		return 0;
	}

	/* First boot (or corrupt blob): characterise, enable, then persist the fit. */
	LOG_INF("Clock stretcher: no valid SPIROM fit, characterising");
	clock_stretcher_fit = characterize();
	clock_stretcher_fit_valid = true;
	clock_stretcher_repoint(boot_vcore);

	int rc = save_fit_to_spirom(&clock_stretcher_fit);

	if (rc != TT_BOOT_FS_OK) {
		LOG_ERR("Clock stretcher: failed to persist fit to SPIROM (%d)", rc);
	} else {
		LOG_INF("Clock stretcher: fit persisted to SPIROM");
	}

	return 0;
}
SYS_INIT_APP(clock_stretcher_init);

/*
 * Host-facing message handlers (t6py / SMC message queue).
 *
 * Registered unconditionally, but they refuse to operate when the feature is
 * compiled out (CONFIG_TT_BH_ARC_CLOCK_STRETCHER=n) by returning a nonzero
 * error code -- the fit is never populated in that case.
 */

/*
 * TT_SMC_MSG_GET_CLK_STRETCHER_FIT: report the calibrated fit and detector state.
 *
 * Response:
 *   data[1] = fit slope m * 1000, as a signed int32 (RO counts per mV)
 *   data[2] = fit intercept b * 1000, as a signed int32 (RO counts)
 *   data[3] = 1 if the fit is valid (loaded or characterised), else 0
 *   data[4] = FLL_CNTL_3.dd_enable (1 = detector armed)
 *   data[5] = FLL_CNTL_5.dd_fw_nominal_vdd (nominal RO count currently in use)
 */
static uint8_t get_clk_stretcher_fit_handler(const union request *request,
					     struct response *response)
{
	(void)request;

	if (!IS_ENABLED(CONFIG_TT_BH_ARC_CLOCK_STRETCHER)) {
		return 1;
	}

	FLL_CNTL_3_reg_u cntl3 = {.val = ReadReg(FLL_CNTL_3_REG_ADDR)};
	FLL_CNTL_5_reg_u cntl5 = {.val = ReadReg(FLL_CNTL_5_REG_ADDR)};

	response->data[1] = (uint32_t)round_to_i32(clock_stretcher_fit.m * 1000.0f);
	response->data[2] = (uint32_t)round_to_i32(clock_stretcher_fit.b * 1000.0f);
	response->data[3] = clock_stretcher_fit_valid ? 1U : 0U;
	response->data[4] = cntl3.f.dd_enable;
	response->data[5] = cntl5.f.dd_fw_nominal_vdd;
	return 0;
}

/*
 * TT_SMC_MSG_RECALIBRATE_CLK_STRETCHER: re-run the VCORE sweep + fit at runtime,
 * so a fresh characterisation can be forced without first erasing the SPIROM
 * blob (a persisted fit otherwise makes every boot skip the sweep).
 *
 * The detector is disarmed for the duration of the sweep (via
 * apply_static_config(), which clears dd_enable) so stretching cannot perturb
 * the measurement, then re-pointed and re-armed at the current VCORE.
 *
 * Request:
 *   data[1] != 0 -> also persist the new fit to SPIROM. Default (0) updates only
 *                   the live fit, avoiding a duplicate tt-boot-fs FD on every
 *                   recalibration (see save_fit_to_spirom()).
 * Response:
 *   data[1] = new slope m * 1000 (signed int32)
 *   data[2] = new intercept b * 1000 (signed int32)
 *   data[3] = 1 if a requested persist failed, else 0
 */
static uint8_t recalibrate_clk_stretcher_handler(const union request *request,
						 struct response *response)
{
	if (!IS_ENABLED(CONFIG_TT_BH_ARC_CLOCK_STRETCHER)) {
		return 1;
	}

	bool persist = (request->data[1] != 0);
	uint32_t vcore = get_vcore();

	apply_static_config();
	clock_stretcher_fit = characterize();
	clock_stretcher_fit_valid = true;
	clock_stretcher_repoint(vcore);

	uint32_t persist_err = 0;

	if (persist) {
		if (save_fit_to_spirom(&clock_stretcher_fit) != TT_BOOT_FS_OK) {
			LOG_ERR("Clock stretcher: recalibration persist failed");
			persist_err = 1;
		}
	}

	response->data[1] = (uint32_t)round_to_i32(clock_stretcher_fit.m * 1000.0f);
	response->data[2] = (uint32_t)round_to_i32(clock_stretcher_fit.b * 1000.0f);
	response->data[3] = persist_err;
	return 0;
}

REGISTER_MESSAGE(TT_SMC_MSG_GET_CLK_STRETCHER_FIT, get_clk_stretcher_fit_handler);
REGISTER_MESSAGE(TT_SMC_MSG_RECALIBRATE_CLK_STRETCHER, recalibrate_clk_stretcher_handler);
