/***************************************************************************//**
 *   @file   iio_max42500.c
 *   @brief  Source file for the MAX42500 IIO Driver
 *   @author Aldrin Abacan (aldrin.abacan@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "no_os_alloc.h"
#include "no_os_error.h"
#include "no_os_units.h"
#include "no_os_util.h"
#include "max42500.h"
#include "iio_max42500.h"

#define REG_MIN			0x00
#define REG_MAX			0xFF

#define OV_MASK			NO_OS_GENMASK(7, 4)
#define UV_MASK			NO_OS_GENMASK(3, 0)

#define BSTU_MASK		NO_OS_BIT(0)
#define BSTO_MASK		NO_OS_BIT(1)
#define EN0_MASK		NO_OS_BIT(2)
#define EN1_MASK		NO_OS_BIT(3)
#define RST_MASK		NO_OS_BIT(4)
#define RSTF_MASK		NO_OS_BIT(5)
#define PAR_MASK		NO_OS_BIT(6)
#define CLKF_MASK		NO_OS_BIT(7)

#define VM1_MASK		NO_OS_BIT(0)
#define VM2_MASK		NO_OS_BIT(1)
#define VM3_MASK		NO_OS_BIT(2)
#define VM4_MASK		NO_OS_BIT(3)
#define VM5_MASK		NO_OS_BIT(4)
#define VM6_MASK		NO_OS_BIT(5)
#define VM7_MASK		NO_OS_BIT(6)
#define VMPD_MASK		NO_OS_BIT(7)

#define IN1_MASK		NO_OS_BIT(0)
#define IN2_MASK		NO_OS_BIT(1)
#define IN3_MASK		NO_OS_BIT(2)
#define IN4_MASK		NO_OS_BIT(3)
#define IN5_MASK		NO_OS_BIT(4)
#define IN6_MASK		NO_OS_BIT(5)
#define IN7_MASK		NO_OS_BIT(6)
#define PARM_MASK		NO_OS_BIT(7)

#define SRR_MASK		NO_OS_BIT(0)
#define FPSE_MASK		NO_OS_BIT(1)
#define DEN_MASK		NO_OS_BIT(2)
#define UEN_MASK		NO_OS_BIT(3)
#define NOTRD_MASK		NO_OS_BIT(4)

#define FDIV_MASK		NO_OS_GENMASK(2,0)
#define FPSEN1_MASK		NO_OS_BIT(3)
#define DVALM_MASK		NO_OS_BIT(4)
#define UVALM_MASK		NO_OS_BIT(5)
#define DVAL_MASK		NO_OS_BIT(6)
#define UVAL_MASK		NO_OS_BIT(7)

#define WDEXP_MASK		NO_OS_BIT(0)
#define WDUV_MASK		NO_OS_BIT(1)
#define LFSR_MASK		NO_OS_BIT(2)
#define OPEN_MASK		NO_OS_BIT(3)

#define WDIV_MASK		NO_OS_GENMASK(5,0)
#define SWW_MASK		NO_OS_BIT(6)

#define CLO_MASK		NO_OS_GENMASK(7,4)
#define OPN_MASK		NO_OS_GENMASK(3,0)

#define UD_MASK			NO_OS_GENMASK(2,0)
#define WDEN_MASK		NO_OS_BIT(3)

#define WDLOCK_MASK		NO_OS_BIT(0)

#define MR1_MASK		NO_OS_BIT(2)
#define RHLD_MASK		NO_OS_GENMASK(1,0)

#define WDT_BASE_TO 	200
#define WDT_BASE_TO_MIN 200
#define WDT_BASE_TO_MAX 12800
#define WDT_WINDOW_MAX	16
#define WDT_WINDOW_MIN	1

/* MAX42500 config2 register. */
enum max42500_config2 {
	MAX42500_BSTU,
	MAX42500_BSTO,
	MAX42500_EN0,
	MAX42500_EN1,
	MAX42500_RST,
	MAX42500_RSTF,
	MAX42500_PAR,
	MAX42500_CLKF,
};

/* MAX42500 fpsstat1 register. */
enum max42500_fpstat1 {
	MAX42500_SRR,
	MAX42500_FPSE,
	MAX42500_DEN,
	MAX42500_UEN,
	MAX42500_NOTRD
};

/* MAX42500 fpsconfig register. */
enum max42500_fpsconfig {
	MAX42500_UVAL,
	MAX42500_DVAL,
	MAX42500_UVALM,
	MAX42500_DVALM,
	MAX42500_FPSEN1,
	MAX42500_FDIV
};

/* MAX42500 wdtstat register. */
enum max42500_wdtstat {
	MAX42500_WDEXP,
	MAX42500_WDUV,
	MAX42500_LFSR,
	MAX42500_OPEN
};

/* MAX42500 wdtstat register. */
enum max42500_wdcdiv {
	MAX42500_SWW,
	MAX42500_WDIV
};

/* MAX42500 wdcfg1 register. */
enum max42500_wdcfg1 {
	MAX42500_CLO,
	MAX42500_OPN
};

/* MAX42500 wdcfg2 register. */
enum max42500_wdcfg2 {
	MAX42500_WDEN,
	MAX42500_1UD
};

/* MAX42500 rstctrl register. */
enum max42500_rstctrl {
	MAX42500_RHLD,
	MAX42500_MR1,
};
static const char *const max42500_comp_status[] = {
	"Low",
	"High",
};

static const char *const max42500_pin_state[] = {
	"OFF",
	"SLEEP",
	"ON"
};

static const char *const max42500_state[] = {
	"DISABLED",
	"ENABLED",
};

static const char *const max42500_fault_status[] = {
	"NO_FAULT",
	"FAULT",
};

static const char *const max42500_uen_states[] = {
	"EN0 low to high",
	"EN1 low to high",
};

static const char *const max42500_den_states[] = {
	"EN0 high to low",
	"EN1 high to low",
};

static const char *const max42500_uval_states[] = {
	"capture not completed",
	"capture completed",
};

static const char *const max42500_dval_states[] = {
	"capture not completed",
	"capture completed",
};

static const char *const max42500_fpsen1_states[] = {
	"EN0 transitions will trigger FPSR",
	"Both EN0 and EN1 will start the FPSR timer",
};

static const char *const max42500_fdiv_sel[] = {
	"25us/tick, 6.375ms total recording time",
	"50us/tick, 12.75ms total recording time",
	"100us/tick, 25.5ms total recording time",
	"200us/tick, 51ms total recording time",
	"400us/tick, 102ms total recording time",
	"800us/tick, 204ms total recording time",
	"1600μs/tick, 408ms total recording time"
	"3200us/tick, 816ms total recording time",
};

static const char *const max42500_wd_mode[] = {
	"Challenge/response watchdog mode",
	"Simple windowed watchdog mode",
};

static const char *const max42500_wd_rhld[] = {
	"0ms (6ms typ, used for interrupt-style functionality)",
	"8ms",
	"16ms",
	"32ms",
};

static const char *const max42500_wd_mr1[] = {
	"RESET asserts after any watchdog violation",
	"RESET asserts after 2 consecutive watchdog violation",
};

static int max42500_iio_get_state(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel);
static int max42500_iio_set_state(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel);
static int max42500_iio_reg_read(void *dev, uint32_t reg, uint32_t *readval);
static int max42500_iio_reg_write(void *dev, uint32_t reg, uint32_t writeval);
static int max42500_iio_read_config2(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_vmon(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_write_vmon(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_rstmap(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_write_rstmap(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);					
static int max42500_iio_set_nominal_voltage(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_get_nominal_voltage(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_comp_statoff(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_comp_statuv(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_comp_statov(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_get_ovthresh1(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_set_ovthresh1(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_get_ovthresh2(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_set_ovthresh2(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_get_uvthresh1(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_set_uvthresh1(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_get_uvthresh2(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_set_uvthresh2(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_fpsstat1(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_fpsconfig(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_write_fpsconfig(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_power_up_timestamp(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_power_down_timestamp(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_wdtstat(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_wdcdiv(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_write_wdcdiv(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_wdcfg1(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_write_wdcfg1(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_1ud(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_write_1ud(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_read_wdkey(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_get_watchdog_enable_status(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_set_watchdog_enable(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_get_watchdog_lock_bit(void *dev, char *buf, uint32_t len,
			     const struct iio_ch_info*channel, intptr_t priv);				
static int max42500_iio_set_watchdog_lock_bit(void *dev, char *buf, uint32_t len,
			     const struct iio_ch_info *channel, intptr_t priv);	
static int max42500_iio_read_rstctrl(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv);
static int max42500_iio_write_rstctrl(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv);				
				 
static struct iio_attribute const max42500_channel_attrs[] = {
	{
		.name = "vm1",
		.store = max42500_iio_set_nominal_voltage,
		.show = max42500_iio_get_nominal_voltage,
		.priv = MAX42500_VM1,
	},
	{
		.name = "vm2",
		.store = max42500_iio_set_nominal_voltage,
		.show = max42500_iio_get_nominal_voltage,
		.priv = MAX42500_VM2,
	},	
	{
		.name = "vm3",
		.store = max42500_iio_set_nominal_voltage,
		.show = max42500_iio_get_nominal_voltage,
		.priv = MAX42500_VM3,
	},
	{
		.name = "vm4",
		.store = max42500_iio_set_nominal_voltage,
		.show = max42500_iio_get_nominal_voltage,
		.priv = MAX42500_VM4,
	},
	{
		.name = "vm5",
		.store = max42500_iio_set_nominal_voltage,
		.show = max42500_iio_get_nominal_voltage,
		.priv = MAX42500_VM5,
	},
	{
		.name = "vm1_threshov",
		.store = max42500_iio_set_ovthresh1,
		.show = max42500_iio_get_ovthresh1,
		.priv = MAX42500_VM1,
	},
	{
		.name = "vm2_threshov",
		.store = max42500_iio_set_ovthresh1,
		.show = max42500_iio_get_ovthresh1,
		.priv = MAX42500_VM2,
	},	
	{
		.name = "vm3_threshov",
		.store = max42500_iio_set_ovthresh1,
		.show = max42500_iio_get_ovthresh1,
		.priv = MAX42500_VM3,
	},
	{
		.name = "vm4_threshov",
		.store = max42500_iio_set_ovthresh1,
		.show = max42500_iio_get_ovthresh1,
		.priv = MAX42500_VM4,
	},
	{
		.name = "vm5_threshov",
		.store = max42500_iio_set_ovthresh1,
		.show = max42500_iio_get_ovthresh1,
		.priv = MAX42500_VM5,
	},
	{
		.name = "vm6_threshov",
		.store = max42500_iio_set_ovthresh2,
		.show = max42500_iio_get_ovthresh2,
		.priv = MAX42500_VM6,
	},
	{
		.name = "vm7_threshov",
		.store = max42500_iio_set_ovthresh2,
		.show = max42500_iio_get_ovthresh2,
		.priv = MAX42500_VM7,
	},
	{
		.name = "vm1_threshuv",
		.store = max42500_iio_set_uvthresh1,
		.show = max42500_iio_get_uvthresh1,
		.priv = MAX42500_VM1,
	},
	{
		.name = "vm2_threshuv",
		.store = max42500_iio_set_uvthresh1,
		.show = max42500_iio_get_uvthresh1,
		.priv = MAX42500_VM2,
	},	
	{
		.name = "vm3_threshuv",
		.store = max42500_iio_set_uvthresh1,
		.show = max42500_iio_get_uvthresh1,
		.priv = MAX42500_VM3,
	},
	{
		.name = "vm4_threshuv",
		.store = max42500_iio_set_uvthresh1,
		.show = max42500_iio_get_uvthresh1,
		.priv = MAX42500_VM4,
	},
	{
		.name = "vm5_threshuv",
		.store = max42500_iio_set_uvthresh1,
		.show = max42500_iio_get_uvthresh1,
		.priv = MAX42500_VM5,
	},
	{
		.name = "vm6_threshuv",
		.store = max42500_iio_set_uvthresh2,
		.show = max42500_iio_get_uvthresh2,
		.priv = MAX42500_VM6,
	},
	{
		.name = "vm7_threshuv",
		.store = max42500_iio_set_uvthresh2,
		.show = max42500_iio_get_uvthresh2,
		.priv = MAX42500_VM7,
	},
	
};

static struct iio_attribute max42500_debug_attrs[] = {
	{
		.name = "statoff_vm1",
		.show = max42500_iio_read_comp_statoff,
		.priv = MAX42500_VM1,
	},
	{
		.name = "statoff_vm2",
		.show = max42500_iio_read_comp_statoff,
		.priv = MAX42500_VM2,
	},
	{
		.name = "statoff_vm3",
		.show = max42500_iio_read_comp_statoff,
		.priv = MAX42500_VM3,
	},
	{
		.name = "statoff_vm4",
		.show = max42500_iio_read_comp_statoff,
		.priv = MAX42500_VM4,
	},
	{
		.name = "statoff_vm5",
		.show = max42500_iio_read_comp_statoff,
		.priv = MAX42500_VM5,
	},
	{
		.name = "statoff_vm6",
		.show = max42500_iio_read_comp_statoff,
		.priv = MAX42500_VM6,
	},
	{
		.name = "statoff_vm7",
		.show = max42500_iio_read_comp_statoff,
		.priv = MAX42500_VM7,
	},
	{
		.name = "statov_vm1",
		.show = max42500_iio_read_comp_statov,
		.priv = MAX42500_VM1,
	},
	{
		.name = "statov_vm2",
		.show = max42500_iio_read_comp_statov,
		.priv = MAX42500_VM2,
	},
	{
		.name = "statov_vm3",
		.show = max42500_iio_read_comp_statov,
		.priv = MAX42500_VM3,
	},
	{
		.name = "statov_vm4",
		.show = max42500_iio_read_comp_statov,
		.priv = MAX42500_VM4,
	},
	{
		.name = "statov_vm5",
		.show = max42500_iio_read_comp_statov,
		.priv = MAX42500_VM5,
	},
	{
		.name = "statov_vm6",
		.show = max42500_iio_read_comp_statov,
		.priv = MAX42500_VM6,
	},
	{
		.name = "statov_vm7",
		.show = max42500_iio_read_comp_statov,
		.priv = MAX42500_VM7,
	},
	{
		.name = "statuv_vm1",
		.show = max42500_iio_read_comp_statuv,
		.priv = MAX42500_VM1,
	},
	{
		.name = "statuv_vm2",
		.show = max42500_iio_read_comp_statuv,
		.priv = MAX42500_VM2,
	},
	{
		.name = "statuv_vm3",
		.show = max42500_iio_read_comp_statuv,
		.priv = MAX42500_VM3,
	},
	{
		.name = "statuv_vm4",
		.show = max42500_iio_read_comp_statuv,
		.priv = MAX42500_VM4,
	},
	{
		.name = "statuv_vm5",
		.show = max42500_iio_read_comp_statuv,
		.priv = MAX42500_VM5,
	},
	{
		.name = "statuv_vm6",
		.show = max42500_iio_read_comp_statuv,
		.priv = MAX42500_VM6,
	},
	{
		.name = "statuv_vm7",
		.show = max42500_iio_read_comp_statuv,
		.priv = MAX42500_VM7,
	},
	{
		.name = "Internal_Oscilator",
		.show = max42500_iio_read_config2,
		.priv = MAX42500_CLKF,
	},
	{
		.name = "Parity",
		.show = max42500_iio_read_config2,
		.priv = MAX42500_PAR,
	},
	{
		.name = "RESET_Fault",
		.show = max42500_iio_read_config2,
		.priv = MAX42500_RSTF,
	},
	{
		.name = "RESET",
		.show = max42500_iio_read_config2,
		.priv = MAX42500_RST,
	},
	{
		.name = "EN1",
		.show = max42500_iio_read_config2,
		.priv = MAX42500_EN1,
	},
	{
		.name = "EN0",
		.show = max42500_iio_read_config2,
		.priv = MAX42500_EN0,
	},
	{
		.name = "BSTO",
		.show = max42500_iio_read_config2,
		.priv = MAX42500_BSTO,
	},
	{
		.name = "BSTU",
		.show = max42500_iio_read_config2,
		.priv = MAX42500_BSTU,
	},
	{
		.name = "fps_not_read",
		.show = max42500_iio_read_fpsstat1,
		.priv = MAX42500_NOTRD,
	},
	{
		.name = "fps_pu_source",
		.show = max42500_iio_read_fpsstat1,
		.priv = MAX42500_UEN,
	},
	{
		.name = "fps_pd_source",
		.show = max42500_iio_read_fpsstat1,
		.priv = MAX42500_DEN,
	},
	{
		.name = "fps_en",
		.show = max42500_iio_read_fpsstat1,
		.priv = MAX42500_FPSE,
	},
	{
		.name = "sequence_rec_running",
		.show = max42500_iio_read_fpsstat1,
		.priv = MAX42500_SRR,
	},
	{
		.name = "utime1",
		.show = max42500_iio_read_power_up_timestamp,
		.priv = MAX42500_VM1,
	},
	{
		.name = "utime2",
		.show = max42500_iio_read_power_up_timestamp,
		.priv = MAX42500_VM2,
	},
	{
		.name = "utime3",
		.show = max42500_iio_read_power_up_timestamp,
		.priv = MAX42500_VM3,
	},
	{
		.name = "utime4",
		.show = max42500_iio_read_power_up_timestamp,
		.priv = MAX42500_VM4,
	},
	{
		.name = "utime5",
		.show = max42500_iio_read_power_up_timestamp,
		.priv = MAX42500_VM5,
	},
	{
		.name = "utime6",
		.show = max42500_iio_read_power_up_timestamp,
		.priv = MAX42500_VM6,
	},
	{
		.name = "utime7",
		.show = max42500_iio_read_power_up_timestamp,
		.priv = MAX42500_VM7,
	},
	{
		.name = "dtime1",
		.show = max42500_iio_read_power_down_timestamp,
		.priv = MAX42500_VM1,
	},
	{
		.name = "dtime2",
		.show = max42500_iio_read_power_down_timestamp,
		.priv = MAX42500_VM2,
	},
	{
		.name = "dtime3",
		.show = max42500_iio_read_power_down_timestamp,
		.priv = MAX42500_VM3,
	},
	{
		.name = "dtime4",
		.show = max42500_iio_read_power_down_timestamp,
		.priv = MAX42500_VM4,
	},
	{
		.name = "dtime5",
		.show = max42500_iio_read_power_down_timestamp,
		.priv = MAX42500_VM5,
	},
	{
		.name = "dtime6",
		.show = max42500_iio_read_power_down_timestamp,
		.priv = MAX42500_VM6,
	},
	{
		.name = "dtime7",
		.show = max42500_iio_read_power_down_timestamp,
		.priv = MAX42500_VM7,
	},
	{
		.name = "wd_window_open",
		.show = max42500_iio_read_wdtstat,
		.priv = MAX42500_OPEN,
	},
	{
		.name = "lfsr_wr_mismatch",
		.show = max42500_iio_read_wdtstat,
		.priv = MAX42500_LFSR,
	},
	{
		.name = "wd_update_violation",
		.show = max42500_iio_read_wdtstat,
		.priv = MAX42500_WDUV,
	},
	{
		.name = "wd_window_expired",
		.show = max42500_iio_read_wdtstat,
		.priv = MAX42500_WDEXP,
	},
	{
		.name = "watchdog_key",
		.show = max42500_iio_read_wdkey,
	},
	
	
	
	
	
	END_ATTRIBUTES_ARRAY
};

static struct iio_attribute max42500_global_attrs[] = {
	{
		.name = "state",
		.store = max42500_iio_set_state,
		.show = max42500_iio_get_state,
	},
	{
		.name = "vmpd_enable",
		.store = max42500_iio_write_vmon,
		.show = max42500_iio_read_vmon,
		.priv = MAX42500_VM_MAX
	},
	{
		.name = "vm1_enable",
		.store = max42500_iio_write_vmon,
		.show = max42500_iio_read_vmon,
		.priv = MAX42500_VM1
	},
	{
		.name = "vm2_enable",
		.store = max42500_iio_write_vmon,
		.show = max42500_iio_read_vmon,
		.priv = MAX42500_VM2
	},
	{
		.name = "vm3_enable",
		.store = max42500_iio_write_vmon,
		.show = max42500_iio_read_vmon,
		.priv = MAX42500_VM3
	},
	{
		.name = "vm4_enable",
		.store = max42500_iio_write_vmon,
		.show = max42500_iio_read_vmon,
		.priv = MAX42500_VM4
	},
	{
		.name = "vm5_enable",
		.store = max42500_iio_write_vmon,
		.show = max42500_iio_read_vmon,
		.priv = MAX42500_VM5
	},
	{
		.name = "vm6_enable",
		.store = max42500_iio_write_vmon,
		.show = max42500_iio_read_vmon,
		.priv = MAX42500_VM6
	},
	{
		.name = "vm7_enable",
		.store = max42500_iio_write_vmon,
		.show = max42500_iio_read_vmon,
		.priv = MAX42500_VM7
	},
	{
		.name = "parm_enable",
		.store = max42500_iio_write_rstmap,
		.show = max42500_iio_read_rstmap,
		.priv = MAX42500_VM_MAX
	},
	{
		.name = "in1_enable",
		.store = max42500_iio_write_rstmap,
		.show = max42500_iio_read_rstmap,
		.priv = MAX42500_VM1
	},
	{
		.name = "in2_enable",
		.store = max42500_iio_write_rstmap,
		.show = max42500_iio_read_rstmap,
		.priv = MAX42500_VM2
	},
	{
		.name = "in3_enable",
		.store = max42500_iio_write_rstmap,
		.show = max42500_iio_read_rstmap,
		.priv = MAX42500_VM3
	},
	{
		.name = "in4_enable",
		.store = max42500_iio_write_rstmap,
		.show = max42500_iio_read_rstmap,
		.priv = MAX42500_VM4
	},
	{
		.name = "in5_enable",
		.store = max42500_iio_write_rstmap,
		.show = max42500_iio_read_rstmap,
		.priv = MAX42500_VM5
	},
	{
		.name = "in6_enable",
		.store = max42500_iio_write_rstmap,
		.show = max42500_iio_read_rstmap,
		.priv = MAX42500_VM6
	},
	{
		.name = "in7_enable",
		.store = max42500_iio_write_rstmap,
		.show = max42500_iio_read_rstmap,
		.priv = MAX42500_VM7
	},	
	{
		.name = "uval",
		.show = max42500_iio_read_fpsconfig,
		.store = max42500_iio_write_fpsconfig,
		.priv = MAX42500_UVAL,
	},
	{
		.name = "dval",
		.show = max42500_iio_read_fpsconfig,
		.store = max42500_iio_write_fpsconfig,
		.priv = MAX42500_DVAL,
	},
	{
		.name = "uvalm",
		.show = max42500_iio_read_fpsconfig,
		.store = max42500_iio_write_fpsconfig,
		.priv = MAX42500_UVALM,
	},
	{
		.name = "dvalm",
		.show = max42500_iio_read_fpsconfig,
		.store = max42500_iio_write_fpsconfig,
		.priv = MAX42500_DVALM,
	},
	{
		.name = "fpsen",
		.show = max42500_iio_read_fpsconfig,
		.store = max42500_iio_write_fpsconfig,
		.priv = MAX42500_FPSEN1,
	},
	{
		.name = "fps_clkdiv",
		.show = max42500_iio_read_fpsconfig,
		.store = max42500_iio_write_fpsconfig,
		.priv = MAX42500_FDIV,
	},
	{
		.name = "watchdog_mode",
		.show = max42500_iio_read_wdcdiv,
		.store = max42500_iio_write_wdcdiv,
		.priv = MAX42500_SWW,
	},
	{
		.name = "wd_clkdiv",
		.show = max42500_iio_read_wdcdiv,
		.store = max42500_iio_write_wdcdiv,
		.priv = MAX42500_WDIV,
	},
	{
		.name = "wd_open_window",
		.show = max42500_iio_read_wdcfg1,
		.store = max42500_iio_write_wdcfg1,
		.priv = MAX42500_OPN,
	},
	{
		.name = "wd_close_window",
		.show = max42500_iio_read_wdcfg1,
		.store = max42500_iio_write_wdcfg1,
		.priv = MAX42500_CLO,
	},
	{
		.name = "watchdog_enable",
		.store = max42500_iio_set_watchdog_enable,
		.show = max42500_iio_get_watchdog_enable_status,
	},
	{
		.name = "first_update_ext",
		.store = max42500_iio_write_1ud,
		.show = max42500_iio_read_1ud,
	},
	{
		.name = "wd_lock",
		.store = max42500_iio_set_watchdog_lock_bit,
		.show = max42500_iio_get_watchdog_lock_bit,
	},
	{
		.name = "wd_violation_count",
		.show = max42500_iio_read_rstctrl,
		.store = max42500_iio_write_rstctrl,
		.priv = MAX42500_MR1,
	},
	{
		.name = "wd_reset_timeout",
		.show = max42500_iio_read_rstctrl,
		.store = max42500_iio_write_rstctrl,
		.priv = MAX42500_RHLD,
	},
	END_ATTRIBUTES_ARRAY
};

static struct iio_channel const max42500_channels[] = {
	{
		.name = "voltage",
		.attributes = max42500_channel_attrs,
		.ch_out = true,
		.scan_type = NULL,
		.indexed = true,
		.channel = 0,
		.address = 0,
		.ch_type = IIO_VOLTAGE,
	},
};

static struct iio_device max42500_iio_dev = {
	.num_ch = NO_OS_ARRAY_SIZE(max42500_channels),
	.channels = max42500_channels,
	.attributes = max42500_global_attrs,
	.debug_attributes = max42500_debug_attrs,
	.debug_reg_read = (int32_t (*)())max42500_iio_reg_read,
	.debug_reg_write = (int32_t (*)())max42500_iio_reg_write,
};

/**
 * @brief Handles the read request for EN0 and EN1 pins.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_get_state(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel)
{
	int ret;
	uint8_t state;
	uint8_t en0;
    uint8_t en1;
	uint32_t num_values = NO_OS_ARRAY_SIZE(max42500_pin_state);
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;
	
	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = no_os_gpio_get_value(max42500->en0, &en0);
    if (ret)
        return ret;

    ret = no_os_gpio_get_value(max42500->en1, &en1);
    if (ret)
        return ret;
	
	if (en0 == NO_OS_GPIO_LOW && en1 == NO_OS_GPIO_LOW) {
        state = MAX42500_STATE_OFF;
    } else if (en0 == NO_OS_GPIO_HIGH && en1 == NO_OS_GPIO_LOW) {
        state = MAX42500_STATE_SLEEP;
    } else if (en0 == NO_OS_GPIO_HIGH && en1 == NO_OS_GPIO_HIGH) {
        state = MAX42500_STATE_ON;
    } else {
        return -EINVAL;
    }
	
	return sprintf(buf, "%s", max42500_pin_state[state]);
}

/**
 * @brief Handles the write request for EN0 and EN1 pins.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_set_state(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel)
{
	int ret;
	uint8_t value;
	uint32_t num_values = NO_OS_ARRAY_SIZE(max42500_pin_state);
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;
	

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	for (value = 0; value < num_values; value++)
		if (!strcmp(buf, max42500_pin_state[value]))
			break;

	if (value == num_values)
		return -EINVAL;

	ret = max42500_set_state(max42500, value);

	return ret;
}

/**
 * @brief Wrapper function for reading data from the MAX42500 device.
 * @param desc - The MAX42500 device structure.
 * @param reg - The register address to read from.
 * @param readval - The data read from the register.
 * @return 0 in case of success, an error code otherwise.
 */
static int max42500_iio_reg_read(void *dev, uint32_t reg, uint32_t *readval)
{
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev || !readval)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	return max42500_reg_read(max42500, reg, readval);
}

/**
 * @brief Wrapper function for writing data to the MAX42500 device.
 * @param desc - The MAX42500 device structure.
 * @param reg - The register address to write to.
 * @param writeval - The data to write to the register.
 * @return 0 in case of success, an error code otherwise.
 */
static int max42500_iio_reg_write(void *dev, uint32_t reg, uint32_t writeval)
{
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	return max42500_reg_write(max42500, reg, writeval);
}

/**
 * @brief Handles the read request for config2 register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_config2(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_CONFIG2, &value);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_BSTU:
		return sprintf(buf, "%s", max42500_fault_status[no_os_field_get(BSTU_MASK, value)]);
	case MAX42500_BSTO:
		return sprintf(buf, "%s", max42500_fault_status[no_os_field_get(BSTO_MASK, value)]);
	case MAX42500_EN0:
		return sprintf(buf, "%s", max42500_comp_status[no_os_field_get(EN0_MASK, value)]);
	case MAX42500_EN1:
		return sprintf(buf, "%s", max42500_comp_status[no_os_field_get(EN1_MASK, value)]);
	case MAX42500_RST:
		return sprintf(buf, "%s", max42500_comp_status[no_os_field_get(RST_MASK, value)]);
	case MAX42500_RSTF:
		return sprintf(buf, "%s", max42500_fault_status[no_os_field_get(RSTF_MASK, value)]);
	case MAX42500_PAR:
		return sprintf(buf, "%s", max42500_fault_status[no_os_field_get(PAR_MASK, value)]);
	case MAX42500_CLKF:
		return sprintf(buf, "%s", max42500_fault_status[no_os_field_get(CLKF_MASK, value)]);
	}
}

/**
 * @brief Handles the read request for vmon register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_vmon(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_VMON, &reg_data);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_VM1:
		value = no_os_field_get(VM1_MASK, reg_data);
		break;
	case MAX42500_VM2:
		value = no_os_field_get(VM2_MASK, reg_data);
		break;
	case MAX42500_VM3:
		value = no_os_field_get(VM3_MASK, reg_data);
		break;
	case MAX42500_VM4:
		value = no_os_field_get(VM4_MASK, reg_data);
		break;
	case MAX42500_VM5:
		value = no_os_field_get(VM5_MASK, reg_data);
		break;
	case MAX42500_VM6:
		value = no_os_field_get(VM6_MASK, reg_data);
		break;
	case MAX42500_VM7:
		value = no_os_field_get(VM7_MASK, reg_data);
		break;
	case MAX42500_VM_MAX:
		value = no_os_field_get(VMPD_MASK, reg_data);
		break;
	}
	
	return sprintf(buf, "%s", max42500_state[value]);
}

/**
 * @brief Handles the write request for VMON attributes.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_write_vmon(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t value;
	uint8_t mask;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_VM1:
		mask = VM1_MASK;
		break;
	case MAX42500_VM2:
		mask = VM2_MASK;
		break;
	case MAX42500_VM3:
		mask = VM3_MASK;
		break;
	case MAX42500_VM4:
		mask = VM4_MASK;
		break;
	case MAX42500_VM5:
		mask = VM5_MASK;
		break;
	case MAX42500_VM6:
		mask = VM6_MASK;
		break;
	case MAX42500_VM7:
		mask = VM7_MASK;
		break;
	case MAX42500_VM_MAX:
		mask = VMPD_MASK;
		break;
	}
    
	max42500_reg_update(max42500, MAX42500_REG_VMON, mask, 
					no_os_field_prep(mask, value));
}

/**
 * @brief Handles the read request for interrupt mapping register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_rstmap(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_RSTMAP, &reg_data);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_VM1:
		value = no_os_field_get(VM1_MASK, reg_data);
		break;
	case MAX42500_VM2:
		value = no_os_field_get(VM2_MASK, reg_data);
		break;
	case MAX42500_VM3:
		value = no_os_field_get(VM3_MASK, reg_data);
		break;
	case MAX42500_VM4:
		value = no_os_field_get(VM4_MASK, reg_data);
		break;
	case MAX42500_VM5:
		value = no_os_field_get(VM5_MASK, reg_data);
		break;
	case MAX42500_VM6:
		value = no_os_field_get(VM6_MASK, reg_data);
		break;
	case MAX42500_VM7:
		value = no_os_field_get(VM7_MASK, reg_data);
		break;
	case MAX42500_VM_MAX:
		value = no_os_field_get(PARM_MASK, reg_data);
		break;
	}
	
	return sprintf(buf, "%s", max42500_state[value]);
}

/**
 * @brief Handles the write request for rstmap attributes.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_write_rstmap(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t value;
	uint8_t mask;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_VM1:
		mask = IN1_MASK;
		break;
	case MAX42500_VM2:
		mask = IN2_MASK;
		break;
	case MAX42500_VM3:
		mask = IN3_MASK;
		break;
	case MAX42500_VM4:
		mask = IN4_MASK;
		break;
	case MAX42500_VM5:
		mask = IN5_MASK;
		break;
	case MAX42500_VM6:
		mask = IN6_MASK;
		break;
	case MAX42500_VM7:
		mask = IN7_MASK;
		break;
	case MAX42500_VM_MAX:
		mask = PARM_MASK;
		break;
	}
    
	max42500_reg_update(max42500, MAX42500_REG_RSTMAP, mask, 
					no_os_field_prep(mask, value));
}

/**
 * @brief Handles the read request for nominal voltage VM1 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_get_nominal_voltage(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_addr = MAX42500_REG_VIN1 + priv;
	float voltage;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, reg_addr, &value);

	if (ret)
		return ret;
	
	if ((value < REG_MIN) || (value > REG_MAX))
		return -EINVAL;
	
	switch (priv) {
	case MAX42500_VM1:
	case MAX42500_VM2:
	case MAX42500_VM3:
	case MAX42500_VM4:
		voltage = (MAX42500_MIN_VNOM + (MAX42500_VNOM_STEP_VM1_VM4 * value));
		break;
	case MAX42500_VM5:
		voltage = (MAX42500_MIN_VNOM + (MAX42500_VNOM_STEP_VM5 * value));
		break;
	default:
		return -EINVAL;
	}
	
	return snprintf(buf, len, "%.4fV", voltage);
}

/**
 * @brief Handles the write request for nominal voltage VM1 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_set_nominal_voltage(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t dec, fract;
	int64_t combined;
	double value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT_PLUS_MICRO, &dec, &fract);
	if (ret)
		return ret;

    value = (double)dec + ((double)fract / MICRO);
	//precision correction
	value +=  0.000001;
	
	return max42500_set_nominal_voltage(max42500, priv, value);
}

/**
 * @brief Handles the read request for off comparator register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_comp_statoff(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_get_comp_status(max42500, priv, MAX42500_COMP_STAT_OFF,
			     &value);
	if (ret)
		return ret;

	return sprintf(buf, "%s", max42500_comp_status[value]);
}

/**
 * @brief Handles the read request for uv comparator register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_comp_statuv(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_get_comp_status(max42500, priv, MAX42500_COMP_STAT_UV,
			     &value);
	if (ret)
		return ret;

	return sprintf(buf, "%s", max42500_comp_status[value]);
}

/**
 * @brief Handles the read request for ov comparator register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_comp_statov(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_get_comp_status(max42500, priv, MAX42500_COMP_STAT_OV,
			     &value);
	if (ret)
		return ret;

	return sprintf(buf, "%s", max42500_comp_status[value]);
}

/**
 * @brief Handles the read request for nominal OV THRESHOLD FOR VM1-VM5 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_get_ovthresh1(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_addr = MAX42500_REG_OVUV1 + priv;
	float thresh;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, reg_addr, &value);
	if (ret)
		return ret;
	
	thresh = 102.5 + (0.5 * (uint8_t)no_os_field_get(OV_MASK, value));
	
	return snprintf(buf, len, "%.1f%%", thresh);
}

/**
 * @brief Handles the write request for OV threshold for VM1-VM5 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_set_ovthresh1(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t dec, fract;
	double value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT_PLUS_MICRO, &dec, &fract);
	if (ret)
		return ret;

    value = dec + ((double)fract / MICRO);

	return max42500_set_ov_thresh1(max42500, priv, value);
}

/**
 * @brief Handles the read request for nominal OV THRESHOLD FOR VM6-VM7 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_get_ovthresh2(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	float thresh;
	uint8_t reg_addr;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	switch (priv) {
	case MAX42500_VM6:
		reg_addr = MAX42500_REG_VINO6;
		break;
	case MAX42500_VM7:
		reg_addr = MAX42500_REG_VINO7;
		break;
	default:
		return -EINVAL;
	}

	ret = max42500_reg_read(max42500, reg_addr, &value);
	if (ret)
		return ret;
	
	thresh = MAX42500_MIN_THRESH_VM6_V7 + (0.005 * value);
	
	return snprintf(buf, len, "%.3fV", thresh);
}

/**
 * @brief Handles the write request for OV threshold for VM6-VM7 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_set_ovthresh2(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t dec, fract;
	double value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT_PLUS_MICRO, &dec, &fract);
	if (ret)
		return ret;

    value = dec + ((double)fract / MICRO);
	//precision correction
	value +=  0.000001;

	return max42500_set_ov_thresh2(max42500, priv, value);
}

/**
 * @brief Handles the read request for nominal UV THRESHOLD FOR VM1-VM5 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_get_uvthresh1(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_addr = MAX42500_REG_OVUV1 + priv;
	float thresh;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, reg_addr, &value);
	if (ret)
		return ret;
	
	thresh = 97.5 - (0.5 * (uint8_t)no_os_field_get(UV_MASK, value));
	
	return snprintf(buf, len, "%.1f%%", thresh);
}

/**
 * @brief Handles the write request for UV threshold for VM1-VM5 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_set_uvthresh1(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t dec, fract;
	double value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT_PLUS_MICRO, &dec, &fract);
	if (ret)
		return ret;

    value = dec + ((double)fract / MICRO);

	return max42500_set_uv_thresh1(max42500, priv, value);
}

/**
 * @brief Handles the read request for nominal UV THRESHOLD FOR VM6-VM7 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_get_uvthresh2(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	float thresh;
	uint8_t reg_addr;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	switch (priv) {
	case MAX42500_VM6:
		reg_addr = MAX42500_REG_VINU6;
		break;
	case MAX42500_VM7:
		reg_addr = MAX42500_REG_VINU7;
		break;
	default:
		return -EINVAL;
	}

	ret = max42500_reg_read(max42500, reg_addr, &value);
	if (ret)
		return ret;
	
	thresh = MAX42500_MIN_THRESH_VM6_V7 + (0.005 * value);
	
	return snprintf(buf, len, "%.3fV", thresh);
}

/**
 * @brief Handles the write request for UV threshold for VM6-VM7 attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_set_uvthresh2(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t dec, fract;
	double value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT_PLUS_MICRO, &dec, &fract);
	if (ret)
		return ret;

    value = dec + ((double)fract / MICRO);
	//precision correction
	value +=  0.000001;

	return max42500_set_uv_thresh2(max42500, priv, value);
}

/**
 * @brief Handles the read request for fps status.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_fpsstat1(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_FPSSTAT1, &reg_data);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_NOTRD:
		value = no_os_field_get(NOTRD_MASK, reg_data);
		return sprintf(buf, "%s", max42500_fault_status[value]);
		break;
	case MAX42500_UEN:
		value = no_os_field_get(UEN_MASK, reg_data);
		return sprintf(buf, "%s", max42500_uen_states[value]);
		break;
	case MAX42500_DEN:
		value = no_os_field_get(DEN_MASK, reg_data);
		return sprintf(buf, "%s", max42500_den_states[value]);
		break;
	case MAX42500_FPSE:
		value = no_os_field_get(FPSE_MASK, reg_data);
		return sprintf(buf, "%s", max42500_state[value]);
		break;
	case MAX42500_SRR:
		value = no_os_field_get(SRR_MASK, reg_data);
		return sprintf(buf, "%s", max42500_state[value]);
		break;
	}
}

/**
 * @brief Handles the read request for fps config register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_fpsconfig(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_FPSCFG1, &reg_data);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_UVAL:
		value = no_os_field_get(UVAL_MASK, reg_data);
		return sprintf(buf, "%s", max42500_uval_states[value]);
		break;
	case MAX42500_DVAL:
		value = no_os_field_get(DVAL_MASK, reg_data);
		return sprintf(buf, "%s", max42500_dval_states[value]);
		break;
	case MAX42500_UVALM:
		value = no_os_field_get(UVALM_MASK, reg_data);
		return sprintf(buf, "%s", max42500_state[value]);
		break;
	case MAX42500_DVALM:
		value = no_os_field_get(DVALM_MASK, reg_data);
		return sprintf(buf, "%s", max42500_state[value]);
		break;
	case MAX42500_FPSEN1:
		value = no_os_field_get(FPSEN1_MASK, reg_data);
		return sprintf(buf, "%s", max42500_fpsen1_states[value]);
	case MAX42500_FDIV:
		value = no_os_field_get(FDIV_MASK, reg_data);
		return sprintf(buf, "%s", max42500_fdiv_sel[value]);
		break;
	}
}

/**
 * @brief Handles the write request for fps config register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_write_fpsconfig(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t value;
	uint8_t mask;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_UVAL:
		mask = UVAL_MASK;
		break;
	case MAX42500_DVAL:
		mask = DVAL_MASK;
		break;
	case MAX42500_UVALM:
		mask = UVALM_MASK;
		break;
	case MAX42500_DVALM:
		mask = DVALM_MASK;
		break;
	case MAX42500_FPSEN1:
		mask = FPSEN1_MASK;
		break;
	case MAX42500_FDIV:
		mask = FDIV_MASK;
		break;
	}
    
	max42500_reg_update(max42500, MAX42500_REG_FPSCFG1, mask, 
					no_os_field_prep(mask, value));
}

/**
 * @brief Handles the read request for power up timestamp.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_power_up_timestamp(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_get_power_up_timestamp(max42500, priv, &value);
	if (ret)
		return ret;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, (int32_t *)&value);
}

/**
 * @brief Handles the read request for power down timestamp.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_power_down_timestamp(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_get_power_down_timestamp(max42500, priv, &value);
	if (ret)
		return ret;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, (int32_t *)&value);
}

/**
 * @brief Handles the read request for watchdog status register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_wdtstat(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_WDSTAT, &reg_data);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_OPEN:
		value = no_os_field_get(OPEN_MASK, reg_data);
		break;
	case MAX42500_LFSR:
		value = no_os_field_get(LFSR_MASK, reg_data);
		break;
	case MAX42500_WDUV:
		value = no_os_field_get(WDUV_MASK, reg_data);
		break;
	case MAX42500_WDEXP:
		value = no_os_field_get(WDEXP_MASK, reg_data);
		break;
	}
	return sprintf(buf, "%s", max42500_fault_status[value]);
}

/**
 * @brief Handles the read request for watchdog clock and div register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_wdcdiv(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_WDCDIV, &reg_data);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_SWW:
		value = no_os_field_get(SWW_MASK, reg_data);
		return sprintf(buf, "%s", max42500_wd_mode[value]);
		break;
	case MAX42500_WDIV:
		value = no_os_field_get(WDIV_MASK, reg_data);
		return sprintf(buf, "%dus", WDT_BASE_TO*(value + 1));
		break;
	}
}

/**
 * @brief Handles the write request for watchdog clock and div register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_write_wdcdiv(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t value;
	uint8_t mask;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_SWW:
		mask = SWW_MASK;
		break;
	case MAX42500_WDIV:
		mask = WDIV_MASK;
		if (value > WDT_BASE_TO_MAX || value < WDT_BASE_TO_MIN)
			return -EINVAL;
		else
			value = (value / WDT_BASE_TO) - 1;
		break;
	}
    
	max42500_reg_update(max42500, MAX42500_REG_WDCDIV, mask, 
					no_os_field_prep(mask, value));
}

/**
 * @brief Handles the read request for watchdog window configuration.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_wdcfg1(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t clkdiv;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_WDCDIV, &reg_data);
	if (ret)
		return ret;
	clkdiv = no_os_field_get(WDIV_MASK, reg_data);
	no_os_udelay(70);
	
	ret = max42500_reg_read(max42500, MAX42500_REG_WDCFG1, &reg_data);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_CLO:
		value = no_os_field_get(CLO_MASK, reg_data);
		break;
	case MAX42500_OPN:
		value = no_os_field_get(OPN_MASK, reg_data);
		break;
	}
	
	return sprintf(buf, "%dus", WDT_BASE_TO * (clkdiv + 1)* 8 *(value + 1));
}

/**
 * @brief Handles the write request for watchdog window configuration.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_write_wdcfg1(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t value;
	uint8_t mask;
	uint8_t clkdiv;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;

	ret = max42500_reg_read(max42500, MAX42500_REG_WDCDIV, &reg_data);
	if (ret)
		return ret;
		clkdiv = no_os_field_get(WDIV_MASK, reg_data);
	no_os_udelay(70);	
	
	if ((value > ((clkdiv + 1)* WDT_BASE_TO * 8 * WDT_WINDOW_MAX)) ||
			(value < ((clkdiv + 1)* WDT_BASE_TO * 8 * WDT_WINDOW_MIN)))
		return -EINVAL;
	else
		value = (value / (WDT_BASE_TO * 8 * (clkdiv + 1))) - 1;
	
	switch (priv) {
	case MAX42500_CLO:
		mask = CLO_MASK;
		break;
	case MAX42500_OPN:
		mask = OPN_MASK;
		break;
	}
    
	max42500_reg_update(max42500, MAX42500_REG_WDCFG1, mask, 
					no_os_field_prep(mask, value));
}

/**
 * @brief Handles the read request for watchdog 1ud attibute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_1ud(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	uint8_t clkdiv;
	uint8_t ud_val;
	uint32_t wdt_per;
	
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_WDCDIV, &reg_data);
	if (ret)
		return ret;
	clkdiv = no_os_field_get(WDIV_MASK, reg_data);
	no_os_udelay(70);
	
	ret = max42500_reg_read(max42500, MAX42500_REG_WDCFG1, &reg_data);
	if (ret)
		return ret;
	wdt_per = no_os_field_get(OPN_MASK, reg_data) + no_os_field_get(CLO_MASK, reg_data);
	no_os_udelay(70);
	
	ret = max42500_reg_read(max42500, MAX42500_REG_WDCFG2, &reg_data);
	if (ret)
		return ret;
	ud_val = no_os_field_get(UD_MASK, reg_data);
	
	return sprintf(buf, "%dus", 8*(wdt_per+2) * WDT_BASE_TO * (clkdiv + 1) * ((ud_val * 2)+1));

}

/**
 * @brief Handles the write request for watchdog 1ud attibute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_write_1ud(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;

	return max42500_reg_update(max42500, MAX42500_REG_WDCFG2, UD_MASK, 
					no_os_field_prep(UD_MASK, value));
}

/**
 * @brief Handles the read request for watchdog key attibute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_wdkey(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_WDKEY, &reg_data);
	if (ret)
		return ret;

	return sprintf(buf, "%X", reg_data);
}

/**
 * @brief Handles getting watchdog enable bit status request.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_get_watchdog_enable_status(void *dev, char *buf, uint32_t len,
			     const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;
	
	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;
	
	ret = max42500_reg_read(max42500, MAX42500_REG_WDCFG2, &value);
	if (ret)
		return ret;
	
	
	return sprintf(buf, "%s", max42500_state[no_os_field_get(WDEN_MASK, value)]);
}

/**
 * @brief Handles setting watchdog enable request.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_set_watchdog_enable(void *dev, char *buf, uint32_t len,
			     const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;
	
	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;
	
	return max42500_set_watchdog_enable(max42500, value);
}

/**
 * @brief Handles getting watchdog lock bit request.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_get_watchdog_lock_bit(void *dev, char *buf, uint32_t len,
			     const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;
	
	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;
	
	ret = max42500_reg_read(max42500, MAX42500_REG_WDLOCK, &value);
	if (ret)
		return ret;
	
	
	return sprintf(buf, "%s", max42500_state[no_os_field_get(WDLOCK_MASK, value)]);
}

/**
 * @brief Handles setting lock bit request.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_set_watchdog_lock_bit(void *dev, char *buf, uint32_t len,
			     const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;
	
	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;
	
	return max42500_reg_update(max42500, MAX42500_REG_WDLOCK, WDLOCK_MASK, 
				no_os_field_prep(WDLOCK_MASK, value));
}

/**
 * @brief Handles the read request for reset control register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int max42500_iio_read_rstctrl(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	uint8_t value;
	uint8_t reg_data;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = max42500_reg_read(max42500, MAX42500_REG_RSTCTRL, &reg_data);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_RHLD:
		value = no_os_field_get(RHLD_MASK, reg_data);
		return sprintf(buf, "%s", max42500_wd_rhld[value]);
		break;
	case MAX42500_MR1:
		value = no_os_field_get(MR1_MASK, reg_data);
		return sprintf(buf, "%s", max42500_wd_mr1[value]);
		break;
	}
}

/**
 * @brief Handles the write request for reset control and div register.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
 */
static int max42500_iio_write_rstctrl(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel, intptr_t priv)
{
	int ret;
	int32_t value;
	uint8_t mask;
	struct max42500_iio_dev *iio_max42500;
	struct max42500_dev *max42500;

	if (!dev)
		return -EINVAL;

	iio_max42500 = (struct max42500_iio_dev *)dev;
	max42500 = iio_max42500->max42500_dev;

	ret = iio_parse_value(buf, IIO_VAL_INT, &value, NULL);
	if (ret)
		return ret;

	switch (priv) {
	case MAX42500_RHLD:
		mask = RHLD_MASK;
		break;
	case MAX42500_MR1:
		mask = MR1_MASK;
		break;
	}
    
	max42500_reg_update(max42500, MAX42500_REG_RSTCTRL, mask, 
					no_os_field_prep(mask, value));
}







/**
 * @brief Initializes the MAX42500 IIO descriptor.
 * @param iio_dev - The iio device descriptor.
 * @param init_param - The structure that contains the device
 * 		initial parameters.
 * @return 0 in case of success, an error code otherwise.
 */
int max42500_iio_init(struct max42500_iio_dev **iio_dev,
		     struct max42500_iio_dev_init_param *init_param)
{
	struct max42500_iio_dev *descriptor;
	int ret;

	if (!iio_dev || !init_param || !init_param->max42500_init_param)
		return -EINVAL;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = max42500_init(&descriptor->max42500_dev,
			   init_param->max42500_init_param);
	if (ret)
		goto free_desc;

	descriptor->iio_dev = &max42500_iio_dev;

	*iio_dev = descriptor;

	return 0;

free_desc:
	max42500_iio_remove(descriptor);

	return ret;
}

/**
 * @brief Free resources allocated by the init function.
 * @param desc - The iio device descriptor.
 * @return 0 in case of success, an error code otherwise.
 */
int max42500_iio_remove(struct max42500_iio_dev *desc)
{
	if (!desc)
		return -ENODEV;

	no_os_free(desc->iio_dev->channels);
	max42500_remove(desc->max42500_dev);
	no_os_free(desc);

	return 0;
}

