/*
 * Copyright (c) 2026 SetZero
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/toolchain.h>

#include <stm32_common.h>
#include <stm32_ll_pwr.h>

/*
 * Power off = STM32N6 Standby mode: the VCORE domain is powered down
 * entirely (all SRAM and register contents are lost except the backup
 * domain), giving the lowest consumption the SoC can reach while still
 * being able to wake. Wake-up sources are the PWR wake-up pins
 * (WKUP1..4) and RTC/TAMP events; the caller is expected to configure
 * and enable them (LL_PWR_EnableWakeUpPin & friends) before calling
 * sys_poweroff(). Wake-up is a full system reset through the BootROM.
 */
void z_sys_poweroff(void)
{
	/* Stale status/wake-up flags would cause an immediate wake */
	LL_PWR_ClearFlag_STOP_SB();
	LL_PWR_ClearFlag_WU1();
	LL_PWR_ClearFlag_WU2();
	LL_PWR_ClearFlag_WU3();
	LL_PWR_ClearFlag_WU4();

	/* Deepsleep = Standby */
	LL_PWR_SetPowerDownModeDS(LL_PWR_POWERDOWN_MODE_DS_STANDBY);

	stm32_enter_poweroff();
}
