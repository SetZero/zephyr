/*
 * Copyright (c) 2026 SetZero
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <soc.h>
#include <zephyr/init.h>

#include <stm32_ll_cortex.h>
#include <stm32_ll_pwr.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

/* clock_stm32_ll_n6.c (the N6 is not part of the "common" LL clock
 * driver family, so clock_stm32_ll_common.h does not apply to it)
 */
int stm32_clock_control_init(const struct device *dev);

/*
 * System PM backend for the STM32N6.
 *
 * Suspend-to-idle substate 1 maps to the N6 Stop mode: the whole VCORE
 * domain is clock-gated but stays powered, so every SRAM and peripheral
 * register survives and execution continues after the WFI. Wake-up
 * sources are EXTI lines (GPIO interrupts, RTC).
 *
 * On Stop exit the CPU runs on HSI and all PLLs are off, but the RCC
 * configuration registers kept their contents — re-running the clock
 * driver init restores the full clock tree. Note that the Cortex-M ISR
 * wrapper calls pm_system_resume() (and thus pm_state_exit_post_ops())
 * with interrupts still locked BEFORE dispatching the wake interrupt's
 * handler, so no ISR or thread ever executes on the degraded HSI clock.
 *
 * There is no LPTIM system-timer support on this series yet: SysTick is
 * the kernel timer and its clock stops in Stop mode, so kernel timeouts
 * do NOT wake the system and uptime pauses for the duration of the
 * suspension. The devicetree state therefore carries a large
 * min-residency-us: entering Stop only makes sense for indefinite,
 * event-woken (EXTI) suspensions, not for timed idle gaps.
 */

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	if (state != PM_STATE_SUSPEND_TO_IDLE) {
		LOG_DBG("Unsupported power state %u", state);
		return;
	}

	switch (substate_id) {
	case 1: /* Stop mode */
		/* Clear stale STOPF/SBF status flags */
		LL_PWR_ClearFlag_STOP_SB();
		/* Deepsleep = Stop, not Standby */
		LL_PWR_SetPowerDownModeDS(LL_PWR_POWERDOWN_MODE_DS_STOP);
		LL_LPM_EnableDeepSleep();
		/* Enter Stop mode: idle unmasks interrupts atomically with
		 * the WFI; the wake IRQ is dispatched only after
		 * pm_state_exit_post_ops() restored the clock tree.
		 */
		k_cpu_idle();
		break;
	default:
		LOG_DBG("Unsupported power state substate-id %u", substate_id);
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	if (state != PM_STATE_SUSPEND_TO_IDLE) {
		LOG_DBG("Unsupported power state %u", state);
	} else {
		switch (substate_id) {
		case 1: /* Stop mode */
			LL_LPM_DisableSleepOnExit();
			/* Clear SLEEPDEEP so a plain idle WFI is Sleep again */
			LL_LPM_EnableSleep();
			break;
		default:
			LOG_DBG("Unsupported power state substate-id %u",
				substate_id);
			break;
		}
		/* Back from HSI to the configured PLL/IC clock tree. The
		 * driver init is re-entrant (it switches to HSI itself
		 * before touching the PLLs); the VDDCORE/VOS recovery path
		 * inside it self-skips because ACTVOS never dropped (the
		 * external supply keeps its level through Stop).
		 */
		stm32_clock_control_init(NULL);
	}

	/*
	 * System is now in active mode. Reenable interrupts which were
	 * disabled when OS started idling code.
	 */
	irq_unlock(0);
}
