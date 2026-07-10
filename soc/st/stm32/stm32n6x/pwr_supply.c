/*
 * Copyright (c) 2026 SetZero
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <soc.h>

#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>

/*
 * PWR supply configuration for the STM32N6.
 *
 * RM0486: "After a system reset, the software must configure the used
 * supply configuration in PWR_CR1 [...] before changing VOS in PWR_VOSCR,
 * or the RCC sys_ck frequency." The write is only accepted once per POR.
 *
 * Nothing in the BootROM writes it. Until it is written, the VOS-ready
 * machinery is inert — PWR_VOSCR reads ACTVOS=1 with VOSRDY/ACTVOSRDY
 * both 0, a state RM0486 documents as impossible with the SMPS converter
 * disabled — and the PWR low-power wake-up sequencer waits forever on
 * its "supply good" step: Stop and Standby modes can be ENTERED but any
 * EXTI/WKUP wake-up stalls after re-powering the rails, with the CPU
 * never fetching an instruction (found the hard way on the
 * STM32N6570-DK; only NRST recovers). Writing the supply configuration
 * flips VOSCR to its documented state (VOS|VOSRDY with external supply)
 * and wake-up completes. This is also why every ST example calls
 * HAL_PWREx_ConfigSupply() as its first PWR operation.
 *
 * The VDDCORE monitor is enabled so the PWR can observe an external
 * supply during low-power transitions, and the BSEC clock is kept on
 * (AN5946 warning + errata ES0620 2.2.2: with BSEC clock off the
 * deepsleep handshake with the RCC wedges and low-power entry fails).
 */
static int stm32n6_pwr_supply_init(void)
{
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_PWR);

	if (IS_ENABLED(CONFIG_POWER_SUPPLY_DIRECT_SMPS)) {
		LL_PWR_ConfigSupply(LL_PWR_SMPS_SUPPLY);
	} else {
		LL_PWR_ConfigSupply(LL_PWR_EXTERNAL_SOURCE_SUPPLY);
	}

	SET_BIT(PWR->CR3, PWR_CR3_VCOREMONEN);
	SET_BIT(RCC->APB4ENR2, RCC_APB4ENR2_BSECEN);

	return 0;
}
SYS_INIT(stm32n6_pwr_supply_init, PRE_KERNEL_1, 0);
