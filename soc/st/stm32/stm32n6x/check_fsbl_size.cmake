# Post-build guard: the signed FSBL must fit the boot partition, or the
# slot-0 image download overwrites its tail on the NOR. The BootROM then
# rejects the corrupted signature and silently falls back to its USB DFU
# loader (black screen, no UART, debug AP gated) - a failure mode that
# cost a day to diagnose. Fail the BUILD instead.
#
# Args: -DBIN=<signed binary> -DMAX=<boot partition size in bytes>

file(SIZE "${BIN}" bin_size)
if(bin_size GREATER ${MAX})
	math(EXPR over "${bin_size} - ${MAX}")
	message(FATAL_ERROR
		"Signed FSBL (${bin_size} B) exceeds the boot partition "
		"(${MAX} B) by ${over} B. Flashing this would corrupt the "
		"FSBL when slot 0 is written. Grow boot_partition in the "
		"board devicetree (and shift slot0_partition) or shrink "
		"the bootloader.")
endif()
