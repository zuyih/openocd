/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * STM32C5x flash driver definitions.
 *
 * Same register map as STM32H5, except that the erase bits are named PER/PNB
 * instead of SER/SNB, and that there is no TrustZone.
 */

#ifndef OPENOCD_FLASH_NOR_STM32C5X
#define OPENOCD_FLASH_NOR_STM32C5X

/* IMPORTANT: this file is included by stm32c5x driver and flashloader,
 * so please when changing this file, do not forget to check the flashloader */

/* #include "helper/bits.h" causes build errors when compiling
 * the flashloader, for now just redefine the needed 'BIT 'macro */

#ifndef BIT
#define BIT(nr)                 (1UL << (nr))
#endif

/* Enough timeouts for flash operations */
#define FLASH_ERASE_TIMEOUT 250
#define FLASH_WRITE_TIMEOUT 50
#define FLASH_TIMEOUT 250

/* Relevant STM32C5 flags ****************************************************/
#define F_NONE              0
/* This flag indicates if the device flash is with dual bank architecture */
#define F_HAS_DUAL_BANK     BIT(0)
/* End of STM32C5 flags ******************************************************/

/* DBGMCU is mirrored at 0xE00E4000 on the debug AP (used by the tcl scripts)
 * and at 0x44024000 on the system bus; only the latter is reachable from the
 * Cortex-M AP this driver runs on. */
#define DBGMCU_IDCODE			0x44024000

/* FLASH_OPTCR register bits */
#define FLASH_OPTLOCK		BIT(0)
#define FLASH_OPTSTRT		BIT(1)

/* FLASH_SR register bits */
#define FLASH_BSY			BIT(0)
#define FLASH_WBNE			BIT(1)
#define FLASH_DBNE			BIT(3)
#define FLASH_EOP			BIT(16)
#define FLASH_WRPERR		BIT(17)
#define FLASH_PGSERR		BIT(18)
#define FLASH_STRBERR		BIT(19)
#define FLASH_INCERR		BIT(20)
#define FLASH_OPTCHANGEERR	BIT(23)

#define FLASH_ERROR (FLASH_WRPERR | FLASH_PGSERR | FLASH_STRBERR | FLASH_INCERR | \
		FLASH_OPTCHANGEERR)

/* FLASH_CR register bits */
#define FLASH_LOCK			BIT(0)
#define FLASH_PG			BIT(1)
#define FLASH_PER			BIT(2)
#define FLASH_BER			BIT(3)
#define FLASH_FW			BIT(4)
#define FLASH_STRT			BIT(5)
#define FLASH_PNB_POS		6
#define FLASH_PNB_MASK		0x3F
#define FLASH_MER			BIT(15)
#define FLASH_BKSEL			BIT(31)

/* FLASH_OPTSR register bits, PSTATE is named RDP_LEVEL in the CMSIS headers */
#define FLASH_PSTATE_POS	8
#define FLASH_PSTATE_MASK	0xFF
#define FLASH_SINGLE_BANK	BIT(30)
#define FLASH_SWAP_BANK		BIT(31)

/* Register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* Option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

/* Supported device IDs */
#define DEVID_STM32C55_C56XX	0x44E
#define DEVID_STM32C53_C54XX	0x44F
#define DEVID_STM32C59_C5AXX	0x45A

/* Known Flash base addresses */
#define STM32_FLASH_BANK_BASE	0x08000000

/* Flash data width (128 bit) */
#define FLASH_DATA_WIDTH		16

/* 100 bytes as loader stack should be large enough for the loader to operate */
#define LDR_STACK_SIZE			100

struct stm32c5x_work_area {
	struct stm32c5x_loader_params {
		uint32_t flash_sr_addr;
		uint32_t flash_cr_addr;
		uint32_t flash_word_size;
	} params;
	uint8_t stack[LDR_STACK_SIZE];
	struct flash_async_algorithm_circbuf {
		/* note: stm32c5x_work_area struct is shared between the loader
		 * and stm32c5x flash driver.
		 *
		 * '*wp' and '*rp' pointers' size is 4 bytes each since stm32c5x
		 * devices have 32-bit processors.
		 * however when used in openocd code, their size depends on the host
		 *   if the host is 32-bit, then the size is 4 bytes each.
		 *   if the host is 64-bit, then the size is 8 bytes each.
		 * to avoid this size difference, change their types depending on the
		 * usage (pointers for the loader, and 32-bit integers in openocd code).
		 */
#ifdef OPENOCD_CONTRIB_LOADERS_FLASH_STM32_STM32C5X
		uint8_t *wp;
		uint8_t *rp;
#else
		uint32_t wp;
		uint32_t rp;
#endif /* OPENOCD_CONTRIB_LOADERS_FLASH_STM32_STM32C5X */
	} fifo;
};

#endif
