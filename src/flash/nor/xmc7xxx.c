// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *                                                                         *
 *   Infineon XMC7000 (Arm Cortex-M0+ + Cortex-M7) flash driver            *
 *                                                                         *
 *   The XMC7000 family shares the SROM-based non-volatile-memory          *
 *   programming architecture with PSoC 6 and the Traveo II (CYT) family.  *
 *                                                                         *
 *   Programs Code (Main) and Work flash via the SROM system-call API,     *
 *   invoked through IPC structure 3 (reserved for the DAP). The SROM      *
 *   code executes on the CM0+ core, so a small infinite-loop "algorithm"  *
 *   is left running on CM0+ while the operations are performed, allowing  *
 *   the SROM IRQ0 handler to service the system calls.                    *
 *                                                                         *
 *   Modelled on the PSoC6 driver (psoc6.c), but with the addresses,       *
 *   opcodes and register offsets that are specific to the XMC7000         *
 *   family:                                                               *
 *     - IPC block at 0x40220000 (PSoC6 uses 0x40230000)                   *
 *     - DAP uses IPC structure 3 (PSoC6 uses 2)                           *
 *     - IPC LOCK_STATUS at offset 0x1C (PSoC6 uses 0x10)                  *
 *     - SROM opcode in bits [31:24]                                       *
 *     - flash-controller write-safety registers must be enabled           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <time.h>

#include "imp.h"
#include "helper/time_support.h"
#include "target/arm_adi_v5.h"
#include "target/target.h"
#include "target/breakpoints.h"
#include "target/algorithm.h"
#include "target/cortex_m.h"

/**************************************************************************************************
 * XMC7000 device definitions
 *************************************************************************************************/
#define MEM_BASE_MFLASH                 0x10000000u
#define MEM_BASE_WFLASH                 0x14000000u

/* Code (main) flash: program one 512-byte page per ProgramRow call.
 * Erase sectors are 32 kB (large) with a 128 kB region of 8 kB (small)
 * sectors at the top of the array. */
#define MFLASH_PROGRAM_SIZE             512u
#define MFLASH_LARGE_SECTOR             (32u * 1024u)
#define MFLASH_SMALL_SECTOR             (8u * 1024u)
#define MFLASH_SMALL_REGION             (128u * 1024u)

/* Work flash: erase sectors are 2 kB (large) with a region of 128-byte
 * (small) sectors (1/4 of the array) at the top. Programmed in pages. */
#define WFLASH_PROGRAM_SIZE             512u
#define WFLASH_LARGE_SECTOR             (2u * 1024u)
#define WFLASH_SMALL_SECTOR             128u

/* Default sizes for the XMC7100 part on hand (silicon ID 0xE94C):
 * 4160 kB code flash, 256 kB work flash. Used when the flash bank is
 * declared with size 0. */
#define MFLASH_DEFAULT_SIZE             (4160u * 1024u)
#define WFLASH_DEFAULT_SIZE             (256u * 1024u)

#define RAM_STACK_WA_SIZE               2048u

#define PROTECTION_UNKNOWN              0x00u
#define PROTECTION_VIRGIN               0x01u
#define PROTECTION_NORMAL               0x02u
#define PROTECTION_SECURE               0x03u
#define PROTECTION_DEAD                 0x04u

/* IPC block (XMC7000) */
#define MEM_BASE_IPC                    0x40220000u
#define IPC_STRUCT_SIZE                 0x20u
#define MEM_IPC(n)                      (MEM_BASE_IPC + (n) * IPC_STRUCT_SIZE)
#define MEM_IPC_ACQUIRE(n)              (MEM_IPC(n) + 0x00u)
#define MEM_IPC_NOTIFY(n)               (MEM_IPC(n) + 0x08u)
#define MEM_IPC_DATA(n)                 (MEM_IPC(n) + 0x0Cu)
#define MEM_IPC_LOCK_STATUS(n)          (MEM_IPC(n) + 0x1Cu)

#define MEM_BASE_IPC_INTR               0x40221000u
#define IPC_INTR_STRUCT_SIZE            0x20u
#define MEM_IPC_INTR(n)                 (MEM_BASE_IPC_INTR + (n) * IPC_INTR_STRUCT_SIZE)
#define MEM_IPC_INTR_MASK(n)            (MEM_IPC_INTR(n) + 0x08u)
#define IPC_ACQUIRE_SUCCESS_MSK         0x80000000u
#define IPC_LOCK_ACQUIRED_MSK           0x80000000u

#define IPC_ID                          3u  /* IPC structure reserved for the DAP */
#define IPC_INTR_ID                     0u  /* notify -> intr struct 0 -> CM0+ IRQ0 */
#define IPC_TIMEOUT_MS                  1500

/* Flash-controller write-safety registers (bit0 = write enable) */
#define FLASHC_MAIN_FLASH_SAFETY        0x4024F400u
#define FLASHC_WORK_FLASH_SAFETY        0x4024F500u
#define FLASH_WRITE_ENABLE              0x00000001u

/* Cortex-M registers used to prepare the CM0+ for SROM IRQ servicing */
#define CM0_VTOR                        0xE000ED08u
#define CM0_NVIC_ISER0                  0xE000E100u

/* SROM API opcodes live in bits [31:24]. The inline/RAM selector is bit0 of
 * the value written to IPC_DATA0: 1 = inline argument, 0 = pointer to SRAM. */
#define SROMAPI_SIID_REQ                    0x00000001u  /* SiliconID, inline */
#define SROMAPI_SIID_REQ_FAMILY_REVISION    (SROMAPI_SIID_REQ | 0x000u)
#define SROMAPI_SIID_REQ_SIID_PROTECTION    (SROMAPI_SIID_REQ | 0x100u)
#define SROMAPI_PROGRAMROW_REQ              0x06000100u  /* blocking, blank-check */
#define SROMAPI_PROGRAMWORKFLASH_REQ        0x30000100u  /* blocking, blank-check */
#define SROMAPI_ERASESECTOR_REQ             0x14000100u  /* blocking */
#define SROMAPI_ERASEALL_REQ                0x0A000100u

/* ProgramRow/ProgramWorkFlash data-size code in param word +0x04, bits[7:0] */
#define SROMAPI_DSIZE_512B                  0x00000009u  /* 4096 bits */

#define SROMAPI_STATUS_MSK                  0xF0000000u
#define SROMAPI_STAT_SUCCESS                0xA0000000u
#define SROMAPI_DATA_LOCATION_MSK           0x00000001u

struct xmc7xxx_target_info {
	uint32_t silicon_id;
	uint8_t protection;
	bool is_work;
	bool is_probed;
};

struct timeout {
	int64_t start_time;
	long timeout_ms;
};

static struct working_area *g_stack_area;
static struct armv7m_algorithm g_armv7m_info;

static void timeout_init(struct timeout *to, long timeout_ms)
{
	to->start_time = timeval_ms();
	to->timeout_ms = timeout_ms;
}

static bool timeout_expired(struct timeout *to)
{
	return (timeval_ms() - to->start_time) > to->timeout_ms;
}

/** ***********************************************************************************************
 * @brief Starts a pseudo flash algorithm (an infinite loop) on the CM0+ and leaves it running.
 * This keeps the core executing so that the SROM IRQ0 handler can service DAP-issued system
 * calls. The vector table is forced to the ROM table (VTOR=0) and the IPC system-call interrupt
 * (CM0+ IRQ0) is enabled in the NVIC, since the boot ROM has not necessarily done so yet at the
 * point a debugger halts the device.
 *************************************************************************************************/
static int sromalgo_prepare(struct target *target)
{
	int hr;

	/* Point the vector table at the ROM table that holds the SROM IRQ0 handler */
	hr = target_write_u32(target, CM0_VTOR, 0x00000000);
	if (hr != ERROR_OK)
		return hr;

	/* Enable the IPC system-call interrupt (CM0+ IRQ0) in the NVIC */
	hr = target_write_u32(target, CM0_NVIC_ISER0, 0x00000001);
	if (hr != ERROR_OK)
		return hr;

	/* Enable embedded flash write operations (safety registers are not retained) */
	hr = target_write_u32(target, FLASHC_MAIN_FLASH_SAFETY, FLASH_WRITE_ENABLE);
	if (hr != ERROR_OK)
		return hr;
	hr = target_write_u32(target, FLASHC_WORK_FLASH_SAFETY, FLASH_WRITE_ENABLE);
	if (hr != ERROR_OK)
		return hr;

	/* Allocate working area for the loop and its stack */
	hr = target_alloc_working_area(target, RAM_STACK_WA_SIZE, &g_stack_area);
	if (hr != ERROR_OK)
		return hr;

	g_armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	g_armv7m_info.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params;
	init_reg_param(&reg_params, "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params.value, 0, 32, g_stack_area->address + g_stack_area->size);

	/* Helper loop: "cpsie i; b ." (0xB662, 0xE7FE).
	 * target_start_algorithm() resumes with debug_execution, which sets
	 * PRIMASK=1 to mask interrupts. The SROM system call is serviced by the
	 * CM0+ IPC IRQ0 handler, so the loop must re-enable interrupts (cpsie i)
	 * before spinning, otherwise the call never completes. */
	hr = target_write_u32(target, g_stack_area->address, 0xE7FEB662);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	hr = target_start_algorithm(target, 0, NULL, 1, &reg_params, g_stack_area->address,
			0, &g_armv7m_info);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	destroy_reg_param(&reg_params);
	return hr;

destroy_rp_free_wa:
	destroy_reg_param(&reg_params);
	target_free_working_area(target, g_stack_area);
	g_stack_area = NULL;
	return hr;
}

static void sromalgo_release(struct target *target)
{
	int hr = ERROR_OK;

	if (g_stack_area) {
		if (target->running_alg) {
			hr = target_halt(target);
			if (hr != ERROR_OK)
				goto exit_free_wa;

			hr = target_wait_algorithm(target, 0, NULL, 0, NULL, 0,
					IPC_TIMEOUT_MS, &g_armv7m_info);
			if (hr != ERROR_OK)
				goto exit_free_wa;
		}

exit_free_wa:
		target_free_working_area(target, g_stack_area);
		g_stack_area = NULL;
	}
}

static int ipc_poll_lock_stat(struct target *target, uint32_t ipc_id, bool lock_expected)
{
	int hr;
	uint32_t reg_val;

	struct timeout to;
	timeout_init(&to, IPC_TIMEOUT_MS);

	while (!timeout_expired(&to)) {
		keep_alive();

		hr = target_read_u32(target, MEM_IPC_LOCK_STATUS(ipc_id), &reg_val);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to read IPC Lock Status register");
			return hr;
		}

		bool is_locked = (reg_val & IPC_LOCK_ACQUIRED_MSK) != 0;
		if (lock_expected == is_locked)
			return ERROR_OK;
	}

	LOG_ERROR("Timeout polling IPC Lock Status");
	return ERROR_TARGET_TIMEOUT;
}

static int ipc_acquire(struct target *target, char ipc_id)
{
	int hr = ERROR_OK;
	bool is_acquired = false;
	uint32_t reg_val;

	struct timeout to;
	timeout_init(&to, IPC_TIMEOUT_MS);

	while (!timeout_expired(&to)) {
		keep_alive();

		hr = target_write_u32(target, MEM_IPC_ACQUIRE(ipc_id), IPC_ACQUIRE_SUCCESS_MSK);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to write to IPC Acquire register");
			return hr;
		}

		hr = target_read_u32(target, MEM_IPC_ACQUIRE(ipc_id), &reg_val);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to read IPC Acquire register");
			return hr;
		}

		is_acquired = (reg_val & IPC_ACQUIRE_SUCCESS_MSK) != 0;
		if (is_acquired) {
			hr = ipc_poll_lock_stat(target, ipc_id, true);
			break;
		}
	}

	if (!is_acquired)
		LOG_ERROR("Timeout acquiring IPC structure");

	return hr;
}

/** ***********************************************************************************************
 * @brief Invokes an SROM API function via IPC structure 3 (DAP).
 * @param req_and_params inline opcode (LSb set) or, for RAM-based calls, the same value is used
 *        only to decide the data location; @p working_area holds the SRAM parameter pointer.
 * @param working_area SRAM address of the parameter block (RAM-based calls), or 0 for inline.
 * @param data_out populated with the SROM status / result word.
 *************************************************************************************************/
static int call_sromapi(struct target *target,
	uint32_t req_and_params,
	uint32_t working_area,
	uint32_t *data_out)
{
	int hr;

	bool is_data_in_ram = (req_and_params & SROMAPI_DATA_LOCATION_MSK) == 0;

	hr = ipc_acquire(target, IPC_ID);
	if (hr != ERROR_OK)
		return hr;

	if (is_data_in_ram)
		hr = target_write_u32(target, MEM_IPC_DATA(IPC_ID), working_area);
	else
		hr = target_write_u32(target, MEM_IPC_DATA(IPC_ID), req_and_params);
	if (hr != ERROR_OK)
		return hr;

	/* Enable notification interrupt of IPC_INTR_STRUCT0 (CM0+) for our IPC structure */
	hr = target_write_u32(target, MEM_IPC_INTR_MASK(IPC_INTR_ID), 1u << (16 + IPC_ID));
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, MEM_IPC_NOTIFY(IPC_ID), 1);
	if (hr != ERROR_OK)
		return hr;

	/* Wait for the SROM handler to release the IPC structure */
	hr = ipc_poll_lock_stat(target, IPC_ID, false);
	if (hr != ERROR_OK)
		return hr;

	if (is_data_in_ram)
		hr = target_read_u32(target, working_area, data_out);
	else
		hr = target_read_u32(target, MEM_IPC_DATA(IPC_ID), data_out);
	if (hr != ERROR_OK) {
		LOG_ERROR("Error reading SROM API status location");
		return hr;
	}

	bool is_success = (*data_out & SROMAPI_STATUS_MSK) == SROMAPI_STAT_SUCCESS;
	if (!is_success) {
		LOG_ERROR("SROM API execution failed. Status: 0x%08" PRIX32, *data_out);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int get_silicon_id(struct target *target, uint32_t *si_id, uint8_t *protection)
{
	int hr;
	uint32_t family_rev, siid_prot;

	hr = sromalgo_prepare(target);
	if (hr != ERROR_OK)
		goto exit;

	hr = call_sromapi(target, SROMAPI_SIID_REQ_FAMILY_REVISION, 0, &family_rev);
	if (hr != ERROR_OK)
		goto exit;

	hr = call_sromapi(target, SROMAPI_SIID_REQ_SIID_PROTECTION, 0, &siid_prot);
	if (hr != ERROR_OK)
		goto exit;

	*si_id  = (siid_prot & 0x0000FFFF) << 16;
	*si_id |= (family_rev & 0x00FF0000) >> 8;
	*si_id |= (family_rev & 0x000000FF) >> 0;

	*protection = (siid_prot & 0x000F0000) >> 0x10;

exit:
	sromalgo_release(target);
	return hr;
}

static int xmc7xxx_protect_check(struct flash_bank *bank)
{
	int is_protected;

	struct xmc7xxx_target_info *info = bank->driver_priv;
	int hr = get_silicon_id(bank->target, &info->silicon_id, &info->protection);
	if (hr != ERROR_OK)
		return hr;

	switch (info->protection) {
	case PROTECTION_VIRGIN:
	case PROTECTION_NORMAL:
		is_protected = 0;
		break;
	case PROTECTION_UNKNOWN:
	case PROTECTION_SECURE:
	case PROTECTION_DEAD:
	default:
		is_protected = 1;
		break;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = is_protected;

	return ERROR_OK;
}

static int xmc7xxx_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	(void)bank;
	(void)set;
	(void)first;
	(void)last;

	LOG_WARNING("Life Cycle transition for XMC7000 is not supported");
	return ERROR_OK;
}

static const char *protection_to_str(uint8_t protection)
{
	switch (protection) {
	case PROTECTION_VIRGIN:
		return "VIRGIN";
	case PROTECTION_NORMAL:
		return "NORMAL";
	case PROTECTION_SECURE:
		return "SECURE";
	case PROTECTION_DEAD:
		return "DEAD";
	case PROTECTION_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

static int xmc7xxx_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct xmc7xxx_target_info *info = bank->driver_priv;

	if (!info->is_probed)
		return ERROR_FAIL;

	int hr = get_silicon_id(bank->target, &info->silicon_id, &info->protection);
	if (hr != ERROR_OK)
		return hr;

	command_print_sameline(cmd,
		"XMC7000 Silicon ID: 0x%08" PRIX32 "\n"
		"Protection: %s\n"
		"%s flash size: %" PRIu32 " kB\n",
		info->silicon_id,
		protection_to_str(info->protection),
		info->is_work ? "Work" : "Main",
		(uint32_t)(bank->size / 1024));

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Builds the (non-uniform) sector map for a Code or Work flash bank. The array is laid out
 * as a region of large sectors followed by a region of small sectors at the top of the bank.
 *************************************************************************************************/
static int xmc7xxx_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct xmc7xxx_target_info *info = bank->driver_priv;

	uint32_t large_sz, small_sz, small_region;

	if (bank->base == MEM_BASE_MFLASH) {
		info->is_work = false;
		large_sz = MFLASH_LARGE_SECTOR;
		small_sz = MFLASH_SMALL_SECTOR;
		small_region = MFLASH_SMALL_REGION;
		if (bank->size == 0)
			bank->size = MFLASH_DEFAULT_SIZE;
	} else if (bank->base == MEM_BASE_WFLASH) {
		info->is_work = true;
		large_sz = WFLASH_LARGE_SECTOR;
		small_sz = WFLASH_SMALL_SECTOR;
		if (bank->size == 0)
			bank->size = WFLASH_DEFAULT_SIZE;
		small_region = bank->size / 4;
	} else {
		LOG_ERROR("xmc7xxx: flash bank base must be 0x%08X (main) or 0x%08X (work)",
			MEM_BASE_MFLASH, MEM_BASE_WFLASH);
		return ERROR_FLASH_BANK_INVALID;
	}

	(void)target;

	if (small_region > bank->size)
		small_region = bank->size;
	uint32_t large_region = bank->size - small_region;

	unsigned int num_large = large_region / large_sz;
	unsigned int num_small = small_region / small_sz;
	unsigned int num_sectors = num_large + num_small;

	if (num_sectors == 0) {
		LOG_ERROR("xmc7xxx: invalid flash bank size 0x%08" PRIx32, (uint32_t)bank->size);
		return ERROR_FLASH_BANK_INVALID;
	}

	free(bank->sectors);
	bank->sectors = calloc(num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors)
		return ERROR_FAIL;

	bank->num_sectors = num_sectors;
	bank->erased_value = 0xFF;
	bank->default_padded_value = 0xFF;

	uint32_t offset = 0;
	for (unsigned int i = 0; i < num_large; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = large_sz;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
		offset += large_sz;
	}
	for (unsigned int i = 0; i < num_small; i++) {
		unsigned int idx = num_large + i;
		bank->sectors[idx].offset = offset;
		bank->sectors[idx].size = small_sz;
		bank->sectors[idx].is_erased = -1;
		bank->sectors[idx].is_protected = -1;
		offset += small_sz;
	}

	info->is_probed = true;
	return ERROR_OK;
}

static int xmc7xxx_auto_probe(struct flash_bank *bank)
{
	struct xmc7xxx_target_info *info = bank->driver_priv;

	if (info->is_probed)
		return ERROR_OK;

	return xmc7xxx_probe(bank);
}

/** ***********************************************************************************************
 * @brief Erases a single flash sector. EraseSector erases the hardware sector that contains the
 * given address, so passing each sector's base address erases exactly that sector regardless of
 * its (large/small) size.
 *************************************************************************************************/
static int xmc7xxx_erase_sector(struct flash_bank *bank, struct working_area *wa, uint32_t addr)
{
	struct target *target = bank->target;
	uint32_t data_out;
	int hr;

	LOG_DEBUG("Erasing SECTOR @%08" PRIX32, addr);

	hr = target_write_u32(target, wa->address + 0x00, SROMAPI_ERASESECTOR_REQ);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, wa->address + 0x04, addr);
	if (hr != ERROR_OK)
		return hr;

	hr = call_sromapi(target, SROMAPI_ERASESECTOR_REQ, wa->address, &data_out);
	if (hr != ERROR_OK)
		LOG_ERROR("SECTOR @%08" PRIX32 " not erased!", addr);

	return hr;
}

static int xmc7xxx_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct working_area *wa = NULL;
	int hr;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	hr = sromalgo_prepare(target);
	if (hr != ERROR_OK)
		goto exit;

	hr = target_alloc_working_area(target, 64, &wa);
	if (hr != ERROR_OK)
		goto exit;

	for (unsigned int i = first; i <= last; i++) {
		hr = xmc7xxx_erase_sector(bank, wa, bank->base + bank->sectors[i].offset);
		if (hr != ERROR_OK)
			goto exit_free_wa;
	}

exit_free_wa:
	target_free_working_area(target, wa);
exit:
	sromalgo_release(target);
	return hr;
}

/** ***********************************************************************************************
 * @brief Programs a single flash page. The SROM parameter block is laid out at the start of the
 * working area, followed by the page data which the block points to.
 *************************************************************************************************/
static int xmc7xxx_program_page(struct flash_bank *bank,
	struct working_area *wa,
	uint32_t addr,
	const uint8_t *buffer,
	uint32_t page_size)
{
	struct target *target = bank->target;
	struct xmc7xxx_target_info *info = bank->driver_priv;
	const uint32_t opcode = info->is_work ?
			SROMAPI_PROGRAMWORKFLASH_REQ : SROMAPI_PROGRAMROW_REQ;
	uint32_t data_out;
	int hr;

	LOG_DEBUG("Programming PAGE @%08" PRIX32, addr);

	/* SRAM_SCRATCH_ADDR + 0x00: opcode, perform blank check, blocking */
	hr = target_write_u32(target, wa->address + 0x00, opcode);
	if (hr != ERROR_OK)
		return hr;

	/* SRAM_SCRATCH_ADDR + 0x04: no FM interrupt mask, data size = 4096 bits (512 B) */
	hr = target_write_u32(target, wa->address + 0x04, SROMAPI_DSIZE_512B);
	if (hr != ERROR_OK)
		return hr;

	/* SRAM_SCRATCH_ADDR + 0x08: destination flash address */
	hr = target_write_u32(target, wa->address + 0x08, addr);
	if (hr != ERROR_OK)
		return hr;

	/* SRAM_SCRATCH_ADDR + 0x0C: pointer to the page data */
	hr = target_write_u32(target, wa->address + 0x0C, wa->address + 0x10);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_buffer(target, wa->address + 0x10, page_size, buffer);
	if (hr != ERROR_OK)
		return hr;

	hr = call_sromapi(target, opcode, wa->address, &data_out);
	if (hr != ERROR_OK)
		LOG_ERROR("PAGE @%08" PRIX32 " not programmed!", addr);

	return hr;
}

static int xmc7xxx_program(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	struct target *target = bank->target;
	struct xmc7xxx_target_info *info = bank->driver_priv;
	const uint32_t page_size = info->is_work ? WFLASH_PROGRAM_SIZE : MFLASH_PROGRAM_SIZE;
	struct working_area *wa = NULL;
	uint8_t *page_buf = NULL;
	int hr;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	page_buf = malloc(page_size);
	if (!page_buf)
		return ERROR_FAIL;

	hr = sromalgo_prepare(target);
	if (hr != ERROR_OK)
		goto exit;

	hr = target_alloc_working_area(target, page_size + 0x10, &wa);
	if (hr != ERROR_OK)
		goto exit;

	while (count) {
		uint32_t page_offset = offset % page_size;
		uint32_t aligned_addr = bank->base + offset - page_offset;
		uint32_t page_bytes = MIN(page_size - page_offset, count);

		memset(page_buf, bank->erased_value, page_size);
		memcpy(&page_buf[page_offset], buffer, page_bytes);

		/* The caller erases the spanned sectors before programming, so a page
		 * that is entirely the erased value is already in its final state.
		 * Skip it to avoid a needless program cycle (e.g. the padding that
		 * flash write_image adds to round the image up to the erase block). */
		bool page_blank = true;
		for (uint32_t i = 0; i < page_size; i++) {
			if (page_buf[i] != bank->erased_value) {
				page_blank = false;
				break;
			}
		}

		if (!page_blank) {
			hr = xmc7xxx_program_page(bank, wa, aligned_addr, page_buf, page_size);
			if (hr != ERROR_OK) {
				LOG_ERROR("Failed to program flash at address 0x%08" PRIX32, aligned_addr);
				goto exit_free_wa;
			}
		}

		buffer += page_bytes;
		offset += page_bytes;
		count -= page_bytes;
	}

exit_free_wa:
	target_free_working_area(target, wa);
exit:
	sromalgo_release(target);
	free(page_buf);
	return hr;
}

COMMAND_HANDLER(xmc7xxx_handle_mass_erase_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int hr = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (hr != ERROR_OK)
		return hr;

	return xmc7xxx_erase(bank, 0, bank->num_sectors - 1);
}

FLASH_BANK_COMMAND_HANDLER(xmc7xxx_flash_bank_command)
{
	struct xmc7xxx_target_info *info;
	int hr = ERROR_OK;

	if (CMD_ARGC < 6) {
		hr = ERROR_COMMAND_SYNTAX_ERROR;
	} else {
		info = calloc(1, sizeof(struct xmc7xxx_target_info));
		if (!info)
			return ERROR_FAIL;
		info->is_probed = false;
		bank->driver_priv = info;
	}
	return hr;
}

static const struct command_registration xmc7xxx_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = xmc7xxx_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank",
		.help = "Erases all sectors of the given flash bank",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration xmc7xxx_command_handlers[] = {
	{
		.name = "xmc7xxx",
		.mode = COMMAND_ANY,
		.help = "XMC7000 flash command group",
		.usage = "",
		.chain = xmc7xxx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver xmc7xxx_flash = {
	.name = "xmc7xxx",
	.commands = xmc7xxx_command_handlers,
	.flash_bank_command = xmc7xxx_flash_bank_command,
	.erase = xmc7xxx_erase,
	.protect = xmc7xxx_protect,
	.write = xmc7xxx_program,
	.read = default_flash_read,
	.probe = xmc7xxx_probe,
	.auto_probe = xmc7xxx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = xmc7xxx_protect_check,
	.info = xmc7xxx_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
