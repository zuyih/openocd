// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Flash driver of the WCH CH579, a Bluetooth LE MCU with Cortex-M0 core.
 * The WCH Keil DFP calls it and its siblings CH577/CH578 "CH57x", but WCH
 * uses that name for the RISC-V CH571/CH573 elsewhere, so this driver is
 * named after the chip. Other chips with the same flash controller can be
 * added to ch579_chips.
 *
 * The code flash at 0 and the data flash behind it are erased in 512 byte
 * sectors and programmed one word at a time through the flash registers,
 * as the flash algorithm CH57x.FLM of the WCH Keil DFP does.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* flash registers */

#define CH579_FLASH_REG_BASE	0x40001800

#define CH579_FLASH_DATA	0x00
#define CH579_FLASH_ADDR	0x04
#define CH579_FLASH_COMMAND	0x08
#define CH579_FLASH_PROTECT	0x09
#define CH579_FLASH_STATUS	0x0A

#define CH579_CMD_PROG		0x9A
#define CH579_CMD_ERASE		0xA6

/* FLASH_PROTECT: bits 7..6 must be written with 10 */
#define CH579_PROTECT_MUST_10	0x80
#define CH579_PROTECT_DATA_WE	BIT(2)
#define CH579_PROTECT_CODE_WE	BIT(3)

/* FLASH_STATUS */
#define CH579_STATUS_TOUT	BIT(0)
#define CH579_STATUS_CMD_ERR	BIT(1)
#define CH579_STATUS_ADDR_OK	BIT(6)
#define CH579_STATUS_READ_FREE	BIT(8)

/* system registers */

#define CH579_CHIP_ID		0x40001041
#define CH579_GLOB_CFG_INFO	0x40001045

/* GLOB_CFG_INFO: after power on the boot loader runs and is mapped at 0, a
 * reset by SYSRESETREQ starts the application with the code flash at 0 */
#define CH579_CFG_BOOT_LOADER	BIT(5)

#define CH579_SECTOR_SIZE	512

struct ch579_chip {
	uint8_t chip_id;
	const char *name;
	uint32_t code_size;
	uint32_t data_addr;
	uint32_t data_size;
};

/* from the CH579 datasheet and the WCH SDK (ID_CH579, DATA_FLASH_ADDR) */
static const struct ch579_chip ch579_chips[] = {
	{
		.chip_id = 0x79,
		.name = "CH579",
		.code_size = 0x3E800,
		.data_addr = 0x3E800,
		.data_size = 0x800,
	},
};

static const uint8_t ch579_flash_write_code[] = {
#include "../../../contrib/loaders/flash/wch/ch579.inc"
};

struct ch579_flash_bank {
	bool probed;
	const struct ch579_chip *chip;
	/* write enable of the FLASH_PROTECT register for this bank */
	uint8_t write_enable;
};

/* flash bank ch579 <base> <size> 0 0 <target#>
 * <base> is 0 for the code flash and the data flash address for the data
 * flash, the size is probed.
 */
FLASH_BANK_COMMAND_HANDLER(ch579_flash_bank_command)
{
	struct ch579_flash_bank *ch579_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch579_info = calloc(1, sizeof(struct ch579_flash_bank));
	if (!ch579_info)
		return ERROR_FAIL;

	bank->driver_priv = ch579_info;

	/* the flash is programmed one word at a time */
	bank->write_start_alignment = 4;
	bank->write_end_alignment = 4;

	return ERROR_OK;
}

static int ch579_set_protect(struct flash_bank *bank, uint8_t write_enable)
{
	return target_write_u8(bank->target, CH579_FLASH_REG_BASE + CH579_FLASH_PROTECT,
			CH579_PROTECT_MUST_10 | write_enable);
}

static int ch579_check_status(struct flash_bank *bank, const char *operation, uint32_t address)
{
	uint8_t status;

	int retval = target_read_u8(bank->target, CH579_FLASH_REG_BASE + CH579_FLASH_STATUS, &status);
	if (retval != ERROR_OK)
		return retval;

	if ((status & (CH579_STATUS_TOUT | CH579_STATUS_CMD_ERR | CH579_STATUS_ADDR_OK)) !=
			CH579_STATUS_ADDR_OK) {
		LOG_ERROR("ch579 %s failed at address 0x%08" PRIx32 " (status 0x%02" PRIx8 ")",
				operation, address, status);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int ch579_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct ch579_flash_bank *ch579_info = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = ch579_set_protect(bank, ch579_info->write_enable);
	if (retval != ERROR_OK)
		goto flash_lock;

	for (unsigned int i = first; i <= last; i++) {
		uint32_t address = bank->base + bank->sectors[i].offset;

		retval = target_write_u32(target, CH579_FLASH_REG_BASE + CH579_FLASH_ADDR, address);
		if (retval != ERROR_OK)
			goto flash_lock;
		/* the erase completes before the next access to the flash registers */
		retval = target_write_u8(target, CH579_FLASH_REG_BASE + CH579_FLASH_COMMAND,
				CH579_CMD_ERASE);
		if (retval != ERROR_OK)
			goto flash_lock;
		retval = ch579_check_status(bank, "erase", address);
		if (retval != ERROR_OK)
			goto flash_lock;
	}

flash_lock:
	{
		int retval2 = ch579_set_protect(bank, 0);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int ch579_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t words_count)
{
	struct target *target = bank->target;
	uint32_t buffer_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct armv7m_algorithm armv7m_info;
	int retval;

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(ch579_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(ch579_flash_write_code), ch579_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	buffer_size = target_get_working_area_avail(target);
	buffer_size = MIN((words_count + 1) * 4 + 8, MAX(buffer_size, 256));
	/* Normally we allocate all available working area.
	 * MIN shrinks buffer_size if the size of the written block is smaller,
	 * a full FIFO keeps one word free, so it needs one word more.
	 * MAX prevents using async algo if the available working area is smaller
	 * than 256, the following allocation fails with
	 * ERROR_TARGET_RESOURCE_NOT_AVAILABLE and slow flashing takes place.
	 */

	retval = target_alloc_working_area(target, buffer_size, &source);
	/* Allocated size is always 32-bit word aligned */
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		/* target_alloc_working_area() may return ERROR_FAIL if area backup fails:
		 * convert any error to ERROR_TARGET_RESOURCE_NOT_AVAILABLE
		 */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct reg_param reg_params[5];

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (words) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, CH579_FLASH_REG_BASE);
	buf_set_u32(reg_params[1].value, 0, 32, words_count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, words_count, 4,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		/* The algorithm returns the flash status in r0 only if properly
		 * finished, re-read the status register to report the errors */
		uint32_t failed_address = buf_get_u32(reg_params[4].value, 0, 32);
		if (ch579_check_status(bank, "write", failed_address) == ERROR_OK)
			LOG_ERROR("flash write failed just before address 0x%" PRIx32, failed_address);
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	return retval;
}

/* program one word at a time through the flash registers */
static int ch579_write_words(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t words_count)
{
	struct target *target = bank->target;

	for (uint32_t i = 0; i < words_count; i++) {
		int retval = target_write_u32(target, CH579_FLASH_REG_BASE + CH579_FLASH_ADDR, address);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, CH579_FLASH_REG_BASE + CH579_FLASH_DATA,
				target_buffer_get_u32(target, buffer));
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u8(target, CH579_FLASH_REG_BASE + CH579_FLASH_COMMAND,
				CH579_CMD_PROG);
		if (retval != ERROR_OK)
			return retval;
		retval = ch579_check_status(bank, "write", address);
		if (retval != ERROR_OK)
			return retval;

		buffer += 4;
		address += 4;
	}

	return ERROR_OK;
}

static int ch579_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct ch579_flash_bank *ch579_info = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The flash write must be aligned to a word boundary.
	 * The flash infrastructure ensures it, do just a security check
	 */
	assert(offset % 4 == 0);
	assert(count % 4 == 0);

	int retval = ch579_set_protect(bank, ch579_info->write_enable);
	if (retval != ERROR_OK)
		goto flash_lock;

	retval = ch579_write_block_async(bank, buffer, bank->base + offset, count / 4);
	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single word accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
		retval = ch579_write_words(bank, buffer, bank->base + offset, count / 4);
	}

flash_lock:
	{
		int retval2 = ch579_set_protect(bank, 0);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int ch579_probe(struct flash_bank *bank)
{
	struct ch579_flash_bank *ch579_info = bank->driver_priv;
	struct target *target = bank->target;
	const struct ch579_chip *chip = NULL;
	uint8_t chip_id, cfg_info;
	uint32_t size;

	ch579_info->probed = false;

	int retval = target_read_u8(target, CH579_CHIP_ID, &chip_id);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < ARRAY_SIZE(ch579_chips); i++) {
		if (ch579_chips[i].chip_id == chip_id) {
			chip = &ch579_chips[i];
			break;
		}
	}
	if (!chip) {
		LOG_ERROR("Cannot identify target as a CH579 (chip id 0x%02" PRIx8 ")", chip_id);
		return ERROR_FAIL;
	}

	if (bank->base == 0) {
		size = chip->code_size;
		ch579_info->write_enable = CH579_PROTECT_CODE_WE;
	} else if (bank->base == chip->data_addr) {
		size = chip->data_size;
		ch579_info->write_enable = CH579_PROTECT_DATA_WE;
	} else {
		LOG_ERROR("%s has no flash at " TARGET_ADDR_FMT ", the code flash is at 0 and the "
				"data flash at 0x%" PRIx32, chip->name, bank->base, chip->data_addr);
		return ERROR_FAIL;
	}

	retval = target_read_u8(target, CH579_GLOB_CFG_INFO, &cfg_info);
	if (retval != ERROR_OK)
		return retval;
	if (cfg_info & CH579_CFG_BOOT_LOADER)
		LOG_WARNING("%s runs the boot loader, it is mapped at 0 instead of the code flash; "
				"reset the target to read the flash", chip->name);

	LOG_INFO("%s %s flash: %" PRIu32 " bytes", chip->name,
			bank->base == 0 ? "code" : "data", size);

	free(bank->sectors);
	bank->size = size;
	bank->num_sectors = size / CH579_SECTOR_SIZE;
	bank->sectors = alloc_block_array(0, CH579_SECTOR_SIZE, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	ch579_info->chip = chip;
	ch579_info->probed = true;

	return ERROR_OK;
}

static int ch579_auto_probe(struct flash_bank *bank)
{
	struct ch579_flash_bank *ch579_info = bank->driver_priv;
	if (ch579_info->probed)
		return ERROR_OK;
	return ch579_probe(bank);
}

static int get_ch579_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct ch579_flash_bank *ch579_info = bank->driver_priv;
	uint16_t status;

	if (!ch579_info->probed) {
		command_print_sameline(cmd, "Unable to find bank information.");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "%s %s flash", ch579_info->chip->name,
			bank->base == 0 ? "code" : "data");

	if (target_read_u16(bank->target, CH579_FLASH_REG_BASE + CH579_FLASH_STATUS,
			&status) == ERROR_OK)
		command_print_sameline(cmd, ", read protection %s",
				(status & CH579_STATUS_READ_FREE) ? "off" : "on");

	return ERROR_OK;
}

const struct flash_driver ch579_flash = {
	.name = "ch579",
	.flash_bank_command = ch579_flash_bank_command,
	.erase = ch579_erase,
	.write = ch579_write,
	.read = default_flash_read,
	.probe = ch579_probe,
	.auto_probe = ch579_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = get_ch579_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
