// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 ***************************************************************************/

/*
 * Flash driver of the WCH CH32F10x and CH32F20x (Cortex-M3).
 *
 * Driver and command names are the ones of the "wch_arm" driver of the WCH
 * fork of OpenOCD, the flash is written with the fast page programming of
 * these chips. Derived from stm32f1x.c, the flash controller is an extended
 * STM32F1 one.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/align.h>
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* flash registers */

#define CH32X_FLASH_REG_BASE	0x40022000

#define CH32X_FLASH_KEYR	0x04
#define CH32X_FLASH_SR		0x0C
#define CH32X_FLASH_CR		0x10
#define CH32X_FLASH_AR		0x14
#define CH32X_FLASH_OBR		0x1C
#define CH32X_FLASH_WRPR	0x20
#define CH32X_FLASH_MODEKEYR	0x24

/* FLASH_CR register bits */

#define FLASH_PG		BIT(0)
#define FLASH_PER		BIT(1)
#define FLASH_MER		BIT(2)
#define FLASH_STRT		BIT(6)
#define FLASH_LOCK		BIT(7)
#define FLASH_FLOCK		BIT(15)	/* fast programming mode lock */

/* FLASH_SR register bits */

#define FLASH_BSY		BIT(0)
#define FLASH_PGERR		BIT(2)	/* CH32F10x only */
#define FLASH_WRPRTERR		BIT(4)

/* FLASH_OBR register bits */

#define FLASH_OBR_RDPRT		BIT(1)

/* register unlock keys */

#define KEY1			0x45670123
#define KEY2			0xCDEF89AB

/* device information */

#define CH32F10X_CHIPID		0x1FFFF884
#define CH32F20X_CHIPID		0x1FFFF704
#define CH32X_FLASH_SIZE	0x1FFFF7E0

/* erased flash reads 0xe339 on the CH32F20x */
#define CH32X_ERASED_HALFWORD	0xe339

/* default flash size of the WCH driver, if the size register is not set */
#define CH32X_DEFAULT_FLASH_SIZE_KB	128

/* one bit of FLASH_WRPR protects 4 KiB, the last one the rest of the flash */
#define CH32X_PROT_BLOCK_SIZE	4096
#define CH32X_MAX_PROT_BLOCKS	32

/* timeout values */

#define FLASH_WRITE_TIMEOUT	10
#define FLASH_ERASE_TIMEOUT	100
#define FLASH_MASS_ERASE_TIMEOUT	1000
/* the flash is not ready yet when BSY goes low after a mass erase, the WCH
 * driver and the WCH SDK wait some time after it */
#define FLASH_MASS_ERASE_DELAY	300

static const uint8_t ch32f1x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/wch/ch32f1x.inc"
};

static const uint8_t ch32f2x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/wch/ch32f2x.inc"
};

struct ch32x_family {
	const char *name;
	/* standard erase page */
	uint32_t page_size;
	/* fast programming page */
	uint32_t fast_page_size;
	/* FLASH_SR error bits */
	uint32_t error_mask;
	const uint8_t *write_code;
	size_t write_code_size;
};

static const struct ch32x_family ch32f10x_family = {
	.name = "CH32F10x",
	.page_size = 1024,
	.fast_page_size = 128,
	.error_mask = FLASH_PGERR | FLASH_WRPRTERR,
	.write_code = ch32f1x_flash_write_code,
	.write_code_size = sizeof(ch32f1x_flash_write_code),
};

static const struct ch32x_family ch32f20x_family = {
	.name = "CH32F20x",
	.page_size = 4096,
	.fast_page_size = 256,
	.error_mask = FLASH_WRPRTERR,
	.write_code = ch32f2x_flash_write_code,
	.write_code_size = sizeof(ch32f2x_flash_write_code),
};

struct ch32x_flash_bank {
	bool probed;
	const struct ch32x_family *family;
	uint32_t chip_id;
	uint32_t user_bank_size;
};

static int ch32x_mass_erase(struct flash_bank *bank);

/* flash bank wch_arm <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(ch32x_flash_bank_command)
{
	struct ch32x_flash_bank *ch32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch32x_info = calloc(1, sizeof(struct ch32x_flash_bank));
	if (!ch32x_info)
		return ERROR_FAIL;

	bank->driver_priv = ch32x_info;
	ch32x_info->user_bank_size = bank->size;

	/* The flash write must be aligned to a halfword boundary */
	bank->write_start_alignment = 2;
	bank->write_end_alignment = 2;

	return ERROR_OK;
}

static int ch32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	uint32_t error_mask = ch32x_info->family ? ch32x_info->family->error_mask : FLASH_WRPRTERR;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = target_read_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_SR, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32, status);
		if ((status & FLASH_BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FLASH_BUSY;
		}
		alive_sleep(1);
	}

	if (status & FLASH_WRPRTERR) {
		LOG_ERROR("ch32x device protected");
		retval = ERROR_FLASH_PROTECTED;
	}

	if (status & error_mask & FLASH_PGERR) {
		LOG_ERROR("ch32x device programming failed / flash not erased");
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

	/* Clear but report errors */
	if (status & error_mask) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_SR, status & error_mask);
	}
	return retval;
}

static int ch32x_unlock(struct target *target, bool fast_mode)
{
	int retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK || !fast_mode)
		return retval;

	retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_MODEKEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;
	return target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_MODEKEYR, KEY2);
}

static int ch32x_lock(struct target *target)
{
	return target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_CR, FLASH_LOCK | FLASH_FLOCK);
}

static int ch32x_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t protection;

	int retval = target_read_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_WRPR, &protection);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = (protection & (1 << i)) ? 0 : 1;

	return ERROR_OK;
}

static int ch32x_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (first == 0 && last == bank->num_sectors - 1)
		return ch32x_mass_erase(bank);

	int retval = ch32x_unlock(target, false);
	if (retval != ERROR_OK)
		goto flash_lock;

	for (unsigned int i = first; i <= last; i++) {
		retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_CR, FLASH_PER);
		if (retval != ERROR_OK)
			goto flash_lock;
		retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_AR,
				bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK)
			goto flash_lock;
		retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_CR,
				FLASH_PER | FLASH_STRT);
		if (retval != ERROR_OK)
			goto flash_lock;

		retval = ch32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			goto flash_lock;
	}

flash_lock:
	{
		int retval2 = ch32x_lock(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int ch32x_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t page_count)
{
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	const struct ch32x_family *family = ch32x_info->family;
	struct target *target = bank->target;
	uint32_t page_size = family->fast_page_size;
	uint32_t buffer_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct armv7m_algorithm armv7m_info;
	int retval;

	/* flash write code */
	if (target_alloc_working_area(target, family->write_code_size,
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			family->write_code_size, family->write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer: the FIFO is a multiple of the page size, so the
	 * algorithm always finds a whole page at the read pointer. A full FIFO
	 * keeps one page free, it needs at least two pages. */
	buffer_size = target_get_working_area_avail(target);
	buffer_size = MIN((page_count + 1) * page_size + 8, buffer_size);
	if (buffer_size < 2 * page_size + 8) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	buffer_size = 8 + ALIGN_DOWN(buffer_size - 8, page_size);

	retval = target_alloc_working_area(target, buffer_size, &source);
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
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (pages) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, CH32X_FLASH_REG_BASE);
	buf_set_u32(reg_params[1].value, 0, 32, page_count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, page_count, page_size,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		/* The algorithm returns the flash status in r0 only if properly
		 * finished, re-read the status register to report the errors */
		int retval2 = ch32x_wait_status_busy(bank, 5);
		if (retval2 != ERROR_OK)
			retval = retval2;

		LOG_ERROR("flash write failed just before address 0x%" PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	return retval;
}

/* standard programming, one halfword at a time */
static int ch32x_write_halfwords(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t count)
{
	struct target *target = bank->target;

	int retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_CR, FLASH_PG);
	if (retval != ERROR_OK)
		return retval;

	for (uint32_t i = 0; i < count; i += 2) {
		retval = target_write_memory(target, address + i, 2, 1, buffer + i);
		if (retval != ERROR_OK)
			return retval;

		retval = ch32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/* Erased flash reads 0xe339 on the CH32F20x, 0xffff (as on the STM32F1) is
 * taken as erased as well. The single byte bank->erased_value of the flash
 * infrastructure can not express it. */
static bool ch32x_is_erased(struct target *target, const uint8_t *buffer, uint32_t count)
{
	for (uint32_t i = 0; i < count; i += 2) {
		uint16_t value = target_buffer_get_u16(target, buffer + i);
		if (value != 0xffff && value != CH32X_ERASED_HALFWORD)
			return false;
	}
	return true;
}

static int ch32x_page_is_erased(struct flash_bank *bank, uint32_t address, bool *erased)
{
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	uint32_t page_size = ch32x_info->family->fast_page_size;
	uint8_t page[256];

	assert(page_size <= sizeof(page));
	int retval = target_read_buffer(bank->target, address, page_size, page);
	if (retval != ERROR_OK)
		return retval;

	*erased = ch32x_is_erased(bank->target, page, page_size);
	return ERROR_OK;
}

static int ch32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t page_size = ch32x_info->family->fast_page_size;
	uint8_t *new_buffer = NULL;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The fast programming writes whole pages, and a page has to be erased
	 * before. Partial pages at the start and at the end are padded with the
	 * erased value if the page is erased, else their data is written with
	 * standard programming. */
	uint32_t data_end = offset + count;
	uint32_t start = ALIGN_DOWN(offset, page_size);
	uint32_t end = ALIGN_UP(data_end, page_size);
	bool head_erased = true;
	bool tail_erased = true;

	if (start != offset) {
		retval = ch32x_page_is_erased(bank, bank->base + start, &head_erased);
		if (retval != ERROR_OK)
			return retval;
	}
	if (end != data_end) {
		if (start != offset && end - page_size == start) {
			tail_erased = head_erased;
		} else {
			retval = ch32x_page_is_erased(bank, bank->base + end - page_size, &tail_erased);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	uint32_t fast_start = head_erased ? start : start + page_size;
	uint32_t fast_end = tail_erased ? end : end - page_size;

	if (start != offset || end != data_end) {
		new_buffer = malloc(end - start);
		if (!new_buffer) {
			LOG_ERROR("No memory for padded buffer");
			return ERROR_FAIL;
		}
		memset(new_buffer, bank->erased_value, end - start);
		memcpy(new_buffer + offset - start, buffer, count);
	}

	retval = ch32x_unlock(target, true);
	if (retval != ERROR_OK)
		goto flash_lock;

	if (fast_start >= fast_end) {
		retval = ch32x_write_halfwords(bank, buffer, bank->base + offset, count);
		goto flash_lock;
	}

	retval = ch32x_write_block_async(bank,
			new_buffer ? new_buffer + fast_start - start : buffer,
			bank->base + fast_start, (fast_end - fast_start) / page_size);
	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
		retval = ch32x_write_halfwords(bank, buffer, bank->base + offset, count);
		goto flash_lock;
	}
	if (retval != ERROR_OK)
		goto flash_lock;

	if (offset < fast_start) {
		retval = ch32x_write_halfwords(bank, buffer, bank->base + offset,
				fast_start - offset);
		if (retval != ERROR_OK)
			goto flash_lock;
	}
	if (fast_end < data_end)
		retval = ch32x_write_halfwords(bank, buffer + fast_end - offset,
				bank->base + fast_end, data_end - fast_end);

flash_lock:
	{
		int retval2 = ch32x_lock(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	free(new_buffer);
	return retval;
}

static int ch32x_erase_check(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint8_t *buffer = malloc(bank->sectors[0].size);
	if (!buffer) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	int retval = ERROR_OK;
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		struct flash_sector *sector = &bank->sectors[i];

		retval = target_read_buffer(target, bank->base + sector->offset, sector->size, buffer);
		if (retval != ERROR_OK)
			break;
		sector->is_erased = ch32x_is_erased(target, buffer, sector->size) ? 1 : 0;
	}

	free(buffer);
	return retval;
}

static int ch32x_get_device_id(struct flash_bank *bank, uint32_t *chip_id,
		const struct ch32x_family **family)
{
	struct target *target = bank->target;

	/* the same order of checks as the WCH driver */
	int retval = target_read_u32(target, CH32F10X_CHIPID, chip_id);
	if (retval != ERROR_OK)
		return retval;
	if ((*chip_id >> 24) == 0x20) {
		*family = &ch32f10x_family;
		return ERROR_OK;
	}

	retval = target_read_u32(target, CH32F20X_CHIPID, chip_id);
	if (retval != ERROR_OK)
		return retval;
	switch (*chip_id >> 20) {
	case 0x203:
	case 0x205:
	case 0x207:
	case 0x208:
		*family = &ch32f20x_family;
		return ERROR_OK;
	}

	LOG_ERROR("Cannot identify target as a CH32F10x or CH32F20x (chip id 0x%08" PRIx32 ")",
			*chip_id);
	return ERROR_FAIL;
}

static int ch32x_probe(struct flash_bank *bank)
{
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	const struct ch32x_family *family;
	uint32_t chip_id;
	uint16_t flash_size_in_kb;

	ch32x_info->probed = false;

	int retval = ch32x_get_device_id(bank, &chip_id, &family);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 " (%s)", chip_id, family->name);

	retval = target_read_u16(bank->target, CH32X_FLASH_SIZE, &flash_size_in_kb);
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("ch32x flash size failed, probe inaccurate - assuming %d KiB flash",
				CH32X_DEFAULT_FLASH_SIZE_KB);
		flash_size_in_kb = CH32X_DEFAULT_FLASH_SIZE_KB;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (ch32x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = ch32x_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %d KiB", flash_size_in_kb);

	unsigned int num_pages = flash_size_in_kb * 1024 / family->page_size;
	if (num_pages == 0) {
		LOG_ERROR("flash size of %d KiB is too small", flash_size_in_kb);
		return ERROR_FAIL;
	}

	free(bank->sectors);
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	bank->size = num_pages * family->page_size;
	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, family->page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* each bit of FLASH_WRPR refers to a 4 KiB protection block,
	 * bit 31 refers to all remaining sectors */
	unsigned int num_prot_blocks = DIV_ROUND_UP(bank->size, CH32X_PROT_BLOCK_SIZE);
	num_prot_blocks = MIN(num_prot_blocks, CH32X_MAX_PROT_BLOCKS);
	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, CH32X_PROT_BLOCK_SIZE, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;
	struct flash_sector *last_block = &bank->prot_blocks[num_prot_blocks - 1];
	last_block->size = bank->size - last_block->offset;

	ch32x_info->family = family;
	ch32x_info->chip_id = chip_id;
	ch32x_info->probed = true;

	return ERROR_OK;
}

static int ch32x_auto_probe(struct flash_bank *bank)
{
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	if (ch32x_info->probed)
		return ERROR_OK;
	return ch32x_probe(bank);
}

static int get_ch32x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	uint32_t obr;

	if (!ch32x_info->probed) {
		command_print_sameline(cmd, "Unable to find bank information.");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "%s - chip id 0x%08" PRIx32,
			ch32x_info->family->name, ch32x_info->chip_id);

	if (target_read_u32(bank->target, CH32X_FLASH_REG_BASE + CH32X_FLASH_OBR, &obr) == ERROR_OK)
		command_print_sameline(cmd, ", read protection %s",
				(obr & FLASH_OBR_RDPRT) ? "on" : "off");

	return ERROR_OK;
}

static int ch32x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = ch32x_unlock(target, false);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* mass erase flash memory */
	retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_CR, FLASH_MER);
	if (retval != ERROR_OK)
		goto flash_lock;
	retval = target_write_u32(target, CH32X_FLASH_REG_BASE + CH32X_FLASH_CR,
			FLASH_MER | FLASH_STRT);
	if (retval != ERROR_OK)
		goto flash_lock;

	retval = ch32x_wait_status_busy(bank, FLASH_MASS_ERASE_TIMEOUT);
	if (retval == ERROR_OK)
		alive_sleep(FLASH_MASS_ERASE_DELAY);

flash_lock:
	{
		int retval2 = ch32x_lock(target);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

COMMAND_HANDLER(ch32x_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32x_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "ch32x mass erase complete");
	else
		command_print(CMD, "ch32x mass erase failed");

	return retval;
}

static const struct command_registration ch32f1x_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = ch32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ch32f1x_command_handlers[] = {
	{
		.name = "ch32f1x",
		.mode = COMMAND_ANY,
		.help = "ch32f1x flash command group",
		.usage = "",
		.chain = ch32f1x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver wch_arm_flash = {
	.name = "wch_arm",
	.commands = ch32f1x_command_handlers,
	.flash_bank_command = ch32x_flash_bank_command,
	.erase = ch32x_erase,
	.write = ch32x_write,
	.read = default_flash_read,
	.probe = ch32x_probe,
	.auto_probe = ch32x_auto_probe,
	.erase_check = ch32x_erase_check,
	.protect_check = ch32x_protect_check,
	.info = get_ch32x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
