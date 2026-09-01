// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * PFLASH driver for Infineon AURIX.
 *
 * The flash is driven through a command sequence interface: operands and
 * opcodes are written to fixed offsets (0x5554, 0xAA50, 0xAA58, 0xAAA8,
 * 0x55F0/0x55F4) of a window, and completion is polled in the DMU. Where that
 * window sits and how completion is reported is per derivative, so it is
 * collected in struct aurix_flash_variant.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "target/aurix/aurix.h"
#include "target/aurix/aurix_ocds.h"
#include <flash/common.h>
#include <flash/nor/core.h>
#include <flash/nor/driver.h>
#include <helper/log.h>
#include <helper/time_support.h>
#include <target/target.h>

/* Command sequence offsets, relative to the variant's cmd_base. */
#define CMD_SEQ_CTRL 0x5554  /* single-word commands: enter page mode, ... */
#define CMD_SEQ_ADDR 0xAA50  /* operand: address */
#define CMD_SEQ_CNT 0xAA58   /* operand: count */
#define CMD_SEQ_CMD 0xAAA8   /* two-word command opcode */
#define CMD_SEQ_DATA_L 0x55F0
#define CMD_SEQ_DATA_U 0x55F4

/* Command opcodes. */
#define CMD_CLEAR_STATUS 0xFA
#define CMD_ENTER_PAGE_MODE 0x50
#define CMD_ERASE_1 0x80
#define CMD_ERASE_2 0x50
#define CMD_WRITE_1 0xA0
#define CMD_WRITE_BURST_2 0xA6
#define CMD_WRITE_PAGE_2 0xAA

/* A logical PFLASH sector is 16 KiB on both families. */
#define AURIX_SECTOR_SIZE 0x4000

/* PFLASH page (the unit a page-mode load fills) is 32 bytes on both families. */
#define AURIX_PAGE_SIZE 32

#define AURIX_FLASH_TIMEOUT_MS 5000

struct aurix_flash_variant {
  const char *name;

  /* Base of the command sequence interface. */
  uint32_t cmd_base;

  /* Completion and error reporting. */
  uint32_t status_reg;
  uint32_t error_reg;
  uint32_t error_mask;
  /* An operation is in progress while any of these bits is set. */
  uint32_t busy_mask;

  /* Length of a write burst, in bytes. */
  uint32_t burst_size;

  /* Erase granularity: a physical sector spans this many logical sectors and
   * an erase request may not cross a physical sector boundary. */
  unsigned int sectors_per_phys_sector;

  /* Longest range one erase command may cover, in logical sectors, where that
   * is tighter than the physical sector. TC3xx refuses a PFLASH range over
   * 512 KB, which is 32 of its 16 KiB logical sectors, with a sequence error.
   * 0 where only the physical sector bounds a request. */
  unsigned int max_erase_sectors;

  /*
   * Per-sector write protection: base of PFLASH bank 0's block, the distance
   * to the next bank's, and how many registers of 32 sectors each a block
   * holds. Zero where the layout has not been established, which leaves the
   * protection state unknown rather than guessed.
   */
  uint32_t protect_reg;
  uint32_t protect_bank_stride;
  unsigned int protect_regs_per_bank;
  /*
   * Register reporting which protections a password has switched off, and the
   * bits in it for program flash: one for all banks, one per bank at
   * protect_off_bank0 + index. Configured protection only takes effect while
   * neither is set.
   */
  uint32_t protect_off_reg;
  uint32_t protect_off_all;
  unsigned int protect_off_bank0;

  /* Identification register, 0 if the family does not provide one. */
  uint32_t chipid_reg;
  uint32_t chipid_mask;
  uint32_t chipid_value;
};

static const struct aurix_flash_variant tc3xx_variant = {
    .name = "tc3xx",
    .cmd_base = 0xAF000000,
    .status_reg = 0xF8040010, /* DMU_HF_STATUS */
    .error_reg = 0xF8040034,  /* DMU_HF_ERRSR */
    .error_mask = 0xFFFFFFFF,
    /* HF_STATUS carries one busy flag per bank: DFLASH in bits 1:0 and the
     * program flash banks in bit x+2. A command is aimed at a bank by address,
     * so watch all six rather than guess which one it landed in; watching only
     * PF0 leaves an operation on any other bank looking finished the moment it
     * is issued, and the next command then hits a busy flash. */
    .busy_mask = 0x3F << 2,
    .burst_size = 256,
    .sectors_per_phys_sector = 64,
    .max_erase_sectors = 32,
    .protect_reg = 0xF8050000, /* DMU_HP_PROCONP00 */
    .protect_bank_stride = 0x100,
    .protect_regs_per_bank = 6,
    .protect_off_reg = 0xF804001C, /* DMU_HF_PROTECT */
    .protect_off_all = 1 << 0,     /* PRODISP */
    .protect_off_bank0 = 8,        /* PRODISPx at bit x + 8 */
    .chipid_reg = 0xF0036140, /* SCU_CHIPID */
    .chipid_mask = 0xC0,      /* CHTEC */
    .chipid_value = 0x80,
};

struct aurix_flash_bank {
  const struct aurix_flash_variant *variant;
  bool probed;
  /* Which program flash bank this is, for the protection registers. The
   * driver cannot tell from the base address alone, since where a bank sits
   * and how large it is are both derivative specific, so a configuration says
   * so; without it the protection state stays unknown. */
  bool has_pflash_index;
  unsigned int pflash_index;
};

/*
 * Start a command sequence from a known state. The error flags are sticky and
 * nothing else clears them, so without this one failed operation would make
 * every command after it fail too, reporting an error it had nothing to do
 * with.
 */
static int aurix_flash_clear_status(struct aurix_ocds *ocds,
                                    const struct aurix_flash_variant *v) {
  return aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_CTRL,
                                        CMD_CLEAR_STATUS);
}

/**
 * Poll the command interface until the pending operation finished.
 *
 * @returns ERROR_OK on success, ERROR_FLASH_OPERATION_FAILED if the flash
 *	reported an error or did not complete within AURIX_FLASH_TIMEOUT_MS.
 */
static int aurix_flash_wait_done(struct aurix_ocds *ocds,
                                 const struct aurix_flash_variant *v) {
  int64_t start = timeval_ms();

  for (;;) {
    uint32_t status = 0;
    uint32_t err = 0;
    int retval;

    retval = aurix_ocds_queue_soc_read_u32(ocds, v->error_reg, &err);
    if (retval != ERROR_OK)
      return retval;
    retval = aurix_ocds_queue_soc_read_u32(ocds, v->status_reg, &status);
    if (retval != ERROR_OK)
      return retval;
    retval = aurix_ocds_run(ocds);
    if (retval != ERROR_OK)
      return retval;

    if (err & v->error_mask) {
      LOG_ERROR("%s: flash operation failed, error register = 0x%08" PRIx32,
                v->name, err);
      return ERROR_FLASH_OPERATION_FAILED;
    }

    if (!(status & v->busy_mask))
      return ERROR_OK;

    if (timeval_ms() - start > AURIX_FLASH_TIMEOUT_MS) {
      LOG_ERROR("%s: flash operation timed out, status = 0x%08" PRIx32, v->name,
                status);
      return ERROR_FLASH_OPERATION_FAILED;
    }
  }
}

static int aurix_flash_probe(struct flash_bank *bank) {
  struct aurix_flash_bank *aurix_bank = bank->driver_priv;
  const struct aurix_flash_variant *v = aurix_bank->variant;
  uint32_t flash_addr = bank->base;

  if (v->chipid_reg) {
    uint32_t chipid;
    int retval = target_read_u32(bank->target, v->chipid_reg, &chipid);
    if (retval != ERROR_OK) {
      LOG_ERROR("Cannot read CHIPID register.");
      return retval;
    }

    if ((chipid & v->chipid_mask) != v->chipid_value) {
      LOG_ERROR("CHIPID register does not match %s.", v->name);
      return ERROR_FAIL;
    }

    LOG_DEBUG("IDCHIP = %08" PRIx32, chipid);
  }

  /* TODO: Check size / DFLASH / UCB */

  /* probe() is asked to look again even when it has already run, so start
   * over rather than leak the table a previous run left behind. */
  free(bank->sectors);
  bank->num_sectors = bank->size / AURIX_SECTOR_SIZE;
  bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
  if (!bank->sectors) {
    LOG_ERROR("Out of memory");
    return ERROR_FAIL;
  }
  for (unsigned int i = 0; i < bank->num_sectors; i++) {
    bank->sectors[i].size = AURIX_SECTOR_SIZE;
    bank->sectors[i].offset = flash_addr - bank->base;
    flash_addr += AURIX_SECTOR_SIZE;
    /* TODO: Check erased */
    bank->sectors[i].is_erased = -1;
    /* TODO: Check UCB for protection */
    bank->sectors[i].is_protected = -1;
  }

  aurix_bank->probed = true;

  return ERROR_OK;
}

static int aurix_flash_auto_probe(struct flash_bank *bank) {
  struct aurix_flash_bank *aurix_bank = bank->driver_priv;

  if (aurix_bank->probed)
    return ERROR_OK;

  return aurix_flash_probe(bank);
}

static int aurix_flash_erase(struct flash_bank *bank, unsigned int first,
                             unsigned int last) {
  struct aurix_flash_bank *aurix_bank = bank->driver_priv;
  const struct aurix_flash_variant *v = aurix_bank->variant;
  struct aurix_ocds *ocds = target_to_aurix(bank->target)->ocds;
  int err;

  while (first <= last) {
    uint32_t addr = bank->base + bank->sectors[first].offset;
    /* An erase request may not cross a physical sector boundary. */
    uint32_t sector_count =
        MIN(last - first + 1,
            v->sectors_per_phys_sector - (first % v->sectors_per_phys_sector));

    if (v->max_erase_sectors)
      sector_count = MIN(sector_count, v->max_erase_sectors);

    first += sector_count;

    err = aurix_flash_clear_status(ocds, v);
    if (err)
      goto err;
    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_ADDR,
                                         addr);
    if (err)
      goto err;
    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_CNT,
                                         sector_count);
    if (err)
      goto err;
    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_CMD,
                                         CMD_ERASE_1);
    if (err)
      goto err;
    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_CMD,
                                         CMD_ERASE_2);
    if (err)
      goto err;

    err = aurix_ocds_run(ocds);
    if (err)
      goto err;

    err = aurix_flash_wait_done(ocds, v);
    if (err != ERROR_OK)
      return err;
  }

  return ERROR_OK;

err:
  LOG_ERROR("Failed to queue flash erase sequence");
  return ERROR_FLASH_OPERATION_FAILED;
}

/* Queue one 32-bit word of a page load. */
static int aurix_flash_load_word(struct aurix_ocds *ocds,
                                 const struct aurix_flash_variant *v,
                                 uint32_t index, uint32_t data) {
  /* The two halves of the 64 bit page load register take alternating words. */
  uint32_t reg = ((index % 8) == 0) ? CMD_SEQ_DATA_L : CMD_SEQ_DATA_U;

  return aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | reg, data);
}

static int aurix_flash_write(struct flash_bank *bank, const uint8_t *buffer,
                             uint32_t offset, uint32_t count) {
  struct aurix_flash_bank *aurix_bank = bank->driver_priv;
  const struct aurix_flash_variant *v = aurix_bank->variant;
  struct aurix_ocds *ocds = target_to_aurix(bank->target)->ocds;
  int err;
  uint32_t page_offset = 0;

  if (offset & (AURIX_PAGE_SIZE - 1))
    return ERROR_FLASH_DST_BREAKS_ALIGNMENT;

  while (page_offset < count) {
    uint32_t chunk;
    uint32_t i;

    err = aurix_flash_clear_status(ocds, v);
    if (err)
      goto err;

    /* Enter page mode */
    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_CTRL,
                                         CMD_ENTER_PAGE_MODE);
    if (err)
      goto err;

    /* A full burst is programmed in one go, anything else page by page.
     * "Write Burst" operates on an aligned group of pages, so it can only be
     * used once the destination reaches a burst boundary. */
    uint32_t addr = bank->base + offset + page_offset;
    chunk = (count - page_offset >= v->burst_size &&
             (addr & (v->burst_size - 1)) == 0)
                ? v->burst_size
                : AURIX_PAGE_SIZE;

    for (i = 0; i < chunk && page_offset + i + 3 < count; i += 4) {
      uint32_t data;
      memcpy(&data, buffer + page_offset + i, 4);
      err = aurix_flash_load_word(ocds, v, i, data);
      if (err)
        goto err;
    }

    /* Write the trailing partial word, if any */
    if (i < chunk && page_offset + i < count) {
      uint32_t data = 0xFFFFFFFF;
      memcpy(&data, buffer + page_offset + i, count - i - page_offset);
      err = aurix_flash_load_word(ocds, v, i, data);
      if (err)
        goto err;
      i += 4;
    }

    /* Fill up to the page boundary */
    for (; i < chunk; i += 4) {
      err = aurix_flash_load_word(ocds, v, i, 0xFFFFFFFF);
      if (err)
        goto err;
    }

    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_ADDR,
                                         addr);
    if (err)
      goto err;
    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_CNT, 0);
    if (err)
      goto err;
    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_CMD,
                                         CMD_WRITE_1);
    if (err)
      goto err;
    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_CMD,
                                         chunk == v->burst_size
                                             ? CMD_WRITE_BURST_2
                                             : CMD_WRITE_PAGE_2);
    if (err)
      goto err;

    page_offset += chunk;

    err = aurix_ocds_run(ocds);
    if (err)
      goto err;

    err = aurix_flash_wait_done(ocds, v);
    if (err != ERROR_OK)
      return err;
  }

  return ERROR_OK;

err:
  LOG_ERROR("Failed to queue flash write sequence");
  return ERROR_FLASH_OPERATION_FAILED;
}

/**
 * Fill in the per-sector write protection.
 *
 * A sector is locked by its bit in one of the bank's protection registers, 32
 * sectors to a register, but only while the protection has not been switched
 * off with a password.
 */
static int aurix_flash_protect_check(struct flash_bank *bank) {
  struct aurix_flash_bank *aurix_bank = bank->driver_priv;
  const struct aurix_flash_variant *v = aurix_bank->variant;
  uint32_t block, disabled_mask, off;
  int ret;

  if (!v->protect_reg || !aurix_bank->has_pflash_index)
    return ERROR_FLASH_OPER_UNSUPPORTED;

  ret = target_read_u32(bank->target, v->protect_off_reg, &off);
  if (ret != ERROR_OK)
    return ret;

  disabled_mask =
      v->protect_off_all | (1u << (v->protect_off_bank0 + aurix_bank->pflash_index));

  block = v->protect_reg + aurix_bank->pflash_index * v->protect_bank_stride;

  for (unsigned int g = 0; g < v->protect_regs_per_bank; g++) {
    uint32_t locked;

    if (g * 32 >= bank->num_sectors)
      break;

    ret = target_read_u32(bank->target, block + g * 4, &locked);
    if (ret != ERROR_OK)
      return ret;

    for (unsigned int i = 0; i < 32; i++) {
      unsigned int sector = g * 32 + i;

      if (sector >= bank->num_sectors)
        break;

      bank->sectors[sector].is_protected =
          (off & disabled_mask) ? 0 : !!(locked & (1u << i));
    }
  }

  return ERROR_OK;
}

static int aurix_flash_info(struct flash_bank *bank,
                            struct command_invocation *cmd) {
  struct aurix_flash_bank *aurix_bank = bank->driver_priv;
  const struct aurix_flash_variant *v = aurix_bank->variant;
  uint32_t off = 0;

  command_print_sameline(
      cmd, "%s: %u logical sectors of %u KiB, erased %u at a time", v->name,
      bank->num_sectors, AURIX_SECTOR_SIZE / 1024,
      v->max_erase_sectors ? v->max_erase_sectors : v->sectors_per_phys_sector);

  if (aurix_bank->has_pflash_index)
    command_print_sameline(cmd, ", program flash bank %u",
                           aurix_bank->pflash_index);

  command_print_sameline(cmd,
                         "\ncommand sequence interface at 0x%08" PRIx32
                         ", %" PRIu32 " byte bursts",
                         v->cmd_base, v->burst_size);

  if (v->protect_off_reg &&
      target_read_u32(bank->target, v->protect_off_reg, &off) == ERROR_OK &&
      off)
    command_print_sameline(cmd, "\nprotection switched off by password: 0x%08" PRIx32,
                           off);

  return ERROR_OK;
}

static int aurix_flash_read(struct flash_bank *bank, uint8_t *buffer,
                            uint32_t offset, uint32_t count) {
  return target_read_buffer(bank->target, bank->base + offset, count, buffer);
}

static int aurix_flash_bank_command(struct command_invocation *cmd,
                                    struct flash_bank *bank,
                                    const struct aurix_flash_variant *variant) {
  struct aurix_flash_bank *aurix_bank;

  aurix_bank = malloc(sizeof(struct aurix_flash_bank));
  if (!aurix_bank)
    return ERROR_FLASH_OPERATION_FAILED;

  aurix_bank->variant = variant;
  aurix_bank->probed = false;
  aurix_bank->has_pflash_index = false;
  aurix_bank->pflash_index = 0;

  /* Optional: which program flash bank this is, for the protection state. */
  if (CMD_ARGC > 6) {
    unsigned int index;

    COMMAND_PARSE_NUMBER(uint, CMD_ARGV[6], index);
    aurix_bank->has_pflash_index = true;
    aurix_bank->pflash_index = index;
  }

  bank->driver_priv = aurix_bank;

  /* Let the flash core pad partial pages for us; without this an image whose
   * segments are not page aligned (which is the norm) is simply rejected. */
  bank->write_start_alignment = AURIX_PAGE_SIZE;
  bank->write_end_alignment = AURIX_PAGE_SIZE;

  return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(tc3xx_flash_bank_command) {
  return aurix_flash_bank_command(CMD, bank, &tc3xx_variant);
}

const struct flash_driver tc3xx_flash = {
    .name = "tc3xx",
    .usage = "<name> tc3xx <base> <size> 0 0 <target> [pflash_bank]",
    .flash_bank_command = tc3xx_flash_bank_command,
    .probe = aurix_flash_probe,
    .auto_probe = aurix_flash_auto_probe,
    .erase = aurix_flash_erase,
    .protect_check = aurix_flash_protect_check,
    .write = aurix_flash_write,
    .read = aurix_flash_read,
    .info = aurix_flash_info,
    .free_driver_priv = default_flash_free_driver_priv,
};
