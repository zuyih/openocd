// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * PFLASH driver for Infineon AURIX TC3xx and TC4x.
 *
 * Both families drive the flash through the same command sequence interface:
 * the opcodes and the offsets they are written to (0x5554, 0xAA50, 0xAA58,
 * 0xAAA8, 0x55F0/0x55F4) are identical. What differs is where that interface
 * is mapped, which registers report completion, how a page is filled, and the
 * burst and physical sector geometry. See struct aurix_flash_variant.
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
#include <target/register.h>
#include <target/target.h>

/* Command sequence offsets, relative to the variant's cmd_base. */
#define CMD_SEQ_CTRL 0x5554  /* single-word commands: enter page mode, ... */
#define CMD_SEQ_ADDR 0xAA50  /* operand: address */
#define CMD_SEQ_CNT 0xAA58   /* operand: count */
#define CMD_SEQ_CMD 0xAAA8   /* two-word command opcode */
#define CMD_SEQ_DATA_L 0x55F0
#define CMD_SEQ_DATA_U 0x55F4

/* Command opcodes, identical on TC3xx and TC4x. */
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

/* Largest write burst of any supported derivative, so that a page or burst can
 * be assembled before it is queued. */
#define AURIX_MAX_BURST 512

#define AURIX_FLASH_TIMEOUT_MS 5000

struct aurix_flash_variant {
  const char *name;

  /* Base of the command sequence interface. */
  uint32_t cmd_base;

  /* Completion and error reporting. */
  uint32_t status_reg;
  uint32_t error_reg;
  uint32_t error_mask;
  /* Exactly one of these is used: if done_mask is non-zero the operation is
   * complete once all of its bits are set, otherwise it is complete once no
   * bit of busy_mask is set. */
  uint32_t busy_mask;
  uint32_t done_mask;

  /* Whether the status has to be cleared once more between filling a page and
   * the write command. TC4x needs it because entering page mode already sets
   * the sticky REQDONE it reports completion with; TC3xx must not have its
   * command sequence interrupted that way and answers with a sequence error. */
  bool clear_status_before_write;

  /* How a page is filled with 32 bit accesses. TC3xx takes the two halves of
   * the 64 bit page load register at 0x55F0 and 0x55F4; TC4x takes every word
   * at 0x55F4. */
  bool page_load_alternates;


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

  /* Bits to force into a flash address before handing it to the command
   * sequence interface, e.g. to reach the non-cached alias. */
  uint32_t cmd_addr_or;

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
    .done_mask = 0,
    .page_load_alternates = true,
    .burst_size = 256,
    .sectors_per_phys_sector = 64,
    .max_erase_sectors = 32,
    /* Unlike TC4x, the address the TC3xx command interface wants has not been
     * established; leave it as the port had it. */
    .cmd_addr_or = 0,
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

/*
 * TC4x replaces the TC3xx "HF" registers with a HOST command interface (HCI).
 * DMU_HCI_STATUS.REQDONE (bit 31) reports completion, DMU_HCI_ERR collects the
 * errors: ADER, SQER, PROER, ABER, CLER, PVER, EVER and OPER.
 */
static const struct aurix_flash_variant tc4xx_variant = {
    .name = "tc4xx",
    .cmd_base = 0xF8080000,
    .status_reg = 0xF8040004, /* DMU_HCI_STATUS */
    .error_reg = 0xF8040010,  /* DMU_HCI_ERR */
    .error_mask = 0x000100F7,
    .busy_mask = 0,
    .done_mask = (1u << 31) | (1u << 30), /* REQDONE | REQACK */
    .clear_status_before_write = true,
    .page_load_alternates = false,
    .burst_size = 512,
    .sectors_per_phys_sector = 32,
    /* The command interface rejects a cached (0x8...) flash address with a
     * sequence error; it wants the non-cached alias, as iLLD uses. */
    .cmd_addr_or = 0x20000000,
    .chipid_reg = 0, /* TC4x has no SCU CHIPID; the TAS device type is used */
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

/* Address as the command sequence interface wants to see it; a bank may be
 * configured through a different alias than the interface accepts. */
static inline uint32_t aurix_flash_cmd_addr(const struct aurix_flash_variant *v,
                                            uint32_t addr) {
  return addr | v->cmd_addr_or;
}

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

    if (v->done_mask) {
      if ((status & v->done_mask) == v->done_mask)
        return ERROR_OK;
    } else {
      if (!(status & v->busy_mask))
        return ERROR_OK;
    }

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
                                         aurix_flash_cmd_addr(v, addr));
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
  uint32_t reg = CMD_SEQ_DATA_U;

  if (v->page_load_alternates)
    reg = ((index % 8) == 0) ? CMD_SEQ_DATA_L : CMD_SEQ_DATA_U;

  return aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | reg, data);
}

static int aurix_flash_write_slow(struct flash_bank *bank,
                                  const uint8_t *buffer, uint32_t offset,
                                  uint32_t count) {
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

    /*
     * Assemble what goes into the page first, padding whatever the image does
     * not cover, so that the queueing below is a plain loop either way.
     */
    uint8_t page[AURIX_MAX_BURST];
    uint32_t have = MIN(chunk, count - page_offset);

    memcpy(page, buffer + page_offset, have);
    memset(page + have, 0xFF, chunk - have);

    /*
     * One request per word is what makes writing slow, but it cannot be
     * helped: a page load targets a single address, so it is not a block
     * transfer, and the 64 bit load iLLD uses is a CPU instruction that the
     * debug interface cannot issue -- a 64 bit TAS write arrives as two 32 bit
     * ones and the command sequence rejects that.
     */
    for (i = 0; i < chunk; i += 4) {
      uint32_t data;
      memcpy(&data, page + i, 4);
      err = aurix_flash_load_word(ocds, v, i, data);
      if (err)
        goto err;
    }

    /* Entering page mode already set REQDONE, so clear the status once more
     * to make it report the outcome of the write command alone. */
    if (v->clear_status_before_write) {
      err = aurix_flash_clear_status(ocds, v);
      if (err)
        goto err;
    }

    err = aurix_ocds_queue_soc_write_u32(ocds, v->cmd_base | CMD_SEQ_ADDR,
                                         aurix_flash_cmd_addr(v, addr));
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


/*
 * Programming from a loader running on the core.
 *
 * Filling a page means writing every word to a single address, which is not a
 * block transfer, so on the debugger's side it costs one request per word and
 * lands at a few KiB/s. Handing the sequence to the core instead reduces the
 * debugger's part to getting the data into RAM, which is a block transfer.
 */

static const uint8_t aurix_flash_loader[] = {
#include "../../../contrib/loaders/flash/aurix/aurix.inc"
};

/* Laid out in the data scratchpad, clear of what an application keeps at the
 * bottom and of the context save area at the top. */
#define AURIX_LOADER_PARAMS_OFFSET 0x20000
#define AURIX_LOADER_BUFFER_OFFSET 0x20100
#define AURIX_LOADER_BUFFER_SIZE (32 * 1024)

/* Sentinel the loader overwrites when it is done. */
#define AURIX_LOADER_RUNNING 0xFFFFFFFF

#define AURIX_LOADER_TIMEOUT_MS 5000

/* struct params of contrib/loaders/flash/aurix/aurix.c, in words. */
enum {
  AURIX_LP_CMD_BASE,
  AURIX_LP_STATUS_REG,
  AURIX_LP_ERROR_REG,
  AURIX_LP_ERROR_MASK,
  AURIX_LP_DONE_MASK,
  AURIX_LP_BUSY_MASK,
  AURIX_LP_CLEAR_STATUS_WRITE,
  AURIX_LP_PAGE_REG_LO,
  AURIX_LP_PAGE_REG_HI,
  AURIX_LP_CMD_BURST,
  AURIX_LP_CMD_PAGE,
  AURIX_LP_BURST_SIZE,
  AURIX_LP_ADDRESS,
  AURIX_LP_DATA,
  AURIX_LP_COUNT,
  AURIX_LP_RESULT,
  AURIX_LP_NUM,
};

/** Whether the loader can be used on this target. */
static bool aurix_flash_can_load(struct flash_bank *bank) {
  struct aurix_ocds *ocds = target_to_aurix(bank->target)->ocds;

  if (!aurix_ocds_dspr(ocds, bank->target->coreid))
    return false;

  return aurix_ocds_dspr_size(ocds, bank->target->coreid) >=
         AURIX_LOADER_BUFFER_OFFSET + AURIX_LOADER_BUFFER_SIZE;
}

/**
 * Run the loader over one buffer's worth of data.
 *
 * The core is left parked in the loader's final loop; the caller halts it.
 */
static int aurix_flash_run_loader(struct flash_bank *bank,
                                  const struct aurix_flash_variant *v,
                                  uint32_t address, const uint8_t *buffer,
                                  uint32_t count) {
  struct target *target = bank->target;
  struct aurix_ocds *ocds = target_to_aurix(target)->ocds;
  uint32_t dspr = aurix_ocds_dspr(ocds, target->coreid);
  uint32_t pspr = aurix_ocds_pspr(ocds, target->coreid);
  uint32_t params_at = dspr + AURIX_LOADER_PARAMS_OFFSET;
  uint32_t buffer_at = dspr + AURIX_LOADER_BUFFER_OFFSET;
  uint32_t params[AURIX_LP_NUM];
  uint8_t words[AURIX_LP_NUM * 4];
  int64_t start;
  int ret;

  params[AURIX_LP_CMD_BASE] = v->cmd_base;
  params[AURIX_LP_STATUS_REG] = v->status_reg;
  params[AURIX_LP_ERROR_REG] = v->error_reg;
  params[AURIX_LP_ERROR_MASK] = v->error_mask;
  params[AURIX_LP_DONE_MASK] = v->done_mask;
  params[AURIX_LP_BUSY_MASK] = v->busy_mask;
  params[AURIX_LP_CLEAR_STATUS_WRITE] = v->clear_status_before_write;
  params[AURIX_LP_PAGE_REG_LO] =
      v->page_load_alternates ? CMD_SEQ_DATA_L : CMD_SEQ_DATA_U;
  params[AURIX_LP_PAGE_REG_HI] = CMD_SEQ_DATA_U;
  params[AURIX_LP_CMD_BURST] = CMD_WRITE_BURST_2;
  params[AURIX_LP_CMD_PAGE] = CMD_WRITE_PAGE_2;
  params[AURIX_LP_BURST_SIZE] = v->burst_size;
  params[AURIX_LP_ADDRESS] = aurix_flash_cmd_addr(v, address);
  params[AURIX_LP_DATA] = buffer_at;
  params[AURIX_LP_COUNT] = count;
  params[AURIX_LP_RESULT] = AURIX_LOADER_RUNNING;

  for (unsigned int i = 0; i < AURIX_LP_NUM; i++)
    target_buffer_set_u32(target, words + i * 4, params[i]);

  ret = target_write_memory(target, buffer_at, 4, count / 4, buffer);
  if (ret != ERROR_OK)
    return ret;
  ret = target_write_memory(target, params_at, 4, AURIX_LP_NUM, words);
  if (ret != ERROR_OK)
    return ret;

  /* Hand the parameter block over in the first argument register. */
  ret = target_write_u32(
      target, aurix_ocds_csfr(ocds, target->coreid, AURIX_CSFR_A4), params_at);
  if (ret != ERROR_OK)
    return ret;

  ret = target_resume(target, false, pspr, false, true);
  if (ret != ERROR_OK)
    return ret;

  start = timeval_ms();
  for (;;) {
    uint32_t result;

    ret = target_read_u32(target,
                          params_at + AURIX_LP_RESULT * 4, &result);
    if (ret != ERROR_OK)
      break;

    if (result != AURIX_LOADER_RUNNING) {
      ret = result ? ERROR_FLASH_OPERATION_FAILED : ERROR_OK;
      if (result)
        LOG_ERROR("%s: loader reported error register 0x%08" PRIx32, v->name,
                  result);
      break;
    }

    if (timeval_ms() - start > AURIX_LOADER_TIMEOUT_MS) {
      LOG_ERROR("%s: flash loader did not finish", v->name);
      ret = ERROR_FLASH_OPERATION_FAILED;
      break;
    }
  }

  /* Park the core; it is sitting in the loader's final loop. */
  target_halt(target);
  target_poll(target);

  return ret;
}

static int aurix_flash_write_loader(struct flash_bank *bank,
                                    const struct aurix_flash_variant *v,
                                    const uint8_t *buffer, uint32_t offset,
                                    uint32_t count) {
  struct target *target = bank->target;
  struct aurix_ocds *ocds = target_to_aurix(target)->ocds;
  uint32_t pspr = aurix_ocds_pspr(ocds, target->coreid);
  uint32_t pc_csfr = aurix_ocds_csfr(ocds, target->coreid, AURIX_CSFR_PC);
  uint32_t icr_csfr = aurix_ocds_csfr(ocds, target->coreid, AURIX_CSFR_ICR);
  uint32_t d_csfr = aurix_ocds_csfr(ocds, target->coreid, AURIX_CSFR_D0);
  uint32_t a_csfr = aurix_ocds_csfr(ocds, target->coreid, AURIX_CSFR_A0);
  uint8_t saved_d[16 * 4], saved_a[16 * 4];
  uint32_t saved_pc, saved_icr;
  uint32_t done = 0;
  int ret;

  /*
   * The loader runs on the core and disables interrupts, so put back
   * everything it disturbs: a resume afterwards then carries on with whatever
   * was being debugged. The register files are contiguous, so this is two
   * block transfers each way rather than sixty-four accesses.
   */
  ret = target_read_u32(target, pc_csfr, &saved_pc);
  if (ret != ERROR_OK)
    return ret;
  ret = target_read_u32(target, icr_csfr, &saved_icr);
  if (ret != ERROR_OK)
    return ret;
  ret = target_read_memory(target, d_csfr, 4, 16, saved_d);
  if (ret != ERROR_OK)
    return ret;
  ret = target_read_memory(target, a_csfr, 4, 16, saved_a);
  if (ret != ERROR_OK)
    return ret;

  ret = target_write_memory(target, pspr, 4,
                            (sizeof(aurix_flash_loader) + 3) / 4,
                            aurix_flash_loader);
  if (ret != ERROR_OK)
    return ret;

  while (done < count) {
    uint32_t chunk = MIN(AURIX_LOADER_BUFFER_SIZE, count - done);

    ret = aurix_flash_run_loader(bank, v, bank->base + offset + done,
                                 buffer + done, chunk);
    if (ret != ERROR_OK)
      break;

    done += chunk;
  }

  target_write_memory(target, d_csfr, 4, 16, saved_d);
  target_write_memory(target, a_csfr, 4, 16, saved_a);
  target_write_u32(target, icr_csfr, saved_icr);
  target_write_u32(target, pc_csfr, saved_pc);
  register_cache_invalidate(target->reg_cache);

  return ret;
}

static int aurix_flash_write(struct flash_bank *bank, const uint8_t *buffer,
                             uint32_t offset, uint32_t count) {
  struct aurix_flash_bank *aurix_bank = bank->driver_priv;

  /* The loader steps a page at a time and has no tail handling; the flash
   * core pads to the declared alignment, so this only guards against a
   * caller that bypasses it. */
  if (aurix_flash_can_load(bank) && (count % AURIX_PAGE_SIZE) == 0)
    return aurix_flash_write_loader(bank, aurix_bank->variant, buffer, offset,
                                    count);

  return aurix_flash_write_slow(bank, buffer, offset, count);
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

  command_print_sameline(cmd, "\nprogrammed %s",
                         aurix_flash_can_load(bank)
                             ? "through a loader on the core"
                             : "a word at a time over the debug link");

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

FLASH_BANK_COMMAND_HANDLER(tc4xx_flash_bank_command) {
  return aurix_flash_bank_command(CMD, bank, &tc4xx_variant);
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

const struct flash_driver tc4xx_flash = {
    .name = "tc4xx",
    .usage = "<name> tc4xx <base> <size> 0 0 <target>",
    .flash_bank_command = tc4xx_flash_bank_command,
    .probe = aurix_flash_probe,
    .auto_probe = aurix_flash_auto_probe,
    .erase = aurix_flash_erase,
    .protect_check = aurix_flash_protect_check,
    .write = aurix_flash_write,
    .read = aurix_flash_read,
    .info = aurix_flash_info,
    .free_driver_priv = default_flash_free_driver_priv,
};
