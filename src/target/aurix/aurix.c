#include "helper/binarybuffer.h"
#include "helper/command.h"
#include "jtag/interface.h"
#include "jtag/tas.h"
#include "target/aurix/aurix_ocds.h"
#include "target/register.h"
#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/time_support.h>
#include <target/breakpoints.h>
#include <target/target.h>
#include <target/target_type.h>

#include "aurix.h"

/*
 * TriCore CSFR offsets inside a core window. These are architectural and
 * identical on TriCore 1.6.x (TC3xx) and TriCore 1.8 (TC4x); only the location
 * of the window itself differs, see struct aurix_device.
 */
#define CSFR_TR0_EVT 0xF000
#define CSFR_TR_EVT(n) (0xF000 + (n) * 8)
#define CSFR_TR_ADR(n) (0xF004 + (n) * 8)
#define CSFR_DBGSR 0xFD00
#define CSFR_TRIG_ACC 0xFD30
#define CSFR_PC 0xFE08
#define CSFR_SYSCON 0xFE14

/* Set while a core waits in boot halt; it starts once software clears it. */
#define SYSCON_BHALT (1 << 24)

#define DBGSR_HALT (1 << 1)
#define DBGSR_HALT_SET (3 << 1)
#define DBGSR_HALT_RESET (2 << 1)

/* Trigger event register. TYP selects the instruction address as the compare
 * input; without it a match is recorded in TRIG_ACC but raises no event. BBM
 * halts on the instruction rather than after it. */
#define TREVT_BBM (1 << 3)
#define TREVT_TYP (1 << 12)
/* With TYP clear the compare input is the data address; AST and ALD pick the
 * access kinds that raise the event. */
#define TREVT_AST (1 << 27)
#define TREVT_ALD (1 << 28)

/*
 * What a trigger does when it matches. TriCore 1.6.2 keeps that in the event
 * register itself: EVTA 010 halts, and BOD suppresses the BRKOUT pulse that
 * would otherwise come with it.
 */
#define TREVT_EVTA_HALT 0x2
#define TREVT_BOD (1 << 4)

/* Bits that arm a trigger so that a match halts the core. */
#define TREVT_ARM (TREVT_BOD | TREVT_EVTA_HALT)

/* Triggers per core; the last one is kept for single stepping. */
#define AURIX_NUM_TRIGGERS 8
#define AURIX_STEP_TRIGGER (AURIX_NUM_TRIGGERS - 1)

#define AURIX_STEP_TIMEOUT_MS 1000

/*
 * The order and the names are the ones the TriCore GDB uses for its built-in
 * register layout, so that a debugger which rejects the target description
 * still lines up. e0/e2/e4/e6 are pseudo registers GDB forms from the data
 * registers itself and are deliberately absent here.
 */
static const struct {
  const char *const name;
  uint16_t reg_offset;
  enum reg_type type;
  bool caller_saved;
} tricore_core_regs[] = {
    {.name = "d0",     .reg_offset = 0xFF00 + 0 * 4, .type = REG_TYPE_INT32,    .caller_saved = true, },
    {.name = "d1",     .reg_offset = 0xFF00 + 1 * 4, .type = REG_TYPE_INT32,    .caller_saved = true, },
    {.name = "d2",     .reg_offset = 0xFF00 + 2 * 4, .type = REG_TYPE_INT32,    .caller_saved = true, },
    {.name = "d3",     .reg_offset = 0xFF00 + 3 * 4, .type = REG_TYPE_INT32,    .caller_saved = true, },
    {.name = "d4",     .reg_offset = 0xFF00 + 4 * 4, .type = REG_TYPE_INT32,    .caller_saved = true, },
    {.name = "d5",     .reg_offset = 0xFF00 + 5 * 4, .type = REG_TYPE_INT32,    .caller_saved = true, },
    {.name = "d6",     .reg_offset = 0xFF00 + 6 * 4, .type = REG_TYPE_INT32,    .caller_saved = true, },
    {.name = "d7",     .reg_offset = 0xFF00 + 7 * 4, .type = REG_TYPE_INT32,    .caller_saved = true, },
    {.name = "d8",     .reg_offset = 0xFF00 + 8 * 4, .type = REG_TYPE_INT32,    },
    {.name = "d9",     .reg_offset = 0xFF00 + 9 * 4, .type = REG_TYPE_INT32,    },
    {.name = "d10",    .reg_offset = 0xFF00 + 10 * 4, .type = REG_TYPE_INT32,    },
    {.name = "d11",    .reg_offset = 0xFF00 + 11 * 4, .type = REG_TYPE_INT32,    },
    {.name = "d12",    .reg_offset = 0xFF00 + 12 * 4, .type = REG_TYPE_INT32,    },
    {.name = "d13",    .reg_offset = 0xFF00 + 13 * 4, .type = REG_TYPE_INT32,    },
    {.name = "d14",    .reg_offset = 0xFF00 + 14 * 4, .type = REG_TYPE_INT32,    },
    {.name = "d15",    .reg_offset = 0xFF00 + 15 * 4, .type = REG_TYPE_INT32,    },
    {.name = "a0",     .reg_offset = 0xFF80 + 0 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a1",     .reg_offset = 0xFF80 + 1 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a2",     .reg_offset = 0xFF80 + 2 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a3",     .reg_offset = 0xFF80 + 3 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a4",     .reg_offset = 0xFF80 + 4 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a5",     .reg_offset = 0xFF80 + 5 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a6",     .reg_offset = 0xFF80 + 6 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a7",     .reg_offset = 0xFF80 + 7 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a8",     .reg_offset = 0xFF80 + 8 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a9",     .reg_offset = 0xFF80 + 9 * 4, .type = REG_TYPE_DATA_PTR, .caller_saved = true, },
    {.name = "a10",    .reg_offset = 0xFF80 + 10 * 4, .type = REG_TYPE_DATA_PTR, },
    {.name = "a11",    .reg_offset = 0xFF80 + 11 * 4, .type = REG_TYPE_DATA_PTR, },
    {.name = "a12",    .reg_offset = 0xFF80 + 12 * 4, .type = REG_TYPE_DATA_PTR, },
    {.name = "a13",    .reg_offset = 0xFF80 + 13 * 4, .type = REG_TYPE_DATA_PTR, },
    {.name = "a14",    .reg_offset = 0xFF80 + 14 * 4, .type = REG_TYPE_DATA_PTR, },
    {.name = "a15",    .reg_offset = 0xFF80 + 15 * 4, .type = REG_TYPE_DATA_PTR, },
    {.name = "lcx",    .reg_offset = 0xFE3C,        .type = REG_TYPE_UINT32,   },
    {.name = "fcx",    .reg_offset = 0xFE38,        .type = REG_TYPE_UINT32,   },
    {.name = "pcx",    .reg_offset = 0xFE00,        .type = REG_TYPE_UINT32,   },
    {.name = "psw",    .reg_offset = 0xFE04,        .type = REG_TYPE_UINT32,   },
    {.name = "pc",     .reg_offset = 0xFE08,        .type = REG_TYPE_CODE_PTR, },
    {.name = "icr",    .reg_offset = 0xFE2C,        .type = REG_TYPE_UINT32,   },
    {.name = "isp",    .reg_offset = 0xFE28,        .type = REG_TYPE_DATA_PTR, },
    {.name = "btv",    .reg_offset = 0xFE24,        .type = REG_TYPE_CODE_PTR, },
    {.name = "biv",    .reg_offset = 0xFE20,        .type = REG_TYPE_CODE_PTR, },
    {.name = "syscon", .reg_offset = 0xFE14,        .type = REG_TYPE_UINT32,   },
    {.name = "pcon0",  .reg_offset = 0x920C,        .type = REG_TYPE_UINT32,   },
    {.name = "dcon0",  .reg_offset = 0x9040,        .type = REG_TYPE_UINT32,   },
};

static int aurix_read_dbgsr(struct target *target, uint32_t *debug_sr) {
  struct aurix_private_config *aurix = target_to_aurix(target);

  return aurix_ocds_atomic_read_u32(
      aurix->ocds, aurix_ocds_csfr(aurix->ocds, target->coreid, CSFR_DBGSR),
      debug_sr);
}

static int aurix_read_syscon(struct target *target, uint32_t *syscon) {
  struct aurix_private_config *aurix = target_to_aurix(target);

  return aurix_ocds_atomic_read_u32(
      aurix->ocds, aurix_ocds_csfr(aurix->ocds, target->coreid, CSFR_SYSCON),
      syscon);
}

static int tricore_breakpoints_clear(struct target *target) {
  /* Clear out any existing breakpoints */
  uint8_t trig[16 * 4];
  memset(trig, 0, 16 * 4);
  return target_write_memory(
      target, aurix_ocds_csfr(target_to_aurix(target)->ocds, target->coreid,
                              CSFR_TR0_EVT),
      4, 16, trig);
}

static int aurix_write_trigger(struct target *target, unsigned int n,
                               uint32_t address, uint32_t evt) {
  struct aurix_ocds *ocds = target_to_aurix(target)->ocds;
  int ret;

  ret = target_write_u32(
      target, aurix_ocds_csfr(ocds, target->coreid, CSFR_TR_ADR(n)), address);
  if (ret != ERROR_OK)
    return ret;

  return target_write_u32(
      target, aurix_ocds_csfr(ocds, target->coreid, CSFR_TR_EVT(n)), evt);
}

/*
 * Refuse to run a core that is still in boot halt. Releasing it is a decision
 * about the system, not about debugging: the application chose not to start
 * this core, and its entry point may never have been set up. Clearing the
 * halt behind the user's back would run it from whatever the boot PC holds.
 */
static int aurix_check_boot_halt(struct target *target) {
  uint32_t syscon;
  int ret;

  ret = aurix_read_syscon(target, &syscon);
  if (ret != ERROR_OK)
    return ret;

  if (syscon & SYSCON_BHALT) {
    LOG_TARGET_ERROR(target,
                     "core is in boot halt and has not been started; it runs "
                     "once software clears its SYSCON.BHALT");
    return ERROR_TARGET_NOT_HALTED;
  }

  return ERROR_OK;
}

static int aurix_poll(struct target *target) {
  enum target_state prev_target_state;
  int ret = ERROR_OK;
  uint32_t dbgsr;
  uint32_t syscon;

  ret = aurix_read_dbgsr(target, &dbgsr);
  if (ret != ERROR_OK)
    return ret;

  ret = aurix_read_syscon(target, &syscon);
  if (ret != ERROR_OK)
    return ret;

  /* A core other than 0 comes out of reset in boot halt and stays there until
   * software clears SYSCON.BHALT, so on a device whose application never
   * starts it, it is stopped without the debug unit having stopped it. On TC4x
   * this offset is CORECON, whose bits 31:18 are reserved, so it reads back
   * zero and the test never fires there. */
  if (syscon & SYSCON_BHALT) {
    target->state = TARGET_HALTED;
    return ERROR_OK;
  }

  if (dbgsr & DBGSR_HALT) {
    prev_target_state = target->state;
    if (prev_target_state != TARGET_HALTED) {
      // senum target_debug_reason debug_reason = target->debug_reason;

      /* We have a halting debug event */
      target->state = TARGET_HALTED;
      LOG_DEBUG("Target %s halted", target_name(target));

      /*
       * Tell a debugger why, so that it does not report every stop as an
       * asynchronous interrupt. A halt we asked for already carries its
       * reason; anything else came from a trigger. TRIG_ACC only records
       * instruction triggers, so an armed data trigger is what is left.
       */
      if (target->debug_reason != DBG_REASON_DBGRQ) {
        struct aurix_private_config *cfg = target_to_aurix(target);
        uint32_t trig = 0;

        aurix_ocds_atomic_read_u32(
            cfg->ocds, aurix_ocds_csfr(cfg->ocds, target->coreid,
                                       CSFR_TRIG_ACC),
            &trig);

        if (trig & cfg->trigger_used & ~cfg->trigger_is_watchpoint)
          target->debug_reason = DBG_REASON_BREAKPOINT;
        else if (cfg->trigger_is_watchpoint)
          target->debug_reason = DBG_REASON_WATCHPOINT;
      }

#if 0
      ret = tricore_debug_entry(target);
      if (ret != ERROR_OK)
        return ret;
#endif

      /* TODO: Multi-core */

      switch (prev_target_state) {
      case TARGET_RUNNING:
      case TARGET_UNKNOWN:
      case TARGET_RESET:
        target_call_event_callbacks(target, TARGET_EVENT_HALTED);
        break;
      case TARGET_DEBUG_RUNNING:
        target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
        break;
      default:
        break;
      }
    }
  } else {
    target->state = TARGET_RUNNING;
  }
  /* TODO: Maybe reset */

  return ret;
}

/* Invoked only from target_arch_state().
 * Issue USER() w/architecture specific status.  */
int aurix_arch_state(struct target *target) { return ERROR_FAIL; }

/* target request support */
int aurix_target_request_data(struct target *target, uint32_t size,
                              uint8_t *buffer) {
  return ERROR_FAIL;
}

/* halt will log a warning, but return ERROR_OK if the target is already halted.
 */
int aurix_halt(struct target *target) {
  int ret = 0;

  if (target->state == TARGET_HALTED) {
    LOG_TARGET_DEBUG(target, "target already halted");
    return ERROR_OK;
  }

  ret = target_write_u32(
      target,
      aurix_ocds_csfr(target_to_aurix(target)->ocds, target->coreid, CSFR_DBGSR),
      DBGSR_HALT_SET);
  if (ret) {
    LOG_TARGET_ERROR(target, "Failed to halt target");
    return ret;
  }

  target->debug_reason = DBG_REASON_DBGRQ;

  return ret;
}
/* See target.c target_resume() for documentation. */
int aurix_resume(struct target *target, bool current, target_addr_t address,
                 bool handle_breakpoints, bool debug_execution) {
  struct aurix_ocds *ocds = target_to_aurix(target)->ocds;
  int ret;

  /* Right after init nothing has polled yet, so refresh rather than refuse. */
  if (target->state != TARGET_HALTED && target->state != TARGET_RUNNING) {
    ret = aurix_poll(target);
    if (ret != ERROR_OK)
      return ret;
  }

  if (target->state == TARGET_RUNNING) {
    LOG_TARGET_DEBUG(target, "target already running");
    return ERROR_OK;
  }

  if (target->state != TARGET_HALTED) {
    LOG_TARGET_ERROR(target, "target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  ret = aurix_check_boot_halt(target);
  if (ret != ERROR_OK)
    return ret;

  /* current == false means: start executing at 'address' instead. */
  if (!current) {
    ret = target_write_u32(
        target, aurix_ocds_csfr(ocds, target->coreid, CSFR_PC), address);
    if (ret != ERROR_OK) {
      LOG_TARGET_ERROR(target, "Failed to set resume address");
      return ret;
    }
  }

  ret = target_write_u32(
      target, aurix_ocds_csfr(ocds, target->coreid, CSFR_DBGSR),
      DBGSR_HALT_RESET);
  if (ret != ERROR_OK) {
    LOG_TARGET_ERROR(target, "Failed to continue target");
    return ret;
  }

  /* registers are now invalid */
  register_cache_invalidate(target->reg_cache);
  target->debug_reason = DBG_REASON_NOTHALTED;

  if (!debug_execution) {
    target->state = TARGET_RUNNING;
    ret = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
    if (ret) {
      return ret;
    }
    // LOG_TARGET_DEBUG(target, "resumed at 0x%08" PRIx32, resume_pc);
  } else {
    target->state = TARGET_DEBUG_RUNNING;
    ret = target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
    if (ret) {
      return ret;
    }
    // LOG_DEBUG("target debug resumed at 0x%08" PRIx32, resume_pc);
  }

  return ERROR_OK;
}

int aurix_step(struct target *target, bool current, target_addr_t address,
               bool handle_breakpoints) {
  struct aurix_ocds *ocds = target_to_aurix(target)->ocds;
  uint32_t pc;
  int ret;

  if (target->state != TARGET_HALTED) {
    LOG_TARGET_ERROR(target, "target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  ret = aurix_check_boot_halt(target);
  if (ret != ERROR_OK)
    return ret;

  if (!current) {
    ret = target_write_u32(
        target, aurix_ocds_csfr(ocds, target->coreid, CSFR_PC), address);
    if (ret != ERROR_OK)
      return ret;
  }

  ret = target_read_u32(target, aurix_ocds_csfr(ocds, target->coreid, CSFR_PC),
                        &pc);
  if (ret != ERROR_OK)
    return ret;

  /*
   * A breakpoint sitting on this very instruction breaks before it executes,
   * so resuming would trip it again and go nowhere. Stand it down for the
   * duration of the step.
   */
  struct breakpoint *stepped_over = NULL;
  for (struct breakpoint *bp = target->breakpoints; bp; bp = bp->next) {
    if (bp->is_set && bp->address == pc &&
        bp->number < AURIX_STEP_TRIGGER) {
      ret = aurix_write_trigger(target, bp->number, 0, 0);
      if (ret != ERROR_OK)
        return ret;
      stepped_over = bp;
      break;
    }
  }

  /*
   * Arm a break-after-make trigger on the instruction that is about to run:
   * the core halts once it has retired, so a taken branch lands on its
   * destination and instruction length does not have to be known.
   */
  ret = aurix_write_trigger(target, AURIX_STEP_TRIGGER, pc,
                            TREVT_ARM | TREVT_TYP);
  if (ret != ERROR_OK)
    goto restore;

  register_cache_invalidate(target->reg_cache);

  ret = target_write_u32(target,
                         aurix_ocds_csfr(ocds, target->coreid, CSFR_DBGSR),
                         DBGSR_HALT_RESET);
  if (ret != ERROR_OK)
    goto out;

  target->state = TARGET_RUNNING;

  int64_t start = timeval_ms();
  for (;;) {
    uint32_t dbgsr;

    ret = aurix_read_dbgsr(target, &dbgsr);
    if (ret != ERROR_OK)
      goto out;
    if (dbgsr & DBGSR_HALT)
      break;

    if (timeval_ms() - start > AURIX_STEP_TIMEOUT_MS) {
      LOG_TARGET_ERROR(target, "timed out stepping from 0x%08" PRIx32, pc);
      ret = ERROR_TARGET_TIMEOUT;
      goto out;
    }
  }

  target->state = TARGET_HALTED;
  target->debug_reason = DBG_REASON_SINGLESTEP;
  target_call_event_callbacks(target, TARGET_EVENT_HALTED);
  ret = ERROR_OK;

out:
  aurix_write_trigger(target, AURIX_STEP_TRIGGER, 0, 0);

restore:
  if (stepped_over)
    aurix_write_trigger(target, stepped_over->number, stepped_over->address,
                        TREVT_ARM | TREVT_BBM | TREVT_TYP);

  return ret;
}
/* target reset control. assert reset can be invoked when OpenOCD and
 * the target is out of sync.
 *
 * A typical example is that the target was power cycled while OpenOCD
 * thought the target was halted or running.
 *
 * assert_reset() can therefore make no assumptions whatsoever about the
 * state of the target
 *
 * Before assert_reset() for the target is invoked, a TRST/tms and
 * chain validation is executed. TRST should not be asserted
 * during target assert unless there is no way around it due to
 * the way reset's are configured.
 *
 */
int aurix_assert_reset(struct target *target) {
  enum reset_types reset_config = jtag_get_reset_config();

  /* Issue some kind of warm reset. */
  if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT))
    target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
  else if (reset_config & RESET_HAS_SRST) {
    bool srst_asserted = false;

    if (target->reset_halt && !(reset_config & RESET_SRST_PULLS_TRST)) {
      if (target_was_examined(target)) {

        if (reset_config & RESET_SRST_NO_GATING) {
          /*
           * SRST needs to be asserted *before* Reset Catch
           * debug event can be set up.
           */
          adapter_assert_reset();
          srst_asserted = true;
        }

        /* Catch logic currently implemented in adapter */
      } else {
        LOG_WARNING(
            "%s: Target not examined, will not halt immediately after reset!",
            target_name(target));
      }
    }

    /* REVISIT handle "pulls" cases, if there's
     * hardware that needs them to work.
     */
    if (!srst_asserted)
      adapter_assert_reset();
  } else {
    LOG_ERROR("%s: how to reset?", target_name(target));
    return ERROR_FAIL;
  }

  /* registers are now invalid */
  register_cache_invalidate(target->reg_cache);

  target->state = TARGET_RESET;

  return ERROR_OK;
}
/**
 * The implementation is responsible for polling the
 * target such that target->state reflects the
 * state correctly.
 *
 * Otherwise the following would fail, as there will not
 * be any "poll" invoked between the "reset run" and
 * "halt".
 *
 * reset run; halt
 */
int aurix_deassert_reset(struct target *target) {
  int ret;

  /* be certain SRST is off */
  adapter_deassert_reset();

  if (!target_was_examined(target))
    return ERROR_OK;

  ret = aurix_poll(target);
  if (ret != ERROR_OK)
    return ret;

  if (target->reset_halt) {
    /* Breakpoints clear */
    tricore_breakpoints_clear(target);

    if (target->state != TARGET_HALTED) {
      LOG_TARGET_WARNING(target, "ran after reset and before halt ...");
      if (target_was_examined(target)) {
        ret = aurix_halt(target);
        if (ret != ERROR_OK)
          return ret;
      } else {
        target->state = TARGET_UNKNOWN;
      }
    }
  }

  return ERROR_OK;
}
int aurix_soft_reset_halt(struct target *target) { return ERROR_FAIL; }

/**
 * Target architecture for GDB.
 *
 * The string returned by this function will not be automatically freed;
 * if dynamic allocation is used for this value, it must be managed by
 * the target, ideally by caching the result for subsequent calls.
 */
const char *aurix_get_gdb_arch(const struct target *target) {
  return "tricore";
}

/**
 * Target register access for GDB.  Do @b not call this function
 * directly, use target_get_gdb_reg_list() instead.
 *
 * Danger! this function will succeed even if the target is running
 * and return a register list with dummy values.
 *
 * The reason is that GDB connection will fail without a valid register
 * list, however it is after GDB is connected that monitor commands can
 * be run to properly initialize the target
 */
int aurix_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
                           int *reg_list_size,
                           enum target_register_class reg_class) {

  switch (reg_class) {
  case REG_CLASS_ALL:
  case REG_CLASS_GENERAL:
    *reg_list_size = ARRAY_SIZE(tricore_core_regs);
    *reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

    int i;
    for (i = 0; i < *reg_list_size; i++) {
      (*reg_list)[i] = &target->reg_cache->reg_list[i];
    }
    return ERROR_OK;
    break;
  default:
    LOG_ERROR("not a valid register class type in query.");
    return ERROR_FAIL;
  }
}

/**
 * Same as get_gdb_reg_list, but doesn't read the register values.
 * */
int aurix_get_gdb_reg_list_noread(struct target *target,
                                  struct reg **reg_list[], int *reg_list_size,
                                  enum target_register_class reg_class) {
  return ERROR_FAIL;
}

/* target memory access
 * size: 1 = byte (8bit), 2 = half-word (16bit), 4 = word (32bit)
 * count: number of items of <size>
 */

/**
 * Target memory read callback.  Do @b not call this function
 * directly, use target_read_memory() instead.
 */
int aurix_read_memory(struct target *target, target_addr_t address,
                      uint32_t size, uint32_t count, uint8_t *buffer) {

  struct aurix_private_config *aurix_cfg = target->private_config;
  int ret;

  ret =
      aurix_ocds_queue_soc_read(aurix_cfg->ocds, address, size, count, buffer);
  if (ret) {
    LOG_ERROR("Failed to enque read");
    goto exit;
  }

  ret = aurix_ocds_run(aurix_cfg->ocds);
  if (ret) {
    LOG_ERROR("Failed to run ocds sequence");
    goto exit;
  }

exit:
  return ret;
}
/**
 * Target memory write callback.  Do @b not call this function
 * directly, use target_write_memory() instead.
 */
int aurix_write_memory(struct target *target, target_addr_t address,
                       uint32_t size, uint32_t count, const uint8_t *buffer) {
  struct aurix_private_config *aurix_cfg = target->private_config;
  int ret;

  ret =
      aurix_ocds_queue_soc_write(aurix_cfg->ocds, address, size, count, buffer);
  if (ret) {
    LOG_ERROR("Failed to queue write request");
    goto exit;
  }

  ret = aurix_ocds_run(aurix_cfg->ocds);
  if (ret) {
    LOG_ERROR("Failed to run OCDS sequence");
    goto exit;
  }

exit:
  return ret;
}

int aurix_checksum_memory(struct target *target, target_addr_t address,
                          uint32_t count, uint32_t *checksum) {
  return ERROR_FAIL;
}
int aurix_blank_check_memory(struct target *target,
                             struct target_memory_check_block *blocks,
                             int num_blocks, uint8_t erased_value) {
  return ERROR_FAIL;
}

/*
 * target break-/watchpoint control
 * rw: 0 = write, 1 = read, 2 = access
 *
 * Target must be halted while this is invoked as this
 * will actually set up breakpoints on target.
 *
 * The breakpoint hardware will be set up upon adding the
 * first breakpoint.
 *
 * Upon GDB connection all breakpoints/watchpoints are cleared.
 */
int aurix_add_breakpoint(struct target *target, struct breakpoint *breakpoint) {
  struct aurix_private_config *cfg = target_to_aurix(target);
  unsigned int n;
  int ret;

  if (breakpoint->is_set)
    return ERROR_OK;

  /* There is no software breakpoint mechanism, so everything becomes a
   * trigger; a debugger asking for a soft breakpoint gets a hard one. */
  for (n = 0; n < AURIX_STEP_TRIGGER; n++)
    if (!(cfg->trigger_used & (1 << n)))
      break;

  if (n == AURIX_STEP_TRIGGER) {
    LOG_TARGET_ERROR(target, "all %u breakpoint triggers are in use",
                     AURIX_STEP_TRIGGER);
    return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
  }

  ret = aurix_write_trigger(target, n, breakpoint->address,
                            TREVT_ARM | TREVT_BBM | TREVT_TYP);
  if (ret != ERROR_OK)
    return ret;

  cfg->trigger_used |= 1 << n;
  cfg->trigger_is_watchpoint &= ~(1 << n);
  breakpoint_hw_set(breakpoint, n);

  return ERROR_OK;
}

int aurix_remove_breakpoint(struct target *target,
                            struct breakpoint *breakpoint) {
  struct aurix_private_config *cfg = target_to_aurix(target);
  int ret;

  if (!breakpoint->is_set)
    return ERROR_OK;

  if (breakpoint->number >= AURIX_STEP_TRIGGER) {
    LOG_TARGET_ERROR(target, "breakpoint on bogus trigger %u",
                     breakpoint->number);
    return ERROR_FAIL;
  }

  ret = aurix_write_trigger(target, breakpoint->number, 0, 0);
  if (ret != ERROR_OK)
    return ret;

  cfg->trigger_used &= ~(1 << breakpoint->number);
  breakpoint->is_set = false;

  return ERROR_OK;
}
int aurix_add_context_breakpoint(struct target *target,
                                 struct breakpoint *breakpoint) {
  return ERROR_FAIL;
}
int aurix_add_hybrid_breakpoint(struct target *target,
                                struct breakpoint *breakpoint) {
  return ERROR_FAIL;
}

/* remove breakpoint. hw will only be updated if the target
 * is currently halted.
 * However, this method can be invoked on unresponsive targets.
 */

/* add watchpoint ... see add_breakpoint() comment above. */
int aurix_add_watchpoint(struct target *target, struct watchpoint *watchpoint) {
  struct aurix_private_config *cfg = target_to_aurix(target);
  unsigned int n;
  uint32_t evt;
  int ret;

  if (watchpoint->is_set)
    return ERROR_OK;

  /* A trigger compares one address, so only accesses starting exactly there
   * are caught. */
  if (watchpoint->length > 4)
    LOG_TARGET_WARNING(target,
                       "watchpoint covers %u bytes but only accesses to "
                       TARGET_ADDR_FMT " itself are caught",
                       watchpoint->length, watchpoint->address);
  if (watchpoint->mask != WATCHPOINT_IGNORE_DATA_VALUE_MASK) {
    LOG_TARGET_ERROR(target, "value matching watchpoints are not supported");
    return ERROR_NOT_IMPLEMENTED;
  }

  evt = TREVT_ARM;
  if (watchpoint->rw == WPT_READ || watchpoint->rw == WPT_ACCESS)
    evt |= TREVT_ALD;
  if (watchpoint->rw == WPT_WRITE || watchpoint->rw == WPT_ACCESS)
    evt |= TREVT_AST;

  for (n = 0; n < AURIX_STEP_TRIGGER; n++)
    if (!(cfg->trigger_used & (1 << n)))
      break;

  if (n == AURIX_STEP_TRIGGER) {
    LOG_TARGET_ERROR(target, "all %u triggers are in use",
                     AURIX_STEP_TRIGGER);
    return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
  }

  ret = aurix_write_trigger(target, n, watchpoint->address, evt);
  if (ret != ERROR_OK)
    return ret;

  cfg->trigger_used |= 1 << n;
  cfg->trigger_is_watchpoint |= 1 << n;
  watchpoint_set(watchpoint, n);

  return ERROR_OK;
}

/* remove watchpoint. hw will only be updated if the target
 * is currently halted.
 * However, this method can be invoked on unresponsive targets.
 */
int aurix_remove_watchpoint(struct target *target,
                            struct watchpoint *watchpoint) {
  struct aurix_private_config *cfg = target_to_aurix(target);
  int ret;

  if (!watchpoint->is_set)
    return ERROR_OK;

  if (watchpoint->number >= AURIX_STEP_TRIGGER) {
    LOG_TARGET_ERROR(target, "watchpoint on bogus trigger %u",
                     watchpoint->number);
    return ERROR_FAIL;
  }

  ret = aurix_write_trigger(target, watchpoint->number, 0, 0);
  if (ret != ERROR_OK)
    return ret;

  cfg->trigger_used &= ~(1 << watchpoint->number);
  cfg->trigger_is_watchpoint &= ~(1 << watchpoint->number);
  watchpoint->is_set = false;

  return ERROR_OK;
}

/* Find out just hit watchpoint. After the target hits a watchpoint, the
 * information could assist gdb to locate where the modified/accessed memory is.
 */
int aurix_hit_watchpoint(struct target *target,
                         struct watchpoint **hit_watchpoint) {
  return ERROR_FAIL;
}

/**
 * Target algorithm support.  Do @b not call this method directly,
 * use target_run_algorithm() instead.
 */
int aurix_run_algorithm(struct target *target, int num_mem_params,
                        struct mem_param *mem_params, int num_reg_params,
                        struct reg_param *reg_param, target_addr_t entry_point,
                        target_addr_t exit_point, unsigned int timeout_ms,
                        void *arch_info) {
  return ERROR_FAIL;
}
int aurix_start_algorithm(struct target *target, int num_mem_params,
                          struct mem_param *mem_params, int num_reg_params,
                          struct reg_param *reg_param,
                          target_addr_t entry_point, target_addr_t exit_point,
                          void *arch_info) {
  return ERROR_FAIL;
}
int aurix_wait_algorithm(struct target *target, int num_mem_params,
                         struct mem_param *mem_params, int num_reg_params,
                         struct reg_param *reg_param, target_addr_t exit_point,
                         unsigned int timeout_ms, void *arch_info) {
  return ERROR_FAIL;
}

const struct command_registration *commands;

/* called when target is created */
int aurix_target_create(struct target *target) {
  return ERROR_OK;
}

static const struct jim_nvp nvp_config_opts[] = {{.name = "-ocds", .value = 0},
                                                 {.name = NULL, .value = -1}};
/* called for various config parameters */
/* returns JIM_CONTINUE - if option not understood */
/* otherwise: JIM_OK, or JIM_ERR, */
int aurix_target_jim_configure(struct target *target,
                               struct jim_getopt_info *goi) {

  int e;
  struct jim_nvp *n;
  struct aurix_private_config *aurix_cfg =
      (struct aurix_private_config *)target->private_config;

  if (!goi->argc)
    return JIM_OK;

  if (aurix_cfg == NULL) {
    aurix_cfg = calloc(1, sizeof(struct aurix_private_config));
    if (!aurix_cfg) {
      LOG_ERROR("Out of memory");
      return JIM_ERR;
    }
    target->private_config = aurix_cfg;
  }

  Jim_SetEmptyResult(goi->interp);

  e = jim_nvp_name2value_obj(goi->interp, nvp_config_opts, goi->argv[0], &n);
  if (e != JIM_OK)
    return JIM_CONTINUE;

  e = jim_getopt_obj(goi, NULL);
  if (e != JIM_OK)
    return e;

  switch (n->value) {
  case 0:
    if (goi->is_configure) {
      Jim_Obj *o_t;
      struct aurix_ocds *ocds;
      e = jim_getopt_obj(goi, &o_t);
      if (e != JIM_OK)
        return e;
      ocds = aurix_ocds_by_jim_obj(goi->interp, o_t);
      if (!ocds) {
        Jim_SetResultString(goi->interp, "OCDS name invalid!", -1);
        return JIM_ERR;
      }
      if (aurix_cfg->ocds && aurix_cfg->ocds != ocds) {
        Jim_SetResultString(goi->interp, "OCDS assignment cannot be changed!",
                            -1);
        return JIM_ERR;
      }
      aurix_cfg->ocds = ocds;
    } else {
      if (goi->argc)
        goto err_no_param;
      if (!aurix_cfg->ocds) {
        Jim_SetResultString(goi->interp, "OCDS not configured", -1);
        return JIM_ERR;
      }
      Jim_SetResultString(goi->interp, aurix_cfg->ocds->name, -1);
    }
    break;
  }

  if (aurix_cfg->ocds) {
    if (target->tap_configured) {
      aurix_cfg->ocds = NULL;
      Jim_SetResultString(
          goi->interp,
          "-chain-position and -ocds configparams are mutually exclusive!", -1);
      return JIM_ERR;
    }
    target->tap = aurix_cfg->ocds->tap;
    /* Deliberately not has_dap: an AURIX target has no ADIv5 DAP, and the
     * 'dap' commands would reinterpret private_config as adiv5_private_config. */
    target->ocds_configured = true;
    target->has_ocds = true;
  }

  return JIM_OK;

err_no_param:
  Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "No parameters");
  return JIM_ERR;
}

/**
 * This method is used to perform target setup that requires
 * JTAG access.
 *
 * This may be called multiple times.  It is called after the
 * scan chain is initially validated, or later after the target
 * is enabled by a JRC.  It may also be called during some
 * parts of the reset sequence.
 *
 * For one-time initialization tasks, use target_was_examined()
 * and target_set_examined().  For example, probe the hardware
 * before setting up chip-specific state, and then set that
 * flag so you don't do that again.
 */
int aurix_examine(struct target *target) {
  struct aurix_ocds *ocds = target_to_aurix(target)->ocds;
  int ret;

  if (ocds->device->num_cores &&
      (target->coreid < 0 ||
       (unsigned int)target->coreid >= ocds->device->num_cores)) {
    LOG_TARGET_ERROR(target, "-coreid %d is out of range, %s has %u cores",
                     target->coreid, ocds->device->name,
                     ocds->device->num_cores);
    return ERROR_TARGET_INVALID;
  }

  /* Memory access is gated on the examined flag, so raise it before touching
   * the core. target_examine_one() clears it again if we return an error. */
  if (!target_was_examined(target)) {
    target_set_examined(target);
  }

  ret = tricore_breakpoints_clear(target);
  if (ret != ERROR_OK) {
    LOG_TARGET_ERROR(target, "Failed to clear breakpoints");
    return ret;
  }
  target_to_aurix(target)->trigger_used = 0;
  target_to_aurix(target)->trigger_is_watchpoint = 0;

  return ERROR_OK;
}

int aurix_reg_get(struct reg *reg) {
  struct tricore_reg *arch_reg = (struct tricore_reg *)reg->arch_info;
  struct target *target = arch_reg->target;

  int ret = target_read_buffer(
      target,
      aurix_ocds_csfr(target_to_aurix(target)->ocds, target->coreid,
                      arch_reg->offset),
      4, arch_reg->value);

  if (ret == 0) {
    reg->valid = true;
    reg->dirty = false;
  }

  return ret;
}
int aurix_reg_set(struct reg *reg, uint8_t *buf) {
  struct tricore_reg *arch_reg = (struct tricore_reg *)reg->arch_info;
  struct target *target = arch_reg->target;

  /* Write through: nothing else ever flushes the cache, and resume discards
   * it, so a cached-only value would silently be lost. */
  int ret = target_write_buffer(
      target,
      aurix_ocds_csfr(target_to_aurix(target)->ocds, target->coreid,
                      arch_reg->offset),
      4, buf);
  if (ret != ERROR_OK) {
    LOG_TARGET_ERROR(target, "Failed to write register %s", reg->name);
    return ret;
  }

  memcpy(arch_reg->value, buf, 4);
  reg->valid = true;
  reg->dirty = false;

  return ERROR_OK;
}
static const struct reg_arch_type aurix_reg_type = {.get = aurix_reg_get,
                                                    .set = aurix_reg_set};

static void tricore_build_reg_cache(struct target *target) {
  int num_regs = ARRAY_SIZE(tricore_core_regs);

  struct reg_cache *cache = malloc(sizeof(struct reg_cache));
  struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
  struct tricore_reg *reg_arch_info =
      calloc(num_regs, sizeof(struct tricore_reg));
  int i;

  if (!cache || !reg_list || !reg_arch_info) {
    LOG_ERROR("Out of memory");
    free(cache);
    free(reg_list);
    free(reg_arch_info);
    target->reg_cache = NULL;
    return;
  }
  target->reg_cache = cache;

  cache->name = "TriCore registers";
  cache->next = NULL;
  cache->reg_list = reg_list;
  cache->num_regs = 0;

  for (i = 0; i < num_regs; i++) {
    reg_arch_info[i].offset = tricore_core_regs[i].reg_offset;
    reg_arch_info[i].target = target;

    reg_list[i].name = tricore_core_regs[i].name;
    reg_list[i].number = i;
    reg_list[i].size = 32;
    reg_list[i].value = reg_arch_info[i].value;
    reg_list[i].type = &aurix_reg_type;
    reg_list[i].arch_info = &reg_arch_info[i];
    reg_list[i].exist = true;

    /* This really depends on the calling convention in use */
    reg_list[i].caller_save = tricore_core_regs[i].caller_saved;

    /* Registers data type, as used by GDB target description */
    reg_list[i].reg_data_type = malloc(sizeof(struct reg_data_type));
    reg_list[i].reg_data_type->type = tricore_core_regs[i].type;

    reg_list[i].feature = malloc(sizeof(struct reg_feature));
    reg_list[i].feature->name = "org.gnu.gdb.tricore.core";
    reg_list[i].group = "general";

    cache->num_regs++;
  }
}

/* Set up structures for target.
 *
 * It is illegal to talk to the target at this stage as this fn is invoked
 * before the JTAG chain has been examined/verified
 * */
int aurix_init_target(struct command_context *cmd_ctx, struct target *target) {
  tricore_build_reg_cache(target);

  return ERROR_OK;
}

/**
 * Free all the resources allocated by the target.
 *
 * WARNING: deinit_target is called unconditionally regardless the target has
 * ever been examined/initialised or not.
 * If a problem has prevented establishing JTAG/SWD/... communication
 *  or
 * if the target was created with -defer-examine flag and has never been
 *  examined
 * then it is not possible to communicate with the target.
 *
 * If you need to talk to the target during deinit, first check if
 * target_was_examined()!
 *
 * @param target The target to deinit
 */
void aurix_deinit_target(struct target *target) {
  struct reg_cache *cache = target->reg_cache;

  if (cache) {
    for (unsigned int i = 0; i < cache->num_regs; i++) {
      free(cache->reg_list[i].reg_data_type);
      free(cache->reg_list[i].feature);
    }
    /* every reg's arch_info points into one array allocated as a block */
    if (cache->num_regs)
      free(cache->reg_list[0].arch_info);
    free(cache->reg_list);
    free(cache);
    target->reg_cache = NULL;
  }

  free(target->private_config);
  target->private_config = NULL;
}

/* translate from virtual to physical address. Default implementation is
 * successful no-op(i.e. virtual==physical).
 */
int aurix_virt2phys(struct target *target, target_addr_t address,
                    target_addr_t *physical) {
  return ERROR_FAIL;
}

/* read directly from physical memory. caches are bypassed and untouched.
 *
 * If the target does not support disabling caches, leaving them untouched,
 * then minimally the actual physical memory location will be read even
 * if cache states are unchanged, flushed, etc.
 *
 * Default implementation is to call read_memory.
 */
int aurix_read_phys_memory(struct target *target, target_addr_t phys_address,
                           uint32_t size, uint32_t count, uint8_t *buffer) {
  return ERROR_FAIL;
}

/*
 * same as read_phys_memory, except that it writes...
 */
int aurix_write_phys_memory(struct target *target, target_addr_t phys_address,
                            uint32_t size, uint32_t count,
                            const uint8_t *buffer) {
  return ERROR_FAIL;
}

int aurix_mmu(struct target *target, bool *enabled) { return ERROR_FAIL; }

/* after reset is complete, the target can check if things are properly set up.
 *
 * This can be used to check if e.g. DCC memory writes have been enabled for
 * arm7/9 targets, which they really should except in the most contrived
 * circumstances.
 */
int aurix_check_reset(struct target *target) { return ERROR_OK; }

/* get GDB file-I/O parameters from target
 */
int aurix_get_gdb_fileio_info(struct target *target,
                              struct gdb_fileio_info *fileio_info) {
  return ERROR_FAIL;
}

/* pass GDB file-I/O response to target
 */
int aurix_gdb_fileio_end(struct target *target, int retcode, int fileio_errno,
                         bool ctrl_c) {
  return ERROR_FAIL;
}

/* Parse target-specific GDB query commands.
 * The string pointer "response_p" is always assigned by the called function
 * to a pointer to a NULL-terminated string, even when the function returns
 * an error. The string memory is not freed by the caller, so this function
 * must pay attention for possible memory leaks if the string memory is
 * dynamically allocated.
 */
int aurix_gdb_query_custom(struct target *target, const char *packet,
                           char **response_p) {
  return ERROR_FAIL;
}

/* do target profiling
 */
int aurix_profiling(struct target *target, uint32_t *samples,
                    uint32_t max_num_samples, uint32_t *num_samples,
                    uint32_t seconds) {
  return ERROR_FAIL;
}

/* Return the number of address bits this target supports. This will
 * typically be 32 for 32-bit targets, and 64 for 64-bit targets. If not
 * implemented, it's assumed to be 32. */
unsigned int aurix_address_bits(struct target *target) { return ERROR_FAIL; }

/* Return the number of system bus data bits this target supports. This
 * will typically be 32 for 32-bit targets, and 64 for 64-bit targets. If
 * not implemented, it's assumed to be 32. */
unsigned int aurix_data_bits(struct target *target) { return ERROR_FAIL; }

static const struct command_registration aurix_commands[] = {
    COMMAND_REGISTRATION_DONE};

struct target_type aurix_target = {
    .name = "aurix",

    .poll = aurix_poll,
    .arch_state = aurix_arch_state,

    .halt = aurix_halt,
    .resume = aurix_resume,
    .step = aurix_step,

    .assert_reset = aurix_assert_reset,
    .deassert_reset = aurix_deassert_reset,
    .soft_reset_halt = aurix_soft_reset_halt,

    .virt2phys = aurix_virt2phys,
    .mmu = aurix_mmu,
    .read_memory = aurix_read_memory,
    .write_memory = aurix_write_memory,

    .checksum_memory = aurix_checksum_memory,

    .get_gdb_arch = aurix_get_gdb_arch,
    .get_gdb_reg_list = aurix_get_gdb_reg_list,

    .run_algorithm = aurix_run_algorithm,
    .start_algorithm = aurix_start_algorithm,
    .wait_algorithm = aurix_wait_algorithm,

    .add_breakpoint = aurix_add_breakpoint,
    .remove_breakpoint = aurix_remove_breakpoint,

    .add_watchpoint = aurix_add_watchpoint,
    .remove_watchpoint = aurix_remove_watchpoint,

    .target_create = aurix_target_create,

    .target_jim_configure = aurix_target_jim_configure,

    .init_target = aurix_init_target,
    .examine = aurix_examine,
    .deinit_target = aurix_deinit_target,

    .commands = aurix_commands,
};
