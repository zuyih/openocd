#ifndef OPENOCD_TARGET_AURIX_H
#define OPENOCD_TARGET_AURIX_H

#include <target/target.h>
#include <helper/command.h>

#include "aurix_ocds.h"


/* CSFR offsets needed outside the target driver: the program counter, and the
 * first argument register of the TriCore ABI, used to hand a parameter block
 * to code running on the core. */
#define AURIX_CSFR_PC 0xFE08
#define AURIX_CSFR_ICR 0xFE2C
#define AURIX_CSFR_D0 0xFF00
#define AURIX_CSFR_A0 0xFF80
#define AURIX_CSFR_A4 (AURIX_CSFR_A0 + 4 * 4)

struct aurix_private_config {
  struct aurix_ocds *ocds;
  /* Bitmaps of the debug triggers handed out, and of which of those watch a
   * data address rather than an instruction. */
  uint8_t trigger_used;
  uint8_t trigger_is_watchpoint;
};

static inline struct aurix_private_config *target_to_aurix(struct target *target) {
  return (struct aurix_private_config*) target->private_config;
}

struct tricore_reg {
  struct target *target;
  uint16_t offset;
  uint8_t value[4];
};
#endif