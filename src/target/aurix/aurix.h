#ifndef OPENOCD_TARGET_AURIX_H
#define OPENOCD_TARGET_AURIX_H

#include <target/target.h>
#include <helper/command.h>

#include "aurix_ocds.h"


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