#ifndef OPENOCD_TARGET_AURIX_AURIX_OCDS_H
#define OPENOCD_TARGET_AURIX_AURIX_OCDS_H

#include "helper/list.h"
#include "helper/log.h"
#include <jtag/jtag.h>
#include <stdatomic.h>

/**
 * Per-derivative properties of the TriCore debug interface.
 *
 * The CSFR offsets inside a core window (DBGSR, PSW, PC, D0.., A0.., ...) are
 * defined by the TriCore architecture and are identical on TC3xx and TC4x.
 * What differs between the families is where the per-core windows live.
 */
struct aurix_device {
  const char *name;
  uint32_t device_type;  /* IEEE 1149.1 device ID as reported by TAS */
  uint32_t csfr_base;    /* CSFR window of core 0 */
  uint32_t csfr_stride;  /* distance between two core windows */
  unsigned int num_cores;
  /* TriCore 1.8 puts the action a debug event takes in a single DBGACT
   * register; on 1.6.x every event register carries its own EVTA field. Both
   * are driven, see aurix_trigger_arm(). */
  bool has_dbgact;
};

struct aurix_ocds {
  const char *name;
  struct list_head lh;
  struct jtag_tap *tap;
  atomic_bool used;
  uint8_t con_id;

  /* Optional substring of the TAS target identifier this OCDS should bind to,
   * to pick one out of several attached devices. NULL: take the first match. */
  char *device_id;

  /* Filled in by the adapter on connect, resolved to @ref device afterwards. */
  uint32_t device_type;
  const struct aurix_device *device;

  const struct aurix_ocds_ops *ops;
  uint8_t *queue_buffer;
};

struct aurix_ocds_ops {
  int (*connect)(struct aurix_ocds *ocds);
  int (*queue_soc_read)(struct aurix_ocds *ocds, uint32_t addr, uint32_t size,
                        uint32_t count, void *buffer);
  int (*queue_soc_write)(struct aurix_ocds *ocds, uint32_t addr, uint32_t size,
                         uint32_t count, const void *buffer);
  int (*run)(struct aurix_ocds *ocds);
};

struct aurix_ocds *aurix_ocds_by_jim_obj(Jim_Interp *interp, Jim_Obj *o);
int ocds_register_commands(struct command_context *cmd_ctx);

static inline int aurix_ocds_queue_soc_read(struct aurix_ocds *ocds,
                                            uint32_t addr, uint32_t size,
                                            uint32_t count, void *data) {
  assert(ocds->ops);
  return ocds->ops->queue_soc_read(ocds, addr, size, count, data);
}

static inline int aurix_ocds_queue_soc_read_u8(struct aurix_ocds *ocds,
                                               uint32_t addr, uint8_t *data) {
  return aurix_ocds_queue_soc_read(ocds, addr, 1, 1, data);
}
static inline int aurix_ocds_queue_soc_read_u16(struct aurix_ocds *ocds,
                                                uint32_t addr, uint16_t *data) {
  return aurix_ocds_queue_soc_read(ocds, addr, 2, 1, data);
}
static inline int aurix_ocds_queue_soc_read_u32(struct aurix_ocds *ocds,
                                                uint32_t addr, uint32_t *data) {
  return aurix_ocds_queue_soc_read(ocds, addr, 4, 1, data);
}

static inline int aurix_ocds_queue_soc_write(struct aurix_ocds *ocds,
                                             uint32_t addr, uint32_t size,
                                             uint32_t count, const void *data) {
  assert(ocds->ops);
  return ocds->ops->queue_soc_write(ocds, addr, size, count, data);
}
static inline int aurix_ocds_queue_soc_write_u8(struct aurix_ocds *ocds,
                                                uint32_t addr, uint8_t data) {
  return aurix_ocds_queue_soc_write(ocds, addr, 1, 1, &data);
}
static inline int aurix_ocds_queue_soc_write_u16(struct aurix_ocds *ocds,
                                                 uint32_t addr, uint16_t data) {
  return aurix_ocds_queue_soc_write(ocds, addr, 2, 1, &data);
}
static inline int aurix_ocds_queue_soc_write_u32(struct aurix_ocds *ocds,
                                                 uint32_t addr, uint32_t data) {
  return aurix_ocds_queue_soc_write(ocds, addr, 4, 1, &data);
}
static inline int aurix_ocds_queue_soc_write_u64(struct aurix_ocds *ocds,
                                                 uint32_t addr, uint64_t data) {
  return aurix_ocds_queue_soc_write(ocds, addr, 8, 1, &data);
}

static inline int aurix_ocds_run(struct aurix_ocds *ocds) {
  return ocds->ops->run(ocds);
}

int aurix_ocds_atomic_read_u32(struct aurix_ocds *ocds, target_addr_t address,
		uint32_t *value);

/**
 * Absolute address of a TriCore CSFR of a given core.
 *
 * @param ocds The OCDS the core is attached to; must have been connected so
 *	that @ref aurix_ocds::device is resolved.
 * @param coreid Index of the core within the device.
 * @param offset CSFR offset inside the core window, e.g. 0xFD00 for DBGSR.
 */
static inline uint32_t aurix_ocds_csfr(const struct aurix_ocds *ocds,
                                       unsigned int coreid, uint32_t offset) {
  assert(ocds->device);
  return ocds->device->csfr_base + ocds->device->csfr_stride * coreid + offset;
}

#endif