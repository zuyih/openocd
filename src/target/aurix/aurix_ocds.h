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

  /* Scratchpad of core 0, and how much lower the next core's sits. The flash
   * loader runs out of the program scratchpad and takes its data from the
   * data one. Zero where the layout has not been established, which leaves
   * flash programming on the slow path. */
  uint32_t dspr0_base;
  uint32_t spr_stride;
  uint32_t pspr_offset;
  uint32_t dspr_size;
  /* Cores from dspr_size_split on have dspr_size_rest instead, which is 0
   * where every core is the same. From TC37x on, cores 0 and 1 get 240 KB of
   * data scratchpad and the rest 96 KB, which is too little for the loader. */
  unsigned int dspr_size_split;
  uint32_t dspr_size_rest;

  /* A six core TC3xx leaves the CORE_ID 5 slot empty and gives its last core
   * CORE_ID 6, which moves both the CSFR window and the scratchpad of that
   * core one stride further along than its index would. Index of the first
   * core affected, 0 where the cores are numbered without a gap. */
  unsigned int core_id_gap;
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

/** CORE_ID of a core, which is its index unless the device leaves a slot out. */
static inline unsigned int aurix_ocds_core_id(const struct aurix_device *dev,
                                              unsigned int coreid) {
  if (dev->core_id_gap && coreid >= dev->core_id_gap)
    return coreid + 1;

  return coreid;
}

/** Data scratchpad of a core, or 0 if the layout is not known. */
static inline uint32_t aurix_ocds_dspr(const struct aurix_ocds *ocds,
                                       unsigned int coreid) {
  if (!ocds->device->dspr0_base)
    return 0;

  return ocds->device->dspr0_base -
         ocds->device->spr_stride * aurix_ocds_core_id(ocds->device, coreid);
}

/** Size of a core's data scratchpad, or 0 if the layout is not known. */
static inline uint32_t aurix_ocds_dspr_size(const struct aurix_ocds *ocds,
                                            unsigned int coreid) {
  const struct aurix_device *dev = ocds->device;

  if (dev->dspr_size_rest && coreid >= dev->dspr_size_split)
    return dev->dspr_size_rest;

  return dev->dspr_size;
}

/** Program scratchpad of a core, or 0 if the layout is not known. */
static inline uint32_t aurix_ocds_pspr(const struct aurix_ocds *ocds,
                                       unsigned int coreid) {
  uint32_t dspr = aurix_ocds_dspr(ocds, coreid);

  return dspr ? dspr + ocds->device->pspr_offset : 0;
}

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
  return ocds->device->csfr_base +
         ocds->device->csfr_stride * aurix_ocds_core_id(ocds->device, coreid) +
         offset;
}

#endif