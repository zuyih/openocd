#include <stdio.h>
#include <string.h>

#include <helper/command.h>
#include <helper/list.h>

#include "aurix_ocds.h"
#include "helper/jim-nvp.h"
#include "jim.h"
#include <jtag/adapter.h>
#include "jtag/interface.h"
#include "jtag/tas.h"
#include "transport/transport.h"

static OOCD_LIST_HEAD(all_ocds);

/*
 * Known derivatives. The device_type values are the IEEE 1149.1 device IDs the
 * TAS server reports; both the initial and the A-step ID are listed where the
 * silicon uses a version nibble.
 */
/*
 * Shared properties. A core window sits 0x20000 apart on TC3xx and 0x40000 on
 * TC4x; the debug event action lives in DBGACT from TriCore 1.8 onwards. A
 * core count of zero means it has not been established, and so does a missing
 * scratchpad layout, which only costs the flash loader. Core counts are filled
 * in from the manual of the derivative, so the entries left bare are the ones
 * whose manual was not to hand. No TC3xx carries a scratchpad layout: the
 * loader has not been made to work on that family, and a bank is erased before
 * it is written, so offering it there turns a failed write into a blank
 * device.
 */
#define AURIX_TC3XX .csfr_base = 0xF8810000, .csfr_stride = 0x20000

#define AURIX_TC4XX \
	.csfr_base = 0xF8810000, .csfr_stride = 0x40000, .has_dbgact = true

/* Scratchpads: DSPR of CORE_ID 0 at 0x70000000 with the next one 0x10000000
 * lower, and the program scratchpad 1 MB above each. TC3xx and TC4Dx agree on
 * this; they differ in how much data scratchpad a core gets. */
#define AURIX_SPR \
	.dspr0_base = 0x70000000, .spr_stride = 0x10000000, \
	.pspr_offset = 0x100000

/* How much data scratchpad a core gets. The smaller TC3xx give every core the
 * same 192 KB; from TC37x on, cores 0 and 1 get 240 KB and the rest 96 KB. */
#define AURIX_SPR_192K AURIX_SPR, .dspr_size = 192 * 1024

#define AURIX_SPR_240K AURIX_SPR, .dspr_size = 240 * 1024

#define AURIX_SPR_240K_96K \
	AURIX_SPR, .dspr_size = 240 * 1024, \
	.dspr_size_split = 2, .dspr_size_rest = 96 * 1024

/*
 * TC39x has six cores but no CORE_ID 5: its last core answers as CORE_ID 6,
 * putting the CSFR window at 0xF88D0000 rather than the 0xF88B0000 an
 * uninterrupted count would give, which is reserved, and the scratchpad at
 * 0x10000000 rather than 0x20000000.
 */
#define AURIX_TC39X \
	AURIX_TC3XX, .num_cores = 6, AURIX_SPR_240K_96K, .core_id_gap = 5

static const struct aurix_device aurix_devices[] = {
	{ .name = "tc33x", .device_type = 0x0020B083, AURIX_TC3XX,
	  .num_cores = 1, AURIX_SPR_192K },
	{ .name = "tc33x", .device_type = 0x1020B083, AURIX_TC3XX,
	  .num_cores = 1, AURIX_SPR_192K },
	{ .name = "tc33xe", .device_type = 0x0020C083, AURIX_TC3XX },
	{ .name = "tc33xe", .device_type = 0x1020C083, AURIX_TC3XX },
	{ .name = "tc35x", .device_type = 0x0020A083, AURIX_TC3XX },
	{ .name = "tc35x", .device_type = 0x1020A083, AURIX_TC3XX },
	{ .name = "tc36x", .device_type = 0x00209083, AURIX_TC3XX,
	  .num_cores = 2, AURIX_SPR_192K },
	{ .name = "tc36x", .device_type = 0x10209083, AURIX_TC3XX,
	  .num_cores = 2, AURIX_SPR_192K },
	{ .name = "tc37x", .device_type = 0x00207083, AURIX_TC3XX,
	  .num_cores = 3, AURIX_SPR_240K_96K },
	{ .name = "tc37x", .device_type = 0x10207083, AURIX_TC3XX,
	  .num_cores = 3, AURIX_SPR_240K_96K },
	{ .name = "tc37xe", .device_type = 0x00208083, AURIX_TC3XX,
	  .num_cores = 3, AURIX_SPR_240K_96K },
	{ .name = "tc37xe", .device_type = 0x10208083, AURIX_TC3XX,
	  .num_cores = 3, AURIX_SPR_240K_96K },
	{ .name = "tc38x", .device_type = 0x00206083, AURIX_TC3XX,
	  .num_cores = 4, AURIX_SPR_240K_96K },
	{ .name = "tc38x", .device_type = 0x10206083, AURIX_TC3XX,
	  .num_cores = 4, AURIX_SPR_240K_96K },
	{ .name = "tc3ex", .device_type = 0x00215083, AURIX_TC3XX },
	{ .name = "tc3ex", .device_type = 0x10215083, AURIX_TC3XX },
	{ .name = "tc39x", .device_type = 0x00205083, AURIX_TC39X },
	{ .name = "tc39x", .device_type = 0x10205083, AURIX_TC39X },
	{ .name = "tc39x", .device_type = 0x20205083, AURIX_TC39X },

	{ .name = "tc41x", .device_type = 0x00218083, AURIX_TC4XX },
	{ .name = "tc41x", .device_type = 0x10218083, AURIX_TC4XX },
	{ .name = "tc42x", .device_type = 0x00219083, AURIX_TC4XX },
	{ .name = "tc42x", .device_type = 0x10219083, AURIX_TC4XX },
	{ .name = "tc44x", .device_type = 0x0021A083, AURIX_TC4XX },
	{ .name = "tc44x", .device_type = 0x1021A083, AURIX_TC4XX },
	{ .name = "tc45x", .device_type = 0x0021B083, AURIX_TC4XX },
	{ .name = "tc45x", .device_type = 0x1021B083, AURIX_TC4XX },
	{ .name = "tc46x", .device_type = 0x0021C083, AURIX_TC4XX },
	{ .name = "tc46x", .device_type = 0x1021C083, AURIX_TC4XX },
	{ .name = "tc48x", .device_type = 0x0021D083, AURIX_TC4XX },
	{ .name = "tc48x", .device_type = 0x1021D083, AURIX_TC4XX },
	{ .name = "tc49xa", .device_type = 0x0021E083, AURIX_TC4XX },
	{ .name = "tc49xa", .device_type = 0x1021E083, AURIX_TC4XX },
	{ .name = "tc49x", .device_type = 0x0022B083, AURIX_TC4XX },
	{ .name = "tc49x", .device_type = 0x1022B083, AURIX_TC4XX },
	{ .name = "tc4dx", .device_type = 0x00225083, AURIX_TC4XX,
	  .num_cores = 6, AURIX_SPR_240K },
	{ .name = "tc4dx", .device_type = 0x10225083, AURIX_TC4XX,
	  .num_cores = 6, AURIX_SPR_240K },
	{ .name = "tc4rx", .device_type = 0x00223083, AURIX_TC4XX },
	{ .name = "tc4rx", .device_type = 0x10223083, AURIX_TC4XX },
};


static const struct aurix_device *aurix_device_by_type(uint32_t device_type) {
  for (size_t i = 0; i < ARRAY_SIZE(aurix_devices); i++)
    if (aurix_devices[i].device_type == device_type)
      return &aurix_devices[i];

  return NULL;
}

/**
 * Synchronous read of a word from memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param ap The MEM-AP to access.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the result will be stored.
 *
 * @return ERROR_OK for success; *value holds the result.
 * Otherwise a fault code.
 */
int aurix_ocds_atomic_read_u32(struct aurix_ocds *ocds, target_addr_t address,
		uint32_t *value)
{
	int retval;

	retval = aurix_ocds_queue_soc_read_u32(ocds, address, value);
	if (retval != ERROR_OK)
		return retval;

	return aurix_ocds_run(ocds);
}


struct aurix_ocds *aurix_ocds_by_jim_obj(Jim_Interp *interp, Jim_Obj *o) {
  struct aurix_ocds *ocds;
  const char *name = Jim_GetString(o, NULL);

  list_for_each_entry(ocds, &all_ocds, lh) {
    if (strcmp(name, ocds->name) == 0) {
      return ocds;
    }
  }

  return NULL;
}

enum ocds_cfg_param {
  CFG_CHAIN_POSITION,
  CFG_DEVICE,
};

static const struct jim_nvp nvp_config_opts[] = {
    {.name = "-chain-position", .value = CFG_CHAIN_POSITION},
    {.name = "-device", .value = CFG_DEVICE},
    {.name = NULL, .value = -1}};

static int aurix_ocds_configure(struct jim_getopt_info *goi,
                                struct aurix_ocds *ocds) {
  struct jim_nvp *n;
  int e;
  const char *name;

  jim_getopt_string(goi, &name, NULL);
  ocds->name = strdup(name);

  /* parse config ... */
  while (goi->argc > 0) {
    Jim_SetEmptyResult(goi->interp);

    e = jim_getopt_nvp(goi, nvp_config_opts, &n);
    if (e != JIM_OK) {
      jim_getopt_nvp_unknown(goi, nvp_config_opts, 0);
      return e;
    }
    switch (n->value) {
    case CFG_CHAIN_POSITION: {
      Jim_Obj *o_t;
      e = jim_getopt_obj(goi, &o_t);
      if (e != JIM_OK)
        return e;

      struct jtag_tap *tap;
      tap = jtag_tap_by_jim_obj(goi->interp, o_t);
      if (!tap) {
        Jim_SetResultString(goi->interp, "-chain-position is invalid", -1);
        return JIM_ERR;
      }
      ocds->tap = tap;
      /* loop for more */
      break;
    }
    case CFG_DEVICE: {
      const char *id;
      e = jim_getopt_string(goi, &id, NULL);
      if (e != JIM_OK)
        return e;

      free(ocds->device_id);
      ocds->device_id = strdup(id);
      if (!ocds->device_id) {
        Jim_SetResultString(goi->interp, "Out of memory", -1);
        return JIM_ERR;
      }
      break;
    }
    default:
      break;
    }
  }

  return JIM_OK;
}

COMMAND_HANDLER(aurix_ocds_create) {
  if (CMD_ARGC < 3)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct aurix_ocds *ocds = calloc(1, sizeof(struct aurix_ocds));
  if (!ocds) {
    LOG_ERROR("Out of memory");
    return ERROR_FAIL;
  }

  struct jim_getopt_info goi;
  jim_getopt_setup(&goi, CMD_CTX->interp, CMD_ARGC, CMD_JIMTCL_ARGV);

  int e = aurix_ocds_configure(&goi, ocds);
  if (e != JIM_OK) {
    int reslen;
    const char *result = Jim_GetString(Jim_GetResult(CMD_CTX->interp), &reslen);
    if (reslen > 0)
      command_print(CMD, "%s", result);
    goto err;
  }

  if (!ocds->tap) {
    command_print(CMD, "-chain-position required when creating OCDS");
    goto err;
  }

  list_add_tail(&ocds->lh, &all_ocds);

  return ERROR_OK;

err:
  free((void *)ocds->name);
  free(ocds->device_id);
  free(ocds);
  return ERROR_COMMAND_ARGUMENT_INVALID;
}

COMMAND_HANDLER(aurix_ocds_init) {
  struct aurix_ocds *ocds;

  list_for_each_entry(ocds, &all_ocds, lh) {
    /* skip taps that are disabled */
    if (!ocds->tap->enabled)
      continue;

    if (transport_is_tas()) {
      ocds->ops = adapter_driver->tas_ops;
      int err = ocds->ops->connect(ocds);
      if (err) {
        return err;
      }

      ocds->device = aurix_device_by_type(ocds->device_type);
      if (!ocds->device) {
        LOG_ERROR("OCDS %s: unsupported AURIX device type 0x%08" PRIx32
                  ", cannot determine the CSFR layout",
                  ocds->name, ocds->device_type);
        return ERROR_FAIL;
      }
      if (ocds->device->num_cores)
        LOG_INFO("OCDS %s: detected %s (device type 0x%08" PRIx32
                 ", %u cores)",
                 ocds->name, ocds->device->name, ocds->device_type,
                 ocds->device->num_cores);
      else
        LOG_INFO("OCDS %s: detected %s (device type 0x%08" PRIx32 ")",
                 ocds->name, ocds->device->name, ocds->device_type);
    }
  }

  return ERROR_OK;
}

static const struct command_registration ocds_subcommand_handlers[] = {
    {
        .name = "init",
        .mode = COMMAND_ANY,
        .handler = aurix_ocds_init,
        .usage = "",
        .help = "Initialize all OCDS systems",
    },
    {
        .name = "create",
        .mode = COMMAND_ANY,
        .handler = aurix_ocds_create,
        .usage = "name '-chain-position' tap ['-device' identifier]",
        .help = "Creates a new DAP instance",
    },
    COMMAND_REGISTRATION_DONE};

static const struct command_registration ocds_commands[] = {
    {
        .name = "ocds",
        .mode = COMMAND_CONFIG,
        .help = "OCDS commands",
        .chain = ocds_subcommand_handlers,
        .usage = "",
    },
    COMMAND_REGISTRATION_DONE};

int ocds_register_commands(struct command_context *cmd_ctx) {
  return register_commands(cmd_ctx, NULL, ocds_commands);
}