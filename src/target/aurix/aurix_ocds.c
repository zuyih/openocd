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
 *
 * TC3xx core windows sit at 0xF881_0000 + 0x2_0000 * n, TC4x ones at
 * 0xF881_0000 + 0x4_0000 * n.
 */
static const struct aurix_device aurix_devices[] = {
	/* name, type, CSFR base, stride, cores (0: unknown), DBGACT */
	{ "tc33x",  0x0020B083, 0xF8810000, 0x20000, 0, false },
	{ "tc33x",  0x1020B083, 0xF8810000, 0x20000, 0, false },
	{ "tc33xe", 0x0020C083, 0xF8810000, 0x20000, 0, false },
	{ "tc33xe", 0x1020C083, 0xF8810000, 0x20000, 0, false },
	{ "tc35x",  0x0020A083, 0xF8810000, 0x20000, 0, false },
	{ "tc35x",  0x1020A083, 0xF8810000, 0x20000, 0, false },
	{ "tc36x",  0x00209083, 0xF8810000, 0x20000, 0, false },
	{ "tc36x",  0x10209083, 0xF8810000, 0x20000, 0, false },
	{ "tc37x",  0x00207083, 0xF8810000, 0x20000, 3, false },
	{ "tc37x",  0x10207083, 0xF8810000, 0x20000, 3, false },
	{ "tc37xe", 0x00208083, 0xF8810000, 0x20000, 0, false },
	{ "tc37xe", 0x10208083, 0xF8810000, 0x20000, 0, false },
	{ "tc38x",  0x00206083, 0xF8810000, 0x20000, 0, false },
	{ "tc38x",  0x10206083, 0xF8810000, 0x20000, 0, false },
	{ "tc3ex",  0x00215083, 0xF8810000, 0x20000, 0, false },
	{ "tc3ex",  0x10215083, 0xF8810000, 0x20000, 0, false },
	{ "tc39x",  0x00205083, 0xF8810000, 0x20000, 0, false },
	{ "tc39x",  0x10205083, 0xF8810000, 0x20000, 0, false },
	{ "tc39x",  0x20205083, 0xF8810000, 0x20000, 0, false },

	{ "tc41x",  0x00218083, 0xF8810000, 0x40000, 0, true },
	{ "tc41x",  0x10218083, 0xF8810000, 0x40000, 0, true },
	{ "tc42x",  0x00219083, 0xF8810000, 0x40000, 0, true },
	{ "tc42x",  0x10219083, 0xF8810000, 0x40000, 0, true },
	{ "tc44x",  0x0021A083, 0xF8810000, 0x40000, 0, true },
	{ "tc44x",  0x1021A083, 0xF8810000, 0x40000, 0, true },
	{ "tc45x",  0x0021B083, 0xF8810000, 0x40000, 0, true },
	{ "tc45x",  0x1021B083, 0xF8810000, 0x40000, 0, true },
	{ "tc46x",  0x0021C083, 0xF8810000, 0x40000, 0, true },
	{ "tc46x",  0x1021C083, 0xF8810000, 0x40000, 0, true },
	{ "tc48x",  0x0021D083, 0xF8810000, 0x40000, 0, true },
	{ "tc48x",  0x1021D083, 0xF8810000, 0x40000, 0, true },
	{ "tc49xa", 0x0021E083, 0xF8810000, 0x40000, 0, true },
	{ "tc49xa", 0x1021E083, 0xF8810000, 0x40000, 0, true },
	{ "tc49x",  0x0022B083, 0xF8810000, 0x40000, 0, true },
	{ "tc49x",  0x1022B083, 0xF8810000, 0x40000, 0, true },
	{ "tc4dx",  0x00225083, 0xF8810000, 0x40000, 6, true },
	{ "tc4dx",  0x10225083, 0xF8810000, 0x40000, 6, true },
	{ "tc4rx",  0x00223083, 0xF8810000, 0x40000, 0, true },
	{ "tc4rx",  0x10223083, 0xF8810000, 0x40000, 0, true },
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