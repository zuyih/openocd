// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * WCH-Link support of the CMSIS-DAP driver.
 *
 * The WCH-Link(E) in DAP mode (USB ID 1a86:8012) is a CMSIS-DAP probe that
 * understands vendor commands of WCH (CMSIS-DAP vendor command 0x81). The
 * WCH fork of OpenOCD sends them when it opens the probe, and with the
 * configuration commands chip_id and flash_protect it enables or disables the
 * read protection of a CH32F through the probe firmware. This file does the
 * same, with the same command names, but only for WCH-Links: other CMSIS-DAP
 * probes get no vendor commands.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include <helper/command.h>
#include <helper/log.h>
#include <helper/replacements.h>

#include "cmsis_dap.h"

#define WCH_LINK_VID		0x1a86
#define WCH_LINK_DAP_PID	0x8012

#define WCH_LINK_TIMEOUT_MS	300

/* chip types of the WCH-Link firmware */
#define WCH_LINK_CHIP_CH32F10X	0x04
#define WCH_LINK_CHIP_CH32F20X	0x08

/* parameters of the read protection command */
#define WCH_LINK_UNPROTECT	0x02
#define WCH_LINK_PROTECT	0x03

/* WCH-Link chip type set by chip_id, 0 if not set */
static uint8_t wch_arm_chip;
/* read protection requested by flash_protect, 0 if not set */
static uint8_t protect_stat;

/* Send a vendor command and return the first bytes of the response. The
 * responses are not CMSIS-DAP ones, so cmsis_dap_xfer() can not be used. */
static int wch_link_cmd(struct cmsis_dap *dap, const uint8_t *cmd, unsigned int cmd_len,
		uint8_t *resp, unsigned int resp_len)
{
	memcpy(dap->command, cmd, cmd_len);
	int retval = dap->backend->write(dap, cmd_len, WCH_LINK_TIMEOUT_MS);
	if (retval < 0)
		return retval;

	retval = dap->backend->read(dap, WCH_LINK_TIMEOUT_MS, CMSIS_DAP_BLOCKING);
	if (retval < 0)
		return retval;

	unsigned int len = MIN((unsigned int)retval, resp_len);
	memset(resp, 0, resp_len);
	memcpy(resp, dap->response, len);
	LOG_DEBUG("WCH-Link command %02x %02x response %02x %02x %02x %02x (%d bytes)",
			cmd[0], cmd[1], resp[0], resp[1], resp[2], resp[3], retval);

	return ERROR_OK;
}

static void wch_link_flash_protect(struct cmsis_dap *dap)
{
	const uint8_t set_chip[] = { 0x81, 0x0c, 0x02, wch_arm_chip, 0x01 };
	const uint8_t protect[] = { 0x81, 0x06, 0x08, protect_stat,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	const uint8_t reset[] = { 0x81, 0x0b, 0x01, 0x00 };
	const uint8_t get_info[] = { 0x81, 0x0d, 0x01, 0x01 };
	const uint8_t check_protect[] = { 0x81, 0x06, 0x01, 0x04 };
	uint8_t resp[4];

	/* the command sequence of the WCH fork, the results are only checked
	 * at the end by reading the protection state back */
	wch_link_cmd(dap, set_chip, sizeof(set_chip), resp, sizeof(resp));
	wch_link_cmd(dap, protect, sizeof(protect), resp, sizeof(resp));
	alive_sleep(500);
	wch_link_cmd(dap, reset, sizeof(reset), resp, sizeof(resp));
	wch_link_cmd(dap, get_info, sizeof(get_info), resp, sizeof(resp));
	wch_link_cmd(dap, set_chip, sizeof(set_chip), resp, sizeof(resp));
	int retval = wch_link_cmd(dap, check_protect, sizeof(check_protect), resp, sizeof(resp));

	if (retval == ERROR_OK && protect_stat == WCH_LINK_PROTECT && resp[3] == 0x01)
		LOG_INFO("Success to Enable Read-Protect");
	else if (retval == ERROR_OK && protect_stat == WCH_LINK_UNPROTECT && resp[3] == 0x00)
		LOG_INFO("Success to Disable Read-Protect");
	else
		LOG_ERROR("Operation failed");
}

void cmsis_dap_wch_open(struct cmsis_dap *dap)
{
	if (dap->vid != WCH_LINK_VID || dap->pid != WCH_LINK_DAP_PID) {
		if (wch_arm_chip || protect_stat)
			LOG_WARNING("chip_id and flash_protect need a WCH-Link, ignored");
		return;
	}

	LOG_INFO("WCH-Link detected");

	const uint8_t get_info[] = { 0x81, 0x0d, 0x01, 0x01 };
	uint8_t resp[7];
	wch_link_cmd(dap, get_info, sizeof(get_info), resp, sizeof(resp));

	/* The WCH fork sends the read protection command with an undefined
	 * parameter if chip_id is set without flash_protect, skip it. */
	if (!protect_stat)
		return;
	if (!wch_arm_chip) {
		LOG_WARNING("flash_protect needs chip_id, ignored");
		return;
	}

	wch_link_flash_protect(dap);
}

COMMAND_HANDLER(cmsis_dap_wch_handle_chip_id_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!strcmp(CMD_ARGV[0], "CH32F1x") || !strcmp(CMD_ARGV[0], "CH32F10x")) {
		wch_arm_chip = WCH_LINK_CHIP_CH32F10X;
	} else if (!strcmp(CMD_ARGV[0], "CH32F2x") || !strcmp(CMD_ARGV[0], "CH32F20x")) {
		wch_arm_chip = WCH_LINK_CHIP_CH32F20X;
	} else {
		command_print(CMD, "unknown chip_id '%s'", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_wch_handle_flash_protect_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bool enable;
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], enable);
	protect_stat = enable ? WCH_LINK_PROTECT : WCH_LINK_UNPROTECT;

	return ERROR_OK;
}

const struct command_registration cmsis_dap_wch_command_handlers[] = {
	{
		.name = "chip_id",
		.handler = &cmsis_dap_wch_handle_chip_id_command,
		.mode = COMMAND_CONFIG,
		.help = "set the CH32F family for flash_protect (WCH-Link only)",
		.usage = "(CH32F1x | CH32F10x | CH32F2x | CH32F20x)",
	},
	{
		.name = "flash_protect",
		.handler = &cmsis_dap_wch_handle_flash_protect_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable the read protection of the CH32F "
			"through the WCH-Link when it is opened (WCH-Link only)",
		.usage = "(on | off)",
	},
	COMMAND_REGISTRATION_DONE
};
