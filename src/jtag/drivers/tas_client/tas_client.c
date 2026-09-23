// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   TAS Client Adapter Interface for Infineon AURIX                       *
 *   Copyright (C) 2026 Infineon Technologies AG                           *
 ***************************************************************************/

#include <stdbool.h>
#include <string.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "helper/command.h"
#include "helper/log.h"
#include "jtag/adapter.h"
#include "jtag/ifxdap.h"
#include "jtag/interface.h"
#include "jtag/jtag.h"
#include "server/server.h"
#include "target/aurix/ocmts.h"
#include "transport/transport.h"

#include "tas_am15_am14.h"
#include "tas_pkt.h"
#include "tas_protocol.h"

static struct tas_client tas_client = {.max_pl2rq_pkt_size = 1024, .max_pl2rsp_pkt_size = 1024, .pl0_max_num_rw = 128};
/* Set by "tas-client host", else the defaults below are used. */
static char *server_host;
static char *server_port;
#define TAS_SERVER_DEFAULT_HOST "localhost"
#define TAS_SERVER_DEFAULT_PORT "24817"
static uint32_t current_address;
static struct tas_client_mem_req mem_reqs[256];
static size_t mem_req_num;
static tas_target_info_st target;
static bool harr;
static bool porst;

/*
 * Without "adapter serial", take the first attached device whose type is one
 * of the IDs a declared TAP expects, so that a board configuration finds its
 * own device when devices of other types are attached as well. With no TAP
 * declared at all, fall back to the first device.
 */
static const tas_target_info_st *tas_client_find_target(const tas_target_info_st *targets, size_t target_num,
														 unsigned int *matches)
{
	const tas_target_info_st *found = NULL;

	*matches = 0;
	if (!jtag_all_taps()) {
		*matches = 1;
		return &targets[0];
	}

	for (size_t i = 0; i < target_num; i++) {
		for (struct jtag_tap *tap = jtag_all_taps(); tap; tap = tap->next_tap) {
			bool hit = false;

			if (!tap->enabled)
				continue;
			for (unsigned int j = 0; j < tap->expected_ids_cnt; j++)
				if (targets[i].device_type == tap->expected_ids[j])
					hit = true;
			if (hit) {
				if (!found)
					found = &targets[i];
				(*matches)++;
				break;
			}
		}
	}

	return found;
}

static int tas_client_init(void)
{
	int err = ERROR_OK, sock = -1;
	struct addrinfo hints;
	struct addrinfo *result, *rp;
	const char *serial = adapter_get_required_serial();
	const char *host = server_host ? server_host : TAS_SERVER_DEFAULT_HOST;
	const char *port = server_port ? server_port : TAS_SERVER_DEFAULT_PORT;
	tas_target_info_st *targets;
	size_t target_num;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;
	hints.ai_protocol = IPPROTO_TCP;

	LOG_INFO("Connecting to TAS server %s:%s", host, port);

	err = getaddrinfo(host, port, &hints, &result);
	if (err != 0) {
		LOG_ERROR("Failed to get address for host %s", host);
		return ERROR_FAIL;
	}

	err = -1;
	for (rp = result; rp; rp = rp->ai_next) {
		sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (sock == -1)
			continue;

		/* Set buffer size for outstanding packets */
		int buffersize = 64*1024;  // 64k
		setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *) &buffersize, sizeof(buffersize));
		setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char *) &buffersize, sizeof(buffersize));

		err = connect(sock, rp->ai_addr, rp->ai_addrlen);
		if (err == 0)
			break;
		/* e.g. ::1 first while the server listens on 127.0.0.1 only */
		close(sock);
		sock = -1;
	}

	freeaddrinfo(result);

	if (err != 0) {
		LOG_ERROR("Failed to connect to %s:%s", host, port);
		return ERROR_CONNECTION_REJECTED;
	}

	if (tas_client_connect(&tas_client, sock) != 0) {
		LOG_ERROR("Failed to connect to TAS server");
		return ERROR_CONNECTION_REJECTED;
	}

	if (tas_client_get_targets(&tas_client, &targets, &target_num) != 0) {
		LOG_ERROR("Failed to receive targets");
		return ERROR_FAIL;
	}

	if (target_num == 0) {
		LOG_ERROR("No targets to connect");
		return ERROR_FAIL;
	}

	if (!serial) {
		unsigned int matches;
		const tas_target_info_st *found = tas_client_find_target(targets, target_num, &matches);

		if (!found) {
			LOG_ERROR("None of the attached devices is of a type a declared TAP expects:");
			for (size_t i = 0; i < target_num; i++)
				LOG_ERROR("  \"%s\", device type 0x%08" PRIx32, targets[i].identifier,
						  targets[i].device_type);
			free(targets);
			return ERROR_FAIL;
		}
		if (matches > 1)
			LOG_WARNING("%u attached devices match, using \"%s\"; pick one with: adapter serial \"<identifier>\"",
						matches, found->identifier);
		target = *found;
	} else {
		bool found = false;
		for (size_t i = 0; i < target_num; i++) {
			if (strcmp(serial, targets[i].identifier) == 0) {
				found = true;
				target = targets[i];
				break;
			}
		}
		if (!found) {
			LOG_ERROR("No target with serial %s found", serial);
			free(targets);
			return ERROR_FAIL;
		}
	}
	free(targets);

	LOG_INFO("Starting session for target %s", target.identifier);

	err = tas_client_session_start(&tas_client, target.identifier);
	if (err) {
		LOG_ERROR("Failed to start session for target %s", target.identifier);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int tas_client_quit(void)
{
	close(tas_client.sock);
	free(server_host);
	server_host = NULL;
	free(server_port);
	server_port = NULL;
	return 0;
}

static int tas_client_reset(int trst, int srst)
{
	int err;
	if (trst != 0)
		return ERROR_NOT_IMPLEMENTED;

	if (srst && !porst) {
		struct tas_client_mem_req ctrl_mem_req = {
			.addr = TAS_AM15_RW_USERPINS_CNTRL,
			.buffer = (uint32_t[]){TAS_UPC_ADD_SFP_RESET},
			.length = 4,
			.is_read = false,
		};
		err = tas_client_execute_mem_req(&tas_client, 15, &ctrl_mem_req);
		if (err) {
			LOG_ERROR("Failed to set user pins for reset");
			return ERROR_FAIL;
		}

		struct tas_client_mem_req upc_mem_req = {
			.addr = TAS_AM15_RW_USERPINS,
			.buffer = (uint32_t[]){0, TAS_UP_SFP_RESET},
			.length = 8,
			.is_read = false,
		};
		err = tas_client_execute_mem_req(&tas_client, 15, &upc_mem_req);
		if (err) {
			LOG_ERROR("Failed to set user pins for reset");
			return ERROR_FAIL;
		}
		porst = true;
	} else {
		if (porst) {
			if (harr)
				err = tas_client_device_connect(&tas_client, TAS_DEV_CON_FEAT_RESET_AND_HALT);
			else
				err = tas_client_device_connect(&tas_client, TAS_DEV_CON_FEAT_RESET);

			harr = false;
			porst = false;
			return err;
		}
	}

	return 0;
}

static int tas_client_op_run(struct ocmts *ocds)
{
	int err = ERROR_OK;
	if (mem_req_num > 0) {
		err = tas_client_execute_mem_reqs(&tas_client, 0, mem_reqs, mem_req_num);
		if (LOG_LEVEL_IS(LOG_LVL_DEBUG)) {
			for (size_t i = 0; i < mem_req_num; i++) {
				struct tas_client_mem_req *req = &mem_reqs[i];
				uint32_t data = 0;
				memcpy(&data, req->buffer, MIN(sizeof(data), req->length));
				LOG_DEBUG("TAS PL0 request[%zu]: %s addr=0x%08" PRIx32 
							" length=%zu data=0x%08" PRIx32,
							i, req->is_read ? "read" : "write", req->addr, req->length, data);
			}
		}	
	}
	mem_req_num = 0;
	if (err)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int tas_client_op_connect(struct ocmts *ocmts)
{
	uint32_t tap;
	bool found = false;

	for (tap = 0; tap < ocmts->tap->expected_ids_cnt; tap++) {
		if (target.device_type == ocmts->tap->expected_ids[tap]) {
			found = true;
			break;
		}
	}

	if (!found) {
		LOG_ERROR("No matching target for OCDS %s found", ocmts->name);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	ocmts->tap->idcode = target.device_type;
	ocmts->tap->has_idcode = true;

	int err;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST)
		err = tas_client_device_connect(&tas_client, TAS_DEV_CON_FEAT_RESET_AND_HALT);
	else
		err = tas_client_device_connect(&tas_client, TAS_DEV_CON_FEAT_NONE);

	if (err) {
		LOG_ERROR("Failed to connect to device %s", target.identifier);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int tas_client_set_address(struct ocmts *ocds, uint32_t addr)
{
	current_address = addr;
	return ERROR_OK;
}

/*
 * Make room in a full queue. The queue is empty afterwards also when running
 * it fails: its requests point into buffers that the caller gives up as soon
 * as it sees the error.
 */
static int tas_client_queue_make_room(void)
{
	int ret;

	if (mem_req_num < ARRAY_SIZE(mem_reqs))
		return ERROR_OK;

	ret = tas_client_execute_mem_reqs(&tas_client, 0, mem_reqs, mem_req_num);
	mem_req_num = 0;
	return ret ? ERROR_FAIL : ERROR_OK;
}

static int tas_client_read_byte(struct ocmts *ocds, uint8_t *data)
{
	if (tas_client_queue_make_room() != ERROR_OK)
		return ERROR_FAIL;
	mem_reqs[mem_req_num].addr = current_address;
	mem_reqs[mem_req_num].buffer = (void *)data;
	mem_reqs[mem_req_num].length = 1;
	mem_reqs[mem_req_num].is_read = true;
	mem_req_num++;

	return ERROR_OK;
}

static int tas_client_read_hword(struct ocmts *ocds, uint16_t *data)
{
	if (current_address & 0x1) {
		LOG_ERROR("Half-Word write address must be 2-byte aligned");
		return ERROR_FAIL;
	}
	if (tas_client_queue_make_room() != ERROR_OK)
		return ERROR_FAIL;
	mem_reqs[mem_req_num].addr = current_address;
	mem_reqs[mem_req_num].buffer = (void *)data;
	mem_reqs[mem_req_num].length = 2;
	mem_reqs[mem_req_num].is_read = true;
	mem_req_num++;

	return ERROR_OK;
}

static int tas_client_read_word(struct ocmts *ocds, uint32_t *data)
{
	if (current_address & 0x3) {
		LOG_ERROR("Word write address must be 4-byte aligned");
		return ERROR_FAIL;
	}
	if (tas_client_queue_make_room() != ERROR_OK)
		return ERROR_FAIL;
	mem_reqs[mem_req_num].addr = current_address;
	mem_reqs[mem_req_num].buffer = (void *)data;
	mem_reqs[mem_req_num].length = 4;
	mem_reqs[mem_req_num].is_read = true;
	mem_req_num++;

	return ERROR_OK;
}

/*
 * Queue a block transfer as requests that each fit a packet. A full 256-word
 * write need not: with the headers around it, it takes 1052 bytes, where the
 * TAS server for the Lite kits allows 1048.
 */
static int tas_client_queue_block(void *data, size_t length, bool is_read)
{
	size_t max_words = tas_client_max_block_words(&tas_client, is_read);
	uint8_t *p = data;

	if (max_words == 0) {
		LOG_ERROR("TAS packets are too small for block transfers");
		return ERROR_FAIL;
	}

	while (length) {
		size_t words = MIN(length, max_words);

		if (tas_client_queue_make_room() != ERROR_OK)
			return ERROR_FAIL;
		mem_reqs[mem_req_num].addr = current_address;
		mem_reqs[mem_req_num].buffer = p;
		mem_reqs[mem_req_num].length = words * 4;
		mem_reqs[mem_req_num].is_read = is_read;
		mem_req_num++;

		current_address += words * 4;
		p += words * 4;
		length -= words;
	}

	return ERROR_OK;
}

static int tas_client_read_block(struct ocmts *ocds, void *data, size_t length)
{
	if (length > 256) {
		LOG_ERROR("Block read length must up to 256 words");
		return ERROR_FAIL;
	}
	if (current_address & 0x3) {
		LOG_ERROR("Block read address must be 4-byte aligned");
		return ERROR_FAIL;
	}

	return tas_client_queue_block(data, length, true);
}

static int tas_client_write_byte(struct ocmts *ocds, const void *data)
{
	if (tas_client_queue_make_room() != ERROR_OK)
		return ERROR_FAIL;
	mem_reqs[mem_req_num].addr = current_address;
	mem_reqs[mem_req_num].buffer = (void *)data;
	mem_reqs[mem_req_num].length = 1;
	mem_reqs[mem_req_num].is_read = false;
	mem_req_num++;

	return ERROR_OK;
}

static int tas_client_write_hword(struct ocmts *ocds, const void *data)
{
	if (current_address & 0x1) {
		LOG_ERROR("Half-Word write address must be 2-byte aligned");
		return ERROR_FAIL;
	}
	if (tas_client_queue_make_room() != ERROR_OK)
		return ERROR_FAIL;
	mem_reqs[mem_req_num].addr = current_address;
	mem_reqs[mem_req_num].buffer = (void *)data;
	mem_reqs[mem_req_num].length = 2;
	mem_reqs[mem_req_num].is_read = false;
	mem_req_num++;

	return ERROR_OK;
}

static int tas_client_write_word(struct ocmts *ocds, const void *data)
{
	if (current_address & 0x3) {
		LOG_ERROR("Word write address must be 4-byte aligned");
		return ERROR_FAIL;
	}
	if (tas_client_queue_make_room() != ERROR_OK)
		return ERROR_FAIL;
	mem_reqs[mem_req_num].addr = current_address;
	mem_reqs[mem_req_num].buffer = (void *)data;
	mem_reqs[mem_req_num].length = 4;
	mem_reqs[mem_req_num].is_read = false;
	mem_req_num++;

	return ERROR_OK;
}

static int tas_client_write_block(struct ocmts *ocds, const void *data, size_t length)
{
	if (length > 256) {
		LOG_ERROR("Block write length must be up to 256 words");
		return ERROR_FAIL;
	}
	if (current_address & 0x3) {
		LOG_ERROR("Block write address must be 4-byte aligned");
		return ERROR_FAIL;
	}

	return tas_client_queue_block((void *)data, length, false);
}

static int tas_client_set_ojconf(struct ocmts *ocds, uint16_t ojconf)
{
	if (ojconf & 0x1)
		harr = (ojconf & 0x2) != 0;

	return ERROR_OK;
}

static const struct ocmts_ops ocmts_ops_interface = {
	.connect = tas_client_op_connect,
	.queue_io_set_address = tas_client_set_address,
	.queue_io_read_byte = tas_client_read_byte,
	.queue_io_read_hword = tas_client_read_hword,
	.queue_io_read_word = tas_client_read_word,
	.queue_io_read_block = tas_client_read_block,
	.queue_io_write_byte = tas_client_write_byte,
	.queue_io_write_hword = tas_client_write_hword,
	.queue_io_write_word = tas_client_write_word,
	.queue_io_write_block = tas_client_write_block,
	.run = tas_client_op_run,
	.queue_io_set_ojconf = tas_client_set_ojconf,
};

static int tas_client_speed(int speed)
{
	struct tas_client_mem_req mem_req = {
		.addr = TAS_AM15_RW_ACC_HW_FREQUENCY,
		.buffer = (uint32_t[]){speed * 1000},
		.length = 4,
		.is_read = false,
	};
	int err = tas_client_execute_mem_req(&tas_client, 15, &mem_req);
	if (err) {
		LOG_ERROR("Failed to set adapter speed.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int tas_client_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int tas_client_khz(int khz, int *speed)
{
	*speed = khz;
	return ERROR_OK;
}

static int tas_client_dap_init(void)
{
	return ERROR_OK;
}

static struct ifxdap_driver ifxdap_ops = {
	.init = tas_client_dap_init,
};

COMMAND_HANDLER(tas_client_cmd_host)
{
	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	free(server_host);
	server_host = strdup(CMD_ARGV[0]);
	if (CMD_ARGC == 2) {
		free(server_port);
		server_port = strdup(CMD_ARGV[1]);
	}
	if (!server_host || (CMD_ARGC == 2 && !server_port)) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const struct command_registration tas_client_subcommand_handlers[] = {{
																				 .name = "host",
																				 .handler = tas_client_cmd_host,
																				 .mode = COMMAND_CONFIG,
																				 .usage = "host [port]",
																				 .help = "set tas client host and port",
																			 },
																			 COMMAND_REGISTRATION_DONE};

static const struct command_registration tas_client_command_handlers[] = {{
																			  .name = "tas-client",
																			  .mode = COMMAND_ANY,
																			  .help = "perform tas client commands",
																			  .usage = "<cmd>",
																			  .chain = tas_client_subcommand_handlers,
																		  },
																		  COMMAND_REGISTRATION_DONE};

struct adapter_driver tas_client_adapter_driver = {
	.name = "tas_client",
	.commands = tas_client_command_handlers,
	/* Only the OCMTS operations are implemented; there is no JTAG queue to
	 * offer a jtag transport with. */
	.transport_ids = TRANSPORT_IFXDAP,
	.transport_preferred_id = TRANSPORT_IFXDAP,
	.ocmts_ops = &ocmts_ops_interface,
	.ifxdap_ops = &ifxdap_ops,
	.init = tas_client_init,
	.quit = tas_client_quit,
	.reset = tas_client_reset,
	/* .srst_asserted = tas_client_srst_asserted, */
	.speed = tas_client_speed,
	.speed_div = tas_client_speed_div,
	.khz = tas_client_khz,
};
