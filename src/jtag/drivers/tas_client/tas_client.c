#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>

#include "helper/command.h"
#include "helper/list.h"
#include "helper/log.h"
#include "jtag/drivers/tas_client/tas_pkt.h"
#include "jtag/jtag.h"
#include "jtag/tas.h"
#include "server/server.h"
#include <jtag/interface.h>

#include "target/aurix/aurix_ocds.h"
#include "target/target.h"
#include "tas_protocol.h"

struct tas_client_pl0_req {
  uint32_t addr;
  uint8_t cmd;
  uint32_t count;
  union {
    void *buffer;
    uint64_t data;
  };
};
struct tas_client_con_queue {
  uint32_t max_pkt_size;
  /* Largest block transfer, in 32-bit words, that fits a single packet. */
  uint32_t max_block_words;
  uint32_t reqs_count;
  uint32_t reqs_size;
  struct tas_client_pl0_req *reqs;
};
struct tas_client_state {
  int sock;
  bool connected;
  const char *ip_addr;
  tas_target_info_st *targets;
  /* Targets already bound to an OCDS; a device carries one session and the
   * server we test against aborts when a second one is opened on it. */
  bool target_taken[32];
  size_t target_num;
  struct tas_client_con_queue con_queues[32];
};

static struct tas_client_state client_state;

/** How the access hardware is wired to the device. */
static const char *tas_client_phys_name(uint8_t phys) {
  switch (phys) {
  case TAS_DEV_CON_PHYS_JTAG:
    return "JTAG";
  case TAS_DEV_CON_PHYS_DAP:
  case TAS_DEV_CON_PHYS_DAP_DAP:
    return "DAP";
  case TAS_DEV_CON_PHYS_DAP_SPD:
    return "DAP/SPD";
  case TAS_DEV_CON_PHYS_DAP_DXCPL:
    return "DAP/DXCPL";
  case TAS_DEV_CON_PHYS_DAP_DXCM:
    return "DAP/DXCM";
  case TAS_DEV_CON_PHYS_SWD:
    return "SWD";
  case TAS_DEV_CON_PHYS_ETH:
    return "Ethernet";
  default:
    return "unknown";
  }
}

/* Sessions on a device are keyed by name; keep it fixed so that a second
 * connection can join the session an earlier one opened. */
#define TAS_SESSION_NAME "openocd" 

static int tas_client_init(void) {
  struct sockaddr_in ipv4_sock_addr;
  const char *host = client_state.ip_addr ? client_state.ip_addr : "127.0.0.1";

  if (inet_aton(host, &ipv4_sock_addr.sin_addr) == 0) {
    LOG_ERROR("Invalid ip addr: %s", host);
    return ERROR_INVALID_NUMBER;
  }
  ipv4_sock_addr.sin_family = AF_INET;
  ipv4_sock_addr.sin_port = htons(24817);

  LOG_INFO("Connecting to TAS server %s:%u", host, 24817);
  client_state.sock = socket(AF_INET, SOCK_STREAM, 0);
  if (client_state.sock == -1) {
    return ERROR_FAIL;
  }

  if (connect(client_state.sock, (struct sockaddr *)&ipv4_sock_addr,
              sizeof(struct sockaddr_in))) {
    LOG_ERROR("Failed to connect to tas server %s: %s", host,
              strerror(errno));
    return ERROR_CONNECTION_REJECTED;
  }

  if (tas_client_connect(client_state.sock) != 0) {
    LOG_ERROR("Failed to connect to TAS server");
    return ERROR_CONNECTION_REJECTED;
  }
  client_state.connected = true;

  if (tas_client_get_targets(client_state.sock, &client_state.targets,
                             &client_state.target_num) != 0) {
    LOG_ERROR("Failed to receive targets");
    return ERROR_FAIL;
  }

  if (client_state.target_num == 0) {
    LOG_WARNING("TAS server reports no targets; is a device connected?");
  } else {
    LOG_INFO("TAS server reports %zu target(s):", client_state.target_num);
    for (size_t i = 0; i < client_state.target_num; i++)
      LOG_INFO("  %-40.64s device type 0x%08" PRIx32 ", over %s, %u client(s)",
               client_state.targets[i].identifier,
               client_state.targets[i].device_type,
               tas_client_phys_name(client_state.targets[i].dev_con_phys),
               client_state.targets[i].num_client);
  }

  return ERROR_OK;
}

static int tas_client_quit(void) {
  /* There is no session-end command in PL1; a session is torn down by closing
   * the connection. Shut it down explicitly so the server sees the EOF rather
   * than just having the descriptor disappear. */
  if (client_state.connected) {
    shutdown(client_state.sock, SHUT_RDWR);
    client_state.connected = false;
  }
  close(client_state.sock);

  for (size_t i = 0; i < ARRAY_SIZE(client_state.con_queues); i++) {
    free(client_state.con_queues[i].reqs);
    client_state.con_queues[i].reqs = NULL;
  }
  free(client_state.targets);
  client_state.targets = NULL;
  client_state.target_num = 0;

  free((void *)client_state.ip_addr);
  client_state.ip_addr = NULL;

  return ERROR_OK;
}

static int tas_client_reset(int trst, int srst) {
  if (trst != 0) {
    return ERROR_NOT_IMPLEMENTED;
  }
  if (!client_state.connected) {
    return ERROR_FAIL;
  }
  if (srst) {
    return tas_client_device_connect(client_state.sock,
                                     TAS_DEV_CON_FEAT_RESET_AND_HALT);
  }

  return 0;
}

static int tas_client_op_run(struct aurix_ocds *ocds) {

  uint32_t base_address = 0xFFFFFFFF;
  uint32_t i;
  uint32_t pl0_buffer[client_state.con_queues[ocds->con_id].max_pkt_size / 4];
  size_t pl0_size = 0;

  for (i = 0; i < client_state.con_queues[ocds->con_id].reqs_count; i++) {
    struct tas_client_pl0_req *req =
        &client_state.con_queues[ocds->con_id].reqs[i];

    if ((req->addr & 0xFFFF0000) != base_address) {
      tas_pl0rq_base_addr32_st base_addr = {
          .wl = 0,
          .cmd = TAS_PL0_CMD_BASE_ADDR32,
          .ba31to16 = req->addr >> 16,
      };
      memcpy(pl0_buffer + pl0_size / sizeof(uint32_t), &base_addr,
             sizeof(tas_pl0rq_base_addr32_st));
      pl0_size += sizeof(tas_pl0rq_base_addr32_st);
      base_address = req->addr & 0xFFFF0000;
    }

    if ((req->cmd & 0x1) == 1 || req->cmd == TAS_PL0_CMD_RDBLK) {
      if (req->cmd < TAS_PL0_CMD_RDBLK) {
        tas_pl0rq_rd_st read_addr = {
            .wl = 0,
            .cmd = req->cmd,
            .a15to0 = req->addr & 0xFFFF,
        };
        memcpy(pl0_buffer + pl0_size / sizeof(uint32_t), &read_addr,
               sizeof(tas_pl0rq_rd_st));
        pl0_size += sizeof(tas_pl0rq_rd_st);
      } else {
        tas_pl0rq_rdblk_st read_addr = {
            .wl = 1,
            .cmd = req->cmd,
            .wlrd = req->count,
            .a15to0 = req->addr & 0xFFFF,
        };
        memcpy(pl0_buffer + pl0_size / sizeof(uint32_t), &read_addr,
               sizeof(tas_pl0rq_rdblk_st));
        pl0_size += sizeof(tas_pl0rq_rdblk_st);
      }
    } else {
      if (req->cmd == TAS_PL0_CMD_WR64) {
        tas_pl0rq_wr64_st write_addr = {
            .wl = 2,
            .cmd = req->cmd,
            .a15to0 = req->addr & 0xFFFF,
            .data = {req->data & 0xFFFFFFFF, req->data >> 32},
        };
        memcpy(pl0_buffer + pl0_size / sizeof(uint32_t), &write_addr,
               sizeof(tas_pl0rq_wr64_st));
        pl0_size += sizeof(tas_pl0rq_wr64_st);
      } else if (req->cmd < TAS_PL0_CMD_WRBLK) {
        tas_pl0rq_wr_st write_addr = {
            .wl = 1,
            .cmd = req->cmd,
            .a15to0 = req->addr & 0xFFFF,
            .data = req->data,
        };
        memcpy(pl0_buffer + pl0_size / sizeof(uint32_t), &write_addr,
               sizeof(tas_pl0rq_wr_st));
        pl0_size += sizeof(tas_pl0rq_wr_st);
      } else {
        tas_pl0rq_wrblk_st write_addr = {
            .wl = req->count,
            .cmd = req->cmd,
            .a15to0 = req->addr & 0xFFFF,
        };
        memcpy(pl0_buffer + pl0_size / sizeof(uint32_t), &write_addr,
               sizeof(tas_pl0rq_wrblk_st));
        pl0_size += sizeof(tas_pl0rq_wrblk_st);
        memcpy(pl0_buffer + pl0_size / sizeof(uint32_t), req->buffer,
               req->count * sizeof(uint32_t));
        pl0_size += req->count * 4;
      }
    }
  }

  int err =
      tas_client_send_pl0(client_state.sock, ocds->con_id, pl0_buffer, pl0_size,
                          client_state.con_queues[ocds->con_id].max_pkt_size,
                          client_state.con_queues[ocds->con_id].reqs_count);
  if (err) {
    client_state.con_queues[ocds->con_id].reqs_count = 0;
    return err;
  }

  size_t pl0_offset = 0;
  for (i = 0; i < client_state.con_queues[ocds->con_id].reqs_count; i++) {
    struct tas_client_pl0_req *req =
        &client_state.con_queues[ocds->con_id].reqs[i];
    tas_pl0rsp_rd_st rsp_rd;
    tas_pl0rsp_wr_st rsp_wr;

    switch (req->cmd) {
    case TAS_PL0_CMD_RD8:
    case TAS_PL0_CMD_RD16:
    case TAS_PL0_CMD_RD32:
    case TAS_PL0_CMD_RD64:
    case TAS_PL0_CMD_RDBLK:
    case TAS_PL0_CMD_RDBLK1KB:
      memcpy(&rsp_rd, pl0_buffer + pl0_offset, sizeof(tas_pl0rsp_rd_st));
      pl0_offset++;
      /* wlrd counts words, so a 64 bit read answers with two per access. */
      if (rsp_rd.cmd != req->cmd || rsp_rd.err != TAS_PL0_ERR_NO_ERROR ||
          rsp_rd.wlrd != req->count *
                             (req->cmd == TAS_PL0_CMD_RD64 ? 2u : 1u)) {
        client_state.con_queues[ocds->con_id].reqs_count = 0;
        return ERROR_FAIL;
      }
      uint32_t size = req->cmd == TAS_PL0_CMD_RD8    ? 1
                      : req->cmd == TAS_PL0_CMD_RD16 ? 2
                      : req->cmd == TAS_PL0_CMD_RD64 ? 8
                                                     : 4;
      memcpy(req->buffer, pl0_buffer + pl0_offset, req->count * size);
      /* The payload is padded out to a whole number of words. */
      pl0_offset += (req->count * size + 3) / 4;
      break;
    case TAS_PL0_CMD_WR8:
    case TAS_PL0_CMD_WR16:
    case TAS_PL0_CMD_WR32:
    case TAS_PL0_CMD_WR64:
    case TAS_PL0_CMD_WRBLK:
      memcpy(&rsp_wr, pl0_buffer + pl0_offset, sizeof(tas_pl0rsp_wr_st));
      pl0_offset++;
      if (rsp_wr.cmd != req->cmd || rsp_wr.err != TAS_PL0_ERR_NO_ERROR ||
          rsp_wr.wlwr != (req->count + (req->cmd == TAS_PL0_CMD_WR64 ? 1 : 0))) {
        client_state.con_queues[ocds->con_id].reqs_count = 0;
        return ERROR_FAIL;
      }
      break;
    }
  }

  client_state.con_queues[ocds->con_id].reqs_count = 0;
  return ERROR_OK;
}

static int tas_client_op_queue_soc_read(struct aurix_ocds *ocds, uint32_t addr,
                                        uint32_t size, uint32_t count,
                                        void *buffer) {
  struct tas_client_con_queue *q;
  uint8_t *dst = buffer;

  if (ocds->con_id >= ARRAY_SIZE(client_state.con_queues) ||
      !client_state.con_queues[ocds->con_id].reqs) {
    return ERROR_FAIL;
  }
  q = &client_state.con_queues[ocds->con_id];

  if (size == 4 && count > 1) {
    while (count) {
      uint32_t chunk = MIN(q->max_block_words, count);

      /* A block transfer fills a packet on its own, so flush what is queued. */
      if (q->reqs_count > 0) {
        int ret = tas_client_op_run(ocds);
        if (ret) {
          return ret;
        }
      }

      q->reqs[q->reqs_count++] = (struct tas_client_pl0_req){
          .addr = addr, .count = chunk, .cmd = TAS_PL0_CMD_RDBLK, .buffer = dst};

      addr += chunk * 4;
      dst += chunk * 4;
      count -= chunk;
    }
  } else {
    for (uint32_t i = 0; i < count; i++) {
      if (q->reqs_count >= q->reqs_size) {
        int ret = tas_client_op_run(ocds);
        if (ret) {
          return ret;
        }
      }

      q->reqs[q->reqs_count++] = (struct tas_client_pl0_req){
          .addr = addr,
          .count = 1,
          .cmd = size == 8   ? TAS_PL0_CMD_RD64
                 : size == 4 ? TAS_PL0_CMD_RD32
                 : size == 2 ? TAS_PL0_CMD_RD16
                             : TAS_PL0_CMD_RD8,
          .buffer = dst};

      addr += size;
      dst += size;
    }
  }
  return ERROR_OK;
}

static int tas_client_op_queue_soc_write(struct aurix_ocds *ocds, uint32_t addr,
                                         uint32_t size, uint32_t count,
                                         const void *buffer) {
  struct tas_client_con_queue *q;
  const uint8_t *src = buffer;

  if (ocds->con_id >= ARRAY_SIZE(client_state.con_queues) ||
      !client_state.con_queues[ocds->con_id].reqs) {
    return ERROR_FAIL;
  }
  q = &client_state.con_queues[ocds->con_id];

  if (size == 4 && count > 1) {
    while (count) {
      uint32_t chunk = MIN(q->max_block_words, count);

      /* A block transfer fills a packet on its own, so flush what is queued. */
      if (q->reqs_count > 0) {
        int ret = tas_client_op_run(ocds);
        if (ret) {
          return ret;
        }
      }

      q->reqs[q->reqs_count++] = (struct tas_client_pl0_req){
          .addr = addr,
          .count = chunk,
          .cmd = TAS_PL0_CMD_WRBLK,
          .buffer = (void *)src};

      addr += chunk * 4;
      src += chunk * 4;
      count -= chunk;
    }
  } else {
    for (uint32_t i = 0; i < count; i++) {
      uint64_t data = 0;

      if (q->reqs_count >= q->reqs_size) {
        int ret = tas_client_op_run(ocds);
        if (ret) {
          return ret;
        }
      }

      memcpy(&data, src, size);
      q->reqs[q->reqs_count++] = (struct tas_client_pl0_req){
          .addr = addr,
          .count = 1,
          .cmd = size == 8   ? TAS_PL0_CMD_WR64
                 : size == 4 ? TAS_PL0_CMD_WR32
                 : size == 2 ? TAS_PL0_CMD_WR16
                             : TAS_PL0_CMD_WR8,
          .data = data};

      addr += size;
      src += size;
    }
  }

  return ERROR_OK;
}

static int tas_client_op_connect(struct aurix_ocds *ocds) {
  uint32_t i, j;
  tas_target_info_st *target = NULL;
  uint32_t target_index = 0;
  tas_con_info_st con_info;
  unsigned int matches = 0;

  /*
   * A target qualifies when the device type the server reports is one of the
   * IDs declared for the TAP. With several devices of the same type attached
   * that is ambiguous, so an OCDS may narrow it down with -device, matched as
   * a substring of the target identifier.
   */
  for (i = 0; i < client_state.target_num; i++) {
    for (j = 0; j < ocds->tap->expected_ids_cnt; j++) {
      if (client_state.targets[i].device_type != ocds->tap->expected_ids[j])
        continue;
      if (ocds->device_id &&
          !strstr(client_state.targets[i].identifier, ocds->device_id))
        continue;

      matches++;
      /* Leave a device that another OCDS already claimed alone, so that
       * several OCDS without -device spread over the attached devices. */
      if (client_state.target_taken[i])
        break;
      if (!target) {
        target = &client_state.targets[i];
        target_index = i;
      }
      break;
    }
  }

  if (!target && matches > 0) {
    LOG_ERROR("OCDS %s: every matching device is already claimed by another "
              "OCDS. One OCDS covers all cores of a device; use -device to "
              "bind each OCDS to a different one.",
              ocds->name);
    return ERROR_COMMAND_ARGUMENT_INVALID;
  }

  if (!target) {
    if (ocds->device_id)
      LOG_ERROR("OCDS %s: no TAS target matches -device \"%s\" with one of the "
                "%u expected TAP ID(s) of %s",
                ocds->name, ocds->device_id, ocds->tap->expected_ids_cnt,
                ocds->tap->dotted_name);
    else
      LOG_ERROR("OCDS %s: the TAS server offers no device whose type matches "
                "any of the %u expected TAP ID(s) declared for %s",
                ocds->name, ocds->tap->expected_ids_cnt,
                ocds->tap->dotted_name);
    for (j = 0; j < ocds->tap->expected_ids_cnt; j++)
      LOG_ERROR("  expected device type 0x%08" PRIx32,
                ocds->tap->expected_ids[j]);
    return ERROR_COMMAND_ARGUMENT_INVALID;
  }

  if (matches > 1) {
    LOG_WARNING("OCDS %s: %u attached devices match, using \"%s\". Narrow it "
                "down with: ocds create %s ... -device <identifier>",
                ocds->name, matches, target->identifier, ocds->name);
    for (i = 0; i < client_state.target_num; i++)
      LOG_WARNING("  %s", client_state.targets[i].identifier);
  }

  ocds->device_type = target->device_type;

  int err = tas_client_session_start(client_state.sock, target->identifier,
                                     TAS_SESSION_NAME, &ocds->con_id, &con_info);
  if (err) {
    LOG_ERROR("Failed to start session for target %s", target->identifier);
    return ERROR_FAIL;
  }

  if (ocds->con_id >= ARRAY_SIZE(client_state.con_queues)) {
    LOG_ERROR("TAS server assigned connection id %u, only %zu are supported",
              ocds->con_id, ARRAY_SIZE(client_state.con_queues));
    return ERROR_FAIL;
  }
  LOG_DEBUG("OCDS %s: TAS session on connection %u", ocds->name, ocds->con_id);
  client_state.target_taken[target_index] = true;

  if (client_state.con_queues[ocds->con_id].reqs) {
    free(client_state.con_queues[ocds->con_id].reqs);
  }
  struct tas_client_con_queue *q = &client_state.con_queues[ocds->con_id];

  LOG_DEBUG("OCDS %s: pl0_max_num_rw %u, rw_mode_mask 0x%04x, "
            "max_pl2rq_pkt_size %" PRIu32 ", max_pl2rsp_pkt_size %" PRIu32,
            ocds->name, con_info.pl0_max_num_rw, con_info.pl0_rw_mode_mask,
            con_info.max_pl2rq_pkt_size, con_info.max_pl2rsp_pkt_size);

  q->max_pkt_size = con_info.max_pl2rq_pkt_size - 4 -
                    sizeof(tas_pl1rq_pl0_start_st) -
                    sizeof(tas_pl1rq_pl0_end_st);

  /*
   * A block transfer occupies a packet on its own: a base address record plus
   * a block header, and the payload has to fit whichever direction is tighter.
   * The response is received back into the request buffer, so it is bounded by
   * max_pkt_size as well.
   */
  size_t rsp_budget = con_info.max_pl2rsp_pkt_size - 4 -
                      sizeof(tas_pl1rsp_pl0_start_st) -
                      sizeof(tas_pl1rsp_pl0_end_st);
  size_t block_budget = MIN(q->max_pkt_size, rsp_budget);
  size_t block_overhead =
      sizeof(tas_pl0rq_base_addr32_st) + sizeof(tas_pl0rq_rdblk_st);

  q->max_block_words =
      block_budget > block_overhead ? (block_budget - block_overhead) / 4 : 0;
  if (q->max_block_words == 0) {
    LOG_ERROR("TAS server negotiated a packet size too small for block access");
    return ERROR_FAIL;
  }
  /*
   * tas_client_op_run() serialises every queued request into a stack buffer of
   * max_pkt_size bytes without re-checking the bound, so the queue must not be
   * able to hold more requests than fit. The largest a single request can get
   * is a base address record (4 bytes) plus a 64-bit write (12 bytes).
   */
  q->reqs_size =
      MIN(MIN(256, con_info.pl0_max_num_rw), q->max_pkt_size / 16);
  if (q->reqs_size == 0) {
    LOG_ERROR("TAS server reports an unusably small packet size (%" PRIu32 ")",
              client_state.con_queues[ocds->con_id].max_pkt_size);
    return ERROR_FAIL;
  }
  client_state.con_queues[ocds->con_id].reqs =
      malloc(sizeof(struct tas_client_pl0_req) *
             client_state.con_queues[ocds->con_id].reqs_size);
  client_state.con_queues[ocds->con_id].reqs_count = 0;
  if (!client_state.con_queues[ocds->con_id].reqs) {
    return ERROR_FAIL;
  }

  enum reset_types jtag_reset_config = jtag_get_reset_config();

  if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
    err = tas_client_device_connect(client_state.sock,
                                    TAS_DEV_CON_FEAT_RESET_AND_HALT);
  } else {
    err = tas_client_device_connect(client_state.sock, TAS_DEV_CON_FEAT_NONE);
  }
  if (err) {
    LOG_ERROR("Failed to connect to device %s", target->identifier);
    return ERROR_FAIL;
  }

  return ERROR_OK;
}

static const struct aurix_ocds_ops tas_ops_interface = {
    .connect = tas_client_op_connect,
    .queue_soc_read = tas_client_op_queue_soc_read,
    .queue_soc_write = tas_client_op_queue_soc_write,
    .run = tas_client_op_run,
};

COMMAND_HANDLER(tas_client_handle_host_command) {
  if (CMD_ARGC != 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct in_addr addr;
  if (inet_aton(CMD_ARGV[0], &addr) == 0) {
    command_print(CMD, "invalid TAS server address: %s", CMD_ARGV[0]);
    return ERROR_COMMAND_ARGUMENT_INVALID;
  }

  free((void *)client_state.ip_addr);
  client_state.ip_addr = strdup(CMD_ARGV[0]);
  if (!client_state.ip_addr) {
    LOG_ERROR("Out of memory");
    return ERROR_FAIL;
  }

  return ERROR_OK;
}

static const struct command_registration tas_client_subcommand_handlers[] = {
    {
        .name = "host",
        .handler = tas_client_handle_host_command,
        .mode = COMMAND_CONFIG,
        .help = "Set the IPv4 address of the TAS/DAS server to connect to "
                "(default 127.0.0.1). Use this to reach a DAS server running "
                "on another machine.",
        .usage = "ipv4_address",
    },
    COMMAND_REGISTRATION_DONE};

static const struct command_registration tas_client_command_handlers[] = {
    {
        .name = "tas_client",
        .mode = COMMAND_ANY,
        .help = "perform tas_client management",
        .usage = "",
        .chain = tas_client_subcommand_handlers,
    },
    COMMAND_REGISTRATION_DONE};

struct adapter_driver tas_client_adapter_driver = {
    .name = "tas_client",
    .transport_ids = TRANSPORT_TAS,
    .transport_preferred_id = TRANSPORT_TAS,
    .commands = tas_client_command_handlers,
    .tas_ops = &tas_ops_interface,
    .init = tas_client_init,
    .quit = tas_client_quit,
    .reset = tas_client_reset,
};