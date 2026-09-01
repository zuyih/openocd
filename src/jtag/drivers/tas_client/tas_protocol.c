#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "helper/log.h"
#include "tas_pkt.h"

/**
 * Receive exactly @a len bytes.
 *
 * recv() on a stream socket is free to return short, which happens routinely
 * once a response grows past a segment. Every reply below is a fixed layout,
 * so a short read desynchronises the stream for good.
 *
 * @returns ERROR_OK, or ERROR_FAIL on error or premature end of stream.
 */
static int tas_recv_all(int sock, void *buffer, size_t len) {
  uint8_t *p = buffer;

  while (len) {
    ssize_t got = recv(sock, p, len, 0);
    if (got <= 0)
      return ERROR_FAIL;
    p += got;
    len -= got;
  }

  return ERROR_OK;
}

/** Receive and discard exactly @a len bytes. */
static int tas_recv_discard(int sock, size_t len) {
  uint8_t buf[256];

  while (len) {
    size_t chunk = MIN(sizeof(buf), len);
    if (tas_recv_all(sock, buf, chunk) != ERROR_OK)
      return ERROR_FAIL;
    len -= chunk;
  }

  return ERROR_OK;
}

int tas_client_connect(int sock) {
  tas_pl1rq_server_connect_st rq_server_connect;
  tas_pl1rsp_server_connect_st rsp_server_connect;
  uint32_t packet_size;

  memset(&rq_server_connect, 0, sizeof(rq_server_connect));
  packet_size = 4 + sizeof(tas_pl1rq_server_connect_st);
  rq_server_connect.wl = sizeof(tas_pl1rq_server_connect_st) / 4 - 1;
  rq_server_connect.cmd = TAS_PL1_CMD_SERVER_CONNECT;
  rq_server_connect.reserved = 0;
  snprintf(rq_server_connect.client_name, TAS_NAME_LEN32, "openocd");
  getlogin_r(rq_server_connect.user_name, TAS_NAME_LEN16);
  rq_server_connect.client_pid = getpid();

  if (send(sock, &packet_size, 4, MSG_MORE) < 0) {
    return ERROR_FAIL;
  }
  if (send(sock, &rq_server_connect, sizeof(tas_pl1rq_server_connect_st), 0) <
      0) {
    return ERROR_FAIL;
  }

  if (tas_recv_all(sock, &packet_size, 4) != ERROR_OK) {
    return ERROR_FAIL;
  }
  if (tas_recv_all(sock, &rsp_server_connect,
                   sizeof(tas_pl1rsp_server_connect_st)) != ERROR_OK) {
    return ERROR_FAIL;
  }

  if (rsp_server_connect.cmd != TAS_PL1_CMD_SERVER_CONNECT ||
      rsp_server_connect.err != TAS_PL_ERR_NO_ERROR) {
    return ERROR_FAIL;
  }

  LOG_INFO("TAS server \"%.*s\" v%u.%u (%.*s)", TAS_NAME_LEN64,
           rsp_server_connect.server_info.server_name,
           rsp_server_connect.server_info.v_major,
           rsp_server_connect.server_info.v_minor,
           (int)sizeof(rsp_server_connect.server_info.date),
           rsp_server_connect.server_info.date);

  return 0;
}

int tas_client_session_start(int sock, const char *device,
                             const char *session_name, uint8_t *con_id,
                             tas_con_info_st *con_info) {
  tas_pl1rq_session_start_st rq_session_start;
  tas_pl1rsp_session_start_st rsp_session_start;
  uint32_t packet_size;

  memset(&rq_session_start, 0, sizeof(rq_session_start));
  packet_size = 4 + sizeof(tas_pl1rq_session_start_st);
  rq_session_start.wl = sizeof(tas_pl1rq_session_start_st) / 4 - 1;
  rq_session_start.cmd = TAS_PL1_CMD_SESSION_START;
  /* The connection id is handed out by the server, not picked by us. */
  rq_session_start.con_id = 0xFF;
  rq_session_start.client_type = TAS_CLIENT_TYPE_RW;
  strncpy(rq_session_start.identifier, device, TAS_NAME_LEN64 - 1);
  snprintf(rq_session_start.session_name, TAS_NAME_LEN16, "%s", session_name);
  /* A device carries a single named session; further clients have to present
   * the same name to join it, so the name must not vary per connection. */
  rq_session_start.session_pw[0] = 0;

  if (send(sock, &packet_size, 4, MSG_MORE) < 0) {
    return ERROR_FAIL;
  }
  if (send(sock, &rq_session_start, sizeof(tas_pl1rq_session_start_st), 0) <
      0) {
    return ERROR_FAIL;
  }

  if (tas_recv_all(sock, &packet_size, 4) != ERROR_OK) {
    return ERROR_FAIL;
  }
  if (tas_recv_all(sock, &rsp_session_start,
                   sizeof(tas_pl1rsp_session_start_st)) != ERROR_OK) {
    return ERROR_FAIL;
  }

  if (rsp_session_start.cmd != TAS_PL1_CMD_SESSION_START ||
      rsp_session_start.err != TAS_PL_ERR_NO_ERROR) {
    LOG_ERROR("TAS session \"%s\" rejected: cmd 0x%02x, err 0x%02x",
              session_name, rsp_session_start.cmd, rsp_session_start.err);
    return ERROR_FAIL;
  }

  if (rsp_session_start.num_instances > 0) {
    LOG_ERROR("TAS session start: identifier matches %u devices, "
              "it has to be unique",
              rsp_session_start.num_instances);
    return ERROR_FAIL;
  }
  *con_id = rsp_session_start.con_id;
  *con_info = rsp_session_start.con_info;

  return 0;
}

int tas_client_device_connect(int sock, tas_dev_con_feat_et dev_con_feat) {
  tas_pl1rq_device_connect_st rq_device_connect;
  tas_pl1rsp_device_connect_st rsp_device_connect;
  uint32_t packet_size;

  packet_size = 4 + sizeof(tas_pl1rq_device_connect_st);
  rq_device_connect.wl = sizeof(tas_pl1rq_device_connect_st) / 4 - 1;
  rq_device_connect.cmd = TAS_PL1_CMD_DEVICE_CONNECT;
  rq_device_connect.con_id = 0xFF;
  rq_device_connect.reserved = 0;
  rq_device_connect.option = dev_con_feat;
  rq_device_connect.reserved1 = 0;

  if (send(sock, &packet_size, 4, MSG_MORE) < 0) {
    return ERROR_FAIL;
  }
  if (send(sock, &rq_device_connect, sizeof(tas_pl1rq_device_connect_st), 0) <
      0) {
    return ERROR_FAIL;
  }

  if (tas_recv_all(sock, &packet_size, 4) != ERROR_OK) {
    return ERROR_FAIL;
  }
  if (tas_recv_all(sock, &rsp_device_connect,
                   sizeof(tas_pl1rsp_device_connect_st)) != ERROR_OK) {
    return ERROR_FAIL;
  }

  if (rsp_device_connect.cmd != TAS_PL1_CMD_DEVICE_CONNECT ||
      rsp_device_connect.err != TAS_PL_ERR_NO_ERROR) {
    return ERROR_FAIL;
  }

  if (rsp_device_connect.feat_used != dev_con_feat) {
    return ERROR_FAIL;
  }

  return 0;
}

int tas_client_get_targets(int sock, tas_target_info_st **targets,
                           size_t *target_num) {
  tas_pl1rq_get_targets_st rq_get_targets;
  tas_pl1rsp_get_targets_st rsp_get_targets;
  uint32_t packet_size;
  if (targets == NULL) {
    return ERROR_FAIL;
  }

  packet_size = 4 + sizeof(tas_pl1rq_get_targets_st);
  rq_get_targets.cmd = TAS_PL1_CMD_GET_TARGETS;
  rq_get_targets.wl = 0;
  rq_get_targets.start_index = 0;

  if (send(sock, &packet_size, 4, MSG_MORE) < 0) {
    return ERROR_FAIL;
  }
  if (send(sock, &rq_get_targets, sizeof(tas_pl1rq_get_targets_st), 0) < 0) {
    return ERROR_FAIL;
  }

  if (tas_recv_all(sock, &packet_size, 4) != ERROR_OK) {
    return ERROR_FAIL;
  }
  if (tas_recv_all(sock, &rsp_get_targets,
                   sizeof(tas_pl1rsp_get_targets_st)) != ERROR_OK) {
    return ERROR_FAIL;
  }

  if (rsp_get_targets.cmd != TAS_PL1_CMD_GET_TARGETS ||
      rsp_get_targets.err != TAS_PL_ERR_NO_ERROR) {
    return ERROR_FAIL;
  }

  *target_num = rsp_get_targets.num_target;
  /* Limit number of targets supported */
  if (*target_num > 32) {
    return ERROR_FAIL;
  }
  if (*target_num > 0) {
    *targets = calloc(*target_num, sizeof(tas_target_info_st));
    if (*targets == NULL) {
      return ERROR_FAIL;
    }
    if (tas_recv_all(sock, *targets,
                     *target_num * sizeof(tas_target_info_st)) != ERROR_OK) {
      return ERROR_FAIL;
    }
  }

  return ERROR_OK;
}

enum {
  PROTOC_VER = 0 //!< \brief TasPkt protocol version implemented in this class
};

static uint16_t pl1_count = 0;

struct tas_client_pl0_req {
  uint32_t addr;
  uint8_t *buffer;
  uint8_t cmd;
};

int tas_client_send_pl0(int sock, uint8_t con_id, uint32_t *pl0_buffer,
                        size_t pl0_len, size_t pl0_capacity,
                        size_t pl0_elements) {

  uint32_t packet_size = 4 + sizeof(tas_pl1rq_pl0_start_st) +
                         sizeof(tas_pl1rq_pl0_end_st) + pl0_len;
  tas_pl1rq_pl0_start_st rq_start = {
      .cmd = TAS_PL1_CMD_PL0_START,
      .wl = 0,
      .con_id = con_id,
      .pl0_addr_map_mask = 1,
      .pl1_cnt = pl1_count++,
      .protoc_ver = PROTOC_VER,

  };
  tas_pl1rq_pl0_end_st rq_end = {
      .wl = 0, .cmd = TAS_PL1_CMD_PL0_END, .num_pl0_rw = pl0_elements};
  tas_pl1rsp_pl0_start_st rsp_start;
  tas_pl1rsp_pl0_end_st rsp_end;

  if (send(sock, &packet_size, 4, MSG_MORE) < 0) {
    return ERROR_FAIL;
  }
  if (send(sock, &rq_start, sizeof(tas_pl1rq_pl0_start_st), MSG_MORE) < 0) {
    return ERROR_FAIL;
  }
  if (send(sock, pl0_buffer, pl0_len, MSG_MORE) < 0) {
    return ERROR_FAIL;
  }
  if (send(sock, &rq_end, sizeof(tas_pl1rq_pl0_end_st), 0) < 0) {
    return ERROR_FAIL;
  }

  if (tas_recv_all(sock, &packet_size, 4) != ERROR_OK) {
    return ERROR_FAIL;
  }
  if (tas_recv_all(sock, &rsp_start, sizeof(tas_pl1rsp_pl0_start_st)) != ERROR_OK) {
    return ERROR_FAIL;
  }

  if (rsp_start.cmd != TAS_PL1_CMD_PL0_START ||
      (rsp_start.err != TAS_PL_ERR_NO_ERROR &&
       rsp_start.err != TAS_PL_ERR_PROTOCOL)) {
    tas_recv_discard(sock, packet_size - 4 - sizeof(tas_pl1rsp_pl0_start_st));
    return ERROR_FAIL;
  }

  pl0_len = packet_size - 4 - sizeof(tas_pl1rsp_pl0_start_st) -
            sizeof(tas_pl1rsp_pl0_end_st);

  /* Keep whatever fits in the caller's buffer, drain the rest so that the
   * stream stays in sync even if the server answered with more than we asked
   * for. */
  size_t keep = MIN(pl0_len, pl0_capacity);
  if (tas_recv_all(sock, pl0_buffer, keep) != ERROR_OK) {
    return ERROR_FAIL;
  }
  if (tas_recv_discard(sock, pl0_len - keep) != ERROR_OK) {
    return ERROR_FAIL;
  }
  if (keep != pl0_len) {
    LOG_ERROR("TAS PL0 response of %zu bytes exceeds the %zu byte buffer",
              pl0_len, pl0_capacity);
    return ERROR_FAIL;
  }

  if (tas_recv_all(sock, &rsp_end, sizeof(tas_pl1rsp_pl0_end_st)) != ERROR_OK) {
    return ERROR_FAIL;
  }
  if (rsp_end.cmd != TAS_PL1_CMD_PL0_END ||
      rsp_end.pl1_cnt != rq_start.pl1_cnt) {
    return ERROR_FAIL;
  }

  return ERROR_OK;
}

/* Address map 15 holds the settings of the access hardware. */
#define TAS_ADDR_MAP_AM15 15

int tas_client_write_am15_u32(int sock, uint8_t con_id, uint32_t addr,
                              uint32_t value) {
  tas_pl1rq_pl0_start_st rq_start = {
      .wl = 1,
      .cmd = TAS_PL1_CMD_PL0_START,
      .con_id = con_id,
      .pl0_addr_map_mask = 1 << TAS_ADDR_MAP_AM15,
      .pl1_cnt = pl1_count++,
      .protoc_ver = PROTOC_VER,
  };
  tas_pl0rq_addr_map_st rq_map = {
      .wl = 0, .cmd = TAS_PL0_CMD_ADDR_MAP, .addr_map = TAS_ADDR_MAP_AM15};
  tas_pl0rq_base_addr32_st rq_base = {
      .wl = 0, .cmd = TAS_PL0_CMD_BASE_ADDR32, .ba31to16 = addr >> 16};
  tas_pl0rq_wr_st rq_wr = {.wl = 1,
                           .cmd = TAS_PL0_CMD_WR32,
                           .a15to0 = addr & 0xFFFF,
                           .data = value};
  tas_pl1rq_pl0_end_st rq_end = {
      .wl = 0, .cmd = TAS_PL1_CMD_PL0_END, .num_pl0_rw = 1};
  tas_pl1rsp_pl0_start_st rsp_start;
  tas_pl0rsp_wr_st rsp_wr;
  tas_pl1rsp_pl0_end_st rsp_end;
  uint8_t buf[4 + sizeof(rq_start) + sizeof(rq_map) + sizeof(rq_base) +
              sizeof(rq_wr) + sizeof(rq_end)];
  uint32_t packet_size = sizeof(buf);
  size_t off = 0;

  memcpy(buf + off, &packet_size, 4);
  off += 4;
  memcpy(buf + off, &rq_start, sizeof(rq_start));
  off += sizeof(rq_start);
  memcpy(buf + off, &rq_map, sizeof(rq_map));
  off += sizeof(rq_map);
  memcpy(buf + off, &rq_base, sizeof(rq_base));
  off += sizeof(rq_base);
  memcpy(buf + off, &rq_wr, sizeof(rq_wr));
  off += sizeof(rq_wr);
  memcpy(buf + off, &rq_end, sizeof(rq_end));

  if (send(sock, buf, sizeof(buf), 0) < 0)
    return ERROR_FAIL;

  if (tas_recv_all(sock, &packet_size, 4) != ERROR_OK)
    return ERROR_FAIL;
  if (packet_size != 4 + sizeof(rsp_start) + sizeof(rsp_wr) + sizeof(rsp_end)) {
    /* Keep the stream in step before giving up. */
    if (packet_size > 4)
      tas_recv_discard(sock, packet_size - 4);
    return ERROR_FAIL;
  }
  if (tas_recv_all(sock, &rsp_start, sizeof(rsp_start)) != ERROR_OK ||
      tas_recv_all(sock, &rsp_wr, sizeof(rsp_wr)) != ERROR_OK ||
      tas_recv_all(sock, &rsp_end, sizeof(rsp_end)) != ERROR_OK)
    return ERROR_FAIL;

  if (rsp_start.cmd != TAS_PL1_CMD_PL0_START ||
      rsp_start.err != TAS_PL_ERR_NO_ERROR || rsp_wr.cmd != TAS_PL0_CMD_WR32 ||
      rsp_wr.err != TAS_PL0_ERR_NO_ERROR || rsp_end.cmd != TAS_PL1_CMD_PL0_END ||
      rsp_end.pl1_cnt != rq_start.pl1_cnt)
    return ERROR_FAIL;

  return ERROR_OK;
}