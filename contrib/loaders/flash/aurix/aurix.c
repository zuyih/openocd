// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Flash loader for Infineon AURIX program flash.
 *
 * Filling a flash page means writing every word to one address, so it is not a
 * block transfer and the debugger has to spend a request per word. Over TAS
 * that puts programming at roughly 3 KiB/s, almost all of it round trip cost.
 * Running the same sequence on the core instead reduces the debugger's job to
 * getting the data into RAM, which is a block transfer.
 *
 * The loader is entered with a pointer to struct params in %a4, runs to the
 * infinite loop at the end and is stopped there by a breakpoint. It calls
 * nothing and touches no stack, so no context save area has to be set up.
 */

#include <stdint.h>

/* Offsets within the command sequence window. */
#define CMD_SEQ_CTRL 0x5554
#define CMD_SEQ_ADDR 0xAA50
#define CMD_SEQ_CNT 0xAA58
#define CMD_SEQ_CMD 0xAAA8

#define CMD_CLEAR_STATUS 0xFA
#define CMD_ENTER_PAGE_MODE 0x50
#define CMD_WRITE_1 0xA0

/*
 * Drain the store buffer.
 *
 * A page is filled by storing every word to one address, and the core is free
 * to merge two consecutive 32 bit stores into one 64 bit store. The flash then
 * sees a load page of a width it was not told about and rejects the sequence,
 * which is why the debugger can fill a page with plain writes and the loader
 * cannot: a request per word over the debug link is never merged. TC3xx
 * requires a dsync after every store of a command sequence, and one before it
 * so that nothing is left over from earlier.
 */
static inline void dsync(void)
{
	__asm__ volatile ("dsync" ::: "memory");
}

struct params {
	uint32_t cmd_base;
	uint32_t status_reg;
	uint32_t error_reg;
	uint32_t error_mask;
	/* An operation is complete once all of done_mask is set, or, when
	 * done_mask is zero, once no bit of busy_mask is. */
	uint32_t done_mask;
	uint32_t busy_mask;
	/* Non-zero if the status has to be cleared once more between filling the
	 * page and the write command. Clearing at the start of the sequence is
	 * unconditional; the error flags are sticky, so a failure left unhandled
	 * would fail every command after it. */
	uint32_t clear_status_write;
	/* Page load register offsets. Both are the same where a page is filled
	 * through a single port. */
	uint32_t page_reg_lo;
	uint32_t page_reg_hi;
	/* Second opcode of the write command, burst or page. */
	uint32_t cmd_burst;
	uint32_t cmd_page;
	uint32_t burst_size;
	/* Destination, as the command interface wants to see it. */
	uint32_t address;
	/* Source in RAM, and how many bytes of it to program. */
	uint32_t data;
	uint32_t count;
	/* Zero on success, otherwise the error register. */
	uint32_t result;
};

void loader(struct params *p)
{
	volatile uint32_t *ctrl = (volatile uint32_t *)(p->cmd_base | CMD_SEQ_CTRL);
	volatile uint32_t *cmd_addr = (volatile uint32_t *)(p->cmd_base | CMD_SEQ_ADDR);
	volatile uint32_t *cmd_cnt = (volatile uint32_t *)(p->cmd_base | CMD_SEQ_CNT);
	volatile uint32_t *cmd = (volatile uint32_t *)(p->cmd_base | CMD_SEQ_CMD);
	volatile uint32_t *page_lo = (volatile uint32_t *)(p->cmd_base | p->page_reg_lo);
	volatile uint32_t *page_hi = (volatile uint32_t *)(p->cmd_base | p->page_reg_hi);
	volatile uint32_t *status = (volatile uint32_t *)p->status_reg;
	volatile uint32_t *error = (volatile uint32_t *)p->error_reg;

	const uint32_t *src = (const uint32_t *)p->data;
	uint32_t address = p->address;
	uint32_t count = p->count;

	/* An interrupt here would vector through a table that may well be in the
	 * flash being written. The caller restores ICR afterwards. */
	__asm__ volatile ("disable");

	while (count) {
		/* A burst covers an aligned group of pages, so it can only be
		 * used once the destination has reached a burst boundary; an
		 * unaligned one is refused with a sequence error. */
		uint32_t chunk = (count >= p->burst_size &&
				  (address & (p->burst_size - 1)) == 0)
					 ? p->burst_size : 32;
		uint32_t i;
		uint32_t st;

		dsync();

		*ctrl = CMD_CLEAR_STATUS;
		dsync();

		*ctrl = CMD_ENTER_PAGE_MODE;
		dsync();

		for (i = 0; i < chunk; i += 8) {
			*page_lo = *src++;
			dsync();
			*page_hi = *src++;
			dsync();
		}

		/* Entering page mode already reports done, so ask again for a
		 * verdict that belongs to the write alone. */
		if (p->clear_status_write) {
			*ctrl = CMD_CLEAR_STATUS;
			dsync();
		}

		*cmd_addr = address;
		dsync();
		*cmd_cnt = 0;
		dsync();
		*cmd = CMD_WRITE_1;
		dsync();
		*cmd = chunk == p->burst_size ? p->cmd_burst : p->cmd_page;
		dsync();

		for (;;) {
			st = *status;
			if (p->done_mask) {
				if ((st & p->done_mask) == p->done_mask)
					break;
			} else if (!(st & p->busy_mask)) {
				break;
			}
		}

		st = *error;
		if (st & p->error_mask) {
			p->result = st;
			goto out;
		}

		address += chunk;
		count -= chunk;
	}

	p->result = 0;

out:
	for (;;)
		;
}
