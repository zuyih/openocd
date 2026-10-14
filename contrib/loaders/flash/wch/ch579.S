/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Word programming of the flash of the WCH CH579 (Cortex-M0),
 * fed from the asynchronous FIFO of target_run_flash_async_algorithm().
 *
 * A flash command completes before the next access to the flash registers,
 * the status tells the result: success is address valid without timeout and
 * without command error.
 */

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

	/* Params:
	 * r0 - flash register base (in), status (out)
	 * r1 - count (words)
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - tmp
	 */

#define FLASH_DATA		0x00
#define FLASH_ADDR		0x04
#define FLASH_COMMAND		0x08
#define FLASH_STATUS		0x0a

#define CMD_PROG		0x9a

/* timeout | command error | address valid */
#define STATUS_MASK		0x43
#define STATUS_OK		0x40

	.thumb_func
	.global _start
_start:
wait_fifo:
	ldr	r6, [r2, #0]		/* read wp */
	cmp	r6, #0			/* abort if wp == 0 */
	beq	exit
	ldr	r5, [r2, #4]		/* read rp */
	cmp	r5, r6			/* wait until rp != wp */
	beq	wait_fifo
	ldr	r6, [r5]		/* program one word */
	str	r4, [r0, #FLASH_ADDR]
	str	r6, [r0, #FLASH_DATA]
	movs	r6, #CMD_PROG
	strb	r6, [r0, #FLASH_COMMAND]
	ldrb	r6, [r0, #FLASH_STATUS]	/* check the result */
	movs	r7, #STATUS_MASK
	ands	r7, r6
	cmp	r7, #STATUS_OK
	bne	error
	adds	r5, #4
	adds	r4, #4
	cmp	r5, r3			/* wrap rp at end of buffer */
	bcc	no_wrap
	mov	r5, r2
	adds	r5, #8
no_wrap:
	str	r5, [r2, #4]		/* store rp */
	subs	r1, r1, #1		/* decrement word count */
	bne	wait_fifo
	b	exit
error:
	movs	r5, #0
	str	r5, [r2, #4]		/* set rp = 0 on error */
exit:
	mov	r0, r6			/* return status in r0 */
	bkpt	#0
