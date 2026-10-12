/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Fast page programming of the WCH CH32F10x flash (128 byte pages, loaded
 * into the page buffer 16 bytes at a time).
 *
 * The flash register sequence is the one of the flash loader of the WCH
 * OpenOCD, fed from the asynchronous FIFO of target_run_flash_async_algorithm().
 * The FIFO size is a multiple of the page size, so a page never wraps.
 */

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

	/* Params:
	 * r0 - flash register base (in), status (out)
	 * r1 - count (pages)
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address, page aligned
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - tmp
	 */

#define FLASH_SR		0x0c
#define FLASH_CR		0x10
#define FLASH_AR		0x14

#define SR_BSY			0x01
#define SR_PGERR		0x04
#define SR_WRPRTERR		0x10

#define CR_STRT			0x40
#define CR_PAGE_PG_SHIFT	16
#define CR_BUF_LOAD_SHIFT	18
#define CR_BUF_RST_SHIFT	19
/* CR_PAGE_PG | CR_BUF_LOAD | CR_BUF_RST */
#define CR_PAGE_BITS		0x0d
#define CR_PAGE_BITS_SHIFT	16

#define PAGE_SIZE		128
#define LOAD_SIZE		16

	.thumb_func
	.global _start
_start:
	movs	r7, #1
	lsls	r7, r7, #CR_PAGE_PG_SHIFT
	ldr	r6, [r0, #FLASH_CR]	/* enable fast page programming */
	orrs	r6, r7
	str	r6, [r0, #FLASH_CR]
wait_fifo:
	ldr	r6, [r2, #0]		/* read wp */
	cmp	r6, #0			/* abort if wp == 0 */
	beq	exit
	ldr	r5, [r2, #4]		/* read rp */
	cmp	r5, r6			/* wait until rp != wp */
	beq	wait_fifo
	movs	r7, #1
	lsls	r7, r7, #CR_BUF_RST_SHIFT
	ldr	r6, [r0, #FLASH_CR]	/* reset the page buffer */
	orrs	r6, r7
	str	r6, [r0, #FLASH_CR]
rst_busy:
	ldr	r6, [r0, #FLASH_SR]	/* wait until BSY flag is reset */
	movs	r7, #SR_BSY
	tst	r6, r7
	bne	rst_busy
load_word:
	ldr	r6, [r5]		/* "*target_address++ = *rp++" */
	str	r6, [r4]
	adds	r5, #4
	adds	r4, #4
	movs	r7, #(LOAD_SIZE - 1)
	tst	r4, r7			/* loop until 16 bytes are written */
	bne	load_word
	movs	r7, #1
	lsls	r7, r7, #CR_BUF_LOAD_SHIFT
	ldr	r6, [r0, #FLASH_CR]	/* load them into the page buffer */
	orrs	r6, r7
	str	r6, [r0, #FLASH_CR]
load_busy:
	ldr	r6, [r0, #FLASH_SR]	/* wait until BSY flag is reset */
	movs	r7, #SR_BSY
	tst	r6, r7
	bne	load_busy
	movs	r7, #(PAGE_SIZE - 1)
	tst	r4, r7			/* loop until the page is loaded */
	bne	load_word
	mov	r6, r4
	subs	r6, #PAGE_SIZE
	str	r6, [r0, #FLASH_AR]	/* program the page */
	movs	r7, #CR_STRT
	ldr	r6, [r0, #FLASH_CR]
	orrs	r6, r7
	str	r6, [r0, #FLASH_CR]
pg_busy:
	ldr	r6, [r0, #FLASH_SR]	/* wait until BSY flag is reset */
	movs	r7, #SR_BSY
	tst	r6, r7
	bne	pg_busy
	movs	r7, #(SR_PGERR | SR_WRPRTERR)	/* check the error bits */
	tst	r6, r7
	bne	error
	cmp	r5, r3			/* wrap rp at end of buffer */
	bcc	no_wrap
	mov	r5, r2
	adds	r5, #8
no_wrap:
	str	r5, [r2, #4]		/* store rp */
	subs	r1, r1, #1		/* decrement page count */
	bne	wait_fifo
	b	exit
error:
	movs	r5, #0
	str	r5, [r2, #4]		/* set rp = 0 on error */
exit:
	movs	r7, #CR_PAGE_BITS
	lsls	r7, r7, #CR_PAGE_BITS_SHIFT
	ldr	r5, [r0, #FLASH_CR]	/* disable fast page programming */
	bics	r5, r7
	str	r5, [r0, #FLASH_CR]
	mov	r0, r6			/* return status in r0 */
	bkpt	#0
