# SPDX-License-Identifier: GPL-2.0-or-later

BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= tricore-elf-

CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS = -mtc162 -Os -ffreestanding -fno-builtin -fomit-frame-pointer \
	 -Wall -Wextra -Wno-unused-parameter
LDFLAGS = -nostdlib -nostartfiles -T link.ld

all: aurix.inc

.PHONY: all clean

%.elf: %.c link.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.bin *.inc
