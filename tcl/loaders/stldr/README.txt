External flash loaders in the STM32CubeProgrammer ".stldr" format, for boards
whose external memory OpenOCD has no driver of its own for. The stldr flash
driver loads one into RAM and calls into it; see the "stldr" section of the
manual and the board configuration that names the file.

A loader is specific to both the board and the memory on it, since it carries
the pin assignment and the command set of that particular chip. Do not expect
one written for another board to work.


ART-Pi2_ST_winbond_64MB.stldr
	The W35T51NWTBIE octal NOR flash on XSPI2 of the RT-Thread ART-Pi2,
	64 MB at 0x70000000. Used by board/art_pi2.cfg.

	Taken unmodified from the ART-Pi2 board support package, at
	tools/download_algorithm/stldr/ in

	  https://github.com/RT-Thread-Studio/sdk-bsp-stm32h7r-realthread-artpi2

	sha256 c952ca743afc503743c4bf6b6e2e16b40e1b3467f2b9f7efc1f33e3a1406e6db

	It is a prebuilt binary. That package states no licence and does not
	carry the sources it was built from, so it is here for convenience and
	not as something this project can relicense or claim. Anything that
	cares about the licensing of what it ships should drop the file and
	name its own through EXTERNAL_LOADER.


MX25UM25645G_ATK-CNN647B_ExtMemLoader.stldr
	The MX25UM25645G octal NOR flash on the XSPI of the ALIENTEK
	ATK-CNN647B, an STM32N647X0 core board, 32 MB at 0x70000000. Used by
	board/atk_cnn647b.cfg. An STM32N6 has no internal flash, so this is
	the only thing to program on it.

	Taken unmodified from the board's software package, at
	External_Loader/MX25UM25645G_ATK-CNN647B/Binary/.

	sha256 0bce846a1f25e4e967d572efb5a3f97f8a839817b53bf60d837aadfee740dcb5

	A prebuilt binary again, but this one the package does carry the
	sources of, next to it, as an STM32CubeIDE project. They say
	"Copyright (c) 2023 STMicroelectronics" and refer to a LICENSE file
	that the package does not come with, which by their own wording
	leaves them as-is. Same caveat as above: here for convenience, not
	something this project can relicense, and replaceable through
	FLASH_LOADERS.
