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
