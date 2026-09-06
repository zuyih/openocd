ENTRY(loader)
SECTIONS { . = 0; .text : { *(.text*) *(.rodata*) } /DISCARD/ : { *(.comment) *(.note*) *(.debug*) } }
