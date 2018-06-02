	.section .vectors,"ax"
	.align	4
	.global __reset
__reset:
	// Exception(RESET) 0x0 
	jr _start
	nop
	nop
	nop
	nop
	nop
	nop
	// Exception(NMI) 0x1 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	// Exception(INTWDT2) 0x2 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	// Exception(NOP) 0x3
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	// Exception(TRAP0) 0x4 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	// Exception(TRAP1) 0x5 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	// Exception(ILGOP/DBG0) 0x6 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	// Exception(NOP) 0x7 
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	//0x80 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x90 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0xa0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0xb0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0xc0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0xd0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0xe0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0xf0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x100 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x110 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x120 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x130 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x140 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x150 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x160 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x170 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x180 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x190 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x1a0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x1b0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x1c0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x1d0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x1e0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x1f0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x200 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x210 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x220 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x230 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x240 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x250 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x260 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x270 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x280 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x290 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x2a0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x2b0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x2c0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x2d0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x2e0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x2f0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x300 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x310 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x320 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x330 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x340 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x350 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x360 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x370 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x380 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x390 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x3a0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x3b0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x3c0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x3d0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x3e0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x3f0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x400 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x410 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x420 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x430 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x440 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x450 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x460 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x470 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x480 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x490 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x4a0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x4b0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x4c0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x4d0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x4e0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x4f0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x500 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x510 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x520 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x530 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x540 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x550 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x560 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x570 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x580 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x590 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x5a0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x5b0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x5c0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x5d0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x5e0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x5f0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x600 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x610 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x620 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x630 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x640 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x650 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x660 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x670 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x680 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x690 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x6a0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x6b0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x6c0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x6d0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x6e0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x6f0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x700 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x710 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x720 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x730 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x740 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x750 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x760 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x770 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x780 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x790 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x7a0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x7b0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop
	//0x7c0 
	jr _default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

