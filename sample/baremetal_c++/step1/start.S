#include "test_reg.h"
#include "v850asm.inc"



.section	".text" , "ax"
.align	4
.globl	_start, _bss_clear, _data_init
_start:
	di
	Lea _stack_data, r3
	addi STACK_SIZE, sp r3

	jarl _bss_clear, lp
	jarl _data_init, lp
	jarl _ctors_init, lp

	ei
	jarl	 _main, r31
	halt
.end