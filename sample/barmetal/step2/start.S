#include "test_reg.h"
#include "v850asm.inc"


.section	".text" , "ax"
.align	4
.globl	_start, _bss_clear, _data_init
.type   _start, @function
_start:
	di
	Lea _stack_data, r3
	addi STACK_SIZE, sp r3

	jarl _bss_clear, lp
	jarl _data_init, lp

	ei
	br	 _main
.size		_start, .-_start

.align	4
.globl	_default_int_handler
.type	_default_int_handler,@function
_default_int_handler:
	br	_default_int_handler
.size		_default_int_handler, .-_default_int_handler
	
