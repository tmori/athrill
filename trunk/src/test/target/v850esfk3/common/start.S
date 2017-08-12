#include "test_serial.h"
#include "test_data.h"
#include "v850asm.inc"

.section	".text" , "ax"
.align	4
.globl	_start
.type   _start, @function
_start:
	Lea _stack_data, r3
	addi 1024, sp r3
	ei
	br	 _main
.size		_start, .-_start

.align	4
.globl	_default_int_handler
.type	_default_int_handler,@function
_default_int_handler:
	br	_default_int_handler
.size		_default_int_handler, .-_default_int_handler
	
