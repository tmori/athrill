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

	/* mpu setting */
	mov 0x00001001, r10
	ldsr r10, bsel
	/* stsr bsel, r11 */
	/*
	 * set MPM
	 */
	mov 0x00000003, r10
	ldsr r10, mpm
	stsr mpm, r11
	/*
	 * set DPA0L
	 */
	mov 0x07FF0001, r10
	ldsr r10, dpa1l
	stsr dpa1l, r11

	/*
	 * set DPA0U
	 */
	mov 0x07FF1000, r10
	ldsr r10, dpa1u
	stsr dpa1u, r11

	/* mpu setting end */
	mov 0x00000000, r10
	ldsr r10, bsel


	/*
	 * set pp npv dmp imp
	 */
	stsr psw, r11
	movhi 0x0F, r11, r10
	ldsr r10, psw

	ei
	jarl	 _main, r31
	halt
.end