#include "test_reg.h"
#include "v850asm.inc"



.section	".text" , "ax"
.align	4
.globl	_start, _bss_clear, _data_init, _svc_table
_start:
	di
	Lea _stack_data, r3
	addi STACK_SIZE, sp r3


	/*
	 * syscall setting
	 */
	mov     SYSCALL_SIZE, r6
	ldsr    r6, sccfg
	Lea     _svc_table, r6
	ldsr    r6, scbp

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

	/************************
	 * KERNEL REGION
	 ************************/
	/*
	 * set IPA0L
	 */
	mov 0x00000001, r10
	ldsr r10, ipa0l

	/*
	 * set IPA0U
	 */
	mov 0x000FFFF0, r10
	ldsr r10, ipa0u

	/*
	 * set DPA0L
	 */
	mov 0x06000001, r10
	ldsr r10, dpa0l

	/*
	 * set DPA0U
	 */
	mov 0x060FFFF0, r10
	ldsr r10, dpa0u

	/************************
	 * USER REGION
	 ************************/
	/*
	 * set IPA1L
	 */
	mov 0x00100000, r10
	ldsr r10, ipa1l

	/*
	 * set IPA1U
	 */
	mov 0x001FFFF3, r10
	ldsr r10, ipa1u

	/*
	 * set DPA1L
	 */
	mov 0x07000000, r10
	ldsr r10, dpa1l

	/*
	 * set DPA0U
	 */
	mov 0x070FFFF6, r10
	ldsr r10, dpa1u


	/* mpu setting end */
	mov 0x00000000, r10
	ldsr r10, bsel

	ei
	jarl	 _main, r31
	halt


.end
