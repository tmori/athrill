#include "test_data.h"
#include "test_reg.h"
#include "v850asm.inc"

/*
 * ADDI imm16, reg1, reg2
 */

.section	".text" , "ax"
.align	4


.global	_do_test_addi_1
.type   _do_test_addi_1, @function
_do_test_addi_1:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_0_test_fail
	 cmp r24, r25
	 bne addi_0_test_fail

	mov r0, r10

addi_0_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_1, .-_do_test_addi_1


.global	_do_test_addi_2
.type   _do_test_addi_2, @function
_do_test_addi_2:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x1, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_1_test_fail
	 cmp r24, r25
	 bne addi_1_test_fail

	mov r0, r10

addi_1_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_2, .-_do_test_addi_2


.global	_do_test_addi_3
.type   _do_test_addi_3, @function
_do_test_addi_3:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x2, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_2_test_fail
	 cmp r24, r25
	 bne addi_2_test_fail

	mov r0, r10

addi_2_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_3, .-_do_test_addi_3


.global	_do_test_addi_4
.type   _do_test_addi_4, @function
_do_test_addi_4:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFFE, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_3_test_fail
	 cmp r24, r25
	 bne addi_3_test_fail

	mov r0, r10

addi_3_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_4, .-_do_test_addi_4


.global	_do_test_addi_5
.type   _do_test_addi_5, @function
_do_test_addi_5:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFFFFFF, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_4_test_fail
	 cmp r24, r25
	 bne addi_4_test_fail

	mov r0, r10

addi_4_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_5, .-_do_test_addi_5


.global	_do_test_addi_6
.type   _do_test_addi_6, @function
_do_test_addi_6:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFFE, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_5_test_fail
	 cmp r24, r25
	 bne addi_5_test_fail

	mov r0, r10

addi_5_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_6, .-_do_test_addi_6


.global	_do_test_addi_7
.type   _do_test_addi_7, @function
_do_test_addi_7:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFFFFFF, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_6_test_fail
	 cmp r24, r25
	 bne addi_6_test_fail

	mov r0, r10

addi_6_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_7, .-_do_test_addi_7


.global	_do_test_addi_8
.type   _do_test_addi_8, @function
_do_test_addi_8:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000001, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_7_test_fail
	 cmp r24, r25
	 bne addi_7_test_fail

	mov r0, r10

addi_7_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_8, .-_do_test_addi_8


.global	_do_test_addi_9
.type   _do_test_addi_9, @function
_do_test_addi_9:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x80000000, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_8_test_fail
	 cmp r24, r25
	 bne addi_8_test_fail

	mov r0, r10

addi_8_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_9, .-_do_test_addi_9


.global	_do_test_addi_10
.type   _do_test_addi_10, @function
_do_test_addi_10:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0x1, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_9_test_fail
	 cmp r24, r25
	 bne addi_9_test_fail

	mov r0, r10

addi_9_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_10, .-_do_test_addi_10


.global	_do_test_addi_11
.type   _do_test_addi_11, @function
_do_test_addi_11:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x2, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_10_test_fail
	 cmp r24, r25
	 bne addi_10_test_fail

	mov r0, r10

addi_10_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_11, .-_do_test_addi_11


.global	_do_test_addi_12
.type   _do_test_addi_12, @function
_do_test_addi_12:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x3, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_11_test_fail
	 cmp r24, r25
	 bne addi_11_test_fail

	mov r0, r10

addi_11_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_12, .-_do_test_addi_12


.global	_do_test_addi_13
.type   _do_test_addi_13, @function
_do_test_addi_13:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFFF, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_12_test_fail
	 cmp r24, r25
	 bne addi_12_test_fail

	mov r0, r10

addi_12_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_13, .-_do_test_addi_13


.global	_do_test_addi_14
.type   _do_test_addi_14, @function
_do_test_addi_14:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x80000000, r23
	mov 0x6, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_13_test_fail
	 cmp r24, r25
	 bne addi_13_test_fail

	mov r0, r10

addi_13_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_14, .-_do_test_addi_14


.global	_do_test_addi_15
.type   _do_test_addi_15, @function
_do_test_addi_15:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFFF, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_14_test_fail
	 cmp r24, r25
	 bne addi_14_test_fail

	mov r0, r10

addi_14_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_15, .-_do_test_addi_15


.global	_do_test_addi_16
.type   _do_test_addi_16, @function
_do_test_addi_16:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x9, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_15_test_fail
	 cmp r24, r25
	 bne addi_15_test_fail

	mov r0, r10

addi_15_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_16, .-_do_test_addi_16


.global	_do_test_addi_17
.type   _do_test_addi_17, @function
_do_test_addi_17:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000002, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_16_test_fail
	 cmp r24, r25
	 bne addi_16_test_fail

	mov r0, r10

addi_16_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_17, .-_do_test_addi_17


.global	_do_test_addi_18
.type   _do_test_addi_18, @function
_do_test_addi_18:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x80000001, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_17_test_fail
	 cmp r24, r25
	 bne addi_17_test_fail

	mov r0, r10

addi_17_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_18, .-_do_test_addi_18


.global	_do_test_addi_19
.type   _do_test_addi_19, @function
_do_test_addi_19:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0x2, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_18_test_fail
	 cmp r24, r25
	 bne addi_18_test_fail

	mov r0, r10

addi_18_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_19, .-_do_test_addi_19


.global	_do_test_addi_20
.type   _do_test_addi_20, @function
_do_test_addi_20:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x3, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_19_test_fail
	 cmp r24, r25
	 bne addi_19_test_fail

	mov r0, r10

addi_19_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_20, .-_do_test_addi_20


.global	_do_test_addi_21
.type   _do_test_addi_21, @function
_do_test_addi_21:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x4, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_20_test_fail
	 cmp r24, r25
	 bne addi_20_test_fail

	mov r0, r10

addi_20_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_21, .-_do_test_addi_21


.global	_do_test_addi_22
.type   _do_test_addi_22, @function
_do_test_addi_22:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x80000000, r23
	mov 0x6, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_21_test_fail
	 cmp r24, r25
	 bne addi_21_test_fail

	mov r0, r10

addi_21_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_22, .-_do_test_addi_22


.global	_do_test_addi_23
.type   _do_test_addi_23, @function
_do_test_addi_23:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x80000001, r23
	mov 0x6, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_22_test_fail
	 cmp r24, r25
	 bne addi_22_test_fail

	mov r0, r10

addi_22_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_23, .-_do_test_addi_23


.global	_do_test_addi_24
.type   _do_test_addi_24, @function
_do_test_addi_24:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x9, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_23_test_fail
	 cmp r24, r25
	 bne addi_23_test_fail

	mov r0, r10

addi_23_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_24, .-_do_test_addi_24


.global	_do_test_addi_25
.type   _do_test_addi_25, @function
_do_test_addi_25:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0x1, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_24_test_fail
	 cmp r24, r25
	 bne addi_24_test_fail

	mov r0, r10

addi_24_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_25, .-_do_test_addi_25


.global	_do_test_addi_26
.type   _do_test_addi_26, @function
_do_test_addi_26:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000003, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_25_test_fail
	 cmp r24, r25
	 bne addi_25_test_fail

	mov r0, r10

addi_25_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_26, .-_do_test_addi_26


.global	_do_test_addi_27
.type   _do_test_addi_27, @function
_do_test_addi_27:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x80000002, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_26_test_fail
	 cmp r24, r25
	 bne addi_26_test_fail

	mov r0, r10

addi_26_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_27, .-_do_test_addi_27


.global	_do_test_addi_28
.type   _do_test_addi_28, @function
_do_test_addi_28:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0x7FFE, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_27_test_fail
	 cmp r24, r25
	 bne addi_27_test_fail

	mov r0, r10

addi_27_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_28, .-_do_test_addi_28


.global	_do_test_addi_29
.type   _do_test_addi_29, @function
_do_test_addi_29:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x7FFF, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_28_test_fail
	 cmp r24, r25
	 bne addi_28_test_fail

	mov r0, r10

addi_28_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_29, .-_do_test_addi_29


.global	_do_test_addi_30
.type   _do_test_addi_30, @function
_do_test_addi_30:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x8000, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_29_test_fail
	 cmp r24, r25
	 bne addi_29_test_fail

	mov r0, r10

addi_29_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_30, .-_do_test_addi_30


.global	_do_test_addi_31
.type   _do_test_addi_31, @function
_do_test_addi_31:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x80007FFC, r23
	mov 0x6, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_30_test_fail
	 cmp r24, r25
	 bne addi_30_test_fail

	mov r0, r10

addi_30_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_31, .-_do_test_addi_31


.global	_do_test_addi_32
.type   _do_test_addi_32, @function
_do_test_addi_32:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x80007FFD, r23
	mov 0x6, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_31_test_fail
	 cmp r24, r25
	 bne addi_31_test_fail

	mov r0, r10

addi_31_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_32, .-_do_test_addi_32


.global	_do_test_addi_33
.type   _do_test_addi_33, @function
_do_test_addi_33:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0x7FFC, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_32_test_fail
	 cmp r24, r25
	 bne addi_32_test_fail

	mov r0, r10

addi_32_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_33, .-_do_test_addi_33


.global	_do_test_addi_34
.type   _do_test_addi_34, @function
_do_test_addi_34:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0x7FFD, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_33_test_fail
	 cmp r24, r25
	 bne addi_33_test_fail

	mov r0, r10

addi_33_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_34, .-_do_test_addi_34


.global	_do_test_addi_35
.type   _do_test_addi_35, @function
_do_test_addi_35:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80007FFF, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_34_test_fail
	 cmp r24, r25
	 bne addi_34_test_fail

	mov r0, r10

addi_34_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_35, .-_do_test_addi_35


.global	_do_test_addi_36
.type   _do_test_addi_36, @function
_do_test_addi_36:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x80007FFE, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32766, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_35_test_fail
	 cmp r24, r25
	 bne addi_35_test_fail

	mov r0, r10

addi_35_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_36, .-_do_test_addi_36


.global	_do_test_addi_37
.type   _do_test_addi_37, @function
_do_test_addi_37:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0x7FFF, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_36_test_fail
	 cmp r24, r25
	 bne addi_36_test_fail

	mov r0, r10

addi_36_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_37, .-_do_test_addi_37


.global	_do_test_addi_38
.type   _do_test_addi_38, @function
_do_test_addi_38:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x8000, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_37_test_fail
	 cmp r24, r25
	 bne addi_37_test_fail

	mov r0, r10

addi_37_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_38, .-_do_test_addi_38


.global	_do_test_addi_39
.type   _do_test_addi_39, @function
_do_test_addi_39:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x8001, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_38_test_fail
	 cmp r24, r25
	 bne addi_38_test_fail

	mov r0, r10

addi_38_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_39, .-_do_test_addi_39


.global	_do_test_addi_40
.type   _do_test_addi_40, @function
_do_test_addi_40:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x80007FFD, r23
	mov 0x6, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_39_test_fail
	 cmp r24, r25
	 bne addi_39_test_fail

	mov r0, r10

addi_39_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_40, .-_do_test_addi_40


.global	_do_test_addi_41
.type   _do_test_addi_41, @function
_do_test_addi_41:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x80007FFE, r23
	mov 0x6, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_40_test_fail
	 cmp r24, r25
	 bne addi_40_test_fail

	mov r0, r10

addi_40_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_41, .-_do_test_addi_41


.global	_do_test_addi_42
.type   _do_test_addi_42, @function
_do_test_addi_42:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0x7FFD, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_41_test_fail
	 cmp r24, r25
	 bne addi_41_test_fail

	mov r0, r10

addi_41_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_42, .-_do_test_addi_42


.global	_do_test_addi_43
.type   _do_test_addi_43, @function
_do_test_addi_43:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0x7FFE, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_42_test_fail
	 cmp r24, r25
	 bne addi_42_test_fail

	mov r0, r10

addi_42_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_43, .-_do_test_addi_43


.global	_do_test_addi_44
.type   _do_test_addi_44, @function
_do_test_addi_44:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80008000, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_43_test_fail
	 cmp r24, r25
	 bne addi_43_test_fail

	mov r0, r10

addi_43_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_44, .-_do_test_addi_44


.global	_do_test_addi_45
.type   _do_test_addi_45, @function
_do_test_addi_45:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x80007FFF, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi 32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_44_test_fail
	 cmp r24, r25
	 bne addi_44_test_fail

	mov r0, r10

addi_44_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_45, .-_do_test_addi_45


.global	_do_test_addi_46
.type   _do_test_addi_46, @function
_do_test_addi_46:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xFFFFFFFE, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_45_test_fail
	 cmp r24, r25
	 bne addi_45_test_fail

	mov r0, r10

addi_45_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_46, .-_do_test_addi_46


.global	_do_test_addi_47
.type   _do_test_addi_47, @function
_do_test_addi_47:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0xFFFFFFFF, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_46_test_fail
	 cmp r24, r25
	 bne addi_46_test_fail

	mov r0, r10

addi_46_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_47, .-_do_test_addi_47


.global	_do_test_addi_48
.type   _do_test_addi_48, @function
_do_test_addi_48:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x9, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_47_test_fail
	 cmp r24, r25
	 bne addi_47_test_fail

	mov r0, r10

addi_47_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_48, .-_do_test_addi_48


.global	_do_test_addi_49
.type   _do_test_addi_49, @function
_do_test_addi_49:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFFC, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_48_test_fail
	 cmp r24, r25
	 bne addi_48_test_fail

	mov r0, r10

addi_48_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_49, .-_do_test_addi_49


.global	_do_test_addi_50
.type   _do_test_addi_50, @function
_do_test_addi_50:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFFFFFD, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_49_test_fail
	 cmp r24, r25
	 bne addi_49_test_fail

	mov r0, r10

addi_49_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_50, .-_do_test_addi_50


.global	_do_test_addi_51
.type   _do_test_addi_51, @function
_do_test_addi_51:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFFC, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_50_test_fail
	 cmp r24, r25
	 bne addi_50_test_fail

	mov r0, r10

addi_50_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_51, .-_do_test_addi_51


.global	_do_test_addi_52
.type   _do_test_addi_52, @function
_do_test_addi_52:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFFFFFD, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_51_test_fail
	 cmp r24, r25
	 bne addi_51_test_fail

	mov r0, r10

addi_51_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_52, .-_do_test_addi_52


.global	_do_test_addi_53
.type   _do_test_addi_53, @function
_do_test_addi_53:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x7FFFFFFF, r23
	mov 0xC, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_52_test_fail
	 cmp r24, r25
	 bne addi_52_test_fail

	mov r0, r10

addi_52_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_53, .-_do_test_addi_53


.global	_do_test_addi_54
.type   _do_test_addi_54, @function
_do_test_addi_54:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x7FFFFFFE, r23
	mov 0xC, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -2, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_53_test_fail
	 cmp r24, r25
	 bne addi_53_test_fail

	mov r0, r10

addi_53_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_54, .-_do_test_addi_54


.global	_do_test_addi_55
.type   _do_test_addi_55, @function
_do_test_addi_55:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xFFFFFFFF, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_54_test_fail
	 cmp r24, r25
	 bne addi_54_test_fail

	mov r0, r10

addi_54_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_55, .-_do_test_addi_55


.global	_do_test_addi_56
.type   _do_test_addi_56, @function
_do_test_addi_56:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x9, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_55_test_fail
	 cmp r24, r25
	 bne addi_55_test_fail

	mov r0, r10

addi_55_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_56, .-_do_test_addi_56


.global	_do_test_addi_57
.type   _do_test_addi_57, @function
_do_test_addi_57:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x1, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_56_test_fail
	 cmp r24, r25
	 bne addi_56_test_fail

	mov r0, r10

addi_56_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_57, .-_do_test_addi_57


.global	_do_test_addi_58
.type   _do_test_addi_58, @function
_do_test_addi_58:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFFD, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_57_test_fail
	 cmp r24, r25
	 bne addi_57_test_fail

	mov r0, r10

addi_57_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_58, .-_do_test_addi_58


.global	_do_test_addi_59
.type   _do_test_addi_59, @function
_do_test_addi_59:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFFFFFE, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_58_test_fail
	 cmp r24, r25
	 bne addi_58_test_fail

	mov r0, r10

addi_58_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_59, .-_do_test_addi_59


.global	_do_test_addi_60
.type   _do_test_addi_60, @function
_do_test_addi_60:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFFD, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_59_test_fail
	 cmp r24, r25
	 bne addi_59_test_fail

	mov r0, r10

addi_59_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_60, .-_do_test_addi_60


.global	_do_test_addi_61
.type   _do_test_addi_61, @function
_do_test_addi_61:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFFFFFE, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_60_test_fail
	 cmp r24, r25
	 bne addi_60_test_fail

	mov r0, r10

addi_60_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_61, .-_do_test_addi_61


.global	_do_test_addi_62
.type   _do_test_addi_62, @function
_do_test_addi_62:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000000, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_61_test_fail
	 cmp r24, r25
	 bne addi_61_test_fail

	mov r0, r10

addi_61_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_62, .-_do_test_addi_62


.global	_do_test_addi_63
.type   _do_test_addi_63, @function
_do_test_addi_63:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x7FFFFFFF, r23
	mov 0xC, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_62_test_fail
	 cmp r24, r25
	 bne addi_62_test_fail

	mov r0, r10

addi_62_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_63, .-_do_test_addi_63


.global	_do_test_addi_64
.type   _do_test_addi_64, @function
_do_test_addi_64:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xFFFF8001, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_63_test_fail
	 cmp r24, r25
	 bne addi_63_test_fail

	mov r0, r10

addi_63_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_64, .-_do_test_addi_64


.global	_do_test_addi_65
.type   _do_test_addi_65, @function
_do_test_addi_65:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0xFFFF8002, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_64_test_fail
	 cmp r24, r25
	 bne addi_64_test_fail

	mov r0, r10

addi_64_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_65, .-_do_test_addi_65


.global	_do_test_addi_66
.type   _do_test_addi_66, @function
_do_test_addi_66:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0xFFFF8003, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_65_test_fail
	 cmp r24, r25
	 bne addi_65_test_fail

	mov r0, r10

addi_65_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_66, .-_do_test_addi_66


.global	_do_test_addi_67
.type   _do_test_addi_67, @function
_do_test_addi_67:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFF7FFF, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_66_test_fail
	 cmp r24, r25
	 bne addi_66_test_fail

	mov r0, r10

addi_66_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_67, .-_do_test_addi_67


.global	_do_test_addi_68
.type   _do_test_addi_68, @function
_do_test_addi_68:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFF8000, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_67_test_fail
	 cmp r24, r25
	 bne addi_67_test_fail

	mov r0, r10

addi_67_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_68, .-_do_test_addi_68


.global	_do_test_addi_69
.type   _do_test_addi_69, @function
_do_test_addi_69:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFF7FFF, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_68_test_fail
	 cmp r24, r25
	 bne addi_68_test_fail

	mov r0, r10

addi_68_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_69, .-_do_test_addi_69


.global	_do_test_addi_70
.type   _do_test_addi_70, @function
_do_test_addi_70:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFF8000, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_69_test_fail
	 cmp r24, r25
	 bne addi_69_test_fail

	mov r0, r10

addi_69_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_70, .-_do_test_addi_70


.global	_do_test_addi_71
.type   _do_test_addi_71, @function
_do_test_addi_71:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x7FFF8002, r23
	mov 0xC, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_70_test_fail
	 cmp r24, r25
	 bne addi_70_test_fail

	mov r0, r10

addi_70_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_71, .-_do_test_addi_71


.global	_do_test_addi_72
.type   _do_test_addi_72, @function
_do_test_addi_72:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x7FFF8001, r23
	mov 0xC, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32767, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_71_test_fail
	 cmp r24, r25
	 bne addi_71_test_fail

	mov r0, r10

addi_71_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_72, .-_do_test_addi_72


.global	_do_test_addi_73
.type   _do_test_addi_73, @function
_do_test_addi_73:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xFFFF8000, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_72_test_fail
	 cmp r24, r25
	 bne addi_72_test_fail

	mov r0, r10

addi_72_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_73, .-_do_test_addi_73


.global	_do_test_addi_74
.type   _do_test_addi_74, @function
_do_test_addi_74:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0xFFFF8001, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_73_test_fail
	 cmp r24, r25
	 bne addi_73_test_fail

	mov r0, r10

addi_73_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_74, .-_do_test_addi_74


.global	_do_test_addi_75
.type   _do_test_addi_75, @function
_do_test_addi_75:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0xFFFF8002, r23
	mov 0x2, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_74_test_fail
	 cmp r24, r25
	 bne addi_74_test_fail

	mov r0, r10

addi_74_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_75, .-_do_test_addi_75


.global	_do_test_addi_76
.type   _do_test_addi_76, @function
_do_test_addi_76:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFF7FFE, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_75_test_fail
	 cmp r24, r25
	 bne addi_75_test_fail

	mov r0, r10

addi_75_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_76, .-_do_test_addi_76


.global	_do_test_addi_77
.type   _do_test_addi_77, @function
_do_test_addi_77:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFF7FFF, r23
	mov 0x8, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_76_test_fail
	 cmp r24, r25
	 bne addi_76_test_fail

	mov r0, r10

addi_76_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_77, .-_do_test_addi_77


.global	_do_test_addi_78
.type   _do_test_addi_78, @function
_do_test_addi_78:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFF7FFE, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_77_test_fail
	 cmp r24, r25
	 bne addi_77_test_fail

	mov r0, r10

addi_77_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_78, .-_do_test_addi_78


.global	_do_test_addi_79
.type   _do_test_addi_79, @function
_do_test_addi_79:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFF7FFF, r23
	mov 0xA, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_78_test_fail
	 cmp r24, r25
	 bne addi_78_test_fail

	mov r0, r10

addi_78_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_79, .-_do_test_addi_79


.global	_do_test_addi_80
.type   _do_test_addi_80, @function
_do_test_addi_80:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x7FFF8001, r23
	mov 0xC, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_79_test_fail
	 cmp r24, r25
	 bne addi_79_test_fail

	mov r0, r10

addi_79_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_80, .-_do_test_addi_80


.global	_do_test_addi_81
.type   _do_test_addi_81, @function
_do_test_addi_81:
	addi	-20, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	st.w	r25, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x7FFF8000, r23
	mov 0xC, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	addi -32768, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne addi_80_test_fail
	 cmp r24, r25
	 bne addi_80_test_fail

	mov r0, r10

addi_80_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_addi_81, .-_do_test_addi_81

