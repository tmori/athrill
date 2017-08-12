#include "test_data.h"
#include "test_reg.h"
#include "v850asm.inc"

/*
 * ANDI imm16, reg1, reg2
 */

.section	".text" , "ax"
.align	4


.global	_do_test_andi_1
.type   _do_test_andi_1, @function
_do_test_andi_1:
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
	andi 0x0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_0_test_fail
	 cmp r24, r25
	 bne andi_0_test_fail

	mov r0, r10

andi_0_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_1, .-_do_test_andi_1


.global	_do_test_andi_2
.type   _do_test_andi_2, @function
_do_test_andi_2:
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
	mov 0x0000FFFF,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_1_test_fail
	 cmp r24, r25
	 bne andi_1_test_fail

	mov r0, r10

andi_1_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_2, .-_do_test_andi_2


.global	_do_test_andi_3
.type   _do_test_andi_3, @function
_do_test_andi_3:
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
	mov 0xFFFF0000,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_2_test_fail
	 cmp r24, r25
	 bne andi_2_test_fail

	mov r0, r10

andi_2_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_3, .-_do_test_andi_3


.global	_do_test_andi_4
.type   _do_test_andi_4, @function
_do_test_andi_4:
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
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_3_test_fail
	 cmp r24, r25
	 bne andi_3_test_fail

	mov r0, r10

andi_3_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_4, .-_do_test_andi_4


.global	_do_test_andi_5
.type   _do_test_andi_5, @function
_do_test_andi_5:
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
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_4_test_fail
	 cmp r24, r25
	 bne andi_4_test_fail

	mov r0, r10

andi_4_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_5, .-_do_test_andi_5


.global	_do_test_andi_6
.type   _do_test_andi_6, @function
_do_test_andi_6:
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
	mov 0x8000,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_5_test_fail
	 cmp r24, r25
	 bne andi_5_test_fail

	mov r0, r10

andi_5_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_6, .-_do_test_andi_6


.global	_do_test_andi_7
.type   _do_test_andi_7, @function
_do_test_andi_7:
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
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x0, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_6_test_fail
	 cmp r24, r25
	 bne andi_6_test_fail

	mov r0, r10

andi_6_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_7, .-_do_test_andi_7


.global	_do_test_andi_8
.type   _do_test_andi_8, @function
_do_test_andi_8:
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
	andi 0xFFFF, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_7_test_fail
	 cmp r24, r25
	 bne andi_7_test_fail

	mov r0, r10

andi_7_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_8, .-_do_test_andi_8


.global	_do_test_andi_9
.type   _do_test_andi_9, @function
_do_test_andi_9:
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
	mov 0x0000FFFF,  r21

	/* set expect */
	mov 0x0000FFFF, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0xFFFF, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_8_test_fail
	 cmp r24, r25
	 bne andi_8_test_fail

	mov r0, r10

andi_8_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_9, .-_do_test_andi_9


.global	_do_test_andi_10
.type   _do_test_andi_10, @function
_do_test_andi_10:
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
	mov 0xFFFF0000,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0xFFFF, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_9_test_fail
	 cmp r24, r25
	 bne andi_9_test_fail

	mov r0, r10

andi_9_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_10, .-_do_test_andi_10


.global	_do_test_andi_11
.type   _do_test_andi_11, @function
_do_test_andi_11:
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
	mov 0x0000FFFF, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0xFFFF, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_10_test_fail
	 cmp r24, r25
	 bne andi_10_test_fail

	mov r0, r10

andi_10_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_11, .-_do_test_andi_11


.global	_do_test_andi_12
.type   _do_test_andi_12, @function
_do_test_andi_12:
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
	andi 0xFFFF, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_11_test_fail
	 cmp r24, r25
	 bne andi_11_test_fail

	mov r0, r10

andi_11_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_12, .-_do_test_andi_12


.global	_do_test_andi_13
.type   _do_test_andi_13, @function
_do_test_andi_13:
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
	mov 0x8000,  r21

	/* set expect */
	mov 0x8000, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0xFFFF, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_12_test_fail
	 cmp r24, r25
	 bne andi_12_test_fail

	mov r0, r10

andi_12_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_13, .-_do_test_andi_13


.global	_do_test_andi_14
.type   _do_test_andi_14, @function
_do_test_andi_14:
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
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0xFFFF, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_13_test_fail
	 cmp r24, r25
	 bne andi_13_test_fail

	mov r0, r10

andi_13_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_14, .-_do_test_andi_14


.global	_do_test_andi_15
.type   _do_test_andi_15, @function
_do_test_andi_15:
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
	andi 0x1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_14_test_fail
	 cmp r24, r25
	 bne andi_14_test_fail

	mov r0, r10

andi_14_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_15, .-_do_test_andi_15


.global	_do_test_andi_16
.type   _do_test_andi_16, @function
_do_test_andi_16:
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
	mov 0x0000FFFF,  r21

	/* set expect */
	mov 0x1, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_15_test_fail
	 cmp r24, r25
	 bne andi_15_test_fail

	mov r0, r10

andi_15_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_16, .-_do_test_andi_16


.global	_do_test_andi_17
.type   _do_test_andi_17, @function
_do_test_andi_17:
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
	mov 0xFFFF0000,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_16_test_fail
	 cmp r24, r25
	 bne andi_16_test_fail

	mov r0, r10

andi_16_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_17, .-_do_test_andi_17


.global	_do_test_andi_18
.type   _do_test_andi_18, @function
_do_test_andi_18:
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
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_17_test_fail
	 cmp r24, r25
	 bne andi_17_test_fail

	mov r0, r10

andi_17_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_18, .-_do_test_andi_18


.global	_do_test_andi_19
.type   _do_test_andi_19, @function
_do_test_andi_19:
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
	andi 0x1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_18_test_fail
	 cmp r24, r25
	 bne andi_18_test_fail

	mov r0, r10

andi_18_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_19, .-_do_test_andi_19


.global	_do_test_andi_20
.type   _do_test_andi_20, @function
_do_test_andi_20:
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
	mov 0x8000,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_19_test_fail
	 cmp r24, r25
	 bne andi_19_test_fail

	mov r0, r10

andi_19_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_20, .-_do_test_andi_20


.global	_do_test_andi_21
.type   _do_test_andi_21, @function
_do_test_andi_21:
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
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x1, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_20_test_fail
	 cmp r24, r25
	 bne andi_20_test_fail

	mov r0, r10

andi_20_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_21, .-_do_test_andi_21


.global	_do_test_andi_22
.type   _do_test_andi_22, @function
_do_test_andi_22:
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
	andi 0x8000, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_21_test_fail
	 cmp r24, r25
	 bne andi_21_test_fail

	mov r0, r10

andi_21_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_22, .-_do_test_andi_22


.global	_do_test_andi_23
.type   _do_test_andi_23, @function
_do_test_andi_23:
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
	mov 0x0000FFFF,  r21

	/* set expect */
	mov 0x8000, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x8000, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_22_test_fail
	 cmp r24, r25
	 bne andi_22_test_fail

	mov r0, r10

andi_22_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_23, .-_do_test_andi_23


.global	_do_test_andi_24
.type   _do_test_andi_24, @function
_do_test_andi_24:
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
	mov 0xFFFF0000,  r21

	/* set expect */
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x8000, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_23_test_fail
	 cmp r24, r25
	 bne andi_23_test_fail

	mov r0, r10

andi_23_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_24, .-_do_test_andi_24


.global	_do_test_andi_25
.type   _do_test_andi_25, @function
_do_test_andi_25:
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
	mov 0x8000, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x8000, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_24_test_fail
	 cmp r24, r25
	 bne andi_24_test_fail

	mov r0, r10

andi_24_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_25, .-_do_test_andi_25


.global	_do_test_andi_26
.type   _do_test_andi_26, @function
_do_test_andi_26:
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
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x8000, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_25_test_fail
	 cmp r24, r25
	 bne andi_25_test_fail

	mov r0, r10

andi_25_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_26, .-_do_test_andi_26


.global	_do_test_andi_27
.type   _do_test_andi_27, @function
_do_test_andi_27:
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
	mov 0x8000,  r21

	/* set expect */
	mov 0x8000, r23
	mov 0x0, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x8000, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_26_test_fail
	 cmp r24, r25
	 bne andi_26_test_fail

	mov r0, r10

andi_26_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_27, .-_do_test_andi_27


.global	_do_test_andi_28
.type   _do_test_andi_28, @function
_do_test_andi_28:
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
	mov 0x0, r23
	mov 0x1, r25
	ldsr r0, psw

	/*
	 * do test
	 */
	andi 0x8000, r21, r22

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r24
	 cmp r22, r23
	 bne andi_27_test_fail
	 cmp r24, r25
	 bne andi_27_test_fail

	mov r0, r10

andi_27_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	ld.w	16[sp], r25
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_andi_28, .-_do_test_andi_28

