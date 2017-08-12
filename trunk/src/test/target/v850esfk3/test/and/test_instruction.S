#include "test_data.h"
#include "test_reg.h"
#include "v850asm.inc"

/*
 * AND reg1, reg2
 */

.section	".text" , "ax"
.align	4


.global	_do_test_and_1
.type   _do_test_and_1, @function
_do_test_and_1:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_0_test_fail
	 cmp r23, r24
	 bne and_0_test_fail

	mov r0, r10

and_0_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_1, .-_do_test_and_1


.global	_do_test_and_2
.type   _do_test_and_2, @function
_do_test_and_2:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_1_test_fail
	 cmp r23, r24
	 bne and_1_test_fail

	mov r0, r10

and_1_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_2, .-_do_test_and_2


.global	_do_test_and_3
.type   _do_test_and_3, @function
_do_test_and_3:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_2_test_fail
	 cmp r23, r24
	 bne and_2_test_fail

	mov r0, r10

and_2_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_3, .-_do_test_and_3


.global	_do_test_and_4
.type   _do_test_and_4, @function
_do_test_and_4:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_3_test_fail
	 cmp r23, r24
	 bne and_3_test_fail

	mov r0, r10

and_3_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_4, .-_do_test_and_4


.global	_do_test_and_5
.type   _do_test_and_5, @function
_do_test_and_5:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_4_test_fail
	 cmp r23, r24
	 bne and_4_test_fail

	mov r0, r10

and_4_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_5, .-_do_test_and_5


.global	_do_test_and_6
.type   _do_test_and_6, @function
_do_test_and_6:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_5_test_fail
	 cmp r23, r24
	 bne and_5_test_fail

	mov r0, r10

and_5_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_6, .-_do_test_and_6


.global	_do_test_and_7
.type   _do_test_and_7, @function
_do_test_and_7:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0x1, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_6_test_fail
	 cmp r23, r24
	 bne and_6_test_fail

	mov r0, r10

and_6_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_7, .-_do_test_and_7


.global	_do_test_and_8
.type   _do_test_and_8, @function
_do_test_and_8:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_7_test_fail
	 cmp r23, r24
	 bne and_7_test_fail

	mov r0, r10

and_7_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_8, .-_do_test_and_8


.global	_do_test_and_9
.type   _do_test_and_9, @function
_do_test_and_9:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_8_test_fail
	 cmp r23, r24
	 bne and_8_test_fail

	mov r0, r10

and_8_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_9, .-_do_test_and_9


.global	_do_test_and_10
.type   _do_test_and_10, @function
_do_test_and_10:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0x1, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_9_test_fail
	 cmp r23, r24
	 bne and_9_test_fail

	mov r0, r10

and_9_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_10, .-_do_test_and_10


.global	_do_test_and_11
.type   _do_test_and_11, @function
_do_test_and_11:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0x1, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_10_test_fail
	 cmp r23, r24
	 bne and_10_test_fail

	mov r0, r10

and_10_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_11, .-_do_test_and_11


.global	_do_test_and_12
.type   _do_test_and_12, @function
_do_test_and_12:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_11_test_fail
	 cmp r23, r24
	 bne and_11_test_fail

	mov r0, r10

and_11_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_12, .-_do_test_and_12


.global	_do_test_and_13
.type   _do_test_and_13, @function
_do_test_and_13:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_12_test_fail
	 cmp r23, r24
	 bne and_12_test_fail

	mov r0, r10

and_12_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_13, .-_do_test_and_13


.global	_do_test_and_14
.type   _do_test_and_14, @function
_do_test_and_14:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_13_test_fail
	 cmp r23, r24
	 bne and_13_test_fail

	mov r0, r10

and_13_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_14, .-_do_test_and_14


.global	_do_test_and_15
.type   _do_test_and_15, @function
_do_test_and_15:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_14_test_fail
	 cmp r23, r24
	 bne and_14_test_fail

	mov r0, r10

and_14_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_15, .-_do_test_and_15


.global	_do_test_and_16
.type   _do_test_and_16, @function
_do_test_and_16:
	addi	-20, sp, sp
	st.w	r20, 0[sp]
	st.w	r21, 4[sp]
	st.w	r22, 8[sp]
	st.w	r23, 12[sp]
	st.w	r24, 16[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	and r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne and_15_test_fail
	 cmp r23, r24
	 bne and_15_test_fail

	mov r0, r10

and_15_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_and_16, .-_do_test_and_16

