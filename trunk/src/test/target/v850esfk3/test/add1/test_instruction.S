#include "test_data.h"
#include "test_reg.h"
#include "v850asm.inc"

/*
 * ADD reg1, reg2
 */

.section	".text" , "ax"
.align	4


.global	_do_test_add1_1
.type   _do_test_add1_1, @function
_do_test_add1_1:
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
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_0_test_fail
	 cmp r23, r24
	 bne add1_0_test_fail

	mov r0, r10

add1_0_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_1, .-_do_test_add1_1


.global	_do_test_add1_2
.type   _do_test_add1_2, @function
_do_test_add1_2:
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
	mov 0x1, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_1_test_fail
	 cmp r23, r24
	 bne add1_1_test_fail

	mov r0, r10

add1_1_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_2, .-_do_test_add1_2


.global	_do_test_add1_3
.type   _do_test_add1_3, @function
_do_test_add1_3:
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
	mov 0x2,  r21
	/* set expect */
	mov 0x2, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_2_test_fail
	 cmp r23, r24
	 bne add1_2_test_fail

	mov r0, r10

add1_2_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_3, .-_do_test_add1_3


.global	_do_test_add1_4
.type   _do_test_add1_4, @function
_do_test_add1_4:
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
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_3_test_fail
	 cmp r23, r24
	 bne add1_3_test_fail

	mov r0, r10

add1_3_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_4, .-_do_test_add1_4


.global	_do_test_add1_5
.type   _do_test_add1_5, @function
_do_test_add1_5:
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
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_4_test_fail
	 cmp r23, r24
	 bne add1_4_test_fail

	mov r0, r10

add1_4_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_5, .-_do_test_add1_5


.global	_do_test_add1_6
.type   _do_test_add1_6, @function
_do_test_add1_6:
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
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0xFFFFFFFE, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_5_test_fail
	 cmp r23, r24
	 bne add1_5_test_fail

	mov r0, r10

add1_5_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_6, .-_do_test_add1_6


.global	_do_test_add1_7
.type   _do_test_add1_7, @function
_do_test_add1_7:
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
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_6_test_fail
	 cmp r23, r24
	 bne add1_6_test_fail

	mov r0, r10

add1_6_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_7, .-_do_test_add1_7


.global	_do_test_add1_8
.type   _do_test_add1_8, @function
_do_test_add1_8:
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
	mov 0x80000001,  r21
	/* set expect */
	mov 0x80000001, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_7_test_fail
	 cmp r23, r24
	 bne add1_7_test_fail

	mov r0, r10

add1_7_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_8, .-_do_test_add1_8


.global	_do_test_add1_9
.type   _do_test_add1_9, @function
_do_test_add1_9:
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
	mov 0x80000000, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_8_test_fail
	 cmp r23, r24
	 bne add1_8_test_fail

	mov r0, r10

add1_8_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_9, .-_do_test_add1_9


.global	_do_test_add1_10
.type   _do_test_add1_10, @function
_do_test_add1_10:
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
	mov 0x1, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_9_test_fail
	 cmp r23, r24
	 bne add1_9_test_fail

	mov r0, r10

add1_9_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_10, .-_do_test_add1_10


.global	_do_test_add1_11
.type   _do_test_add1_11, @function
_do_test_add1_11:
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
	mov 0x2, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_10_test_fail
	 cmp r23, r24
	 bne add1_10_test_fail

	mov r0, r10

add1_10_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_11, .-_do_test_add1_11


.global	_do_test_add1_12
.type   _do_test_add1_12, @function
_do_test_add1_12:
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
	mov 0x2,  r21
	/* set expect */
	mov 0x3, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_11_test_fail
	 cmp r23, r24
	 bne add1_11_test_fail

	mov r0, r10

add1_11_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_12, .-_do_test_add1_12


.global	_do_test_add1_13
.type   _do_test_add1_13, @function
_do_test_add1_13:
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
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_12_test_fail
	 cmp r23, r24
	 bne add1_12_test_fail

	mov r0, r10

add1_12_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_13, .-_do_test_add1_13


.global	_do_test_add1_14
.type   _do_test_add1_14, @function
_do_test_add1_14:
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
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_13_test_fail
	 cmp r23, r24
	 bne add1_13_test_fail

	mov r0, r10

add1_13_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_14, .-_do_test_add1_14


.global	_do_test_add1_15
.type   _do_test_add1_15, @function
_do_test_add1_15:
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
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_14_test_fail
	 cmp r23, r24
	 bne add1_14_test_fail

	mov r0, r10

add1_14_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_15, .-_do_test_add1_15


.global	_do_test_add1_16
.type   _do_test_add1_16, @function
_do_test_add1_16:
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
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_15_test_fail
	 cmp r23, r24
	 bne add1_15_test_fail

	mov r0, r10

add1_15_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_16, .-_do_test_add1_16


.global	_do_test_add1_17
.type   _do_test_add1_17, @function
_do_test_add1_17:
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
	mov 0x80000001,  r21
	/* set expect */
	mov 0x80000002, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_16_test_fail
	 cmp r23, r24
	 bne add1_16_test_fail

	mov r0, r10

add1_16_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_17, .-_do_test_add1_17


.global	_do_test_add1_18
.type   _do_test_add1_18, @function
_do_test_add1_18:
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
	mov 0x80000001, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_17_test_fail
	 cmp r23, r24
	 bne add1_17_test_fail

	mov r0, r10

add1_17_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_18, .-_do_test_add1_18


.global	_do_test_add1_19
.type   _do_test_add1_19, @function
_do_test_add1_19:
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
	mov 0x2,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0x2, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_18_test_fail
	 cmp r23, r24
	 bne add1_18_test_fail

	mov r0, r10

add1_18_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_19, .-_do_test_add1_19


.global	_do_test_add1_20
.type   _do_test_add1_20, @function
_do_test_add1_20:
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
	mov 0x2,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0x3, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_19_test_fail
	 cmp r23, r24
	 bne add1_19_test_fail

	mov r0, r10

add1_19_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_20, .-_do_test_add1_20


.global	_do_test_add1_21
.type   _do_test_add1_21, @function
_do_test_add1_21:
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
	mov 0x2,  r20
	mov 0x2,  r21
	/* set expect */
	mov 0x4, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_20_test_fail
	 cmp r23, r24
	 bne add1_20_test_fail

	mov r0, r10

add1_20_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_21, .-_do_test_add1_21


.global	_do_test_add1_22
.type   _do_test_add1_22, @function
_do_test_add1_22:
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
	mov 0x2,  r20
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_21_test_fail
	 cmp r23, r24
	 bne add1_21_test_fail

	mov r0, r10

add1_21_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_22, .-_do_test_add1_22


.global	_do_test_add1_23
.type   _do_test_add1_23, @function
_do_test_add1_23:
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
	mov 0x2,  r20
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0x80000001, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_22_test_fail
	 cmp r23, r24
	 bne add1_22_test_fail

	mov r0, r10

add1_22_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_23, .-_do_test_add1_23


.global	_do_test_add1_24
.type   _do_test_add1_24, @function
_do_test_add1_24:
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
	mov 0x2,  r20
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_23_test_fail
	 cmp r23, r24
	 bne add1_23_test_fail

	mov r0, r10

add1_23_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_24, .-_do_test_add1_24


.global	_do_test_add1_25
.type   _do_test_add1_25, @function
_do_test_add1_25:
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
	mov 0x2,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0x1, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_24_test_fail
	 cmp r23, r24
	 bne add1_24_test_fail

	mov r0, r10

add1_24_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_25, .-_do_test_add1_25


.global	_do_test_add1_26
.type   _do_test_add1_26, @function
_do_test_add1_26:
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
	mov 0x2,  r20
	mov 0x80000001,  r21
	/* set expect */
	mov 0x80000003, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_25_test_fail
	 cmp r23, r24
	 bne add1_25_test_fail

	mov r0, r10

add1_25_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_26, .-_do_test_add1_26


.global	_do_test_add1_27
.type   _do_test_add1_27, @function
_do_test_add1_27:
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
	mov 0x2,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0x80000002, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_26_test_fail
	 cmp r23, r24
	 bne add1_26_test_fail

	mov r0, r10

add1_26_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_27, .-_do_test_add1_27


.global	_do_test_add1_28
.type   _do_test_add1_28, @function
_do_test_add1_28:
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
	mov 0x7FFFFFFE,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_27_test_fail
	 cmp r23, r24
	 bne add1_27_test_fail

	mov r0, r10

add1_27_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_28, .-_do_test_add1_28


.global	_do_test_add1_29
.type   _do_test_add1_29, @function
_do_test_add1_29:
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
	mov 0x7FFFFFFE,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_28_test_fail
	 cmp r23, r24
	 bne add1_28_test_fail

	mov r0, r10

add1_28_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_29, .-_do_test_add1_29


.global	_do_test_add1_30
.type   _do_test_add1_30, @function
_do_test_add1_30:
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
	mov 0x7FFFFFFE,  r20
	mov 0x2,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_29_test_fail
	 cmp r23, r24
	 bne add1_29_test_fail

	mov r0, r10

add1_29_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_30, .-_do_test_add1_30


.global	_do_test_add1_31
.type   _do_test_add1_31, @function
_do_test_add1_31:
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
	mov 0x7FFFFFFE,  r20
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0xFFFFFFFC, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_30_test_fail
	 cmp r23, r24
	 bne add1_30_test_fail

	mov r0, r10

add1_30_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_31, .-_do_test_add1_31


.global	_do_test_add1_32
.type   _do_test_add1_32, @function
_do_test_add1_32:
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
	mov 0x7FFFFFFE,  r20
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0xFFFFFFFD, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_31_test_fail
	 cmp r23, r24
	 bne add1_31_test_fail

	mov r0, r10

add1_31_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_32, .-_do_test_add1_32


.global	_do_test_add1_33
.type   _do_test_add1_33, @function
_do_test_add1_33:
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
	mov 0x7FFFFFFE,  r20
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0x7FFFFFFC, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_32_test_fail
	 cmp r23, r24
	 bne add1_32_test_fail

	mov r0, r10

add1_32_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_33, .-_do_test_add1_33


.global	_do_test_add1_34
.type   _do_test_add1_34, @function
_do_test_add1_34:
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
	mov 0x7FFFFFFE,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0x7FFFFFFD, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_33_test_fail
	 cmp r23, r24
	 bne add1_33_test_fail

	mov r0, r10

add1_33_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_34, .-_do_test_add1_34


.global	_do_test_add1_35
.type   _do_test_add1_35, @function
_do_test_add1_35:
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
	mov 0x7FFFFFFE,  r20
	mov 0x80000001,  r21
	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_34_test_fail
	 cmp r23, r24
	 bne add1_34_test_fail

	mov r0, r10

add1_34_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_35, .-_do_test_add1_35


.global	_do_test_add1_36
.type   _do_test_add1_36, @function
_do_test_add1_36:
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
	mov 0x7FFFFFFE,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0xFFFFFFFE, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_35_test_fail
	 cmp r23, r24
	 bne add1_35_test_fail

	mov r0, r10

add1_35_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_36, .-_do_test_add1_36


.global	_do_test_add1_37
.type   _do_test_add1_37, @function
_do_test_add1_37:
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
	mov 0x7FFFFFFF,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_36_test_fail
	 cmp r23, r24
	 bne add1_36_test_fail

	mov r0, r10

add1_36_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_37, .-_do_test_add1_37


.global	_do_test_add1_38
.type   _do_test_add1_38, @function
_do_test_add1_38:
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
	mov 0x7FFFFFFF,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_37_test_fail
	 cmp r23, r24
	 bne add1_37_test_fail

	mov r0, r10

add1_37_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_38, .-_do_test_add1_38


.global	_do_test_add1_39
.type   _do_test_add1_39, @function
_do_test_add1_39:
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
	mov 0x7FFFFFFF,  r20
	mov 0x2,  r21
	/* set expect */
	mov 0x80000001, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_38_test_fail
	 cmp r23, r24
	 bne add1_38_test_fail

	mov r0, r10

add1_38_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_39, .-_do_test_add1_39


.global	_do_test_add1_40
.type   _do_test_add1_40, @function
_do_test_add1_40:
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
	mov 0x7FFFFFFF,  r20
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0xFFFFFFFD, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_39_test_fail
	 cmp r23, r24
	 bne add1_39_test_fail

	mov r0, r10

add1_39_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_40, .-_do_test_add1_40


.global	_do_test_add1_41
.type   _do_test_add1_41, @function
_do_test_add1_41:
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
	mov 0x7FFFFFFF,  r20
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0xFFFFFFFE, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_40_test_fail
	 cmp r23, r24
	 bne add1_40_test_fail

	mov r0, r10

add1_40_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_41, .-_do_test_add1_41


.global	_do_test_add1_42
.type   _do_test_add1_42, @function
_do_test_add1_42:
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
	mov 0x7FFFFFFF,  r20
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0x7FFFFFFD, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_41_test_fail
	 cmp r23, r24
	 bne add1_41_test_fail

	mov r0, r10

add1_41_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_42, .-_do_test_add1_42


.global	_do_test_add1_43
.type   _do_test_add1_43, @function
_do_test_add1_43:
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
	mov 0x7FFFFFFF,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_42_test_fail
	 cmp r23, r24
	 bne add1_42_test_fail

	mov r0, r10

add1_42_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_43, .-_do_test_add1_43


.global	_do_test_add1_44
.type   _do_test_add1_44, @function
_do_test_add1_44:
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
	mov 0x7FFFFFFF,  r20
	mov 0x80000001,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_43_test_fail
	 cmp r23, r24
	 bne add1_43_test_fail

	mov r0, r10

add1_43_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_44, .-_do_test_add1_44


.global	_do_test_add1_45
.type   _do_test_add1_45, @function
_do_test_add1_45:
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
	mov 0x7FFFFFFF,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_44_test_fail
	 cmp r23, r24
	 bne add1_44_test_fail

	mov r0, r10

add1_44_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_45, .-_do_test_add1_45


.global	_do_test_add1_46
.type   _do_test_add1_46, @function
_do_test_add1_46:
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
	mov 0xFFFFFFFE,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0xFFFFFFFE, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_45_test_fail
	 cmp r23, r24
	 bne add1_45_test_fail

	mov r0, r10

add1_45_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_46, .-_do_test_add1_46


.global	_do_test_add1_47
.type   _do_test_add1_47, @function
_do_test_add1_47:
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
	mov 0xFFFFFFFE,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_46_test_fail
	 cmp r23, r24
	 bne add1_46_test_fail

	mov r0, r10

add1_46_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_47, .-_do_test_add1_47


.global	_do_test_add1_48
.type   _do_test_add1_48, @function
_do_test_add1_48:
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
	mov 0xFFFFFFFE,  r20
	mov 0x2,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_47_test_fail
	 cmp r23, r24
	 bne add1_47_test_fail

	mov r0, r10

add1_47_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_48, .-_do_test_add1_48


.global	_do_test_add1_49
.type   _do_test_add1_49, @function
_do_test_add1_49:
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
	mov 0xFFFFFFFE,  r20
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0x7FFFFFFC, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_48_test_fail
	 cmp r23, r24
	 bne add1_48_test_fail

	mov r0, r10

add1_48_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_49, .-_do_test_add1_49


.global	_do_test_add1_50
.type   _do_test_add1_50, @function
_do_test_add1_50:
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
	mov 0xFFFFFFFE,  r20
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0x7FFFFFFD, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_49_test_fail
	 cmp r23, r24
	 bne add1_49_test_fail

	mov r0, r10

add1_49_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_50, .-_do_test_add1_50


.global	_do_test_add1_51
.type   _do_test_add1_51, @function
_do_test_add1_51:
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
	mov 0xFFFFFFFE,  r20
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0xFFFFFFFC, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_50_test_fail
	 cmp r23, r24
	 bne add1_50_test_fail

	mov r0, r10

add1_50_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_51, .-_do_test_add1_51


.global	_do_test_add1_52
.type   _do_test_add1_52, @function
_do_test_add1_52:
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
	mov 0xFFFFFFFE,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0xFFFFFFFD, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_51_test_fail
	 cmp r23, r24
	 bne add1_51_test_fail

	mov r0, r10

add1_51_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_52, .-_do_test_add1_52


.global	_do_test_add1_53
.type   _do_test_add1_53, @function
_do_test_add1_53:
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
	mov 0xFFFFFFFE,  r20
	mov 0x80000001,  r21
	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_52_test_fail
	 cmp r23, r24
	 bne add1_52_test_fail

	mov r0, r10

add1_52_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_53, .-_do_test_add1_53


.global	_do_test_add1_54
.type   _do_test_add1_54, @function
_do_test_add1_54:
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
	mov 0xFFFFFFFE,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_53_test_fail
	 cmp r23, r24
	 bne add1_53_test_fail

	mov r0, r10

add1_53_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_54, .-_do_test_add1_54


.global	_do_test_add1_55
.type   _do_test_add1_55, @function
_do_test_add1_55:
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
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_54_test_fail
	 cmp r23, r24
	 bne add1_54_test_fail

	mov r0, r10

add1_54_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_55, .-_do_test_add1_55


.global	_do_test_add1_56
.type   _do_test_add1_56, @function
_do_test_add1_56:
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
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_55_test_fail
	 cmp r23, r24
	 bne add1_55_test_fail

	mov r0, r10

add1_55_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_56, .-_do_test_add1_56


.global	_do_test_add1_57
.type   _do_test_add1_57, @function
_do_test_add1_57:
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
	mov 0x2,  r21
	/* set expect */
	mov 0x1, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_56_test_fail
	 cmp r23, r24
	 bne add1_56_test_fail

	mov r0, r10

add1_56_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_57, .-_do_test_add1_57


.global	_do_test_add1_58
.type   _do_test_add1_58, @function
_do_test_add1_58:
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
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0x7FFFFFFD, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_57_test_fail
	 cmp r23, r24
	 bne add1_57_test_fail

	mov r0, r10

add1_57_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_58, .-_do_test_add1_58


.global	_do_test_add1_59
.type   _do_test_add1_59, @function
_do_test_add1_59:
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
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_58_test_fail
	 cmp r23, r24
	 bne add1_58_test_fail

	mov r0, r10

add1_58_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_59, .-_do_test_add1_59


.global	_do_test_add1_60
.type   _do_test_add1_60, @function
_do_test_add1_60:
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
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0xFFFFFFFD, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_59_test_fail
	 cmp r23, r24
	 bne add1_59_test_fail

	mov r0, r10

add1_59_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_60, .-_do_test_add1_60


.global	_do_test_add1_61
.type   _do_test_add1_61, @function
_do_test_add1_61:
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
	mov 0xFFFFFFFE, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_60_test_fail
	 cmp r23, r24
	 bne add1_60_test_fail

	mov r0, r10

add1_60_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_61, .-_do_test_add1_61


.global	_do_test_add1_62
.type   _do_test_add1_62, @function
_do_test_add1_62:
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
	mov 0x80000001,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_61_test_fail
	 cmp r23, r24
	 bne add1_61_test_fail

	mov r0, r10

add1_61_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_62, .-_do_test_add1_62


.global	_do_test_add1_63
.type   _do_test_add1_63, @function
_do_test_add1_63:
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
	mov 0x7FFFFFFF, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_62_test_fail
	 cmp r23, r24
	 bne add1_62_test_fail

	mov r0, r10

add1_62_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_63, .-_do_test_add1_63


.global	_do_test_add1_64
.type   _do_test_add1_64, @function
_do_test_add1_64:
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
	mov 0x80000001,  r20
	mov 0x0,  r21
	/* set expect */
	mov 0x80000001, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_63_test_fail
	 cmp r23, r24
	 bne add1_63_test_fail

	mov r0, r10

add1_63_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_64, .-_do_test_add1_64


.global	_do_test_add1_65
.type   _do_test_add1_65, @function
_do_test_add1_65:
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
	mov 0x80000001,  r20
	mov 0x1,  r21
	/* set expect */
	mov 0x80000002, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_64_test_fail
	 cmp r23, r24
	 bne add1_64_test_fail

	mov r0, r10

add1_64_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_65, .-_do_test_add1_65


.global	_do_test_add1_66
.type   _do_test_add1_66, @function
_do_test_add1_66:
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
	mov 0x80000001,  r20
	mov 0x2,  r21
	/* set expect */
	mov 0x80000003, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_65_test_fail
	 cmp r23, r24
	 bne add1_65_test_fail

	mov r0, r10

add1_65_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_66, .-_do_test_add1_66


.global	_do_test_add1_67
.type   _do_test_add1_67, @function
_do_test_add1_67:
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
	mov 0x80000001,  r20
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_66_test_fail
	 cmp r23, r24
	 bne add1_66_test_fail

	mov r0, r10

add1_66_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_67, .-_do_test_add1_67


.global	_do_test_add1_68
.type   _do_test_add1_68, @function
_do_test_add1_68:
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
	mov 0x80000001,  r20
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_67_test_fail
	 cmp r23, r24
	 bne add1_67_test_fail

	mov r0, r10

add1_67_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_68, .-_do_test_add1_68


.global	_do_test_add1_69
.type   _do_test_add1_69, @function
_do_test_add1_69:
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
	mov 0x80000001,  r20
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_68_test_fail
	 cmp r23, r24
	 bne add1_68_test_fail

	mov r0, r10

add1_68_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_69, .-_do_test_add1_69


.global	_do_test_add1_70
.type   _do_test_add1_70, @function
_do_test_add1_70:
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
	mov 0x80000001,  r20
	mov 0xFFFFFFFF,  r21
	/* set expect */
	mov 0x80000000, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_69_test_fail
	 cmp r23, r24
	 bne add1_69_test_fail

	mov r0, r10

add1_69_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_70, .-_do_test_add1_70


.global	_do_test_add1_71
.type   _do_test_add1_71, @function
_do_test_add1_71:
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
	mov 0x80000001,  r20
	mov 0x80000001,  r21
	/* set expect */
	mov 0x2, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_70_test_fail
	 cmp r23, r24
	 bne add1_70_test_fail

	mov r0, r10

add1_70_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_71, .-_do_test_add1_71


.global	_do_test_add1_72
.type   _do_test_add1_72, @function
_do_test_add1_72:
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
	mov 0x80000001,  r20
	mov 0x80000000,  r21
	/* set expect */
	mov 0x1, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_71_test_fail
	 cmp r23, r24
	 bne add1_71_test_fail

	mov r0, r10

add1_71_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_72, .-_do_test_add1_72


.global	_do_test_add1_73
.type   _do_test_add1_73, @function
_do_test_add1_73:
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
	mov 0x80000000, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_72_test_fail
	 cmp r23, r24
	 bne add1_72_test_fail

	mov r0, r10

add1_72_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_73, .-_do_test_add1_73


.global	_do_test_add1_74
.type   _do_test_add1_74, @function
_do_test_add1_74:
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
	mov 0x80000001, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_73_test_fail
	 cmp r23, r24
	 bne add1_73_test_fail

	mov r0, r10

add1_73_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_74, .-_do_test_add1_74


.global	_do_test_add1_75
.type   _do_test_add1_75, @function
_do_test_add1_75:
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
	mov 0x2,  r21
	/* set expect */
	mov 0x80000002, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_74_test_fail
	 cmp r23, r24
	 bne add1_74_test_fail

	mov r0, r10

add1_74_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_75, .-_do_test_add1_75


.global	_do_test_add1_76
.type   _do_test_add1_76, @function
_do_test_add1_76:
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
	mov 0x7FFFFFFE,  r21
	/* set expect */
	mov 0xFFFFFFFE, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_75_test_fail
	 cmp r23, r24
	 bne add1_75_test_fail

	mov r0, r10

add1_75_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_76, .-_do_test_add1_76


.global	_do_test_add1_77
.type   _do_test_add1_77, @function
_do_test_add1_77:
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
	mov 0x7FFFFFFF,  r21
	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_76_test_fail
	 cmp r23, r24
	 bne add1_76_test_fail

	mov r0, r10

add1_76_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_77, .-_do_test_add1_77


.global	_do_test_add1_78
.type   _do_test_add1_78, @function
_do_test_add1_78:
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
	mov 0xFFFFFFFE,  r21
	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_77_test_fail
	 cmp r23, r24
	 bne add1_77_test_fail

	mov r0, r10

add1_77_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_78, .-_do_test_add1_78


.global	_do_test_add1_79
.type   _do_test_add1_79, @function
_do_test_add1_79:
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
	mov 0x7FFFFFFF, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_78_test_fail
	 cmp r23, r24
	 bne add1_78_test_fail

	mov r0, r10

add1_78_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_79, .-_do_test_add1_79


.global	_do_test_add1_80
.type   _do_test_add1_80, @function
_do_test_add1_80:
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
	mov 0x80000001,  r21
	/* set expect */
	mov 0x1, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_79_test_fail
	 cmp r23, r24
	 bne add1_79_test_fail

	mov r0, r10

add1_79_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_80, .-_do_test_add1_80


.global	_do_test_add1_81
.type   _do_test_add1_81, @function
_do_test_add1_81:
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
	mov 0x0, r22
	mov 0xD, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add r20, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add1_80_test_fail
	 cmp r23, r24
	 bne add1_80_test_fail

	mov r0, r10

add1_80_test_fail:
	ld.w	0[sp], r20
	ld.w	4[sp], r21
	ld.w	8[sp], r22
	ld.w	12[sp], r23
	ld.w	16[sp], r24
	
	addi	20, sp, sp
	jmp		[lp]
.size		_do_test_add1_81, .-_do_test_add1_81

