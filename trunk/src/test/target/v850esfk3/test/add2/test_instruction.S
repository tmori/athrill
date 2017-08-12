#include "test_data.h"
#include "test_reg.h"
#include "v850asm.inc"

/*
 * ADD imm5, reg2
 */

.section	".text" , "ax"
.align	4


.global	_do_test_add2_1
.type   _do_test_add2_1, @function
_do_test_add2_1:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0x0, r22
	mov 0x1, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_0_test_fail
	 cmp r23, r24
	 bne add2_0_test_fail

	mov r0, r10

add2_0_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_1, .-_do_test_add2_1


.global	_do_test_add2_2
.type   _do_test_add2_2, @function
_do_test_add2_2:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x1, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_1_test_fail
	 cmp r23, r24
	 bne add2_1_test_fail

	mov r0, r10

add2_1_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_2, .-_do_test_add2_2


.global	_do_test_add2_3
.type   _do_test_add2_3, @function
_do_test_add2_3:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x2, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_2_test_fail
	 cmp r23, r24
	 bne add2_2_test_fail

	mov r0, r10

add2_2_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_3, .-_do_test_add2_3


.global	_do_test_add2_4
.type   _do_test_add2_4, @function
_do_test_add2_4:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_3_test_fail
	 cmp r23, r24
	 bne add2_3_test_fail

	mov r0, r10

add2_3_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_4, .-_do_test_add2_4


.global	_do_test_add2_5
.type   _do_test_add2_5, @function
_do_test_add2_5:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_4_test_fail
	 cmp r23, r24
	 bne add2_4_test_fail

	mov r0, r10

add2_4_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_5, .-_do_test_add2_5


.global	_do_test_add2_6
.type   _do_test_add2_6, @function
_do_test_add2_6:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFFE, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_5_test_fail
	 cmp r23, r24
	 bne add2_5_test_fail

	mov r0, r10

add2_5_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_6, .-_do_test_add2_6


.global	_do_test_add2_7
.type   _do_test_add2_7, @function
_do_test_add2_7:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_6_test_fail
	 cmp r23, r24
	 bne add2_6_test_fail

	mov r0, r10

add2_6_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_7, .-_do_test_add2_7


.global	_do_test_add2_8
.type   _do_test_add2_8, @function
_do_test_add2_8:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000001, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_7_test_fail
	 cmp r23, r24
	 bne add2_7_test_fail

	mov r0, r10

add2_7_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_8, .-_do_test_add2_8


.global	_do_test_add2_9
.type   _do_test_add2_9, @function
_do_test_add2_9:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x80000000, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 0, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_8_test_fail
	 cmp r23, r24
	 bne add2_8_test_fail

	mov r0, r10

add2_8_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_9, .-_do_test_add2_9


.global	_do_test_add2_10
.type   _do_test_add2_10, @function
_do_test_add2_10:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0x1, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_9_test_fail
	 cmp r23, r24
	 bne add2_9_test_fail

	mov r0, r10

add2_9_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_10, .-_do_test_add2_10


.global	_do_test_add2_11
.type   _do_test_add2_11, @function
_do_test_add2_11:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x2, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_10_test_fail
	 cmp r23, r24
	 bne add2_10_test_fail

	mov r0, r10

add2_10_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_11, .-_do_test_add2_11


.global	_do_test_add2_12
.type   _do_test_add2_12, @function
_do_test_add2_12:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x3, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_11_test_fail
	 cmp r23, r24
	 bne add2_11_test_fail

	mov r0, r10

add2_11_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_12, .-_do_test_add2_12


.global	_do_test_add2_13
.type   _do_test_add2_13, @function
_do_test_add2_13:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_12_test_fail
	 cmp r23, r24
	 bne add2_12_test_fail

	mov r0, r10

add2_12_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_13, .-_do_test_add2_13


.global	_do_test_add2_14
.type   _do_test_add2_14, @function
_do_test_add2_14:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x80000000, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_13_test_fail
	 cmp r23, r24
	 bne add2_13_test_fail

	mov r0, r10

add2_13_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_14, .-_do_test_add2_14


.global	_do_test_add2_15
.type   _do_test_add2_15, @function
_do_test_add2_15:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_14_test_fail
	 cmp r23, r24
	 bne add2_14_test_fail

	mov r0, r10

add2_14_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_15, .-_do_test_add2_15


.global	_do_test_add2_16
.type   _do_test_add2_16, @function
_do_test_add2_16:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_15_test_fail
	 cmp r23, r24
	 bne add2_15_test_fail

	mov r0, r10

add2_15_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_16, .-_do_test_add2_16


.global	_do_test_add2_17
.type   _do_test_add2_17, @function
_do_test_add2_17:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000002, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_16_test_fail
	 cmp r23, r24
	 bne add2_16_test_fail

	mov r0, r10

add2_16_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_17, .-_do_test_add2_17


.global	_do_test_add2_18
.type   _do_test_add2_18, @function
_do_test_add2_18:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x80000001, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_17_test_fail
	 cmp r23, r24
	 bne add2_17_test_fail

	mov r0, r10

add2_17_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_18, .-_do_test_add2_18


.global	_do_test_add2_19
.type   _do_test_add2_19, @function
_do_test_add2_19:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0x2, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_18_test_fail
	 cmp r23, r24
	 bne add2_18_test_fail

	mov r0, r10

add2_18_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_19, .-_do_test_add2_19


.global	_do_test_add2_20
.type   _do_test_add2_20, @function
_do_test_add2_20:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x3, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_19_test_fail
	 cmp r23, r24
	 bne add2_19_test_fail

	mov r0, r10

add2_19_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_20, .-_do_test_add2_20


.global	_do_test_add2_21
.type   _do_test_add2_21, @function
_do_test_add2_21:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x4, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_20_test_fail
	 cmp r23, r24
	 bne add2_20_test_fail

	mov r0, r10

add2_20_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_21, .-_do_test_add2_21


.global	_do_test_add2_22
.type   _do_test_add2_22, @function
_do_test_add2_22:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x80000000, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_21_test_fail
	 cmp r23, r24
	 bne add2_21_test_fail

	mov r0, r10

add2_21_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_22, .-_do_test_add2_22


.global	_do_test_add2_23
.type   _do_test_add2_23, @function
_do_test_add2_23:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x80000001, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_22_test_fail
	 cmp r23, r24
	 bne add2_22_test_fail

	mov r0, r10

add2_22_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_23, .-_do_test_add2_23


.global	_do_test_add2_24
.type   _do_test_add2_24, @function
_do_test_add2_24:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_23_test_fail
	 cmp r23, r24
	 bne add2_23_test_fail

	mov r0, r10

add2_23_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_24, .-_do_test_add2_24


.global	_do_test_add2_25
.type   _do_test_add2_25, @function
_do_test_add2_25:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0x1, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_24_test_fail
	 cmp r23, r24
	 bne add2_24_test_fail

	mov r0, r10

add2_24_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_25, .-_do_test_add2_25


.global	_do_test_add2_26
.type   _do_test_add2_26, @function
_do_test_add2_26:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000003, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_25_test_fail
	 cmp r23, r24
	 bne add2_25_test_fail

	mov r0, r10

add2_25_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_26, .-_do_test_add2_26


.global	_do_test_add2_27
.type   _do_test_add2_27, @function
_do_test_add2_27:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x80000002, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_26_test_fail
	 cmp r23, r24
	 bne add2_26_test_fail

	mov r0, r10

add2_26_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_27, .-_do_test_add2_27


.global	_do_test_add2_28
.type   _do_test_add2_28, @function
_do_test_add2_28:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xE, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_27_test_fail
	 cmp r23, r24
	 bne add2_27_test_fail

	mov r0, r10

add2_27_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_28, .-_do_test_add2_28


.global	_do_test_add2_29
.type   _do_test_add2_29, @function
_do_test_add2_29:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0xF, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_28_test_fail
	 cmp r23, r24
	 bne add2_28_test_fail

	mov r0, r10

add2_28_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_29, .-_do_test_add2_29


.global	_do_test_add2_30
.type   _do_test_add2_30, @function
_do_test_add2_30:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x10, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_29_test_fail
	 cmp r23, r24
	 bne add2_29_test_fail

	mov r0, r10

add2_29_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_30, .-_do_test_add2_30


.global	_do_test_add2_31
.type   _do_test_add2_31, @function
_do_test_add2_31:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x8000000C, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_30_test_fail
	 cmp r23, r24
	 bne add2_30_test_fail

	mov r0, r10

add2_30_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_31, .-_do_test_add2_31


.global	_do_test_add2_32
.type   _do_test_add2_32, @function
_do_test_add2_32:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x8000000D, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_31_test_fail
	 cmp r23, r24
	 bne add2_31_test_fail

	mov r0, r10

add2_31_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_32, .-_do_test_add2_32


.global	_do_test_add2_33
.type   _do_test_add2_33, @function
_do_test_add2_33:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xC, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_32_test_fail
	 cmp r23, r24
	 bne add2_32_test_fail

	mov r0, r10

add2_32_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_33, .-_do_test_add2_33


.global	_do_test_add2_34
.type   _do_test_add2_34, @function
_do_test_add2_34:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xD, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_33_test_fail
	 cmp r23, r24
	 bne add2_33_test_fail

	mov r0, r10

add2_33_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_34, .-_do_test_add2_34


.global	_do_test_add2_35
.type   _do_test_add2_35, @function
_do_test_add2_35:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x8000000F, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_34_test_fail
	 cmp r23, r24
	 bne add2_34_test_fail

	mov r0, r10

add2_34_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_35, .-_do_test_add2_35


.global	_do_test_add2_36
.type   _do_test_add2_36, @function
_do_test_add2_36:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x8000000E, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 14, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_35_test_fail
	 cmp r23, r24
	 bne add2_35_test_fail

	mov r0, r10

add2_35_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_36, .-_do_test_add2_36


.global	_do_test_add2_37
.type   _do_test_add2_37, @function
_do_test_add2_37:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xF, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_36_test_fail
	 cmp r23, r24
	 bne add2_36_test_fail

	mov r0, r10

add2_36_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_37, .-_do_test_add2_37


.global	_do_test_add2_38
.type   _do_test_add2_38, @function
_do_test_add2_38:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x10, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_37_test_fail
	 cmp r23, r24
	 bne add2_37_test_fail

	mov r0, r10

add2_37_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_38, .-_do_test_add2_38


.global	_do_test_add2_39
.type   _do_test_add2_39, @function
_do_test_add2_39:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x11, r22
	mov 0x0, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_38_test_fail
	 cmp r23, r24
	 bne add2_38_test_fail

	mov r0, r10

add2_38_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_39, .-_do_test_add2_39


.global	_do_test_add2_40
.type   _do_test_add2_40, @function
_do_test_add2_40:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x8000000D, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_39_test_fail
	 cmp r23, r24
	 bne add2_39_test_fail

	mov r0, r10

add2_39_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_40, .-_do_test_add2_40


.global	_do_test_add2_41
.type   _do_test_add2_41, @function
_do_test_add2_41:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x8000000E, r22
	mov 0x6, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_40_test_fail
	 cmp r23, r24
	 bne add2_40_test_fail

	mov r0, r10

add2_40_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_41, .-_do_test_add2_41


.global	_do_test_add2_42
.type   _do_test_add2_42, @function
_do_test_add2_42:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xD, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_41_test_fail
	 cmp r23, r24
	 bne add2_41_test_fail

	mov r0, r10

add2_41_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_42, .-_do_test_add2_42


.global	_do_test_add2_43
.type   _do_test_add2_43, @function
_do_test_add2_43:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xE, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_42_test_fail
	 cmp r23, r24
	 bne add2_42_test_fail

	mov r0, r10

add2_42_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_43, .-_do_test_add2_43


.global	_do_test_add2_44
.type   _do_test_add2_44, @function
_do_test_add2_44:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000010, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_43_test_fail
	 cmp r23, r24
	 bne add2_43_test_fail

	mov r0, r10

add2_43_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_44, .-_do_test_add2_44


.global	_do_test_add2_45
.type   _do_test_add2_45, @function
_do_test_add2_45:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x8000000F, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add 15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_44_test_fail
	 cmp r23, r24
	 bne add2_44_test_fail

	mov r0, r10

add2_44_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_45, .-_do_test_add2_45


.global	_do_test_add2_46
.type   _do_test_add2_46, @function
_do_test_add2_46:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xFFFFFFFE, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_45_test_fail
	 cmp r23, r24
	 bne add2_45_test_fail

	mov r0, r10

add2_45_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_46, .-_do_test_add2_46


.global	_do_test_add2_47
.type   _do_test_add2_47, @function
_do_test_add2_47:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_46_test_fail
	 cmp r23, r24
	 bne add2_46_test_fail

	mov r0, r10

add2_46_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_47, .-_do_test_add2_47


.global	_do_test_add2_48
.type   _do_test_add2_48, @function
_do_test_add2_48:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_47_test_fail
	 cmp r23, r24
	 bne add2_47_test_fail

	mov r0, r10

add2_47_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_48, .-_do_test_add2_48


.global	_do_test_add2_49
.type   _do_test_add2_49, @function
_do_test_add2_49:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFFC, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_48_test_fail
	 cmp r23, r24
	 bne add2_48_test_fail

	mov r0, r10

add2_48_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_49, .-_do_test_add2_49


.global	_do_test_add2_50
.type   _do_test_add2_50, @function
_do_test_add2_50:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFFFFFD, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_49_test_fail
	 cmp r23, r24
	 bne add2_49_test_fail

	mov r0, r10

add2_49_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_50, .-_do_test_add2_50


.global	_do_test_add2_51
.type   _do_test_add2_51, @function
_do_test_add2_51:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFFC, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_50_test_fail
	 cmp r23, r24
	 bne add2_50_test_fail

	mov r0, r10

add2_50_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_51, .-_do_test_add2_51


.global	_do_test_add2_52
.type   _do_test_add2_52, @function
_do_test_add2_52:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFFFFFD, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_51_test_fail
	 cmp r23, r24
	 bne add2_51_test_fail

	mov r0, r10

add2_51_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_52, .-_do_test_add2_52


.global	_do_test_add2_53
.type   _do_test_add2_53, @function
_do_test_add2_53:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_52_test_fail
	 cmp r23, r24
	 bne add2_52_test_fail

	mov r0, r10

add2_52_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_53, .-_do_test_add2_53


.global	_do_test_add2_54
.type   _do_test_add2_54, @function
_do_test_add2_54:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -2, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_53_test_fail
	 cmp r23, r24
	 bne add2_53_test_fail

	mov r0, r10

add2_53_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_54, .-_do_test_add2_54


.global	_do_test_add2_55
.type   _do_test_add2_55, @function
_do_test_add2_55:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xFFFFFFFF, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_54_test_fail
	 cmp r23, r24
	 bne add2_54_test_fail

	mov r0, r10

add2_54_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_55, .-_do_test_add2_55


.global	_do_test_add2_56
.type   _do_test_add2_56, @function
_do_test_add2_56:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0x0, r22
	mov 0x9, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_55_test_fail
	 cmp r23, r24
	 bne add2_55_test_fail

	mov r0, r10

add2_55_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_56, .-_do_test_add2_56


.global	_do_test_add2_57
.type   _do_test_add2_57, @function
_do_test_add2_57:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0x1, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_56_test_fail
	 cmp r23, r24
	 bne add2_56_test_fail

	mov r0, r10

add2_56_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_57, .-_do_test_add2_57


.global	_do_test_add2_58
.type   _do_test_add2_58, @function
_do_test_add2_58:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFFD, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_57_test_fail
	 cmp r23, r24
	 bne add2_57_test_fail

	mov r0, r10

add2_57_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_58, .-_do_test_add2_58


.global	_do_test_add2_59
.type   _do_test_add2_59, @function
_do_test_add2_59:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFFFFFE, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_58_test_fail
	 cmp r23, r24
	 bne add2_58_test_fail

	mov r0, r10

add2_58_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_59, .-_do_test_add2_59


.global	_do_test_add2_60
.type   _do_test_add2_60, @function
_do_test_add2_60:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFFD, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_59_test_fail
	 cmp r23, r24
	 bne add2_59_test_fail

	mov r0, r10

add2_59_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_60, .-_do_test_add2_60


.global	_do_test_add2_61
.type   _do_test_add2_61, @function
_do_test_add2_61:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFFFFFE, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_60_test_fail
	 cmp r23, r24
	 bne add2_60_test_fail

	mov r0, r10

add2_60_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_61, .-_do_test_add2_61


.global	_do_test_add2_62
.type   _do_test_add2_62, @function
_do_test_add2_62:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x80000000, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_61_test_fail
	 cmp r23, r24
	 bne add2_61_test_fail

	mov r0, r10

add2_61_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_62, .-_do_test_add2_62


.global	_do_test_add2_63
.type   _do_test_add2_63, @function
_do_test_add2_63:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x7FFFFFFF, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -1, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_62_test_fail
	 cmp r23, r24
	 bne add2_62_test_fail

	mov r0, r10

add2_62_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_63, .-_do_test_add2_63


.global	_do_test_add2_64
.type   _do_test_add2_64, @function
_do_test_add2_64:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xFFFFFFF1, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_63_test_fail
	 cmp r23, r24
	 bne add2_63_test_fail

	mov r0, r10

add2_63_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_64, .-_do_test_add2_64


.global	_do_test_add2_65
.type   _do_test_add2_65, @function
_do_test_add2_65:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0xFFFFFFF2, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_64_test_fail
	 cmp r23, r24
	 bne add2_64_test_fail

	mov r0, r10

add2_64_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_65, .-_do_test_add2_65


.global	_do_test_add2_66
.type   _do_test_add2_66, @function
_do_test_add2_66:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0xFFFFFFF3, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_65_test_fail
	 cmp r23, r24
	 bne add2_65_test_fail

	mov r0, r10

add2_65_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_66, .-_do_test_add2_66


.global	_do_test_add2_67
.type   _do_test_add2_67, @function
_do_test_add2_67:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFEF, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_66_test_fail
	 cmp r23, r24
	 bne add2_66_test_fail

	mov r0, r10

add2_66_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_67, .-_do_test_add2_67


.global	_do_test_add2_68
.type   _do_test_add2_68, @function
_do_test_add2_68:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFFFFF0, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_67_test_fail
	 cmp r23, r24
	 bne add2_67_test_fail

	mov r0, r10

add2_67_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_68, .-_do_test_add2_68


.global	_do_test_add2_69
.type   _do_test_add2_69, @function
_do_test_add2_69:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFEF, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_68_test_fail
	 cmp r23, r24
	 bne add2_68_test_fail

	mov r0, r10

add2_68_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_69, .-_do_test_add2_69


.global	_do_test_add2_70
.type   _do_test_add2_70, @function
_do_test_add2_70:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFFFFF0, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_69_test_fail
	 cmp r23, r24
	 bne add2_69_test_fail

	mov r0, r10

add2_69_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_70, .-_do_test_add2_70


.global	_do_test_add2_71
.type   _do_test_add2_71, @function
_do_test_add2_71:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x7FFFFFF2, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_70_test_fail
	 cmp r23, r24
	 bne add2_70_test_fail

	mov r0, r10

add2_70_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_71, .-_do_test_add2_71


.global	_do_test_add2_72
.type   _do_test_add2_72, @function
_do_test_add2_72:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x7FFFFFF1, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -15, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_71_test_fail
	 cmp r23, r24
	 bne add2_71_test_fail

	mov r0, r10

add2_71_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_72, .-_do_test_add2_72


.global	_do_test_add2_73
.type   _do_test_add2_73, @function
_do_test_add2_73:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x0,  r21

	/* set expect */
	mov 0xFFFFFFF0, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_72_test_fail
	 cmp r23, r24
	 bne add2_72_test_fail

	mov r0, r10

add2_72_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_73, .-_do_test_add2_73


.global	_do_test_add2_74
.type   _do_test_add2_74, @function
_do_test_add2_74:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x1,  r21

	/* set expect */
	mov 0xFFFFFFF1, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_73_test_fail
	 cmp r23, r24
	 bne add2_73_test_fail

	mov r0, r10

add2_73_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_74, .-_do_test_add2_74


.global	_do_test_add2_75
.type   _do_test_add2_75, @function
_do_test_add2_75:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x2,  r21

	/* set expect */
	mov 0xFFFFFFF2, r22
	mov 0x2, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_74_test_fail
	 cmp r23, r24
	 bne add2_74_test_fail

	mov r0, r10

add2_74_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_75, .-_do_test_add2_75


.global	_do_test_add2_76
.type   _do_test_add2_76, @function
_do_test_add2_76:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFE,  r21

	/* set expect */
	mov 0x7FFFFFEE, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_75_test_fail
	 cmp r23, r24
	 bne add2_75_test_fail

	mov r0, r10

add2_75_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_76, .-_do_test_add2_76


.global	_do_test_add2_77
.type   _do_test_add2_77, @function
_do_test_add2_77:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x7FFFFFFF,  r21

	/* set expect */
	mov 0x7FFFFFEF, r22
	mov 0x8, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_76_test_fail
	 cmp r23, r24
	 bne add2_76_test_fail

	mov r0, r10

add2_76_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_77, .-_do_test_add2_77


.global	_do_test_add2_78
.type   _do_test_add2_78, @function
_do_test_add2_78:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFE,  r21

	/* set expect */
	mov 0xFFFFFFEE, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_77_test_fail
	 cmp r23, r24
	 bne add2_77_test_fail

	mov r0, r10

add2_77_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_78, .-_do_test_add2_78


.global	_do_test_add2_79
.type   _do_test_add2_79, @function
_do_test_add2_79:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0xFFFFFFFF,  r21

	/* set expect */
	mov 0xFFFFFFEF, r22
	mov 0xA, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_78_test_fail
	 cmp r23, r24
	 bne add2_78_test_fail

	mov r0, r10

add2_78_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_79, .-_do_test_add2_79


.global	_do_test_add2_80
.type   _do_test_add2_80, @function
_do_test_add2_80:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000001,  r21

	/* set expect */
	mov 0x7FFFFFF1, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_79_test_fail
	 cmp r23, r24
	 bne add2_79_test_fail

	mov r0, r10

add2_79_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_80, .-_do_test_add2_80


.global	_do_test_add2_81
.type   _do_test_add2_81, @function
_do_test_add2_81:
	addi	-16, sp, sp
	st.w	r21, 0[sp]
	st.w	r22, 4[sp]
	st.w	r23, 8[sp]
	st.w	r24, 12[sp]
	/*
	 * prepare
	 */
	mov -1, r10
	 
	/* set input */
	mov 0x80000000,  r21

	/* set expect */
	mov 0x7FFFFFF0, r22
	mov 0xC, r24
	ldsr r0, psw

	/*
	 * do test
	 */
	add -16, r21

	/*
	 * done
	 */
	 /* get test results*/
	 stsr psw, r23
	 cmp r21, r22
	 bne add2_80_test_fail
	 cmp r23, r24
	 bne add2_80_test_fail

	mov r0, r10

add2_80_test_fail:
	ld.w	0[sp], r21
	ld.w	4[sp], r22
	ld.w	8[sp], r23
	ld.w	12[sp], r24
	
	addi	16, sp, sp
	jmp		[lp]
.size		_do_test_add2_81, .-_do_test_add2_81

