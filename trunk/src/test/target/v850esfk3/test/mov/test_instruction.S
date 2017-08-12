#include "test_data.h"
#include "test_reg.h"
#include "v850asm.inc"

/*
 * （ 1） MOV reg1, reg2
 */

.section	".text" , "ax"
.align	4
.global	_do_test_mov1_1
.type   _do_test_mov1_1, @function
_do_test_mov1_1:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov		UINT32_MAX, REG_IN
	mov		REG_IN, REG_OUT
	cmp		REG_IN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov1_1, .-_do_test_mov1_1

.global	_do_test_mov1_2
.type   _do_test_mov1_2, @function
_do_test_mov1_2:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov 	UINT32_MIN, REG_IN
	mov 	REG_IN, REG_OUT
	cmp 	REG_IN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov1_2, .-_do_test_mov1_2


.global	_do_test_mov1_3
.type   _do_test_mov1_3, @function
_do_test_mov1_3:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov 	SINT32_MAX, REG_IN
	mov 	REG_IN, REG_OUT
	cmp 	REG_IN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov1_3, .-_do_test_mov1_3

.global	_do_test_mov1_4
.type   _do_test_mov1_4, @function
_do_test_mov1_4:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov 	SINT32_MIN, REG_IN
	mov 	REG_IN, REG_OUT
	cmp 	REG_IN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov1_4, .-_do_test_mov1_4

/*
 * （ 2） MOV imm5, reg2
 */
.global	_do_test_mov2_1
.type   _do_test_mov2_1, @function
_do_test_mov2_1:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov 	IMM5_SIGN_MAX, REG_OUT
	cmp 	IMM5_SIGN_MAX, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov2_1, .-_do_test_mov2_1

.global	_do_test_mov2_2
.type   _do_test_mov2_2, @function
_do_test_mov2_2:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov 	IMM5_SIGN_MAX_MINUS_1, REG_OUT
	cmp 	IMM5_SIGN_MAX_MINUS_1, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov2_2, .-_do_test_mov2_2

.global	_do_test_mov2_3
.type   _do_test_mov2_3, @function
_do_test_mov2_3:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov 	IMM5_ZERO, REG_OUT
	cmp 	IMM5_ZERO, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov2_3, .-_do_test_mov2_3

.global	_do_test_mov2_4
.type   _do_test_mov2_4, @function
_do_test_mov2_4:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov 	IMM5_SIGN_MIN, REG_OUT
	cmp 	IMM5_SIGN_MIN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov2_4, .-_do_test_mov2_4

.global	_do_test_mov2_5
.type   _do_test_mov2_5, @function
_do_test_mov2_5:
	add		-8, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]

	/*
	 * テストコード
	 */
	mov 	IMM5_SIGN_MIN_PLUS_1, REG_OUT
	cmp 	IMM5_SIGN_MIN_PLUS_1, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	add		8, sp
	jmp		[lp]
.size		_do_test_mov2_5, .-_do_test_mov2_5

/*
 * （ 3） MOV imm32, reg1
 */
.global	_do_test_mov3_1
.type   _do_test_mov3_1, @function
_do_test_mov3_1:
	add		-12, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]
	st.w	REG_ADDR, 8[sp]

	/*
	 * テストコード
	 */
	mov		SINT32_MAX, REG_OUT
	mov		REG_OUT_ADDR, REG_ADDR
	ld.w	0[REG_ADDR], REG_IN
	cmp		REG_IN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	ld.w	8[sp], REG_ADDR
	add		12, sp
	jmp		[lp]
.size		_do_test_mov3_1, .-_do_test_mov3_1

.global	_do_test_mov3_2
.type   _do_test_mov3_2, @function
_do_test_mov3_2:
	add		-12, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]
	st.w	REG_ADDR, 8[sp]

	/*
	 * テストコード
	 */
	mov		SINT32_MIN, REG_OUT
	mov		REG_OUT_ADDR, REG_ADDR
	ld.w	0[REG_ADDR], REG_IN
	cmp		REG_IN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	ld.w	8[sp], REG_ADDR
	add		12, sp
	jmp		[lp]
.size		_do_test_mov3_2, .-_do_test_mov3_2

.global	_do_test_mov3_3
.type   _do_test_mov3_3, @function
_do_test_mov3_3:
	add		-12, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]
	st.w	REG_ADDR, 8[sp]

	/*
	 * テストコード
	 */
	mov		IMM5_SIGN_MAX_PLUS_1, REG_OUT
	mov		REG_OUT_ADDR, REG_ADDR
	ld.w	0[REG_ADDR], REG_IN
	cmp		REG_IN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	ld.w	8[sp], REG_ADDR
	add		12, sp
	jmp		[lp]
.size		_do_test_mov3_3, .-_do_test_mov3_3


.global	_do_test_mov3_4
.type   _do_test_mov3_4, @function
_do_test_mov3_4:
	add		-12, sp
	st.w	REG_IN, 0[sp]
	st.w	REG_OUT, 4[sp]
	st.w	REG_ADDR, 8[sp]

	/*
	 * テストコード
	 */
	mov		IMM5_SIGN_MIN_MINUS_1, REG_OUT
	mov		REG_OUT_ADDR, REG_ADDR
	ld.w	0[REG_ADDR], REG_IN
	cmp		REG_IN, REG_OUT
	cmov	NZ, -1, r0, r10

	ld.w	0[sp], REG_IN
	ld.w	4[sp], REG_OUT
	ld.w	8[sp], REG_ADDR
	add		12, sp
	jmp		[lp]
.size		_do_test_mov3_4, .-_do_test_mov3_4
