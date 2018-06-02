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



