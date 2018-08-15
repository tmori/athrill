#include "v850asm.inc"
#include "test_reg.h"

.section	.text , "ax"
.align	4
.globl	_svc_table
.type	_svc_table, @function
_svc_table:
	.word (12) /* get_data */
	.word (8) /* set_data */
	.word (4) /* printf */
	jr _svc_exception
.size		_svc_table, .-_svc_table

#define SVC_PUSH_SP     -28
#define SVC_POP_SP       28
#define SVC_POS_EP      0
#define SVC_POS_R11     4
#define SVC_POS_R12     8
#define SVC_POS_R13     12
#define SVC_POS_R31     16
#define SVC_POS_EIPC    20
#define SVC_POS_EIPSW   24

.section	.text , "ax"
.align	4
.globl	_svc_exception, _svc_call_table
_svc_exception:
	addi   SVC_PUSH_SP , sp , sp
	st.w	ep,     SVC_POS_EP[sp]
	mov		sp ,    ep
	sst.w	r11 ,   SVC_POS_R11[ep]
	sst.w	r12 ,   SVC_POS_R12[ep]
	sst.w	r13 ,   SVC_POS_R13[ep]
	sst.w	r31 ,   SVC_POS_R31[ep]
	stsr	eipc,   r11
	sst.w	r11 ,   SVC_POS_EIPC[ep]
	stsr	eipsw,  r11
	sst.w	r11 ,   SVC_POS_EIPSW[ep]
	stsr    eiic,   r13

	/*
	 * index
	 */
	andi   0x00FF, r13, r12
	shl    2, r12 /* r12 * 4 */
	Lea	    _svc_call_table , r11
	add    r12, r11
	ld.w    0[r11], r11

	Lea		end_svc_handler, lp
	/* SVCハンドラを呼び出す */
	jmp		[r11]

end_svc_handler:
	/*
	 * di 状態で呼び出されること
	 * レジスタ復帰
	 */
	mov		sp , ep	/* EPへ切り替え */

	sld.w	SVC_POS_EIPC[ep] , r11
	ldsr	r11 , eipc
	sld.w   SVC_POS_EIPSW[ep] , r11
	ldsr    r11 , eipsw
	sld.w	SVC_POS_R11[ep] , r11
	sld.w	SVC_POS_R12[ep] , r12
	sld.w	SVC_POS_R13[ep] , r13
	sld.w	SVC_POS_R31[ep] , r31
	addi	SVC_POP_SP , ep, sp
	sld.w	SVC_POS_EP[ep] , r30

	eiret	/* SVC発行元へ */



.section	".text_user1" , "ax"
.align	4
.globl	_user1_start, _user1_task
.type   _user1_start, @function
_user1_start:
	di

	/*
	 * set user stack
	 */
	Lea _user_stack_data, r3
	addi USER_STACK_SIZE, sp r3
	
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
	 * USER1 ROM REGION(RX:ENABLE, RW:DISABLE)
	 ************************/
	/*
	 * set IPA1L(RX)
	 */
	mov 0x00100001, r10
	ldsr r10, ipa1l

	/*
	 * set IPA1U(RX)
	 */
	mov 0x0017FFF3, r10
	ldsr r10, ipa1u

	/*
	 * set DPA1L(RW)
	 */
	mov 0x00100001, r10
	ldsr r10, dpa1l

	/*
	 * set DPA1U(RW)
	 */
	mov 0x0017FFF2, r10
	ldsr r10, dpa1u

	/************************
	 * USER1 RAM REGION(RX:DISABLE, RW:ENABLE)
	 ************************/
	/*
	 * set IPA2L(RX)
	 */
	mov 0x07000001, r10
	ldsr r10, ipa2l

	/*
	 * set IPA2U(RX)
	 */
	mov 0x0707FFF2, r10
	ldsr r10, ipa2u

	/*
	 * set DPA2L(RW)
	 */
	mov 0x07000001, r10
	ldsr r10, dpa2l

	/*
	 * set DPA2U(RW)
	 */
	mov 0x0707FFF6, r10
	ldsr r10, dpa2u

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
	jarl	 _user1_task, r31
	halt
.size		_user1_start, .-_user1_start

.section	".text_user2" , "ax"
.align	4
.globl	_user2_start, _user2_task
.type   _user2_start, @function
_user2_start:
	di

	/*
	 * set user stack
	 */
	Lea _user_stack_data, r3
	addi USER_STACK_SIZE, sp r3
	
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
	 * USER2 ROM REGION(RX:ENABLE, RW:DISABLE)
	 ************************/
	/*
	 * set IPA1L(RX)
	 */
	mov 0x00200001, r10
	ldsr r10, ipa1l

	/*
	 * set IPA1U(RX)
	 */
	mov 0x0027FFF3, r10
	ldsr r10, ipa1u

	/*
	 * set DPA1L(RW)
	 */
	mov 0x00200001, r10
	ldsr r10, dpa1l

	/*
	 * set DPA1U(RW)
	 */
	mov 0x0027FFF2, r10
	ldsr r10, dpa1u

	/************************
	 * USER1 RAM REGION(RX:DISABLE, RW:ENABLE)
	 ************************/
	/*
	 * set IPA2L(RX)
	 */
	mov 0x08000001, r10
	ldsr r10, ipa2l

	/*
	 * set IPA2U(RX)
	 */
	mov 0x0807FFF2, r10
	ldsr r10, ipa2u

	/*
	 * set DPA2L(RW)
	 */
	mov 0x08000001, r10
	ldsr r10, dpa2l

	/*
	 * set DPA2U(RW)
	 */
	mov 0x0807FFF6, r10
	ldsr r10, dpa2u

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
	jarl	 _user2_task, r31
	halt
.size		_user2_start, .-_user2_start


.section	".text_user" , "ax"
.align	4
.globl	_svc_call_get_data
.type   _svc_call_get_data, @function
_svc_call_get_data:
	syscall 0
	jmp	[lp]
.size		_svc_call_get_data, .-_svc_call_get_data

.section	".text_user" , "ax"
.align	4
.globl	_svc_call_set_data
.type   _svc_call_set_data, @function
_svc_call_set_data:
	syscall 1
	jmp	[lp]
.size		_svc_call_set_data, .-_svc_call_set_data

.section	".text_user" , "ax"
.align	4
.globl	_svc_call_printf
.type   _svc_call_printf, @function
_svc_call_printf:
	syscall 2
	jmp	[lp]
.size		_svc_call_printf, .-_svc_call_printf
