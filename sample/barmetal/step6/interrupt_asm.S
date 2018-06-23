#include "v850asm.inc"
#include "test_reg.h"


.section	.text , "ax"
.align	4
.globl	_interrupt
_interrupt:
	addi    INTR_PUSH_SPSZ , sp , sp
	st.w	ep,   POS_EP[sp]
	mov		sp ,  ep 
	sst.w	r1 ,  POS_R1[ep]
	sst.w	r6 ,  POS_R6[ep]
	sst.w	r7 ,  POS_R7[ep]
	sst.w	r8 ,  POS_R8[ep]
	sst.w	r9 ,  POS_R9[ep]
	sst.w	r10 , POS_R10[ep]
	sst.w	r11 , POS_R11[ep]
	sst.w	r12 , POS_R12[ep]
	sst.w	r13 , POS_R13[ep]
	sst.w	r14 , POS_R14[ep]
	sst.w	r15 , POS_R15[ep]
	sst.w	r16 , POS_R16[ep]
	sst.w	r17 , POS_R17[ep]
	sst.w	r18 , POS_R18[ep]
	sst.w	r19 , POS_R19[ep]
	sst.w	r31 , POS_R31[ep]
	/* 多重割込みのため，現在のEIPCとEIPSWを保存 */
	stsr	eipc,   r11
	sst.w	r11 ,   POS_EIPC[ep]
	stsr	eipsw,  r11
	sst.w	r11 ,   POS_EIPSW[ep]
	stsr	ecr,	r6
	
	Lea	_interrupt_handler , r10
	Lea		end_int_handler, lp
	/* 割込みハンドラを呼び出す */
	jmp		[r10]

end_int_handler:
	/*
	 * di 状態で呼び出されること
	 * レジスタ復帰
	 */
	mov		sp , ep	/* EPへ切り替え */

	sld.w	POS_EIPC[ep] , r11
	ldsr	r11 , eipc
	sld.w   POS_EIPSW[ep] , r11
	ldsr    r11 , eipsw
	sld.w	POS_R1[ep] , r1
	sld.w	POS_R6[ep] , r6
	sld.w	POS_R7[ep] , r7
	sld.w	POS_R8[ep] , r8
	sld.w	POS_R9[ep] , r9
	sld.w	POS_R10[ep] , r10
	sld.w	POS_R11[ep] , r11
	sld.w	POS_R12[ep] , r12
	sld.w	POS_R13[ep] , r13
	sld.w	POS_R14[ep] , r14
	sld.w	POS_R15[ep] , r15
	sld.w	POS_R16[ep] , r16
	sld.w	POS_R17[ep] , r17
	sld.w	POS_R18[ep] , r18
	sld.w	POS_R19[ep] , r19
	sld.w	POS_R31[ep] , r31
	addi	INTR_POP_SPSZ , ep, sp
	sld.w	POS_EP[ep] , r30

	reti	/* 割込み元へ */


