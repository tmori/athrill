#include "v850asm.inc"

.section	.text , "ax"
.align	4
.globl	_interrupt
_interrupt:
	/*
	 *  残りのレジスタ退避， EPでのアクセスに切り替えて行う
	 *  (r10-r12は退避済み)
	 */
	st.w	ep,   12[sp]
	mov		sp ,  ep 
	sst.w	r1 ,  72[ep]
	sst.w	r6 ,  68[ep]
	sst.w	r7 ,  64[ep]
	sst.w	r8 ,  60[ep]
	sst.w	r9 ,  56[ep]
	sst.w	r13 , 40[ep]
	sst.w	r14 , 36[ep]
	sst.w	r15 , 32[ep]
	sst.w	r16 , 28[ep]
	sst.w	r17 , 24[ep]
	sst.w	r18 , 20[ep]
	sst.w	r19 , 16[ep]
	sst.w	r31 ,  8[ep]
	/* 多重割込みのため，現在のEIPCとEIPSWを保存 */
	stsr	eipc,   r6
	sst.w	r6 ,   4[ep]
	stsr	eipsw,  r6
	sst.w	r6 ,   0[ep]

	Lea		end_int_handler, lp
	/* 割込みハンドラを呼び出す */
	jmp		[r10]

end_int_handler:
	/*
	 * di 状態で呼び出されること
	 * レジスタ復帰
	 */
	mov		sp , ep	/* EPへ切り替え */

	sld.w	 4[ep] , r10
	ldsr	r10 , eipc
	sld.w    0[ep] , r10
	ldsr    r10 , eipsw
	sld.w	72[ep] , r1
	sld.w	68[ep] , r6
	sld.w	64[ep] , r7
	sld.w	60[ep] , r8
	sld.w	56[ep] , r9
	sld.w	52[ep] , r10
	sld.w	48[ep] , r11
	sld.w	44[ep] , r12
	sld.w	40[ep] , r13
	sld.w	36[ep] , r14
	sld.w	32[ep] , r15
	sld.w	28[ep] , r16
	sld.w	24[ep] , r17
	sld.w	20[ep] , r18
	sld.w	16[ep] , r19
	sld.w	 8[ep] , r31
	addi	80 , ep, sp
	sld.w	12[ep] , r30

	reti	/* 割込み元へ */


