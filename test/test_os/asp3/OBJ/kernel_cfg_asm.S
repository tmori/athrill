/*
 *  割込みベクタと各割込み入口処理
 */

#define TOPPERS_LABEL_ASM
#define TOPPERS_MACRO_ONLY
#include <v850asm.inc>


	.section .vectors,"ax"
	.align	4
/*
 *  割込みベクタと各割込み入口処理
 */

	.global _start
	.global interrupt
	.global _default_exc_handler
	.global _default_int_handler
	.global __reset

__reset:
	jr _start
	nop
	nop
	nop
	nop
	nop
	nop

_ext_handler_1:
	jr _goto_default_exc_handler
	nop
	nop
	nop
	nop
	nop
	nop

_ext_handler_2:
	jr _goto_default_exc_handler
	nop
	nop
	nop
	nop
	nop
	nop

_ext_handler_3:
	jr _goto_default_exc_handler
	nop
	nop
	nop
	nop
	nop
	nop

_ext_handler_4:
	jr _goto_default_exc_handler
	nop
	nop
	nop
	nop
	nop
	nop

_ext_handler_5:
	jr _goto_default_exc_handler
	nop
	nop
	nop
	nop
	nop
	nop

_ext_handler_6:
	jr _goto_default_exc_handler
	nop
	nop
	nop
	nop
	nop
	nop
security_id:
	.long 	0xFFFFFFFF
	.long 	0xFFFFFFFF
	.long 	0xFFFFFFFF
	.long 	0xFFFFFFFF

_int_handler_0:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_1:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_2:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_3:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_4:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_5:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_6:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_7:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_8:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_9:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_10:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_11:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_12:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_13:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_14:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_15:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_16:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_17:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_18:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_19:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_20:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_21:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_22:
	jr _goto_kernel_int_handler_22
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_23:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_24:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_25:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_26:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_27:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_28:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_29:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_30:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_31:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_32:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_33:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_34:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_35:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_36:
	jr _goto_kernel_int_handler_36
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_37:
	jr _goto_kernel_int_handler_37
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_38:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_39:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_40:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_41:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_42:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_43:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_44:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_45:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_46:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_47:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_48:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_49:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_50:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_51:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_52:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_53:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_54:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_55:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_56:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_57:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_58:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_59:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_60:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_61:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_62:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_63:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_64:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_65:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_66:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_67:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_68:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_69:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_70:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_71:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_72:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_73:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_74:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_75:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_76:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_77:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_78:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_79:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_80:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_81:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_82:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_83:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_84:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_85:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_86:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_87:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_88:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_89:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_90:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_91:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_92:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_93:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_94:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_95:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_96:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_97:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_98:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_99:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_100:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_101:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_102:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_103:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_104:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_105:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_106:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_107:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_108:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_109:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_110:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_111:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_112:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_113:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_114:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_115:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop

_int_handler_116:
	jr _goto_default_int_handler
	nop
	nop
	nop
	nop
	nop
	nop


	.section .text,"ax"
	.align	4

	.global	__kernel_target_hrt_handler
	.global	_goto_kernel_int_handler_22
_goto_kernel_int_handler_22:
	addi	-80 , sp , sp
	st.w	r10 , 52[sp]
	st.w	r11 , 48[sp]
	st.w	r12 , 44[sp]
	Lea	__kernel_target_hrt_handler , r10
	mov	1 , r11
	mov	22 , r12
	jr		interrupt

	.global	__kernel_inthdr_36
	.global	_goto_kernel_int_handler_36
_goto_kernel_int_handler_36:
	addi	-80 , sp , sp
	st.w	r10 , 52[sp]
	st.w	r11 , 48[sp]
	st.w	r12 , 44[sp]
	Lea	__kernel_inthdr_36 , r10
	mov	4 , r11
	mov	36 , r12
	jr		interrupt

	.global	__kernel_inthdr_37
	.global	_goto_kernel_int_handler_37
_goto_kernel_int_handler_37:
	addi	-80 , sp , sp
	st.w	r10 , 52[sp]
	st.w	r11 , 48[sp]
	st.w	r12 , 44[sp]
	Lea	__kernel_inthdr_37 , r10
	mov	4 , r11
	mov	37 , r12
	jr		interrupt


_goto_default_int_handler:
	addi	-80 , sp , sp
	st.w	r10 , 0[sp]
	Lea	_default_int_handler , r10
	jr		interrupt


_goto_default_exc_handler:
	addi	-80 , sp , sp
	st.w	r10 , 0[sp]
	Lea	_default_exc_handler , r10
	jr		exception
	.end
