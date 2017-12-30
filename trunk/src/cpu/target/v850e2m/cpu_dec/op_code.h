#ifndef _OP_CODE_H_
#define _OP_CODE_H_


#define OP_CODE_ADD_1		(0b001110)	/* format1:6 */
#define OP_CODE_ADD_2		(0b010010)	/* format2:6 */
#define OP_CODE_ADDI		(0b110000)	/* format6:6 */
#define OP_CODE_AND			(0b001010)	/* format1:6 */
#define OP_CODE_ANDI		(0b110110)	/* format6:6 */
#define OP_CODE_BCOND		(0b1011)	/* format3:4 */
/* BSH */
/* BSW */
/* CALLT */
/* CLR1 */
/* CMOV */
#define OP_CODE_CMP_1		(0b001111)	/* format1:6 */
#define OP_CODE_CMP_2		(0b010011)	/* format2:6 */
/* CTRET */
/* DBRET */
/* DBTRAP */
#define OP_CODE_DI			(0b111111)	/* format10:6 */
#define SOP_CODE_DI			(0b001011)	/* format10:6 */
#define OP_CODE_DISPOSE		(0b11001)	/* format13:5 */
/* DIV */
/* DIVH */
/* DIVHU */
/* DIVU */
#define OP_CODE_EI			(0b111111)	/* format10:6 */
#define SOP_CODE_EI			(0b001011)	/* format10:6 */
#define OP_CODE_HALT		(0b111111)	/* format10:6 */
#define SOP_CODE_HALT		(0b001001)	/* format10:6 */
/* HSW */
#define OP_CODE_JARL		(0b11110)	/* format5:5 */
#define OP_CODE_JMP			(0b000011)	/* format1:6 */
#define OP_CODE_JR			(0b11110)	/* format5:5 */
#define OP_CODE_LDB			(0b111000)	/* format7:6 */
#define OP_CODE_LDBU		(0b11110)	/* format7:5 */

/* LDBU */
#define OP_CODE_LDH			(0b111001)	/* format7:6 */
#define OP_CODE_LDHU		(0b111111)	/* format7:6 */

#define OP_CODE_LDW			(0b111001)	/* format7:6 */
#define OP_CODE_LDSR		(0b111111)	/* format9:6 */
#define SOP_CODE_LDSR		(0b000001)	/* format9:6 */
#define OP_CODE_MOV_1		(0b000000)	/* format1:6 */
#define OP_CODE_MOV_2		(0b010000)	/* format2:6 */
#define OP_CODE_MOV_6		(0b110001)	/* format6:6 */
#define OP_CODE_MOVEA		(0b110001)	/* format6:6 */
#define OP_CODE_MOVHI		(0b110010)	/* format6:6 */
#define OP_CODE_MUL_11		(0b111111)	/* format11:6 10-5 */
#define SOP_CODE_MUL_11		(0b010001)	/* format11:6 26-21 */
#define OP_CODE_MUL_12		(0b111111)	/* format12:6 10-5 */


#define SOP_CODE_CMOV_11		(0b011001)	/* format11 */

#define SOP_CODE_DIVHX1_11		(0b010100)
#define SOP_CODE_DIVHX2_11		(0b010101)
#define SOP_CODE_DIVX1_11		(0b010110)
#define SOP_CODE_DIVX2_11		(0b010111)


#define SOP_CODE_MUL_12		(0b0100)	/* format12:6 26-23 */
#define SOP_CODE_CMOV_12	(0b0110)	/* format12:6 26-23 */

#define SSOP_CODE_MUL_12	(0b0)		/* format12:6 17 */
#define OP_CODE_MULH_1		(0b000111)	/* format1:6 10-5 */
#define OP_CODE_MULH_2		(0b010111)	/* format2:6 10-5 */
#define OP_CODE_MULHI		(0b110111)	/* format6:6 10-5 */
/* MULU */
#define OP_CODE_NOP			(0b000000)	/* format1:6 */
#define OP_CODE_NOT			(0b000001)	/* format1:6 */
/* NOT1 */
#define OP_CODE_OR			(0b001000)	/* format1:6 */
#define OP_CODE_ORI			(0b110100)	/* format6:6 */
#define OP_CODE_PREPARE		(0b11110)	/* format13:5 */

#define OP_CODE_RETI		(0b111111)	/* format10:6 10-5 */
#define SOP_CODE_RETI		(0b001010)	/* format10:6 26-21 */
#define OP_CODE_SAR_9		(0b111111)	/* format9:6 10-5 */
#define SOP_CODE_SAR_9		(0b000101)	/* format9:6 26-21 */
#define OP_CODE_SAR_2		(0b010101)	/* format2:6 */
/* SASF */
#define OP_CODE_SATADD_1	(0b000110)	/* format1:6 */
#define OP_CODE_SATADD_2	(0b010001)	/* format2:6 */
#define OP_CODE_SATSUB_1	(0b000101)	/* format1:6 */
#define OP_CODE_SATSUBI		(0b110011)	/* format6:6 */
#define OP_CODE_SATSUBR		(0b000100)	/* format1:6 */
/* SET1 */
#define OP_CODE_SETF_9		(0b111111)	/* format9:6 10-5 */
#define SOP_CODE_SETF_9		(0b000000)	/* format9:6 26-21 */
#define OP_CODE_SHL_9		(0b111111)	/* format9:6 10-5 */
#define SOP_CODE_SHL_9		(0b000110)	/* format9:6 26-21 */
#define OP_CODE_SHL_2		(0b010110)	/* format2:6 */
#define OP_CODE_SHR_9		(0b111111)	/* format9:6 10-5 */
#define SOP_CODE_SHR_9		(0b000100)	/* format9:6 26-21 */
#define OP_CODE_SHR_2		(0b010100)	/* format2:6 */
#define OP_CODE_SLDB		(0b0110)	/* format4:4 */
#define OP_CODE_SLDBU		(0b0000)
#define OP_CODE_SLDH		(0b1000)	/* format4:4 */
/* SLHU */
#define OP_CODE_SLDW		(0b1010)	/* format4:4 */
#define OP_CODE_SSTB		(0b0111)	/* format4:4 */
#define OP_CODE_SSTH		(0b1001)	/* format4:4 */
#define OP_CODE_SSTW		(0b1010)	/* format4:4 */
#define OP_CODE_STB			(0b111010)	/* format7:6 */
#define OP_CODE_STH			(0b111011)	/* format7:6 */
#define OP_CODE_STW			(0b111011)	/* format7:6 */
#define OP_CODE_STSR		(0b111111)	/* format9:6 10-5 */
#define SOP_CODE_STSR		(0b000010)	/* format9:6 26-21 */

#define SOP_CODE_BITOPS		(0b000111)	/* format9:6 26-21 */

#define OP_CODE_SUB			(0b001101)	/* format1:6 */
#define OP_CODE_SUBR		(0b001100)	/* format1:6 */
#define OP_CODE_SWITCH		(0b000010)	/* format1:6 */
#define OP_CODE_SXB			(0b000101)	/* format1:6 */
#define OP_CODE_SXH			(0b000111)	/* format1:6 */

#define OP_CODE_TRAP		(0b111111)	/* format10 */
#define SOP_CODE_TRAP		(0b001000)	/* format10 */

/* TRAP */
#define OP_CODE_TST			(0b001011)	/* format1:6 */
/* TST1 */
#define OP_CODE_XOR			(0b001001)	/* format1:6 */
#define OP_CODE_XORI		(0b110101)	/* format6:6 */
#define OP_CODE_ZXB			(0b000100)	/* format1:6 */
#define OP_CODE_ZXH			(0b000110)	/* format1:6 */



#endif /* _OP_CODE_H_ */
