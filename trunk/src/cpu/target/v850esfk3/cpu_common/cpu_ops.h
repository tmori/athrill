#include "target_cpu.h"

#define CPU_PSW_ID		(1 << 5)
#define CPU_PSW_SAT		(1 << 4)
#define CPU_PSW_CY		(1 << 3)
#define CPU_PSW_OV		(1 << 2)
#define CPU_PSW_S		(1 << 1)
#define CPU_PSW_Z		(1 << 0)

#define CPU_PSW_EP		(1 << 6)
#define CPU_PSW_NP		(1 << 7)

#define CPU_SET_EP(cpu)		(cpu)->psw |= CPU_PSW_EP
#define CPU_CLR_EP(cpu)		(cpu)->psw &= ~CPU_PSW_EP
#define CPU_ISSET_EP(cpu)	( ((cpu)->psw & CPU_PSW_EP) == CPU_PSW_EP )

#define CPU_SET_NP(cpu)		(cpu)->psw |= CPU_PSW_NP
#define CPU_CLR_NP(cpu)		(cpu)->psw &= ~CPU_PSW_NP
#define CPU_ISSET_NP(cpu)	( ((cpu)->psw & CPU_PSW_NP) == CPU_PSW_NP )


#define CPU_SET_ID(cpu)		(cpu)->psw |= CPU_PSW_ID
#define CPU_CLR_ID(cpu)		(cpu)->psw &= ~CPU_PSW_ID
#define CPU_ISSET_ID(cpu)	( ((cpu)->psw & CPU_PSW_ID) == CPU_PSW_ID )

#define CPU_SET_SAT(cpu)	(cpu)->psw |= CPU_PSW_SAT
#define CPU_CLR_SAT(cpu)	(cpu)->psw &= ~CPU_PSW_SAT
#define CPU_ISSET_SAT(cpu)	( ((cpu)->psw & CPU_PSW_SAT) == CPU_PSW_SAT)

#define CPU_SET_CY(cpu)		(cpu)->psw |= CPU_PSW_CY
#define CPU_CLR_CY(cpu)		(cpu)->psw &= ~CPU_PSW_CY
#define CPU_ISSET_CY(cpu)	( ((cpu)->psw & CPU_PSW_CY) == CPU_PSW_CY)

#define CPU_SET_OV(cpu)		(cpu)->psw |= CPU_PSW_OV
#define CPU_CLR_OV(cpu)		(cpu)->psw &= ~CPU_PSW_OV
#define CPU_ISSET_OV(cpu)	( ((cpu)->psw & CPU_PSW_OV) == CPU_PSW_OV)

#define CPU_SET_S(cpu)		(cpu)->psw |= CPU_PSW_S
#define CPU_CLR_S(cpu)		(cpu)->psw &= ~CPU_PSW_S
#define CPU_ISSET_S(cpu)	( ((cpu)->psw & CPU_PSW_S) == CPU_PSW_S)

#define CPU_SET_Z(cpu)		(cpu)->psw |= CPU_PSW_Z
#define CPU_CLR_Z(cpu)		(cpu)->psw &= ~CPU_PSW_Z
#define CPU_ISSET_Z(cpu)	( ((cpu)->psw & CPU_PSW_Z) == CPU_PSW_Z)

extern int op_chk_and_set_carry(CpuRegisterType *cpu, uint32 a, uint32 b);
extern int op_chk_and_set_borrow(CpuRegisterType *cpu, uint32 a, uint32 b);
extern int op_chk_and_set_overflow(CpuRegisterType *cpu, sint64 a, sint64 b);
extern int op_chk_and_set_zero(CpuRegisterType *cpu, sint32 result);
extern int op_chk_and_set_sign(CpuRegisterType *cpu, sint32 result);


static inline sint32 op_addi(
		CpuRegisterType *cpu,
		sint32 imm_data, sint32 reg1_data)
{
	sint32 result;

	result = reg1_data + imm_data;

	op_chk_and_set_carry(cpu, imm_data, reg1_data);
	op_chk_and_set_overflow(cpu, imm_data, reg1_data);
	op_chk_and_set_zero(cpu, result);
	op_chk_and_set_sign(cpu, result);
	return result;
}
static inline sint32 op_andi(
		CpuRegisterType *cpu,
		sint32 imm_data, sint32 reg1_data)
{
	sint32 result = reg1_data & imm_data;

	op_chk_and_set_zero(cpu, result);
	op_chk_and_set_sign(cpu, result);
	CPU_CLR_OV(cpu);
	return result;
}
static inline sint32 op_and(
		CpuRegisterType *cpu,
		sint32 reg2_data, sint32 reg1_data)
{
	sint32 result = reg1_data & reg2_data;

	op_chk_and_set_zero(cpu, result);
	op_chk_and_set_sign(cpu, result);
	CPU_CLR_OV(cpu);
	return result;
}
static inline sint32 op_or(
		CpuRegisterType *cpu,
		sint32 reg2_data, sint32 reg1_data)
{
	sint32 result = reg1_data | reg2_data;

	op_chk_and_set_zero(cpu, result);
	op_chk_and_set_sign(cpu, result);
	CPU_CLR_OV(cpu);
	return result;
}

static inline sint32 op_movea(
		CpuRegisterType *cpu,
		sint32 imm_data, sint32 reg1_data)
{
	sint32 ret = reg1_data + imm_data;
	return ret;
}
static inline sint32 op_movhi(
		CpuRegisterType *cpu,
		sint32 imm_data, sint32 reg1_data)
{
	sint32 ret = reg1_data + (imm_data << 16);
	return ret;
}
static inline sint32 op_mulhi(
		CpuRegisterType *cpu,
		sint32 imm_data, sint32 reg1_data)
{
	sint32 ret = reg1_data * (imm_data);
	return ret;
}
static inline sint32 op_ori(
		CpuRegisterType *cpu,
		uint16 imm_data, sint32 reg1_data)
{
	sint32 ret = ( reg1_data | ((uint32)(imm_data)) );
	if (ret == 0) {
		CPU_SET_Z(cpu);
	}
	else {
		CPU_CLR_Z(cpu);
	}
	if (ret & (0x80000000)) {
		CPU_SET_S(cpu);
	}
	else {
		CPU_CLR_S(cpu);
	}
	return ret;
}

static inline sint32 op_satadd(
		CpuRegisterType *cpu,
		sint32 data_l, sint32 data_r)
{
	sint32 ret;
	sint64 data_r64 = data_r;
	sint64 data_l64 = data_l;
	sint64 ret64;

	ret64 = data_l64 + data_r64;

	op_chk_and_set_carry(cpu, data_l, data_r);
	op_chk_and_set_overflow(cpu, data_l, data_r);

	if (ret64 > (sint64)CPU_REG_PLUS_MAX) {
		CPU_SET_SAT(cpu);
		ret = CPU_REG_PLUS_MAX;
	}
	else if (ret64 < CPU_REG_MINUS_MAX) {
		CPU_SET_SAT(cpu);
		ret = CPU_REG_MINUS_MAX;
	}
	else {
		ret = (sint32)ret64;
	}
	if (ret64 < 0) {
		CPU_SET_S(cpu);
	}
	else {
		CPU_CLR_S(cpu);
	}
	if (ret64 == 0) {
		CPU_SET_Z(cpu);
	}
	else {
		CPU_CLR_Z(cpu);
	}
	return ret;
}
static inline sint32 op_xori(
		CpuRegisterType *cpu,
		uint16 imm_data, sint32 reg1_data)
{
	sint32 ret = ( reg1_data ^ ((uint32)(imm_data)) );
	if (ret == 0) {
		CPU_SET_Z(cpu);
	}
	else {
		CPU_CLR_Z(cpu);
	}
	if (ret < 0) {
		CPU_SET_S(cpu);
	}
	else {
		CPU_CLR_S(cpu);
	}
	CPU_CLR_OV(cpu);
	return ret;
}

/*
 * 指定されたbitに符号ビットがある場合は31bitまで1埋めする(符号拡張)
 */
static inline sint32 op_sign_extend(uint32 bit, uint32 data)
{
	int i;
	if (data & (1 << bit)) {
		for (i = bit; i < 32; i++) {
			data = ( data | (1 << i) );//符号拡張する
		}
	}
	return data;
}
#define OP_FORMAT2_IMM_SIGN_EXTEND(data)	op_sign_extend(4, (data))

/*
 * 指定されたbitから31bitまで０埋めする
 */
static inline uint32 op_zero_extend(uint32 bit, uint32 data)
{
	int i;
	for (i = bit; i < 32; i++) {
		data = ( data & ~(1 << i) );
	}
	return data;
}
#define OP_FORMAT2_IMM_ZERO_EXTEND(data)	op_zero_extend(5, (data))

static inline char *addr2devregname(uint32 addr)
{
	switch (addr) {
	case 0xFFFFF1FE:
		return "PSC";
	case 0xFFFFF308:
		return "SELCNT0";
	case 0xFFFFF30A:
		return "SELCNT1";
	case 0xFFFFF30C:
		return "SELCNT2";
	case 0xFFFFF30E:
		return "SELCNT3";
	case 0xFFFFF3F8:
		return "SELCNT4";
	case 0xFFFFF3FA:
		return "SELCNT5";
	case 0xFFFFF3F0:
		return "SSCGCTL";
	case 0xFFFFF3F1:
		return "SFC0";
	case 0xFFFFF3F2:
		return "SFC1";
	case 0xFFFFF6C0:
		return "OSTS";
	case 0xFFFFF6C1:
		return "PLLS";
	case 0xFFFFF6C2:
		return "OSTC";
	case 0xFFFFF80C:
		return "RCM";
	case 0xFFFFF820:
		return "PSMR";
	case 0xFFFFF824:
		return "LOCKR";
	case 0xFFFFF828:
		return "PCC";
	case 0xFFFFF82C:
		return "PLLCTL";
	case 0xFFFFF82E:
		return "CCLS";
	case 0xFFFFF82F:
		return "PCLM";
	case 0xFFFFF860:
		return "MCM";
	case 0xFFFFF870:
		return "CLM";
	case 0xFFFFF8B0:
		return "PRSM0";
	case 0xFFFFF8B1:
		return "PRSCM0";
	default:
		break;
	}
	return "NULL";
}

