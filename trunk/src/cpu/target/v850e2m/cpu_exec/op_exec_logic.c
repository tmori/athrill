#include "cpu_exec/op_exec_ops.h"
#include "cpu.h"

/*
 * Format1
 */
int op_exec_or(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint32 reg2 = cpu->decoded_code.type1.reg2;
	sint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	result = op_or(&cpu->reg, cpu->reg.r[reg2], cpu->reg.r[reg1]);
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: OR r%d(%d),r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], result));

	cpu->reg.r[reg2] = result;

	cpu->reg.pc += 2;
	return 0;

}
int op_exec_zxb(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint8 data;
	uint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	data = (uint8)cpu->reg.r[reg1];
	result = (uint32)data;
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: ZXB r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1],  result));

	cpu->reg.r[reg1] = result;

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_zxh(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint16 data;
	uint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	data = (uint16)cpu->reg.r[reg1];
	result = (uint32)data;
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: ZXH r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1],  result));

	cpu->reg.r[reg1] = result;

	cpu->reg.pc += 2;
	return 0;
}

int op_exec_tst(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint32 reg2 = cpu->decoded_code.type1.reg2;
	sint32 result = 0;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	result = op_and(&cpu->reg, cpu->reg.r[reg2], cpu->reg.r[reg1]);
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: TST r%d(%d), r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2],  result));

	cpu->reg.pc += 2;
	return 0;
}

int op_exec_and(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint32 reg2 = cpu->decoded_code.type1.reg2;
	sint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	result = op_and(&cpu->reg, cpu->reg.r[reg2], cpu->reg.r[reg1]);
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: AND r%d(%d), r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2],  result));
	cpu->reg.r[reg2] = result;

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_not(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint32 reg2 = cpu->decoded_code.type1.reg2;
	sint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	result = ~cpu->reg.r[reg1];
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: NOT r%d(%d), r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2],  result));

	cpu->reg.r[reg2] = result;
	op_chk_and_set_zero(&cpu->reg, cpu->reg.r[reg2]);
	op_chk_and_set_sign(&cpu->reg, cpu->reg.r[reg2]);
	CPU_CLR_OV(&cpu->reg);

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_xor(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint32 reg2 = cpu->decoded_code.type1.reg2;
	sint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	result = cpu->reg.r[reg2] ^ cpu->reg.r[reg1];
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: XOR r%d(%d), r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2],  result));
	cpu->reg.r[reg2] = result;

	op_chk_and_set_zero(&cpu->reg, cpu->reg.r[reg2]);
	op_chk_and_set_sign(&cpu->reg, cpu->reg.r[reg2]);
	CPU_CLR_OV(&cpu->reg);

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_sxh(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint16 tmp;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	tmp = ((sint16)cpu->reg.r[reg1]);
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SXH r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1],  tmp));

	cpu->reg.r[reg1] = (sint32)tmp;

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_sxb(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint8 tmp;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	tmp = ((sint8)cpu->reg.r[reg1]);
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SXB r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1],  tmp));
	cpu->reg.r[reg1] = (sint32)tmp;

	cpu->reg.pc += 2;
	return 0;
}

/*
 * Format2
 */
static void op_chk_and_set_shl_carry(CpuRegisterType *cpu, uint32 data, uint32 sh)
{
	if (sh > 0) {
		if ( data & (1 << (32 - sh)) ) {
			CPU_SET_CY(cpu);
		}
		else {
			CPU_CLR_CY(cpu);
		}
	}
	else {
		CPU_CLR_CY(cpu);
	}
	return;
}
static void op_chk_and_set_shr_carry(CpuRegisterType *cpu, uint32 data, uint32 sh)
{
	if (sh > 0) {
		if ( data & (1 << (sh - 1)) ) {
			CPU_SET_CY(cpu);
		}
		else {
			CPU_CLR_CY(cpu);
		}
	}
	else {
		CPU_CLR_CY(cpu);
	}
	return;
}

int op_exec_shl_2(TargetCoreType *cpu)
{
	uint32 reg2 = cpu->decoded_code.type2.reg2;
	uint32 imm_data;
	uint32 reg2_data;
	uint32 result;

	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	reg2_data = cpu->reg.r[reg2];
	imm_data = OP_FORMAT2_IMM_ZERO_EXTEND(cpu->decoded_code.type2.imm);

	result = reg2_data << imm_data;
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SHL imm5(%d),r%d(%d):%d\n", cpu->reg.pc, imm_data, reg2, cpu->reg.r[reg2], result));

	cpu->reg.r[reg2] = (sint32)result;

	CPU_CLR_OV(&cpu->reg);
	op_chk_and_set_shl_carry(&cpu->reg, reg2_data, imm_data);
	op_chk_and_set_zero(&cpu->reg, cpu->reg.r[reg2]);
	op_chk_and_set_sign(&cpu->reg, cpu->reg.r[reg2]);

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_shr_2(TargetCoreType *cpu)
{
	uint32 reg2 = cpu->decoded_code.type2.reg2;
	uint32 imm_data;
	uint32 reg2_data;
	uint32 result;

	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	reg2_data = cpu->reg.r[reg2];
	imm_data = OP_FORMAT2_IMM_ZERO_EXTEND(cpu->decoded_code.type2.imm);

	result = reg2_data >> imm_data;
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SHR imm5(%d),r%d(%d):%d\n", cpu->reg.pc, imm_data, reg2, cpu->reg.r[reg2], result));
	cpu->reg.r[reg2] = (sint32)result;

	CPU_CLR_OV(&cpu->reg);
	op_chk_and_set_shr_carry(&cpu->reg, reg2_data, imm_data);
	op_chk_and_set_zero(&cpu->reg, cpu->reg.r[reg2]);
	op_chk_and_set_sign(&cpu->reg, cpu->reg.r[reg2]);

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_sar_2(TargetCoreType *cpu)
{
	uint32 reg2 = cpu->decoded_code.type2.reg2;
	uint32 imm_data;
	sint32 reg2_data;
	sint32 result;

	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	reg2_data = cpu->reg.r[reg2];
	imm_data = OP_FORMAT2_IMM_ZERO_EXTEND(cpu->decoded_code.type2.imm);

	result = reg2_data >> imm_data;
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SAR imm5(%d),r%d(%d):%d\n", cpu->reg.pc, imm_data, reg2, cpu->reg.r[reg2], result));

	cpu->reg.r[reg2] = (sint32)result;

	CPU_CLR_OV(&cpu->reg);
	op_chk_and_set_shr_carry(&cpu->reg, reg2_data, imm_data);
	op_chk_and_set_zero(&cpu->reg, cpu->reg.r[reg2]);
	op_chk_and_set_sign(&cpu->reg, cpu->reg.r[reg2]);

	cpu->reg.pc += 2;

	return 0;
}



/*
 * Format6
 */

int op_exec_andi(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type6.reg1;
	uint32 reg2 = cpu->decoded_code.type6.reg2;
	uint32 imm_data = op_zero_extend(16, cpu->decoded_code.type6.imm);
	sint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	result = op_andi(&cpu->reg, imm_data, cpu->reg.r[reg1]);
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: ANDI imm5(%d),r%d(%d) r%d(%d):%d\n", cpu->reg.pc, imm_data, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], result));

	cpu->reg.r[reg2] = result;

	cpu->reg.pc += 4;
	return 0;
}

int op_exec_ori(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type6.reg1;
	uint32 reg2 = cpu->decoded_code.type6.reg2;
	uint32 imm_data = op_zero_extend(16, cpu->decoded_code.type6.imm);
	sint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	result = op_ori(&cpu->reg, imm_data, cpu->reg.r[reg1]);
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: ORI imm5(%d),r%d(%d) r%d(%d):%d\n", cpu->reg.pc, imm_data, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], result));
	cpu->reg.r[reg2] = result;

	cpu->reg.pc += 4;
	return 0;
}

int op_exec_xori(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type6.reg1;
	uint32 reg2 = cpu->decoded_code.type6.reg2;
	uint32 imm_data = op_zero_extend(16, cpu->decoded_code.type6.imm);
	sint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	result = op_xori(&cpu->reg, imm_data, cpu->reg.r[reg1]);
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: XORI imm5(%d),r%d(%d) r%d(%d):%d\n", cpu->reg.pc, imm_data, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], result));
	cpu->reg.r[reg2] = result;

	cpu->reg.pc += 4;
	return 0;
}

/*
 * Format9
 */
int op_exec_shl_9(TargetCoreType *cpu)
{
	uint32 reg2 = cpu->decoded_code.type9.reg2;
	uint32 reg1 = cpu->decoded_code.type9.gen;
	uint32 reg2_data;
	uint32 reg1_data;
	uint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	reg2_data = cpu->reg.r[reg2];
	reg1_data = (cpu->reg.r[reg1] & 0x0000001F);

	result = reg2_data << reg1_data;
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SHL r%d(%d),r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], result));

	cpu->reg.r[reg2] = (sint32)result;

	CPU_CLR_OV(&cpu->reg);
	op_chk_and_set_shl_carry(&cpu->reg, reg2_data, reg1_data);
	op_chk_and_set_zero(&cpu->reg, cpu->reg.r[reg2]);
	op_chk_and_set_sign(&cpu->reg, cpu->reg.r[reg2]);

	cpu->reg.pc += 4;

	return 0;
}

int op_exec_shr_9(TargetCoreType *cpu)
{
	uint32 reg2 = cpu->decoded_code.type9.reg2;
	uint32 reg1 = cpu->decoded_code.type9.gen;
	uint32 reg2_data;
	uint32 reg1_data;
	uint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	reg2_data = cpu->reg.r[reg2];
	reg1_data = (cpu->reg.r[reg1] & 0x0000001F);

	result = reg2_data >> reg1_data;
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SHR r%d(%d),r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], result));
	cpu->reg.r[reg2] = (sint32)result;

	CPU_CLR_OV(&cpu->reg);
	op_chk_and_set_shr_carry(&cpu->reg, reg2_data, reg1_data);
	op_chk_and_set_zero(&cpu->reg, cpu->reg.r[reg2]);
	op_chk_and_set_sign(&cpu->reg, cpu->reg.r[reg2]);

	cpu->reg.pc += 4;

	return 0;
}

int op_exec_sar_9(TargetCoreType *cpu)
{
	uint32 reg2 = cpu->decoded_code.type9.reg2;
	uint32 reg1 = cpu->decoded_code.type9.gen;
	sint32 reg2_data;
	sint32 reg1_data;
	sint32 result;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	reg2_data = cpu->reg.r[reg2];
	reg1_data = (cpu->reg.r[reg1] & 0x0000001F);

	result = reg2_data >> reg1_data;
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SAR r%d(%d),r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], result));

	cpu->reg.r[reg2] = (sint32)result;

	CPU_CLR_OV(&cpu->reg);
	op_chk_and_set_shr_carry(&cpu->reg, reg2_data, reg1_data);
	op_chk_and_set_zero(&cpu->reg, cpu->reg.r[reg2]);
	op_chk_and_set_sign(&cpu->reg, cpu->reg.r[reg2]);

	cpu->reg.pc += 4;

	return 0;
}
