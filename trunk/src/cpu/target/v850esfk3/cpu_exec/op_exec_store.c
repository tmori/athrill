#include <stdlib.h>

#include "cpu_exec/op_exec_ops.h"
#include "cpu.h"
#include "bus.h"

/*
 * Format4
 */
int op_exec_sstb(TargetCoreType *cpu)
{
	uint32 addr;
	uint32 disp;
	uint32 reg1 = CPU_REG_EP;
	uint32 reg2 = cpu->decoded_code.type4_1.reg2;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	disp = cpu->decoded_code.type4_1.disp;
	disp = (disp << 1) | cpu->decoded_code.type4_1.gen;
	disp = op_zero_extend(7, disp);
	addr = cpu->reg.r[reg1] + disp;

	err = bus_put_data8(cpu->core_id, addr, (uint8)cpu->reg.r[reg2]);
	if (err != STD_E_OK) {
		return -1;
	}
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SST.B r%d(0x%x), disp7(0x%x) r%d(0x%x):0x%x\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], disp, reg2, cpu->reg.r[reg2], (uint8)cpu->reg.r[reg2]));

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_ssth(TargetCoreType *cpu)
{
	uint32 addr;
	uint32 disp;
	uint32 reg1 = CPU_REG_EP;
	uint32 reg2 = cpu->decoded_code.type4_1.reg2;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}


	disp = cpu->decoded_code.type4_1.disp;
	disp = (disp << 1) | cpu->decoded_code.type4_1.gen;
	disp = op_zero_extend(7, disp);
	disp = disp << 1;
	addr = cpu->reg.r[reg1] + disp;

	err = bus_put_data16(cpu->core_id, addr, (uint16)cpu->reg.r[reg2]);
	if (err != STD_E_OK) {
		return -1;
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SST.H r%d(0x%x), disp8(0x%x) r%d(0x%x):0x%x\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], disp, reg2, cpu->reg.r[reg2], (uint16)cpu->reg.r[reg2]));

	cpu->reg.pc += 2;
	return 0;
}

int op_exec_sstw(TargetCoreType *cpu)
{
	uint32 addr;
	uint32 disp;
	uint32 reg1 = CPU_REG_EP;
	uint32 reg2 = cpu->decoded_code.type4_1.reg2;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	disp = cpu->decoded_code.type4_1.disp;
	disp = op_zero_extend(6, disp);
	disp = disp << 2;
	addr = cpu->reg.r[reg1] + disp;

	err = bus_put_data32(cpu->core_id, addr, (uint32)cpu->reg.r[reg2]);
	if (err != STD_E_OK) {
		return -1;
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SST.W r%d(0x%x), disp7(0x%x) r%d(0x%x):0x%x\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], disp, reg2, cpu->reg.r[reg2], (uint32)cpu->reg.r[reg2]));

	cpu->reg.pc += 2;
	return 0;
}

/*
 * Format7
 */

int op_exec_sthw(TargetCoreType *cpu)
{
	uint32 addr;
	sint32 disp;
	uint32 reg1 = cpu->decoded_code.type7.reg1;
	uint32 reg2 = cpu->decoded_code.type7.reg2;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}


	if (cpu->decoded_code.type7.gen == 0x00) {
		//ST.H
		disp = op_sign_extend(15, (cpu->decoded_code.type7.disp << 1) );
		addr = cpu->reg.r[reg1] + disp;

		err = bus_put_data16(cpu->core_id, addr, (sint16)cpu->reg.r[reg2]);
		if (err != STD_E_OK) {
			return -1;
		}

		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: ST.H r%d(0x%x), disp16(%d) r%d(0x%x):0x%x\n", cpu->reg.pc, reg2, cpu->reg.r[reg2], disp, reg1, cpu->reg.r[reg1], (sint16)cpu->reg.r[reg2]));
	}
	else {
		//ST.W
		disp = op_sign_extend(15, (cpu->decoded_code.type7.disp << 1) );
		addr = cpu->reg.r[reg1] + disp;

		err = bus_put_data32(cpu->core_id, addr, (sint32)cpu->reg.r[reg2]);
		if (err != STD_E_OK) {
			return -1;
		}

		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: ST.W r%d(0x%x), disp16(%d) r%d(0x%x):0x%x\n", cpu->reg.pc, reg2, cpu->reg.r[reg2], disp, reg1, cpu->reg.r[reg1], (sint32)cpu->reg.r[reg2]));
	}

	cpu->reg.pc += 4;
	return 0;
}


int op_exec_stb(TargetCoreType *cpu)
{
	uint32 addr;
	sint32 disp;
	uint32 reg1 = cpu->decoded_code.type7.reg1;
	uint32 reg2 = cpu->decoded_code.type7.reg2;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	disp = op_sign_extend(15, (cpu->decoded_code.type7.disp << 1) | cpu->decoded_code.type7.gen);
	addr = cpu->reg.r[reg1] + disp;


	err = bus_put_data8(cpu->core_id, addr, (uint8)cpu->reg.r[reg2]);
	if (err != STD_E_OK) {
		return -1;
	}


	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: ST.B r%d(0x%x), disp16(%d) r%d(0x%x):0x%x\n", cpu->reg.pc, reg2, cpu->reg.r[reg2], disp, reg1, cpu->reg.r[reg1], (uint8)cpu->reg.r[reg2]));

	cpu->reg.pc += 4;
	return 0;
}
