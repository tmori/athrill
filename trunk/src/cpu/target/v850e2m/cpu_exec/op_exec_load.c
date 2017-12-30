#include <stdlib.h>

#include "cpu_exec/op_exec_ops.h"
#include "cpu.h"
#include "bus.h"


/*
 * Format4
 */
int op_exec_sldb(TargetCoreType *cpu)
{
	uint32 addr;
	sint32 ret;
	uint32 disp;
	uint32 reg1 = CPU_REG_EP;
	uint32 reg2 = cpu->decoded_code.type4_1.reg2;
	sint8 data8;
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

	err = bus_get_data8(cpu->core_id, addr, (uint8*)&data8);
	if (err != STD_E_OK) {
		return -1;
	}
	ret = data8;

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SLD.B disp7(%d),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], ret));

	cpu->reg.r[reg2] = ret;

	cpu->reg.pc += 2;
	return 0;
}
int op_exec_sldbu(TargetCoreType *cpu)
{
	uint32 addr;
	uint32 ret;
	uint32 disp;
	uint32 reg1 = CPU_REG_EP;
	uint32 reg2 = cpu->decoded_code.type4_2.reg2;
	uint8 data8;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	disp = cpu->decoded_code.type4_2.disp;
	disp = op_zero_extend(3, disp);
	addr = cpu->reg.r[reg1] + disp;

	err = bus_get_data8(cpu->core_id, addr, &data8);
	if (err != STD_E_OK) {
		return -1;
	}
	ret = data8;

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SLD.BU disp4(%u),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], ret));

	cpu->reg.r[reg2] = ret;

	cpu->reg.pc += 2;
	return 0;
}

int op_exec_sldhu(TargetCoreType *cpu)
{
	uint32 addr;
	uint32 ret;
	uint32 disp;
	uint32 reg1 = CPU_REG_EP;
	uint32 reg2 = cpu->decoded_code.type4_2.reg2;
	uint16 data16;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	disp = ( (cpu->decoded_code.type4_2.disp) << 1U );
	disp = op_zero_extend(4, disp);
	addr = cpu->reg.r[reg1] + disp;

	err = bus_get_data16(cpu->core_id, addr, &data16);
	if (err != STD_E_OK) {
		return -1;
	}
	ret = data16;

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SLD.HU disp4(%u),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], ret));

	cpu->reg.r[reg2] = ret;

	cpu->reg.pc += 2;
	return 0;
}

int op_exec_sldh(TargetCoreType *cpu)
{
	uint32 addr;
	sint32 ret;
	uint32 disp;
	uint32 reg1 = CPU_REG_EP;
	uint32 reg2 = cpu->decoded_code.type4_1.reg2;
	sint16 data16;
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

	err = bus_get_data16(cpu->core_id, addr, (uint16*)&data16);
	if (err != STD_E_OK) {
		return -1;
	}
	ret = data16;

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SLD.H disp8(%d),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], ret));

	cpu->reg.r[reg2] = ret;

	cpu->reg.pc += 2;
	return 0;
}

int op_exec_sldw(TargetCoreType *cpu)
{
	uint32 addr;
	uint32 ret;
	uint32 disp;
	uint32 reg1 = CPU_REG_EP;
	uint32 reg2 = cpu->decoded_code.type4_1.reg2;
	uint32 data32;
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

	//TODO:comm_hook_load_reg32(cpu, addr);

	err = bus_get_data32(cpu->core_id, addr, &data32);
	if (err != STD_E_OK) {
		return -1;
	}
	ret = data32;

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SLD.W disp8(%d),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], ret));

	cpu->reg.r[reg2] = ret;

	cpu->reg.pc += 2;
	return 0;
}
/*
 * Format7
 */

int op_exec_ldb(TargetCoreType *cpu)
{
	uint32 addr;
	sint32 disp;
	uint32 reg1 = cpu->decoded_code.type7.reg1;
	uint32 reg2 = cpu->decoded_code.type7.reg2;
	sint8 data8;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	disp = op_sign_extend(15, (cpu->decoded_code.type7.disp << 1) | cpu->decoded_code.type7.gen);

	addr = cpu->reg.r[reg1] + disp;

	err = bus_get_data8(cpu->core_id, addr, (uint8*)&data8);
	if (err != STD_E_OK) {
		return -1;
	}
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: LD.B disp16(%d),r%d(0x%x), r%d(0x%x) addr=0x%x:0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], addr, data8));

	cpu->reg.r[reg2] = data8;

	cpu->reg.pc += 4;
	return 0;
}

int op_exec_ldbu(TargetCoreType *cpu)
{
	uint32 addr;
	sint32 disp;
	uint32 reg1 = cpu->decoded_code.type7.reg1;
	uint32 reg2 = cpu->decoded_code.type7.reg2;
	sint32 disp_bit;
	uint8 data8;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	disp_bit = (cpu->decoded_code.type7.opcode & 0x0001);
	disp = op_sign_extend(15, (cpu->decoded_code.type7.disp << 1) | disp_bit);

	addr = cpu->reg.r[reg1] + disp;

	err = bus_get_data8(cpu->core_id, addr, &data8);
	if (err != STD_E_OK) {
		return -1;
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: LD.BU disp16(%d),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], data8));

	cpu->reg.r[reg2] = data8;

	cpu->reg.pc += 4;
	return 0;
}
int op_exec_ldhw(TargetCoreType *cpu)
{
	uint32 addr;
	sint32 ret;
	sint32 disp;
	uint32 reg1 = cpu->decoded_code.type7.reg1;
	uint32 reg2 = cpu->decoded_code.type7.reg2;
	sint16 data16;
	sint32 data32;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}


	if (cpu->decoded_code.type7.gen == 0x00) {
		//LD.H
		disp = op_sign_extend(15, (cpu->decoded_code.type7.disp << 1) );
		addr = cpu->reg.r[reg1] + disp;

		err = bus_get_data16(cpu->core_id, addr, (uint16*)&data16);
		if (err != STD_E_OK) {
			return -1;
		}
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: LD.H disp16(%d),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], data16));
		ret = data16;
	}
	else {
		//LD.W
		disp = op_sign_extend(15, (cpu->decoded_code.type7.disp << 1) );
		addr = cpu->reg.r[reg1] + disp;

		//TODO: comm_hook_load_reg32(cpu, addr);

		err = bus_get_data32(cpu->core_id, addr, (uint32*)&data32);
		if (err != STD_E_OK) {
			return -1;
		}

		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: LD.W disp16(%d),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], data32));
		ret = data32;

	}
	cpu->reg.r[reg2] = ret;

	cpu->reg.pc += 4;
	return 0;
}

int op_exec_ldhu(TargetCoreType *cpu)
{
	uint32 addr;
	uint32 ret;
	uint32 disp;
	uint32 reg1 = cpu->decoded_code.type7.reg1;
	uint32 reg2 = cpu->decoded_code.type7.reg2;
	uint16 data16;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}

	disp = op_zero_extend(15, (cpu->decoded_code.type7.disp << 1) );
	addr = cpu->reg.r[reg1] + disp;

	err = bus_get_data16(cpu->core_id, addr, &data16);
	if (err != STD_E_OK) {
		return -1;
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: LD.HU disp16(%d),r%d(0x%x), r%d(0x%x):0x%x\n", cpu->reg.pc, disp, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], data16));
	ret = data16;

	cpu->reg.r[reg2] = ret;

	cpu->reg.pc += 4;
	return 0;
}
