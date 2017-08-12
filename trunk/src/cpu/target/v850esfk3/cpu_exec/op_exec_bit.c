#include "../cpu_exec/op_exec_ops.h"
#include "cpu.h"
#include "bus.h"


/*
 * Format8
 */
int op_exec_tst1_8(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type8.reg1;
	sint32 disp16 = cpu->decoded_code.type8.disp;
	sint32 bit3 = cpu->decoded_code.type8.bit;
	uint32 addr;
	uint8 bit;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}

	addr = cpu->reg.r[reg1] + disp16;

	err = bus_get_data8(cpu->core_id, addr, &bit);
	if (err != STD_E_OK) {
		return -1;
	}


	if ((bit & (1 << bit3)) == (1 << bit3)) {
		CPU_CLR_Z(&cpu->reg);
	}
	else {
		CPU_SET_Z(&cpu->reg);
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: TST1 bit#3(%d), disp16(%d),r%d(0x%x):psw=0x%x\n", cpu->reg.pc, bit3, disp16, reg1, cpu->reg.r[reg1], cpu->reg.psw));

	cpu->reg.pc += 4;

	return 0;
}
int op_exec_set1_8(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type8.reg1;
	sint32 disp16 = cpu->decoded_code.type8.disp;
	sint32 bit3 = cpu->decoded_code.type8.bit;
	uint32 addr;
	uint8 org_bit;
	uint8 bit;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}

	addr = cpu->reg.r[reg1] + disp16;

	err = bus_get_data8(cpu->core_id, addr, &bit);
	if (err != STD_E_OK) {
		return -1;
	}


	org_bit = bit;

	bit |= (1 << bit3);
	err = bus_put_data8(cpu->core_id, addr, bit);
	if (err != STD_E_OK) {
		return -1;
	}

	if (((org_bit) & (1 << bit3)) == (1 << bit3)) {
		CPU_CLR_Z(&cpu->reg);
	}
	else {
		CPU_SET_Z(&cpu->reg);
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SET1 bit#3(%d), disp16(%d), addr=0x%x r%d(0x%x):psw=0x%x, bit=0x%x\n", cpu->reg.pc, bit3, disp16, addr, reg1, cpu->reg.r[reg1], cpu->reg.psw, bit));

	cpu->reg.pc += 4;

	return 0;
}

int op_exec_clr1_8(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type8.reg1;
	sint32 disp16 = cpu->decoded_code.type8.disp;
	sint32 bit3 = cpu->decoded_code.type8.bit;
	uint32 addr;
	uint8 org_bit;
	uint8 bit;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}

	addr = cpu->reg.r[reg1] + disp16;
	err = bus_get_data8(cpu->core_id, addr, &bit);
	if (err != STD_E_OK) {
		return -1;
	}

	org_bit = bit;

	bit &= ~(1 << bit3);
	err = bus_put_data8(cpu->core_id, addr, bit);
	if (err != STD_E_OK) {
		return -1;
	}

	if (((org_bit) & (1 << bit3)) == (1 << bit3)) {
		CPU_CLR_Z(&cpu->reg);
	}
	else {
		CPU_SET_Z(&cpu->reg);
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: CLR1 bit#3(%d), disp16(%d), addr=0x%x r%d(0x%x):psw=0x%x, bit=0x%x\n", cpu->reg.pc, bit3, disp16, addr, reg1, cpu->reg.r[reg1], cpu->reg.psw, bit));

	cpu->reg.pc += 4;

	return 0;
}

int op_exec_not1_8(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type8.reg1;
	sint32 disp16 = cpu->decoded_code.type8.disp;
	sint32 bit3 = cpu->decoded_code.type8.bit;
	uint32 addr;
	uint8 org_bit;
	uint8 bit;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}

	addr = cpu->reg.r[reg1] + disp16;
	err = bus_get_data8(cpu->core_id, addr, &bit);
	if (err != STD_E_OK) {
		return -1;
	}

	org_bit = bit;


	if (((org_bit) & (1 << bit3)) == (1 << bit3)) {
		CPU_CLR_Z(&cpu->reg);
		bit &= ~(1 << bit3);
	}
	else {
		CPU_SET_Z(&cpu->reg);
		bit |= (1 << bit3);
	}
	err = bus_put_data8(cpu->core_id, addr, bit);
	if (err != STD_E_OK) {
		return -1;
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(),
			"0x%x: NOT1 bit#3(%d), disp16(%d), addr=0x%x r%d(0x%x):psw=0x%x, bit=0x%x\n",
			cpu->reg.pc, bit3, disp16, addr, reg1, cpu->reg.r[reg1], cpu->reg.psw, bit));

	cpu->reg.pc += 4;

	return 0;
}


/*
 * Format9
 */
int op_exec_set1_9(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type9.gen;
	sint32 reg2 = cpu->decoded_code.type9.reg2;
	uint32 addr;
	uint8 org_bit;
	uint8 bit3;
	uint8 bit;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	/*
	 * （ 2） adr ← GR [reg1]
	 * Zフラグ ← Not (Load-memory-bit (adr, reg2) )
	 * Store-memory-bit (adr, reg2, 1)
	 */

	addr = cpu->reg.r[reg1];
	err = bus_get_data8(cpu->core_id, addr, &bit);
	if (err != STD_E_OK) {
		return -1;
	}

	org_bit = bit;

	bit3 = (cpu->reg.r[reg2] & 0x07);
	bit |= (1 << bit3);
	err = bus_put_data8(cpu->core_id, addr, bit);
	if (err != STD_E_OK) {
		return -1;
	}

	if (((org_bit) & (1 << bit3)) == (1 << bit3)) {
		CPU_CLR_Z(&cpu->reg);
	}
	else {
		CPU_SET_Z(&cpu->reg);
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(),
			"0x%x: SET1 bit#3(%d), addr=0x%x r%d(0x%x) r%d(0x%x):psw=0x%x, bit=0x%x\n",
			cpu->reg.pc,
			bit3, addr, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2],cpu->reg.psw, bit));

	cpu->reg.pc += 4;

	return 0;
}
int op_exec_clr1_9(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type9.gen;
	sint32 reg2 = cpu->decoded_code.type9.reg2;
	uint32 addr;
	uint8 bit;
	Std_ReturnType err;
	uint8 org_bit;
	uint8 bit3;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	/*
	 * （ 2） adr ← GR [reg1]
	 * Zフラグ ← Not (Load-memory-bit (adr, reg2) )
	 * Store-memory-bit (adr, reg2, 0)
	 */

	addr = cpu->reg.r[reg1];
	err = bus_get_data8(cpu->core_id, addr, &bit);
	if (err != STD_E_OK) {
		return -1;
	}
	org_bit = bit;

	bit3 = (cpu->reg.r[reg2] & 0x07);
	bit &= ~(1 << bit3);
	err = bus_put_data8(cpu->core_id, addr, bit);
	if (err != STD_E_OK) {
		return -1;
	}
	if (((org_bit) & (1 << bit3)) == (1 << bit3)) {
		CPU_CLR_Z(&cpu->reg);
	}
	else {
		CPU_SET_Z(&cpu->reg);
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(),
			"0x%x: CLR1 bit#3(%d), addr=0x%x r%d(0x%x) r%d(0x%x):psw=0x%x, bit=0x%x\n",
			cpu->reg.pc,
			bit3, addr, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2],cpu->reg.psw, bit));

	cpu->reg.pc += 4;

	return 0;
}

int op_exec_tst1_9(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type9.gen;
	sint32 reg2 = cpu->decoded_code.type9.reg2;
	uint32 addr;
	Std_ReturnType err;
	uint8 bit3;
	uint8 bit;


	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	addr = cpu->reg.r[reg1];

	err = bus_get_data8(cpu->core_id, addr, &bit);
	if (err != STD_E_OK) {
		return -1;
	}

	bit3 = (cpu->reg.r[reg2] & 0x07);

	if ((bit & (1 << bit3)) == (1 << bit3)) {
		CPU_CLR_Z(&cpu->reg);
	}
	else {
		CPU_SET_Z(&cpu->reg);
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(),
			"0x%x: TST1 bit#3(%d), r%d(0x%x),r%d(0x%x):psw=0x%x\n",
			cpu->reg.pc, bit3, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], cpu->reg.psw));

	cpu->reg.pc += 4;

	return 0;
}
int op_exec_not1_9(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type9.gen;
	sint32 reg2 = cpu->decoded_code.type9.reg2;
	uint32 addr;
	Std_ReturnType err;
	uint8 bit;
	uint8 org_bit;
	uint8 bit3;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}
	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	/*
	 * （ 2） adr ← GR [reg1]
	 * Zフラグ ← Not (Load-memory-bit (adr, reg2) )
	 * Store-memory-bit (adr, reg2, Zフラグ)
	 */
	addr = cpu->reg.r[reg1];
	err = bus_get_data8(cpu->core_id, addr, &bit);
	if (err != STD_E_OK) {
		return -1;
	}

	org_bit = bit;

	bit3 = (cpu->reg.r[reg2] & 0x07);

	if (((org_bit) & (1 << bit3)) == (1 << bit3)) {
		CPU_CLR_Z(&cpu->reg);
		bit &= ~(1 << bit3);
	}
	else {
		CPU_SET_Z(&cpu->reg);
		bit |= (1 << bit3);
	}
	err = bus_put_data8(cpu->core_id, addr, bit);
	if (err != STD_E_OK) {
		return -1;
	}

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(),
			"0x%x: NOT1 bit#3(%d), addr=0x%x r%d(0x%x) r%d(0x%x):psw=0x%x, bit=0x%x\n",
			cpu->reg.pc,
			bit3, addr, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2],cpu->reg.psw, bit));

	cpu->reg.pc += 4;

	return 0;
}

