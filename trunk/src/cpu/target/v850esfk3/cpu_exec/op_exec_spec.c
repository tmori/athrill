#include "cpu_exec/op_exec_ops.h"
#include "cpu.h"
#include "bus.h"
#include "device.h"
#include <stdio.h> //TODO

static int get_sysreg(CpuRegisterType *cpu, uint32 regid, uint32 **regp)
{
	int ret = 0;
	switch (regid) {
	case 0:
		*regp = &cpu->eipc;
		break;
	case 1:
		*regp = &cpu->eipsw;
		break;
	case 2:
		*regp = &cpu->fepc;
		break;
	case 3:
		*regp = &cpu->fepsw;
		break;
	case 4:
		*regp = &cpu->ecr;
		break;
	case 5:
		*regp = &cpu->psw;
		break;
	case 20:
		*regp = &cpu->ctbp;
		break;
	default:
		ret =-1;
		break;
	}
	return ret;
}

/*
 * Format9
 */
int op_exec_ldsr(TargetCoreType *cpu)
{
	int ret;
	uint32 *sysreg;
	/*
	 * [ユーザーズマニュアルから抜粋]
	 * 注意 この命令では，ニモニック記述の都合上，ソース・レジスタを reg2としていますが，
	 * オペコード上はreg1のフィールドを使用しています。したがって，ニモニック記述と
	 * オペコードにおいて，レジスタ指定の意味付けがほかの命令と異なります。
	 * rrrrr： regID指定
	 * RRRRR： reg2指定
	 */
	uint32 regid = cpu->decoded_code.type9.reg2;
	uint32 reg2 = cpu->decoded_code.type9.gen;

	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	if (regid >= CPU_SYSREG_NUM) {
		return -1;
	}
	ret = get_sysreg(&cpu->reg, regid, &sysreg);
	if (ret < 0) {
		return -1;
	}
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: LDSR r%d(0x%x) regID(%d):0x%x\n", cpu->reg.pc, reg2, cpu->reg.r[reg2], regid, *sysreg));
	*sysreg = cpu->reg.r[reg2];

	cpu->reg.pc += 4;

	return 0;
}

int op_exec_stsr(TargetCoreType *cpu)
{
	int ret;
	uint32 *sysreg;
	uint32 regid = cpu->decoded_code.type9.gen;
	uint32 reg2 = cpu->decoded_code.type9.reg2;

	if (reg2 >= CPU_GREG_NUM) {
		return -1;
	}
	if (regid >= CPU_SYSREG_NUM) {
		return -1;
	}
	ret = get_sysreg(&cpu->reg, regid, &sysreg);
	if (ret < 0) {
		return -1;
	}
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: STSR regID(%d) r%d(0x%x):0x%x\n", cpu->reg.pc, regid, reg2, cpu->reg.r[reg2], *sysreg));
	cpu->reg.r[reg2] = *sysreg;

	cpu->reg.pc += 4;
	return 0;
}

/*
 * Format10
 */


int op_exec_diei(TargetCoreType *cpu)
{
	if (cpu->decoded_code.type10.gen1 == 0x04) {
		/* EI */
		CPU_CLR_ID(&cpu->reg);
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: EI\n", cpu->reg.pc));
	}
	else {
		/* DI */
		CPU_SET_ID(&cpu->reg);
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: DI\n", cpu->reg.pc));

	}
	cpu->reg.pc += 4;

	return 0;
}

int op_exec_nop(TargetCoreType *cpu)
{
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: NOP\n", cpu->reg.pc));

	cpu->reg.pc += 2;

	return 0;
}
int op_exec_reti(TargetCoreType *cpu)
{
	if (CPU_ISSET_EP(&cpu->reg)) {
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: RETI:0x%x\n", cpu->reg.pc, cpu->reg.eipc));
		cpu->reg.pc = cpu->reg.eipc;
		cpu->reg.psw = cpu->reg.eipsw;
		//CPU例外の場合は，ISPRの設定は行わないため不要
		//intc_clr_currlvl_ispr(cpu);
	}
	else if (CPU_ISSET_NP(&cpu->reg)) {
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: RETI:0x%x\n", cpu->reg.pc, cpu->reg.fepc));
		cpu->reg.pc = cpu->reg.fepc;
		cpu->reg.psw = cpu->reg.fepsw;
		intc_clr_nmi(cpu);
	}
	else {
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: RETI:0x%x\n", cpu->reg.pc, cpu->reg.eipc));
		//printf("0x%x: RETI:0x%x\n", cpu->cpu.pc, cpu->cpu.eipc);
		//fflush(stdout);
		cpu->reg.pc = cpu->reg.eipc;
		cpu->reg.psw = cpu->reg.eipsw;
		//intc_clr_currlvl_ispr(cpu);
	}
	return 0;
}
int op_exec_halt(TargetCoreType *cpu)
{
	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: HALT:0x%x\n", cpu->reg.pc, cpu->reg.pc + 4));
	//printf("0x%x: HALT:0x%x\n", cpu->reg.pc, cpu->reg.pc + 4);
	//fflush(stdout);
	cpu->is_halt = TRUE;
	cpu->reg.pc += 4;
	return 0;
}


int op_exec_trap(TargetCoreType *cpu)
{
	int ret = -1;
	uint32 pc;
	uint32 eicc;
	uint32 ecr;
	uint32 vector = cpu->decoded_code.type10.gen2;

	if (vector <= 0x0F) {
		ret = 0;
		pc = 0x40;
		eicc = 0x40;
	}
	else if (vector <= 0x1F) {
		ret = 0;
		pc = 0x50;
		eicc = 0x50;
	}

	if (ret == 0) {
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: TRAP:0x%x\n", cpu->reg.pc, pc));
		cpu->reg.eipc = cpu->reg.pc + 4;
		cpu->reg.eipsw = cpu->reg.psw;
		ecr = cpu->reg.ecr;
		ecr = ecr & 0x00FF;
		ecr |= (eicc << 16);
		cpu->reg.ecr = ecr;
		CPU_SET_EP(&cpu->reg);
		CPU_SET_ID(&cpu->reg);
		cpu->reg.pc = pc;
	}

	return 0;
}

int op_exec_switch(TargetCoreType *cpu)
{
	uint32 reg1 = cpu->decoded_code.type1.reg1;
	uint32 reg1_data;
	uint32 addr;
	sint32 tmp_pc;
	uint32 next_pc;
	sint16 data16;
	Std_ReturnType err;

	if (reg1 >= CPU_GREG_NUM) {
		return -1;
	}

	reg1_data = cpu->reg.r[reg1];

	addr = (cpu->reg.pc + 2U) + (reg1_data << 1U);
	/*
	 * Load-memory (adr, Half-word)
	 */
	err = bus_get_data16(cpu->core_id, addr, (uint16*)&data16);
	if (err != STD_E_OK) {
		printf("ERROR:SWITCH pc=0x%x reg1=%u(0x%x) addr=0x%x\n", cpu->reg.pc, reg1, reg1_data, addr);
		return -1;
	}
	/*
	 * (sign-extend (Load-memory (adr, Half-word) ))
	 */
	tmp_pc = (sint32)( data16 );
	/*
	 * (sign-extend (Load-memory (adr, Half-word) ) ) logically shift left by 1
	 */
	tmp_pc <<= 1U;

	/*
	 * (PC + 2) + (sign-extend (Load-memory (adr, Half-word) ) ) logically shift left by 1
	 */
	next_pc = (cpu->reg.pc + 2U) + ((uint32)tmp_pc);

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: SWITCH r%d(%d):%d\n", cpu->reg.pc, reg1, cpu->reg.r[reg1], next_pc));



	cpu->reg.pc = next_pc;
	return 0;
}

/*
 * Format13
 */
int op_exec_prepare(TargetCoreType *cpu)
{
	uint16 subop = cpu->decoded_code.type13.gen & 0x0007;
	uint16 ff = cpu->decoded_code.type13.gen >> 3U;
	uint16 start_reg = 20U;
	uint16 i;
	uint32 addr;
	uint32 *addrp;
	uint32 *sp = (uint32*)&(cpu->reg.r[3]);	//sp:r3
	uint32 imm = ( cpu->decoded_code.type13.imm << 2U );
	Std_ReturnType err;

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: PREPARE sp=0x%x ", cpu->reg.pc, *sp));
	for (i = start_reg; i < 32; i++) {
		if (cpu->decoded_code.type13.list[i] == 0) {
			continue;
		}
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "r%u(0x%x) ", i, cpu->reg.r[i]));

		addr = (*sp) - 4U;
		err = bus_get_pointer(cpu->core_id, addr, (uint8**)&addrp);
		if (err != STD_E_OK) {
			printf("ERROR:PREPARE pc=0x%x sp=0x%x\n", cpu->reg.pc, *sp);
			return -1;
		}
		*addrp = cpu->reg.r[i];
		*sp = addr;
	}
	*sp = (*sp) - imm;

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "imm5(%u) ", imm));

	if (subop == 1U) {
		cpu->reg.pc += 4;
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), ":sp=0x%x\n", *sp));
		return 0;
	}

	addr = cpu->reg.pc + 4U;

	err = bus_get_pointer(cpu->core_id, addr, (uint8**)&addrp);
	if (err != STD_E_OK) {
		printf("ERROR:PREPARE pc=0x%x sp=0x%x\n", cpu->reg.pc, *sp);
		return -1;
	}

	switch (ff) {
	case 0b00:
		cpu->reg.r[30] = *sp;
		cpu->reg.pc += 4;
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "ep=0x%x\n", cpu->reg.r[30]));
		break;
	case 0b01:
		cpu->reg.r[30] = (sint32)(*((sint16*)addrp));
		cpu->reg.pc += 6;
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "ep=0x%x\n", cpu->reg.r[30]));
		break;
	case 0b10:
		cpu->reg.r[30] = ((uint32)(*((uint16*)addrp))) << 16U;
		cpu->reg.pc += 6;
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "ep=0x%x\n", cpu->reg.r[30]));
		break;
	case 0b11:
		cpu->reg.r[30] = (*((uint32*)addrp));
		cpu->reg.pc += 8;
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "ep=0x%x\n", cpu->reg.r[30]));
		break;
	default:
		printf("ERROR:PREPARE pc=0x%x sp=0x%x\n", cpu->reg.pc, *sp);
		return -1;
	}
	return 0;
}

int op_exec_dispose(TargetCoreType *cpu)
{
	uint16 reg1 = cpu->decoded_code.type13.gen;
	uint16 start_reg = 20U;
	uint16 i;
	uint32 addr;
	uint32 *addrp;
	uint32 *sp = (uint32*)&(cpu->reg.r[3]);	//sp:r3
	uint32 imm = ( cpu->decoded_code.type13.imm << 2U );
	Std_ReturnType err;

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: DISPOSE imm=0x%x sp=0x%x ", cpu->reg.pc, imm, *sp));

	*sp = (*sp) + imm;
	for (i = 31; i >= start_reg; i--) {
		if (cpu->decoded_code.type13.list[i] == 0) {
			continue;
		}

		addr = (*sp);
		err = bus_get_pointer(cpu->core_id, addr, (uint8**)&addrp);
		if (err != STD_E_OK) {
			printf("ERROR:DISPOSE pc=0x%x sp=0x%x\n", cpu->reg.pc, *sp);
			return -1;
		}
		cpu->reg.r[i] = *addrp;
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "r%u(0x%x) ", i, cpu->reg.r[i]));
		*sp = addr + 4;
	}

	if (reg1 != 0U) {
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), ":pc=r%u(0x%x) sp=0x%x\n", reg1, cpu->reg.r[reg1], cpu->reg.r[3]));
		cpu->reg.pc = cpu->reg.r[reg1];
	}
	else {
		cpu->reg.pc += 4;
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), ":pc=r%u(0x%x) sp=0x%x\n", reg1, cpu->reg.pc, cpu->reg.r[3]));
	}

	return 0;
}
