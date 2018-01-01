#include "cpu_exec/op_exec_ops.h"
#include "cpu.h"
#include "bus.h"
#include "device.h"
#include <stdio.h> //TODO

static int get_sysreg(CpuRegisterType *cpu, uint32 regid, uint32 **regp)
{
	if (regid >= CPU_GREG_NUM) {
		return -1;
	}

	*regp = cpu_get_sysreg(&cpu->sys, regid);

	return 0;
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
		printf("ERROR: ldsr reg=%d regID=%d\n", reg2, regid);
		return -1;
	}
	if (regid >= CPU_GREG_NUM) {
		printf("ERROR: ldsr reg=%d regID=%d\n", reg2, regid);
		return -1;
	}
	ret = get_sysreg(&cpu->reg, regid, &sysreg);
	if (ret < 0) {
		printf("ERROR: ldsr reg=%d regID=%d\n", reg2, regid);
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
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: RETI:0x%x\n", cpu->reg.pc, sys_get_cpu_base(&cpu->reg)->r[SYS_REG_EIPC]));
		cpu->reg.pc = sys_get_cpu_base(&cpu->reg)->r[SYS_REG_EIPC];
		sys_get_cpu_base(&cpu->reg)->r[SYS_REG_PSW] = sys_get_cpu_base(&cpu->reg)->r[SYS_REG_EIPSW];
		//CPU例外の場合は，ISPRの設定は行わないため不要
		//intc_clr_currlvl_ispr(cpu);
	}
	else if (CPU_ISSET_NP(&cpu->reg)) {
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: RETI:0x%x\n", cpu->reg.pc, sys_get_cpu_base(&cpu->reg)->r[SYS_REG_FEPC]));
		cpu->reg.pc = sys_get_cpu_base(&cpu->reg)->r[SYS_REG_FEPC];
		sys_get_cpu_base(&cpu->reg)->r[SYS_REG_PSW] = sys_get_cpu_base(&cpu->reg)->r[SYS_REG_FEPSW];
		intc_clr_nmi(cpu);
	}
	else {
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: RETI:0x%x\n", cpu->reg.pc, sys_get_cpu_base(&cpu->reg)->r[SYS_REG_EIPC]));
		//printf("0x%x: RETI:0x%x\n", cpu->cpu.pc, cpu->cpu.eipc);
		//fflush(stdout);
		cpu->reg.pc = sys_get_cpu_base(&cpu->reg)->r[SYS_REG_EIPC];
		sys_get_cpu_base(&cpu->reg)->r[SYS_REG_PSW] = sys_get_cpu_base(&cpu->reg)->r[SYS_REG_EIPSW];
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
		sys_get_cpu_base(&cpu->reg)->r[SYS_REG_EIPC] = cpu->reg.pc + 4;
		sys_get_cpu_base(&cpu->reg)->r[SYS_REG_EIPSW] = sys_get_cpu_base(&cpu->reg)->r[SYS_REG_PSW];

		ecr = sys_get_cpu_base(&cpu->reg)->r[SYS_REG_ECR];
		ecr = ecr & 0x00FF;
		ecr |= (eicc << 16);
		sys_get_cpu_base(&cpu->reg)->r[SYS_REG_ECR] = ecr;
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

/*
 * ［命令形式］ CAXI [reg1], reg2, reg3
 *
 * ［オペレーション］ adr ← GR[reg1]注
 * token ← Load-memory(adr, Word)
 * result ← GR[reg2] – token
 * If result == 0
 * then Store-memory(adr, GR[reg3],Word)
 * GR[reg3] ← token
 * else Store-memory(adr, token,Word)
 * GR[reg3] ← token
 * 注 GR[reg1]の下位 2 ビットは， 0 にマスクしadr とします。
 */
int op_exec_caxi(TargetCoreType *cpu)
{
	Std_ReturnType err;
	uint16 reg1 = cpu->decoded_code.type11.reg1;;
	uint16 reg2 = cpu->decoded_code.type11.reg2;
	uint16 reg3 = cpu->decoded_code.type11.reg3;
	sint16 token;
	sint16 reg2_data;
	sint16 reg3_data;
	sint16 result;
	uint16 put_data;

	uint32 reg1_addr = (cpu->reg.r[reg1] & 0xFFFFFFFC);
	uint32 reg2_addr = cpu->reg.r[reg2];
	uint32 reg3_addr = cpu->reg.r[reg3];

	/*
	 * Load-memory (adr, Half-word)
	 */
	err = bus_get_data16(cpu->core_id, reg1_addr, (uint16*)&token);
	if (err != STD_E_OK) {
		printf("ERROR:CAXI pc=0x%x reg1=%u reg1_addr=%d\n", cpu->reg.pc, reg1, reg1_addr);
		return -1;
	}
	err = bus_get_data16(cpu->core_id, reg2_addr, (uint16*)&reg2_data);
	if (err != STD_E_OK) {
		printf("ERROR:CAXI pc=0x%x reg2=%u reg2_addr=%d\n", cpu->reg.pc, reg2, reg2_addr);
		return -1;
	}
	err = bus_get_data16(cpu->core_id, reg3_addr, (uint16*)&reg3_data);
	if (err != STD_E_OK) {
		printf("ERROR:CAXI pc=0x%x reg3=%u reg3_addr=%d\n", cpu->reg.pc, reg3, reg3_addr);
		return -1;
	}

	result = reg2_data - token;
	if (result == 0) {
		put_data = (uint16)reg3_data;
	}
	else {
		put_data = (uint16)token;
	}
	err = bus_put_data16(cpu->core_id, reg1_addr, put_data);
	if (err != STD_E_OK) {
		return -1;
	}
	cpu->reg.r[reg3] = (sint32)((uint32)((uint16)token));

	op_chk_and_set_borrow(&cpu->reg, reg2_data, token);
	op_chk_and_set_overflow(&cpu->reg, reg2_data, -token);
	op_chk_and_set_zero(&cpu->reg, result);
	op_chk_and_set_sign(&cpu->reg, result);

	DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "0x%x: CAXI r%d(%d),r%d(0x%x), r%d(0x%x):0x%x\n",
			cpu->reg.pc, reg1, cpu->reg.r[reg1], reg2, cpu->reg.r[reg2], reg3, cpu->reg.r[reg3], token));

	cpu->reg.pc += 4;

	return 0;
}

