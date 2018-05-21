#include "cpu.h"
#include "bus.h"
#include "std_cpu_ops.h"
#include <stdio.h>
#include "cpu_common/cpu_ops.h"
#include "cpu_common/op_exec.h"
#include "cpu_dec/op_dec.h"

CpuType virtual_cpu = {
	.cores = {
		/*
		 * INDEX 0
		 */
		{
			.core.core_id	=	CPU_CONFIG_CORE_ID_0,
		},
		/*
		 * INDEX 1
		 */
		{
			.core.core_id	=	CPU_CONFIG_CORE_ID_1,
		}
	},
};

void cpu_init(void)
{
	cpu_reset(CPU_CONFIG_CORE_ID_0);
	cpu_reset(CPU_CONFIG_CORE_ID_1);
	return;
}

static void private_cpu_reset(TargetCoreType *cpu)
{
	uint32 *sysreg;
	cpu->reg.pc = 0x00;
	cpu->reg.r[0] = 0;

	cpu->reg.sys.current_grp = SYS_GRP_CPU;
	cpu->reg.sys.current_bnk = SYS_GRP_CPU_BNK_0;
	for (int regId = 0; regId < CPU_GREG_NUM; regId++) {
		sysreg = cpu_get_sysreg(&cpu->reg.sys, regId);
		*sysreg = 0;
	}
	sys_get_cpu_base(&cpu->reg)->r[SYS_REG_PSW] = 0x20;

	cpu->is_halt = FALSE;
	return;
}

void cpu_reset(CoreIdType core_id)
{
	private_cpu_reset(&virtual_cpu.cores[core_id].core);
	return;
}
bool cpu_is_halt(CoreIdType core_id)
{
	return virtual_cpu.cores[core_id].core.is_halt;
}
void cpu_set_current_core(CoreIdType core_id)
{
	virtual_cpu.current_core = &virtual_cpu.cores[core_id];
	return;
}

Std_ReturnType cpu_supply_clock(CoreIdType core_id)
{
	int ret;
	Std_ReturnType err;
	uint32 inx;
	CachedOperationCodeType *cached_code;

	if (virtual_cpu.cores[core_id].core.is_halt == TRUE) {
		return STD_E_OK;
	}
	cached_code = virtual_cpu_get_cached_code(virtual_cpu.cores[core_id].core.reg.pc);
	inx = virtual_cpu.cores[core_id].core.reg.pc - cached_code->code_start_addr;
	if (cached_code->codes[inx].op_exec == NULL) {
		/*
		 * 命令取得する
		 */
		err = bus_get_data32(core_id,
				virtual_cpu.cores[core_id].core.reg.pc,
				(uint32*)virtual_cpu.cores[core_id].core.current_code);
		if (err != STD_E_OK) {
			return err;
		}

		/*
		 * デコード
		 */
		ret = OpDecode(virtual_cpu.cores[core_id].core.current_code,
				&cached_code->codes[inx].decoded_code);
		if (ret < 0) {
			printf("Decode Error\n");
			return STD_E_DECODE;
		}
		virtual_cpu.cores[core_id].core.decoded_code = &cached_code->codes[inx].decoded_code;
		virtual_cpu.cores[core_id].core.op_exec = NULL;
		/*
		 * 命令実行
		 */
		ret = OpExec(&virtual_cpu.cores[core_id].core);
		if (ret < 0) {
			printf("Exec Error code[0]=0x%x code[1]=0x%x type_id=0x%x\n",
					virtual_cpu.cores[core_id].core.current_code[0],
					virtual_cpu.cores[core_id].core.current_code[1],
					virtual_cpu.cores[core_id].core.decoded_code->type_id);
			return STD_E_EXEC;
		}
		cached_code->codes[inx].op_exec = virtual_cpu.cores[core_id].core.op_exec;
	}
	else {
		virtual_cpu.cores[core_id].core.decoded_code = &cached_code->codes[inx].decoded_code;
		ret = cached_code->codes[inx].op_exec(&virtual_cpu.cores[core_id].core);
		if (ret < 0) {
			printf("Exec Error code[0]=0x%x code[1]=0x%x type_id=0x%x\n",
					virtual_cpu.cores[core_id].core.current_code[0],
					virtual_cpu.cores[core_id].core.current_code[1],
					virtual_cpu.cores[core_id].core.decoded_code->type_id);
			return STD_E_EXEC;
		}

	}

	return STD_E_OK;
}

void cpu_illegal_opcode_trap(CoreIdType core_id)
{
	uint32 eicc;
	uint32 ecr;

	eicc = 0x60;
	sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_EIPC] = virtual_cpu.cores[core_id].core.reg.pc - 4;
	sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_EIPSW] = sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_PSW];

	ecr = sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_ECR];
	ecr = ecr & 0x00FF;
	ecr |= (eicc << 16);
	sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_ECR] = ecr;
	CPU_SET_NP(&virtual_cpu.cores[core_id].core.reg);
	CPU_SET_EP(&virtual_cpu.cores[core_id].core.reg);
	CPU_SET_ID(&virtual_cpu.cores[core_id].core.reg);
	virtual_cpu.cores[core_id].core.reg.pc = 0x60;

	return;
}
