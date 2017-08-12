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
				}
		},
};

void cpu_init(void)
{
	cpu_reset(CPU_CONFIG_CORE_ID_0);
	return;
}

static void private_cpu_reset(TargetCoreType *cpu)
{
	cpu->reg.pc = 0x00;
	cpu->reg.r[0] = 0;

	cpu->reg.eipc = 0;
	cpu->reg.eipsw = 0;
	cpu->reg.fepc = 0;
	cpu->reg.fepsw = 0;
	cpu->reg.ecr = 0;
	cpu->reg.psw = 0x20;
	cpu->reg.ctbp = 0;
	cpu->is_halt = FALSE;
	return;
}

void cpu_reset(CoreIdType core_id)
{
	private_cpu_reset(&virtual_cpu.cores[core_id].core);
	return;
}

Std_ReturnType cpu_supply_clock(CoreIdType core_id)
{
	int ret;
	Std_ReturnType err;

	virtual_cpu.current_core = &virtual_cpu.cores[core_id];

	if (virtual_cpu.cores[core_id].core.is_halt == TRUE) {
		return STD_E_OK;
	}

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
			&virtual_cpu.cores[core_id].core.decoded_code);
	if (ret < 0) {
		printf("Decode Error\n");
		return STD_E_DECODE;
	}

	/*
	 * 命令実行
	 */
	ret = OpExec(&virtual_cpu.cores[core_id].core);
	if (ret < 0) {
		printf("Exec Error code[0]=0x%x code[1]=0x%x type_id=0x%x\n",
				virtual_cpu.cores[core_id].core.current_code[0],
				virtual_cpu.cores[core_id].core.current_code[1],
				virtual_cpu.cores[core_id].core.decoded_code.type_id);
		return STD_E_EXEC;
	}

	return STD_E_OK;
}

void cpu_illegal_opcode_trap(CoreIdType core_id)
{
	uint32 eicc;
	uint32 ecr;

	eicc = 0x60;
	virtual_cpu.cores[core_id].core.reg.eipc = virtual_cpu.cores[core_id].core.reg.pc - 4;
	virtual_cpu.cores[core_id].core.reg.eipsw = virtual_cpu.cores[core_id].core.reg.psw;
	ecr = virtual_cpu.cores[core_id].core.reg.ecr;
	ecr = ecr & 0x00FF;
	ecr |= (eicc << 16);
	virtual_cpu.cores[core_id].core.reg.ecr = ecr;
	CPU_SET_NP(&virtual_cpu.cores[core_id].core.reg);
	CPU_SET_EP(&virtual_cpu.cores[core_id].core.reg);
	CPU_SET_ID(&virtual_cpu.cores[core_id].core.reg);
	virtual_cpu.cores[core_id].core.reg.pc = 0x60;

	return;
}
