#include "cpu_common/op_exec.h"
#include "cpu_dec/op_code.h"
#include "cpu_exec/op_exec_ops.h"
#include "cpu.h"
#include "mpu_types.h"
#include "std_errno.h"
#include <stdio.h>

static int OpExec1(TargetCoreType *cpu);
static int OpExec2(TargetCoreType *cpu);
static int OpExec3(TargetCoreType *cpu);
static int OpExec4(TargetCoreType *cpu);
static int OpExec5(TargetCoreType *cpu);
static int OpExec6(TargetCoreType *cpu);
static int OpExec7(TargetCoreType *cpu);
static int OpExec8(TargetCoreType *cpu);
static int OpExec9(TargetCoreType *cpu);
static int OpExec10(TargetCoreType *cpu);
static int OpExec11(TargetCoreType *cpu);
static int OpExec12(TargetCoreType *cpu);
static int OpExec13(TargetCoreType *cpu);

static Std_ReturnType cpu_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType cpu_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);

MpuAddressRegionOperationType cpu_register_operation = {
		.get_data8 = NULL,
		.get_data16 = NULL,
		.get_data32 = cpu_get_data32,
		.put_data8 = NULL,
		.put_data16 = NULL,
		.put_data32 = cpu_put_data32,
};
static uint32 *get_cpu_register_addr(MpuAddressRegionType *region, TargetCoreType *core, uint32 addr)
{
	uint32 inx = (addr - CPU_CONFIG_DEBUG_REGISTER_ADDR) / sizeof(uint32);

	//printf("get_cpu_register_addr:inx=%u\n", inx);
	if (inx >= 0 && inx <= 31) {
		return (uint32*)&core->reg.r[inx];
	}
	else if (addr == CPU_CONFIG_ADDR_PEID) {
		inx = (addr - CPU_CONFIG_DEBUG_REGISTER_ADDR) * core->core_id;
		return (uint32*)&region->data[inx];
	}
	else if ((addr >= CPU_CONFIG_ADDR_MEV_0) && (addr <= CPU_CONFIG_ADDR_MEV_7)) {
		inx = (addr - CPU_CONFIG_DEBUG_REGISTER_ADDR);
		return (uint32*)&region->data[inx];
	}
	else if ((addr >= CPU_CONFIG_ADDR_MIR_0) && (addr <= CPU_CONFIG_ADDR_MIR_1)) {
		inx = (addr - CPU_CONFIG_DEBUG_REGISTER_ADDR);
		return (uint32*)&region->data[inx];
	}
	return NULL;
}
static Std_ReturnType cpu_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 *registerp = get_cpu_register_addr(region, &virtual_cpu.current_core->core, addr);
	if (registerp == NULL) {
		return STD_E_SEGV;
	}
	else if (addr == CPU_CONFIG_ADDR_PEID) {
		*registerp = (core_id + 1);
	}
	*data = *registerp;
	return STD_E_OK;
}

static Std_ReturnType cpu_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 *registerp = get_cpu_register_addr(region, &virtual_cpu.current_core->core, addr);
	if (registerp == NULL) {
		return STD_E_SEGV;
	}
	else if (addr == CPU_CONFIG_ADDR_PEID) {
		return STD_E_SEGV;
	}
	else if ((addr == CPU_CONFIG_ADDR_MIR_0)) {
		intc_cpu_trigger_interrupt(core_id, CPU_CONFIG_ADDR_MIR_0_INTNO);
		return STD_E_OK;
	}
	else if ((addr == CPU_CONFIG_ADDR_MIR_1)) {
		intc_cpu_trigger_interrupt(core_id, CPU_CONFIG_ADDR_MIR_1_INTNO);
		return STD_E_OK;
	}
	*registerp = data;
	return STD_E_OK;
}


uint32 cpu_get_pc(const TargetCoreType *core)
{
	return core->reg.pc;
}
uint32 cpu_get_ep(const TargetCoreType *core)
{
	return core->reg.r[30];
}
uint32 cpu_get_current_core_id(void)
{
	return ((const TargetCoreType *)virtual_cpu.current_core)->core_id;
}
uint32 cpu_get_current_core_pc(void)
{
	return cpu_get_pc((const TargetCoreType *)virtual_cpu.current_core);
}

uint32 cpu_get_current_core_register(uint32 inx)
{
	return ((TargetCoreType *)virtual_cpu.current_core)->reg.r[inx];
}

uint32 cpu_get_sp(const TargetCoreType *core)
{
	return core->reg.r[3];
}
uint32 cpu_get_current_core_sp(void)
{
	return cpu_get_sp((const TargetCoreType *)virtual_cpu.current_core);
}
uint32 cpu_get_current_core_ep(void)
{
	return cpu_get_ep((const TargetCoreType *)virtual_cpu.current_core);
}


uint32 cpu_get_return_addr(const TargetCoreType *core)
{
	return core->reg.r[31];
}
CoreIdType cpu_get_core_id(const TargetCoreType *core)
{
	return core->core_id;
}

int OpExec(TargetCoreType *cpu)
{
	int ret = -1;
	switch (cpu->decoded_code->type_id) {
	case OP_CODE_FORMAT_1:
		ret = OpExec1(cpu);
		break;
	case OP_CODE_FORMAT_2:
		ret = OpExec2(cpu);
		break;
	case OP_CODE_FORMAT_3:
		ret = OpExec3(cpu);
		break;
	case OP_CODE_FORMAT_4:
		ret = OpExec4(cpu);
		break;
	case OP_CODE_FORMAT_5:
		ret = OpExec5(cpu);
		break;
	case OP_CODE_FORMAT_6:
		ret = OpExec6(cpu);
		break;
	case OP_CODE_FORMAT_7:
		ret = OpExec7(cpu);
		break;
	case OP_CODE_FORMAT_8:
		ret = OpExec8(cpu);
		break;
	case OP_CODE_FORMAT_9:
		ret = OpExec9(cpu);
		break;
	case OP_CODE_FORMAT_10:
		ret = OpExec10(cpu);
		break;
	case OP_CODE_FORMAT_11:
		ret = OpExec11(cpu);
		break;
	case OP_CODE_FORMAT_12:
		ret = OpExec12(cpu);
		break;
	case OP_CODE_FORMAT_13:
		ret = OpExec13(cpu);
		break;
	default:
		break;
	}
	cpu->reg.r[0] = 0U;
	return ret;
}

static int OpExec1(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type1.opcode) {
	case OP_CODE_ADD_1:
		cpu->op_exec = op_exec_add_1;
		ret = op_exec_add_1(cpu);
		break;
	case OP_CODE_AND:
		cpu->op_exec = op_exec_and;
		ret = op_exec_and(cpu);
		break;
	case OP_CODE_CMP_1:
		cpu->op_exec = op_exec_cmp_1;
		ret = op_exec_cmp_1(cpu);
		break;
	case OP_CODE_JMP:
		cpu->op_exec = op_exec_jmp;
		ret = op_exec_jmp(cpu);
		break;
	case OP_CODE_MOV_1:
		if (cpu->decoded_code->type1.reg2 > 0) {
			cpu->op_exec = op_exec_mov_1;
			ret = op_exec_mov_1(cpu);
		}
		else {
			cpu->op_exec = op_exec_nop;
			ret = op_exec_nop(cpu);
		}
		break;
	case OP_CODE_MULH_1:
		if (cpu->decoded_code->type1.reg2 > 0) {
			//OP_CODE_MULH_1
			cpu->op_exec = op_exec_mulh_1;
			ret = op_exec_mulh_1(cpu);
		}
		else {
			//OP_CODE_SXH
			cpu->op_exec = op_exec_sxh;
			ret = op_exec_sxh(cpu);
		}
		break;
	case OP_CODE_NOT:
		cpu->op_exec = op_exec_not;
		ret = op_exec_not(cpu);
		break;
	case OP_CODE_SATADD_1:
/*	case OP_CODE_ZXH: */
		if (cpu->decoded_code->type1.reg2 > 0) {
			//SATADD
			cpu->op_exec = op_exec_satadd_1;
			ret = op_exec_satadd_1(cpu);
		}
		else {
			//ZXH
			cpu->op_exec = op_exec_zxh;
			ret = op_exec_zxh(cpu);
		}
		break;
	case OP_CODE_SATSUB_1:
/* 	case OP_CODE_SXB: */
		if (cpu->decoded_code->type1.reg2 > 0) {
			//OP_CODE_SATSUB
			cpu->op_exec = op_exec_satsub_1;
			ret = op_exec_satsub_1(cpu);
		}
		else {
			cpu->op_exec = op_exec_sxb;
			ret = op_exec_sxb(cpu);
		}
		break;
	case OP_CODE_SATSUBR:
/*	case OP_CODE_ZXB: */
		if (cpu->decoded_code->type1.reg2 > 0) {
			//SATSUBR
			//TODO
		}
		else {
			cpu->op_exec = op_exec_zxb;
			ret = op_exec_zxb(cpu);
		}
		break;
	case OP_CODE_SUB:
		cpu->op_exec = op_exec_sub;
		ret = op_exec_sub(cpu);
		break;
	case OP_CODE_SUBR:
		cpu->op_exec = op_exec_subr;
		ret = op_exec_subr(cpu);
		break;
	case OP_CODE_XOR:
		cpu->op_exec = op_exec_xor;
		ret = op_exec_xor(cpu);
		break;
	case OP_CODE_OR:
		cpu->op_exec = op_exec_or;
		ret = op_exec_or(cpu);
		break;
	case OP_CODE_TST:
		cpu->op_exec = op_exec_tst;
		ret = op_exec_tst(cpu);
		break;
	case OP_CODE_SWITCH:
		if ((cpu->decoded_code->type1.reg1 > 0) && (cpu->decoded_code->type1.reg2 == 0)) {
			cpu->op_exec = op_exec_switch;
			ret = op_exec_switch(cpu);
		}
		else if ((cpu->decoded_code->type1.reg1 == 0) && (cpu->decoded_code->type1.reg2 > 0)) {
			printf("OpExec1 Error:Unknown OP:0x%x\n", cpu->decoded_code->type1.opcode);
		}
		else {
			cpu->op_exec = op_exec_divh_1;
			ret = op_exec_divh_1(cpu);
		}
		break;
	default:
		printf("OpExec1 Error:Unknown OP:0x%x\n", cpu->decoded_code->type1.opcode);
		break;
	}
	return ret;
}
static int OpExec2(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type2.opcode) {
	case OP_CODE_ADD_2:
		cpu->op_exec = op_exec_add_2;
		ret = op_exec_add_2(cpu);
		break;
	case OP_CODE_CMP_2:
		cpu->op_exec = op_exec_cmp_2;
		ret = op_exec_cmp_2(cpu);
		break;
	case OP_CODE_MOV_2:
		if (cpu->decoded_code->type2.reg2 != 0) {
			cpu->op_exec = op_exec_mov_2;
			ret = op_exec_mov_2(cpu);
		}
		else {
			//TODO CALLT
		}
		break;
	case OP_CODE_MULH_2:
		cpu->op_exec = op_exec_mulh_2;
		ret = op_exec_mulh_2(cpu);
		break;
	case OP_CODE_SAR_2:
		cpu->op_exec = op_exec_sar_2;
		ret = op_exec_sar_2(cpu);
		break;
	case OP_CODE_SATADD_2:
		cpu->op_exec = op_exec_satadd_2;
		ret = op_exec_satadd_2(cpu);
		break;
	case OP_CODE_SHL_2:
		cpu->op_exec = op_exec_shl_2;
		ret = op_exec_shl_2(cpu);
		break;
	case OP_CODE_SHR_2:
		cpu->op_exec = op_exec_shr_2;
		ret = op_exec_shr_2(cpu);
		break;
	default:
		printf("OpExec2 Error:Unknown OP:0x%x\n", cpu->decoded_code->type2.opcode);
		break;
	}
	return ret;
}
static int OpExec3(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type3.opcode) {
	case OP_CODE_BCOND:
		cpu->op_exec = op_exec_bcond;
		ret = op_exec_bcond(cpu);
		break;
	default:
		printf("OpExec3 Error:Unknown OP:0x%x\n", cpu->decoded_code->type3.opcode);
		break;
	}
	return ret;
}

static int OpExec4(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type4_1.opcode) {
	case OP_CODE_SLDB:
		cpu->op_exec = op_exec_sldb;
		ret = op_exec_sldb(cpu);
		break;
	case OP_CODE_SLDBU:
		if ((cpu->decoded_code->type4_2.opcode & 0x1) == 0x00) {
			cpu->op_exec = op_exec_sldbu;
			ret = op_exec_sldbu(cpu);
		}
		else {
			//SLD.HU
			cpu->op_exec = op_exec_sldhu;
			ret = op_exec_sldhu(cpu);
		}
		break;
	case OP_CODE_SLDH:
		cpu->op_exec = op_exec_sldh;
		ret = op_exec_sldh(cpu);
		break;
	case OP_CODE_SLDW:
/*	case OP_CODE_SSTW: */
		if (cpu->decoded_code->type4_1.gen == 0x00) {
			//SLDW
			cpu->op_exec = op_exec_sldw;
			ret = op_exec_sldw(cpu);
		}
		else {
			//SSTW
			cpu->op_exec = op_exec_sstw;
			ret = op_exec_sstw(cpu);
		}
		break;
	case OP_CODE_SSTB:
		cpu->op_exec = op_exec_sstb;
		ret = op_exec_sstb(cpu);
		break;
	case OP_CODE_SSTH:
		cpu->op_exec = op_exec_ssth;
		ret = op_exec_ssth(cpu);
		break;
	default:
		printf("OpExec4 Error:Unknown OP:0x%x\n", cpu->decoded_code->type4_1.opcode);
		break;
	}
	return ret;
}

static int OpExec5(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type5.opcode) {
	case OP_CODE_JARL:
/*	case OP_CODE_JR: */
		cpu->op_exec = op_exec_jr;
		ret = op_exec_jr(cpu);
		break;
	default:
		printf("OpExec5 Error:Unknown OP:0x%x\n", cpu->decoded_code->type5.opcode);
		break;
	}
	return ret;
}

static int OpExec6(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type6.opcode) {
	case OP_CODE_ADDI:
		cpu->op_exec = op_exec_addi;
		ret = op_exec_addi(cpu);
		break;
	case OP_CODE_ANDI:
		cpu->op_exec = op_exec_andi;
		ret = op_exec_andi(cpu);
		break;
	case OP_CODE_MOVEA:
/*	case OP_CODE_MOV_6: */
		if (cpu->decoded_code->type6.reg2 != 0U) {
			cpu->op_exec = op_exec_movea;
			ret = op_exec_movea(cpu);
		}
		else {
			cpu->op_exec = op_exec_mov_6;
			ret = op_exec_mov_6(cpu);
		}
		break;
	case OP_CODE_MOVHI:
		cpu->op_exec = op_exec_movhi;
		ret = op_exec_movhi(cpu);
		break;
	case OP_CODE_MULHI:
		cpu->op_exec = op_exec_mulhi;
		ret = op_exec_mulhi(cpu);
		break;
	case OP_CODE_ORI:
		cpu->op_exec = op_exec_ori;
		ret = op_exec_ori(cpu);
		break;
	case OP_CODE_SATSUBI:
		cpu->op_exec = op_exec_satsubi;
		ret = op_exec_satsubi(cpu);
		break;
	case OP_CODE_XORI:
		cpu->op_exec = op_exec_xori;
		ret = op_exec_xori(cpu);
		break;
	default:
		printf("OpExec6 Error:Unknown OP:0x%x\n", cpu->decoded_code->type6.opcode);
		break;
	}
	return ret;
}

static int OpExec7(TargetCoreType *cpu)
{
	int ret = -1;

	if (( (cpu->decoded_code->type7.opcode >> 1U) & 0x1F ) == OP_CODE_LDBU) {
		cpu->op_exec = op_exec_ldbu;
		ret = op_exec_ldbu(cpu);
		return ret;
	}

	switch (cpu->decoded_code->type7.opcode) {
	case OP_CODE_LDB:
		cpu->op_exec = op_exec_ldb;
		ret = op_exec_ldb(cpu);
		break;
	case OP_CODE_LDHU:
		cpu->op_exec = op_exec_ldhu;
		ret = op_exec_ldhu(cpu);
		break;
	case OP_CODE_LDH:
/*	case OP_CODE_LDW: */
		cpu->op_exec = op_exec_ldhw;
		ret = op_exec_ldhw(cpu);
		break;
	case OP_CODE_STB:
		cpu->op_exec = op_exec_stb;
		ret = op_exec_stb(cpu);
		break;
	case OP_CODE_STH:
/*	case OP_CODE_STW: */
		cpu->op_exec = op_exec_sthw;
		ret = op_exec_sthw(cpu);
		break;
	default:
		printf("OpExec7 Error:Unknown OP:0x%x\n", cpu->decoded_code->type7.opcode);
		break;
	}
	return ret;
}

static int OpExec8(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type8.sub) {
	case 0b11:
		cpu->op_exec = op_exec_tst1_8;
		ret = op_exec_tst1_8(cpu);
		break;
	case 0b00:
		cpu->op_exec = op_exec_set1_8;
		ret = op_exec_set1_8(cpu);
		break;
	case 0b10:
		cpu->op_exec = op_exec_clr1_8;
		ret = op_exec_clr1_8(cpu);
		break;
	case 0b01:
		cpu->op_exec = op_exec_not1_8;
		ret = op_exec_not1_8(cpu);
		break;
	default:
		printf("OpExec8 Error:Unknown OP:0x%x sub=0x%x\n", cpu->decoded_code->type8.opcode, cpu->decoded_code->type8.sub);
		break;
	}
	return ret;
}

static int OpExec9(TargetCoreType *cpu)
{
	int ret = -1;
	uint16 subopbits = (cpu->decoded_code->type9.rfu1 & 0x0003);

	switch (cpu->decoded_code->type9.sub) {
	case SOP_CODE_LDSR:
		cpu->op_exec = op_exec_ldsr;
		ret = op_exec_ldsr(cpu);
		break;
	case SOP_CODE_SAR_9:
		cpu->op_exec = op_exec_sar_9;
		ret = op_exec_sar_9(cpu);
		break;
	case SOP_CODE_SETF_9:
		cpu->op_exec = op_exec_setf;
		ret = op_exec_setf(cpu);
		break;
	case SOP_CODE_SHL_9:
		cpu->op_exec = op_exec_shl_9;
		ret = op_exec_shl_9(cpu);
		break;
	case SOP_CODE_SHR_9:
		cpu->op_exec = op_exec_shr_9;
		ret = op_exec_shr_9(cpu);
		break;
	case SOP_CODE_STSR:
		cpu->op_exec = op_exec_stsr;
		ret = op_exec_stsr(cpu);
		break;
	case SOP_CODE_BITOPS:
		switch (subopbits) {
		case 0b00:
			cpu->op_exec = op_exec_set1_9;
			ret = op_exec_set1_9(cpu);
			break;
		case 0b01:
			cpu->op_exec = op_exec_not1_9;
			ret = op_exec_not1_9(cpu);
			break;
		case 0b10:
			cpu->op_exec = op_exec_clr1_9;
			ret = op_exec_clr1_9(cpu);
			break;
		case 0b11:
			cpu->op_exec = op_exec_tst1_9;
			ret = op_exec_tst1_9(cpu);
			break;
		default:
			break;
		}
		break;
	default:
		printf("OpExec9 Error:Unknown OP:0x%x\n", cpu->decoded_code->type9.opcode);
		break;
	}
	return ret;
}

static int OpExec10(TargetCoreType *cpu)
{
	int ret = -1;
	uint16 retibits = (cpu->decoded_code->type10.rfu2 & 0x0003);

	switch (cpu->decoded_code->type10.sub) {
	case SOP_CODE_DI:
/*	case SOP_CODE_EI: */
		cpu->op_exec = op_exec_diei;
		ret = op_exec_diei(cpu);
		break;
	case SOP_CODE_HALT:
		cpu->op_exec = op_exec_halt;
		ret = op_exec_halt(cpu);
		break;
	case SOP_CODE_RETI:
		switch (retibits) {
		case 0b00:
			cpu->op_exec = op_exec_reti;
			ret = op_exec_reti(cpu);
			break;
		case 0b01:
		case 0b10:
			//TODO CTRET
			printf("OpExec10 Error:Unknown(CTRET) OP:0x%x\n", cpu->decoded_code->type10.opcode);
			break;
		case 0b11:
		default:
			break;
		}
		break;
	case SOP_CODE_TRAP:
		cpu->op_exec = op_exec_trap;
		ret = op_exec_trap(cpu);
		break;
	default:
		printf("OpExec10 Error:Unknown OP:0x%x\n", cpu->decoded_code->type10.opcode);
		break;
	}
	return ret;
}

static int OpExec11(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type11.sub1) {
	case SOP_CODE_CMOV_11:
		cpu->op_exec = op_exec_cmov_11;
		ret = op_exec_cmov_11(cpu);
		break;
	case SOP_CODE_DIVX1_11:
	case SOP_CODE_DIVX2_11:
		if (cpu->decoded_code->type11.sub2 == 1U) {
			cpu->op_exec = op_exec_divu;
			ret = op_exec_divu(cpu);
		}
		else {
			cpu->op_exec = op_exec_div;
			ret = op_exec_div(cpu);
		}
		break;
	case SOP_CODE_MUL_11:
		if (cpu->decoded_code->type11.sub2 == 1U) {
			cpu->op_exec = op_exec_mulu;
			ret = op_exec_mulu(cpu);
		}
		else {
			cpu->op_exec = op_exec_mul;
			ret = op_exec_mul(cpu);
		}
		break;
	case SOP_CODE_DIVHX1_11:
	case SOP_CODE_DIVHX2_11:
		if (cpu->decoded_code->type11.sub2 == 1U) {
			cpu->op_exec = op_exec_divhu;
			ret = op_exec_divhu(cpu);
		}
		else {
			cpu->op_exec = op_exec_divh_11;
			ret = op_exec_divh_11(cpu);
		}
		break;
	case SOP_CODE_CAXI_11:
		cpu->op_exec = op_exec_caxi;
		ret = op_exec_caxi(cpu);
		break;
	default:
		printf("OpExec11 Error:Unknown sub1=0x%x sub2=0x%x\n", cpu->decoded_code->type11.sub1, cpu->decoded_code->type11.sub2);
		break;
	}
	return ret;
}

static int OpExec12(TargetCoreType *cpu)
{
	int ret = -1;

	switch (cpu->decoded_code->type12.sub1) {
	case SOP_CODE_CMOV_12:
		cpu->op_exec = op_exec_cmov_12;
		ret = op_exec_cmov_12(cpu);
		break;
	case SOP_CODE_MUL_12:
		if (cpu->decoded_code->type12.sub2 == 1U) {
			cpu->op_exec = op_exec_mul_12;
			ret = op_exec_mul_12(cpu);
		}
		else {
			cpu->op_exec = op_exec_mulu_12;
			ret = op_exec_mulu_12(cpu);
		}
		break;
	default:
		printf("OpExec12 Error:Unknown sub1=0x%x sub2=0x%x imm_low=0x%x imm_high=0x%x\n",
				cpu->decoded_code->type12.sub1, cpu->decoded_code->type12.sub2,
				cpu->decoded_code->type12.imml, cpu->decoded_code->type12.immh);
		break;
	}

	return ret;
}

static int OpExec13(TargetCoreType *cpu)
{
	int ret = -1;
	switch (cpu->decoded_code->type13.opcode) {
	case OP_CODE_DISPOSE:
		cpu->op_exec = op_exec_dispose;
		ret = op_exec_dispose(cpu);
		break;
	case OP_CODE_PREPARE:
		cpu->op_exec = op_exec_prepare;
		ret = op_exec_prepare(cpu);
		break;
	default:
		printf("OpExec13 Error:Unknown OP\n");
		break;
	}
	return ret;
}


