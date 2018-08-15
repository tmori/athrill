#include "cpu.h"
#include "bus.h"
#include "std_cpu_ops.h"
#include <stdio.h>
#include "cpu_common/cpu_ops.h"
#include "cpu_dec/op_parse.h"
#include "cpu_exec/op_exec.h"
#include "mpu_types.h"

CpuType virtual_cpu;

void cpu_init(void)
{
	CoreIdType i;
	for (i = 0; i < cpu_config_get_core_id_num(); i++) {
		virtual_cpu.cores[i].core.core_id = i;
		cpu_reset(i);
	}
	return;
}
typedef struct {
	CpuMemoryAccessType access_type;
	uint32 start;
	uint32 end;
} CpuMemoryCheckType;

/*
 *  not overlap confitions
 *
 *    access1                                  access2
 * |------------|                           |------------|
 * s            e                           s            e
 *               |------------------------|
 *               s        config          e
 */
#define IS_NOT_OVERLAP_RANGE(access_start, access_end, config_start, config_end)	\
	( \
			   ((access_start) > (config_end))	\
			|| ((access_end) < (config_start))	\
	)

static bool is_overlap(TargetCoreMpuConfigType *config, CpuMemoryCheckType *check_arg)
{
	uint32 config_start;
	uint32 config_end;

	if (config->is_mask_method == FALSE) {
		//upper lower
		config_start = config->al;
		config_end = config->au;
	}
	else {
		//mask method
		config_start = (config->al & (~config->au));
		config_end = (config->al | config->au);
	}

	if (IS_NOT_OVERLAP_RANGE(config_start, config_end, check_arg->start, check_arg->end)) {
		return FALSE;
	}
	else {
		//printf("ERROR:no permission(access_type=%u): access_addr=0x%x access_end=0x%x config_addr=0x%x config_end=0x%x\n", 
		//	check_arg->access_type, check_arg->start, check_arg->end, config_start, config_end);
		return TRUE;
	}
}
/*
 * FALSE: permission is not allowed
 * TRUE:  permission is allowed
 */
static bool dmp_object_filter(const void *p, const void *arg)
{
	TargetCoreMpuDataConfigType *config = (TargetCoreMpuDataConfigType*)p;
	CpuMemoryCheckType *check_arg = (CpuMemoryCheckType*)arg;

	if (config->common.enable_protection == FALSE) {
		return FALSE;
	}

	if (is_overlap(&config->common, check_arg) == FALSE) {
		return FALSE;
	}
	if (check_arg->access_type == CpuMemoryAccess_READ) {
		return config->enable_read;
	}
	else if (check_arg->access_type == CpuMemoryAccess_WRITE) {
		return config->enable_write;
	}
	//EXEC
	return FALSE;
}

/*
 * FALSE: permission is not allowed
 * TRUE:  permission is allowed
 */
static bool dip_object_filter(const void *p, const void *arg)
{
	TargetCoreMpuExecConfigType *config = (TargetCoreMpuExecConfigType*)p;
	CpuMemoryCheckType *check_arg = (CpuMemoryCheckType*)arg;

	if (config->common.enable_protection == FALSE) {
		return FALSE;
	}

	if (is_overlap(&config->common, check_arg) == FALSE) {
		return FALSE;
	}
	if (check_arg->access_type == CpuMemoryAccess_EXEC) {
		return config->enable_exec;
	}
	else if (check_arg->access_type == CpuMemoryAccess_READ) {
		return config->enable_read;
	}

	//WRITE
	/* do not check hear. write access must be checked on dmp */
	return FALSE;
}

static bool cpu_has_permission_dmp(CoreIdType core_id, CpuMemoryAccessType access_type, uint32 addr, uint32 size)
{
	ObjectContainerType *container = virtual_cpu.cores[core_id].core.mpu.data_configs.region_permissions;
	void *obj;
	CpuMemoryCheckType arg;

	arg.access_type = access_type;
	arg.start = addr;
	arg.end = addr + (size - 1);

	obj = object_container_find_first2(container, dmp_object_filter, &arg);
	if (obj != NULL) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}

static bool cpu_has_permission_imp(CoreIdType core_id, CpuMemoryAccessType access_type, uint32 addr, uint32 size)
{
	ObjectContainerType *container = virtual_cpu.cores[core_id].core.mpu.exec_configs.region_permissions;
	void *obj;
	CpuMemoryCheckType arg;

	arg.access_type = access_type;
	arg.start = addr;
	arg.end = addr + (size - 1);

	obj = object_container_find_first2(container, dip_object_filter, &arg);
	if (obj != NULL) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}
bool cpu_has_permission(CoreIdType core_id, MpuAddressRegionEnumType region_type, CpuMemoryAccessType access_type, uint32 addr, uint32 size)
{
	uint32 psw = cpu_get_psw(&virtual_cpu.cores[core_id].core.reg.sys);
	bool permission = FALSE;
	bool dmp_permission = FALSE;
	bool imp_permission = FALSE;

	switch (region_type) {
	case GLOBAL_MEMORY:
	case READONLY_MEMORY:
		/* dmp check */
		if ((access_type == CpuMemoryAccess_EXEC) || IS_TRUSTED_DMP(psw)) {
			dmp_permission = TRUE;
		}
		else { /* READ or WRITE */
			dmp_permission = cpu_has_permission_dmp(core_id, access_type, addr, size);
		}
		if (dmp_permission == FALSE) {
			virtual_cpu.cores[core_id].core.mpu.exception_error_code = CpuExceptionError_MDP;
			virtual_cpu.cores[core_id].core.mpu.error_address = addr;
			virtual_cpu.cores[core_id].core.mpu.error_access = access_type;
			break;
		}
		/* imp check */
		if ((access_type == CpuMemoryAccess_WRITE) || IS_TRUSTED_IMP(psw)) {
			imp_permission = TRUE;
		}
		else { /* READ or EXEC */
			imp_permission = cpu_has_permission_imp(core_id, access_type, addr, size);
		}
		if (imp_permission == FALSE) {
			virtual_cpu.cores[core_id].core.mpu.exception_error_code = CpuExceptionError_MIP;
			virtual_cpu.cores[core_id].core.mpu.error_address = cpu_get_current_core_pc();
			virtual_cpu.cores[core_id].core.mpu.error_access = access_type;
		}
		if ((dmp_permission == TRUE) && (imp_permission == TRUE)) {
			permission = TRUE;
		}
		break;
	case DEVICE:
		permission = IS_TRUSTED_PP(psw);
		if (permission == FALSE) {
			virtual_cpu.cores[core_id].core.mpu.exception_error_code = CpuExceptionError_PPI;
			virtual_cpu.cores[core_id].core.mpu.error_address = addr;
			virtual_cpu.cores[core_id].core.mpu.error_access = access_type;
		}
		break;
	default:
		break;
	}
	return permission;
}

bool cpu_illegal_access(CoreIdType core_id)
{
	if (virtual_cpu.cores[core_id].core.mpu.exception_error_code == CpuExceptionError_None) {
		return FALSE;
	}
	uint32 fepc = cpu_get_current_core_pc();
	uint32 fepsw = cpu_get_psw(&virtual_cpu.cores[core_id].core.reg.sys);
	uint32 feic = 0x0;
	uint32 handler_code;
	uint32 ecr;
	uint32 *factor_sysreg;
	uint32 *setting_sysreg;

	factor_sysreg = cpu_get_mpu_illegal_factor_sysreg(&virtual_cpu.cores[core_id].core.reg.sys);
	setting_sysreg = cpu_get_mpu_settign_sysreg(&virtual_cpu.cores[core_id].core.reg.sys);

	factor_sysreg[SYS_REG_MPV_VMADR] = virtual_cpu.cores[core_id].core.mpu.error_address;
	factor_sysreg[SYS_REG_MPV_VMTID] = setting_sysreg[SYS_REG_MPU_TID];

	switch (virtual_cpu.cores[core_id].core.mpu.exception_error_code) {
	case CpuExceptionError_MIP:
		feic = 0x0430;
		handler_code = 0x0030;
		break;
	case CpuExceptionError_MDP:
		feic = 0x0431;
		handler_code = 0x0030;
		break;
	case CpuExceptionError_PPI:
		feic = 0x0432;
		handler_code = 0x0030;
		break;
	default:
		/* not reached */
		break;
	}

	switch (virtual_cpu.cores[core_id].core.mpu.error_access) {
	case CpuMemoryAccess_READ:
		factor_sysreg[SYS_REG_MPV_VMECR] |= CPU_VMECR_VMR;
		factor_sysreg[SYS_REG_MPV_VMECR] &= ~CPU_VMECR_VMW;
		factor_sysreg[SYS_REG_MPV_VMECR] &= ~CPU_VMECR_VMX;
		break;
	case CpuMemoryAccess_WRITE:
		factor_sysreg[SYS_REG_MPV_VMECR] |= CPU_VMECR_VMW;
		factor_sysreg[SYS_REG_MPV_VMECR] &= ~CPU_VMECR_VMR;
		factor_sysreg[SYS_REG_MPV_VMECR] &= ~CPU_VMECR_VMX;
		break;
	case CpuMemoryAccess_EXEC:
		factor_sysreg[SYS_REG_MPV_VMECR] |= CPU_VMECR_VMX;
		factor_sysreg[SYS_REG_MPV_VMECR] &= ~CPU_VMECR_VMW;
		factor_sysreg[SYS_REG_MPV_VMECR] &= ~CPU_VMECR_VMR;
		break;
	default:
		/* not reached */
		break;
	}

	CPU_SET_EP(&virtual_cpu.cores[core_id].core.reg);
	CPU_SET_NP(&virtual_cpu.cores[core_id].core.reg);
	CPU_SET_ID(&virtual_cpu.cores[core_id].core.reg);

	CPU_CLR_PP(&virtual_cpu.cores[core_id].core.reg);
	CPU_CLR_NPV(&virtual_cpu.cores[core_id].core.reg);
	CPU_CLR_IMP(&virtual_cpu.cores[core_id].core.reg);
	CPU_CLR_DMP(&virtual_cpu.cores[core_id].core.reg);


	sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_FEIC] = feic;
	sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_FEPSW] = fepsw;
	sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_FEPC] = fepc;

	ecr = sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_ECR];
	ecr = ecr & 0x00FF;
	ecr |= (feic << 16);
	sys_get_cpu_base(&virtual_cpu.cores[core_id].core.reg)->r[SYS_REG_ECR] = ecr;
	virtual_cpu.cores[core_id].core.reg.pc = handler_code;

	virtual_cpu.cores[core_id].core.mpu.exception_error_code = CpuExceptionError_None;
	virtual_cpu.cores[core_id].core.mpu.error_access = CpuMemoryAccess_NONE;
	return TRUE;
}
static void private_cpu_mpu_set_common_obj(TargetCoreMpuConfigType *config, uint32 au, uint32 al)
{
	config->al = (al & 0xFFFFFF0);
	config->au = ( (au & 0xFFFFFF0) | 0x0F );

	if ((al & 0x04) != 0x00) {
		config->is_mask_method = TRUE;
	}
	else {
		config->is_mask_method = FALSE;
	}

	if ((al & 0x01) != 0x00) {
		config->enable_protection = TRUE;
	}
	else {
		config->enable_protection = FALSE;
	}
	return;
}

static void private_cpu_mpu_construct_containers(TargetCoreType *cpu)
{
	int i;
	uint32 *setting_sysreg;

	setting_sysreg = cpu_get_mpu_settign_sysreg(&cpu->reg.sys);

	if (cpu->mpu.exec_configs.region_permissions != NULL) {
		object_container_delete(cpu->mpu.exec_configs.region_permissions);
		cpu->mpu.exec_configs.region_permissions = NULL;
	}
	if (cpu->mpu.data_configs.region_permissions != NULL) {
		object_container_delete(cpu->mpu.data_configs.region_permissions);
		cpu->mpu.data_configs.region_permissions = NULL;
	}
	cpu->mpu.exec_configs.region_permissions = object_container_create(sizeof(TargetCoreMpuExecConfigType), TARGET_CORE_MPU_CONFIG_EXEC_MAXNUM);
	cpu->mpu.data_configs.region_permissions = object_container_create(sizeof(TargetCoreMpuDataConfigType), TARGET_CORE_MPU_CONFIG_DATA_MAXNUM);

	/*
	 * exec
	 */
	ObjectContainerType	*container;

	container = cpu->mpu.exec_configs.region_permissions;
	for (i = 0; i < TARGET_CORE_MPU_CONFIG_EXEC_MAXNUM; i++) {
		TargetCoreMpuExecConfigType *obj = (TargetCoreMpuExecConfigType *)object_container_create_element(container);
		uint32 al = setting_sysreg[SYS_REG_MPU_IPA0L + (i * 2)];
		uint32 au = setting_sysreg[SYS_REG_MPU_IPA0U + (i * 2)];

		private_cpu_mpu_set_common_obj(&obj->common, au, al);
		if ((au & 0x02) != 0x00) {
			obj->enable_read = TRUE;
		}
		else {
			obj->enable_read = FALSE;
		}

		if ((au & 0x01) != 0x00) {
			obj->enable_exec = TRUE;
		}
		else {
			obj->enable_exec = FALSE;
		}
	}

	/*
	 * data
	 */
	container = cpu->mpu.data_configs.region_permissions;
	for (i = 0; i < TARGET_CORE_MPU_CONFIG_DATA_MAXNUM; i++) {
		TargetCoreMpuDataConfigType *obj = (TargetCoreMpuDataConfigType *)object_container_create_element(container);
		uint32 al = setting_sysreg[SYS_REG_MPU_DPA0L + (i * 2)];
		uint32 au = setting_sysreg[SYS_REG_MPU_DPA0U + (i * 2)];

		private_cpu_mpu_set_common_obj(&obj->common, au, al);
		if ((au & 0x02) != 0x00) {
			obj->enable_read = TRUE;
		}
		else {
			obj->enable_read = FALSE;
		}

		if ((au & 0x04) != 0x00) {
			obj->enable_write = TRUE;
		}
		else {
			obj->enable_write = FALSE;
		}
	}

	return;
}


static void debug_print_mpu_status(TargetCoreType *cpu)
{
	int i;
	uint32 *setting_sysreg;
	uint32 *factor_sysreg;
	ObjectContainerType	*container;

	factor_sysreg = cpu_get_mpu_illegal_factor_sysreg(&cpu->reg.sys);
	setting_sysreg = cpu_get_mpu_settign_sysreg(&cpu->reg.sys);

	printf("VSECR = 0x%08x\n", factor_sysreg[SYS_REG_MPV_VSECR]);
	printf("VSTID = 0x%08x\n", factor_sysreg[SYS_REG_MPV_VSTID]);
	printf("VSADR = 0x%08x\n", factor_sysreg[SYS_REG_MPV_VSADR]);
	printf("VMECR = 0x%08x\n", factor_sysreg[SYS_REG_MPV_VMECR]);
	printf("VMTID = 0x%08x\n", factor_sysreg[SYS_REG_MPV_VMTID]);
	printf("VMADR = 0x%08x\n", factor_sysreg[SYS_REG_MPV_VMADR]);


	printf("MPM = 0x%08x\n", setting_sysreg[SYS_REG_MPU_MPM]);
	printf("MPC = 0x%08x\n", setting_sysreg[SYS_REG_MPU_MPC]);
	printf("TID = 0x%08x\n", setting_sysreg[SYS_REG_MPU_TID]);

	/*
	 * exec
	 */
	container = cpu->mpu.exec_configs.region_permissions;
	for (i = 0; i < TARGET_CORE_MPU_CONFIG_EXEC_MAXNUM; i++) {
		TargetCoreMpuExecConfigType *obj = (TargetCoreMpuExecConfigType *)object_container_get_element(container, i);
		uint32 al = setting_sysreg[SYS_REG_MPU_IPA0L + (i * 2)];
		uint32 au = setting_sysreg[SYS_REG_MPU_IPA0U + (i * 2)];
		if (obj->common.enable_protection == TRUE) {
			printf("IPA%uL = 0x%08x IPA%uU = 0x%08x mask_method = %u\n", i, al, i, au, obj->common.is_mask_method);
			printf(" al = 0x%08x au = 0x%08x\n", obj->common.al, obj->common.au);
			printf(" enable_read = %u enable_exec = %u\n", obj->enable_read, obj->enable_exec);
		}
	}

	/*
	 * data
	 */
	container = cpu->mpu.data_configs.region_permissions;
	for (i = 0; i < TARGET_CORE_MPU_CONFIG_DATA_MAXNUM; i++) {
		TargetCoreMpuDataConfigType *obj = (TargetCoreMpuDataConfigType *)object_container_get_element(container, i);
		uint32 al = setting_sysreg[SYS_REG_MPU_DPA0L + (i * 2)];
		uint32 au = setting_sysreg[SYS_REG_MPU_DPA0U + (i * 2)];
		if (obj->common.enable_protection == TRUE) {
			printf("DPA%uL = 0x%08x DPA%uU = 0x%08x mask_method = %u\n", i, al, i, au, obj->common.is_mask_method);
			printf(" al = 0x%08x au = 0x%08x\n", obj->common.al, obj->common.au);
			printf(" enable_read = %u enable_write = %u\n", obj->enable_read, obj->enable_write);
		}
	}

	return;
}
void cpu_debug_print_mpu_status(CoreIdType core_id)
{
	debug_print_mpu_status(&virtual_cpu.cores[core_id].core);
	return;
}

void cpu_mpu_construct_containers(CoreIdType core_id)
{
	private_cpu_mpu_construct_containers(&virtual_cpu.cores[core_id].core);
	//debug_print_mpu_status(&virtual_cpu.cores[core_id].core);
	return;
}

static void private_cpu_mpu_init(TargetCoreType *cpu)
{
	uint32 *setting_sysreg;

	cpu->mpu.exception_error_code = CpuExceptionError_None;

	//mpu register initial values
	setting_sysreg = cpu_get_mpu_settign_sysreg(&cpu->reg.sys);
	//IPA0L-IPA4L 			0000 0002H
	setting_sysreg[SYS_REG_MPU_IPA0L] = 0x00000002;
	setting_sysreg[SYS_REG_MPU_IPA1L] = 0x00000002;
	setting_sysreg[SYS_REG_MPU_IPA2L] = 0x00000002;
	setting_sysreg[SYS_REG_MPU_IPA3L] = 0x00000002;
	setting_sysreg[SYS_REG_MPU_IPA4L] = 0x00000002;

	//IPA0U-IPA4U 			0000 0000H
	setting_sysreg[SYS_REG_MPU_IPA0U] = 0x00000000;
	setting_sysreg[SYS_REG_MPU_IPA1U] = 0x00000000;
	setting_sysreg[SYS_REG_MPU_IPA2U] = 0x00000000;
	setting_sysreg[SYS_REG_MPU_IPA3U] = 0x00000000;
	setting_sysreg[SYS_REG_MPU_IPA4U] = 0x00000000;

	//DPA0L 				0000 0002H
	setting_sysreg[SYS_REG_MPU_DPA0L] = 0x00000002;
	//DPA1L-DPA4L 			0000 0002H
	setting_sysreg[SYS_REG_MPU_DPA1L] = 0x00000002;
	setting_sysreg[SYS_REG_MPU_DPA2L] = 0x00000002;
	setting_sysreg[SYS_REG_MPU_DPA3L] = 0x00000002;
	setting_sysreg[SYS_REG_MPU_DPA4L] = 0x00000002;
	//DPA5L 				0000 0006H
	setting_sysreg[SYS_REG_MPU_DPA5L] = 0x00000006;

	//DPA0U 				0000 0006H
	setting_sysreg[SYS_REG_MPU_DPA0U] = 0x00000006;
	//DPA1U-DPA4U 			0000 0000H
	setting_sysreg[SYS_REG_MPU_DPA1U] = 0x00000000;
	setting_sysreg[SYS_REG_MPU_DPA2U] = 0x00000000;
	setting_sysreg[SYS_REG_MPU_DPA3U] = 0x00000000;
	setting_sysreg[SYS_REG_MPU_DPA4U] = 0x00000000;
	//DPA5U 				0000 0000H
	setting_sysreg[SYS_REG_MPU_DPA5U] = 0x00000000;

	private_cpu_mpu_construct_containers(cpu);
	//debug_print_mpu_status(cpu);
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

	/*
	 * MPU
	 */
	private_cpu_mpu_init(cpu);
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

static Std_ReturnType cpu_supply_clock_not_cached(CoreIdType core_id, CachedOperationCodeType *cached_code, uint32 inx)
{
	int ret;
	Std_ReturnType err;
	static OpDecodedCodeType	decoded_code;
	OpDecodedCodeType		*p_decoded_code;
	OperationCodeType optype;
	bool permission;

	if (cached_code != NULL) {
		p_decoded_code = &cached_code->codes[inx].decoded_code;
		virtual_cpu.cores[core_id].core.decoded_code = &cached_code->codes[inx].decoded_code;
	}
	else {
		p_decoded_code = &decoded_code;
		virtual_cpu.cores[core_id].core.decoded_code = &decoded_code;
	}
	/*
	 * 命令取得する
	 */
	err = bus_get_pointer(core_id,
			virtual_cpu.cores[core_id].core.reg.pc,
				(uint8**)&(virtual_cpu.cores[core_id].core.current_code));
	if (err != STD_E_OK) {
		return err;
	}

	/*
	 * デコード
	 */
	ret = op_parse(virtual_cpu.cores[core_id].core.current_code,
			p_decoded_code, &optype);
	if (ret < 0) {
		printf("Decode Error\n");
		return STD_E_DECODE;
	}

	permission = cpu_has_permission(core_id,
			READONLY_MEMORY,
			CpuMemoryAccess_EXEC,
			virtual_cpu.cores[core_id].core.reg.pc,
			OpFormatSize[p_decoded_code->type_id]);
	if (permission == FALSE) {
		return STD_E_SEGV;
	}

	if (op_exec_table[optype.code_id].exec == NULL) {
		printf("Not supported code(%d fmt=%d) Error code[0]=0x%x code[1]=0x%x type_id=0x%x\n",
				optype.code_id, optype.format_id,
				virtual_cpu.cores[core_id].core.current_code[0],
				virtual_cpu.cores[core_id].core.current_code[1],
				virtual_cpu.cores[core_id].core.decoded_code->type_id);
		return STD_E_EXEC;
	}

	/*
	 * 命令実行
	 */
	ret = op_exec_table[optype.code_id].exec(&virtual_cpu.cores[core_id].core);
	if (ret < 0) {
		printf("Exec Error code[0]=0x%x code[1]=0x%x type_id=0x%x code_id=%u\n",
				virtual_cpu.cores[core_id].core.current_code[0],
				virtual_cpu.cores[core_id].core.current_code[1],
				virtual_cpu.cores[core_id].core.decoded_code->type_id,
				optype.code_id);
		return STD_E_EXEC;
	}

	if (cached_code != NULL) {
		cached_code->codes[inx].op_exec = op_exec_table[optype.code_id].exec;
	}
	return STD_E_OK;
}

Std_ReturnType cpu_supply_clock(CoreIdType core_id)
{
	int ret;
	Std_ReturnType err;
	uint32 inx;
	CachedOperationCodeType *cached_code;
	bool permission;

	if (virtual_cpu.cores[core_id].core.is_halt == TRUE) {
		return STD_E_OK;
	}
	virtual_cpu.cores[core_id].core.reg.r[0] = 0U;

	cached_code = virtual_cpu_get_cached_code(virtual_cpu.cores[core_id].core.reg.pc);
	if (cached_code != NULL) {
		inx = virtual_cpu.cores[core_id].core.reg.pc - cached_code->code_start_addr;
	}
	if ((cached_code == NULL) || (cached_code->codes[inx].op_exec == NULL)) {
		err = cpu_supply_clock_not_cached(core_id, cached_code, inx);
		if (err != STD_E_OK) {
			return err;
		}
		virtual_cpu.cores[core_id].core.reg.r[0] = 0U;
	}
	else {
		virtual_cpu.cores[core_id].core.decoded_code = &cached_code->codes[inx].decoded_code;
		permission = cpu_has_permission(core_id,
				READONLY_MEMORY,
				CpuMemoryAccess_EXEC,
				virtual_cpu.cores[core_id].core.reg.pc,
				OpFormatSize[cached_code->codes[inx].decoded_code.type_id]);
		if (permission == FALSE) {
			return STD_E_SEGV;
		}
		ret = cached_code->codes[inx].op_exec(&virtual_cpu.cores[core_id].core);
		if (ret < 0) {
			printf("Exec Error code[0]=0x%x code[1]=0x%x type_id=0x%x\n",
					virtual_cpu.cores[core_id].core.current_code[0],
					virtual_cpu.cores[core_id].core.current_code[1],
					virtual_cpu.cores[core_id].core.decoded_code->type_id);
			return STD_E_EXEC;
		}
		virtual_cpu.cores[core_id].core.reg.r[0] = 0U;
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

