#ifndef _CPU_REGISTER_H_
#define _CPU_REGISTER_H_

#include "cpu_dec/op_dec.h"
#include "std_types.h"

#define CPU_GREG_NUM			(32U)
#define CPU_SYSREG_NUM			(28U)
#define CPU_COMMON_SYSREG_NUM	(4U)
#define CPU_SYSBNK_NUM			(3U)

typedef enum {
	SYS_GRP_CPU = 0,
	SYS_GRP_PROSESSOR,
	SYS_GRP_PMU,
	SYS_GRP_FPU,
	SYS_GRP_USER,
	SYS_GRP_NUM,
} SysGrpuType;

typedef enum {
	SYS_GRP_CPU_BNK_0 = 0,
	SYS_GRP_CPU_BNK_1,
	SYS_GRP_CPU_BNK_2,
	SYS_GRP_CPU_BNK_NUM,
} SysGrpCpuBnkType;

typedef enum {
	SYS_REG_EIPC = 0,
	SYS_REG_EIPSW,
	SYS_REG_FEPC,
	SYS_REG_FEPSW,
	SYS_REG_ECR,
	SYS_REG_PSW,
	/* RESERVE 6-10 */
	SYS_REG_RESERVE_6,
	SYS_REG_RESERVE_7,
	SYS_REG_RESERVE_8,
	SYS_REG_RESERVE_9,
	SYS_REG_RESERVE_10,
	SYS_REG_SCCFG,
	SYS_REG_SCBP,
	SYS_REG_EIIC,
	SYS_REG_FEIC,
	SYS_REG_DBIC,
	SYS_REG_CTPC,
	SYS_REG_CTPSW,
	SYS_REG_DBPC,
	SYS_REG_DBPSW,
	SYS_REG_CTBP,
	SYS_REG_DIR,
	/* 22-27 デバッグ機能レジスタ － － － */
} SysGrpCpuBnkBaseRegisterType;


#define CPU_REG_UINT_MAX	0xFFFFFFFFULL
#define CPU_REG_PLUS_MAX	2147483647LL
#define CPU_REG_MINUS_MAX	-2147483648LL

#define CPU_REG_SP		(3)
#define CPU_REG_EP		(30)
#define CPU_REG_LP		(31)


typedef struct {
	uint32 r[CPU_SYSREG_NUM];
} CpuSystemRegisterDataType;

typedef struct {
	uint32						current_grp;
	uint32						current_bnk;
	CpuSystemRegisterDataType	grp[SYS_GRP_NUM][CPU_SYSBNK_NUM];
	uint32						sysreg[CPU_COMMON_SYSREG_NUM];
} CpuSystemRegisterType;

typedef struct {
	uint32 pc;
	sint32 r[CPU_GREG_NUM];
	CpuSystemRegisterType	sys;
} CpuRegisterType;


static inline uint32 *cpu_get_sysreg(CpuSystemRegisterType *sys, uint32 inx) {
	if (inx < CPU_SYSREG_NUM) {
		return &sys->grp[sys->current_grp][sys->current_bnk].r[inx];
	}
	else {
		return &sys->sysreg[inx - CPU_SYSREG_NUM];
	}
}
static inline CpuSystemRegisterDataType *sys_get_cpu_base(CpuRegisterType *reg) {
	return &reg->sys.grp[SYS_GRP_CPU][SYS_GRP_CPU_BNK_0];
}


typedef struct {
	CoreIdType				core_id;
	CpuRegisterType 		reg;
	bool					is_halt;
	uint16 					current_code[OP_DECODE_MAX];
	OpDecodedCodeType		*decoded_code;
	void					*op_exec;
} TargetCoreType;

#endif /* _CPU_REGISTER_H_ */
