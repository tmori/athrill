#ifndef _CPU_REGISTER_H_
#define _CPU_REGISTER_H_

#include "cpu_dec/op_dec.h"
#include "std_types.h"
#include "object_container.h"

#define CPU_GREG_NUM			(32U)
#define CPU_SYSREG_NUM			(28U)
#define CPU_COMMON_SYSREG_NUM	(4U)
#define CPU_SYSBNK_NUM			(3U)

#define CPU_SYSREG_EIWR			(0U)
#define CPU_SYSREG_FEWR			(1U)
#define CPU_SYSREG_DBWR			(2U)
#define CPU_SYSREG_BSEL			(3U)
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

typedef enum {
	SYS_REG_SW_CTL = 0,
	SYS_REG_SW_CFG,
	SYS_REG_SW_RESERVE_2,
	SYS_REG_SW_BASE,
	SYS_REG_SW_RESERVE_4,
	SYS_REG_SW_RESERVE_5,
	SYS_REG_SW_RESERVE_6,
	SYS_REG_SW_RESERVE_7,
	SYS_REG_SW_RESERVE_8,
	SYS_REG_SW_RESERVE_9,
	SYS_REG_SW_RESERVE_10,
	SYS_REG_SW_RESERVE_11,
	SYS_REG_SW_RESERVE_12,
	SYS_REG_SW_RESERVE_13,
	SYS_REG_SW_RESERVE_14,
	SYS_REG_SW_RESERVE_15,
	SYS_REG_SW_RESERVE_16,
	SYS_REG_SW_RESERVE_17,
	SYS_REG_SW_RESERVE_18,
	SYS_REG_SW_RESERVE_19,
	SYS_REG_SW_RESERVE_20,
	SYS_REG_SW_RESERVE_21,
	SYS_REG_SW_RESERVE_22,
	SYS_REG_SW_RESERVE_23,
	SYS_REG_SW_RESERVE_24,
	SYS_REG_SW_RESERVE_25,
	SYS_REG_SW_RESERVE_26,
	SYS_REG_SW_RESERVE_27,
} SysGrpCpuBnk0ExceptionRegisterType;

typedef enum {
	SYS_REG_EH_RESERVE0 = 0,
	SYS_REG_EH_CFG,
	SYS_REG_EH_RESET,
	SYS_REG_EH_BASE,
	SYS_REG_EH_RESERVE_4,
	SYS_REG_EH_RESERVE_5,
	SYS_REG_EH_RESERVE_6,
	SYS_REG_EH_RESERVE_7,
	SYS_REG_EH_RESERVE_8,
	SYS_REG_EH_RESERVE_9,
	SYS_REG_EH_RESERVE_10,
	SYS_REG_EH_RESERVE_11,
	SYS_REG_EH_RESERVE_12,
	SYS_REG_EH_RESERVE_13,
	SYS_REG_EH_RESERVE_14,
	SYS_REG_EH_RESERVE_15,
	SYS_REG_EH_RESERVE_16,
	SYS_REG_EH_RESERVE_17,
	SYS_REG_EH_RESERVE_18,
	SYS_REG_EH_RESERVE_19,
	SYS_REG_EH_RESERVE_20,
	SYS_REG_EH_RESERVE_21,
	SYS_REG_EH_RESERVE_22,
	SYS_REG_EH_RESERVE_23,
	SYS_REG_EH_RESERVE_24,
	SYS_REG_EH_RESERVE_25,
	SYS_REG_EH_RESERVE_26,
	SYS_REG_EH_RESERVE_27,
} SysGrpCpuBnk1ExceptionRegisterType;


typedef enum {
	SYS_REG_MPV_VSECR = 0,
	SYS_REG_MPV_VSTID,
	SYS_REG_MPV_VSADR,
	SYS_REG_MPV_RESERVE_3,
	SYS_REG_MPV_VMECR,
	SYS_REG_MPV_VMTID,
	SYS_REG_MPV_VMADR,
	SYS_REG_MPV_RESERVE_7,
	SYS_REG_MPV_RESERVE_8,
	SYS_REG_MPV_RESERVE_9,
	SYS_REG_MPV_RESERVE_10,
	SYS_REG_MPV_RESERVE_11,
	SYS_REG_MPV_RESERVE_12,
	SYS_REG_MPV_RESERVE_13,
	SYS_REG_MPV_RESERVE_14,
	SYS_REG_MPV_RESERVE_15,
	SYS_REG_MPV_RESERVE_16,
	SYS_REG_MPV_RESERVE_17,
	SYS_REG_MPV_RESERVE_18,
	SYS_REG_MPV_RESERVE_19,
	SYS_REG_MPV_RESERVE_20,
	SYS_REG_MPV_RESERVE_21,
	SYS_REG_MPV_RESERVE_22,
	SYS_REG_MPV_RESERVE_23,
	SYS_REG_MPV_MCA,
	SYS_REG_MPV_MCS,
	SYS_REG_MPV_MCC,
	SYS_REG_MPV_MCR,
} SysGrpProcessorProtectErrorBnkRegisterType;


typedef enum {
	SYS_REG_MPU_MPM = 0,
	SYS_REG_MPU_MPC,
	SYS_REG_MPU_TID,
	SYS_REG_MPU_RESERVE_3,
	SYS_REG_MPU_RESERVE_4,
	SYS_REG_MPU_RESERVE_5,
	SYS_REG_MPU_IPA0L,
	SYS_REG_MPU_IPA0U,
	SYS_REG_MPU_IPA1L,
	SYS_REG_MPU_IPA1U,
	SYS_REG_MPU_IPA2L,
	SYS_REG_MPU_IPA2U,
	SYS_REG_MPU_IPA3L,
	SYS_REG_MPU_IPA3U,
	SYS_REG_MPU_IPA4L,
	SYS_REG_MPU_IPA4U,
	SYS_REG_MPU_DPA0L,
	SYS_REG_MPU_DPA0U,
	SYS_REG_MPU_DPA1L,
	SYS_REG_MPU_DPA1U,
	SYS_REG_MPU_DPA2L,
	SYS_REG_MPU_DPA2U,
	SYS_REG_MPU_DPA3L,
	SYS_REG_MPU_DPA3U,
	SYS_REG_MPU_DPA4L,
	SYS_REG_MPU_DPA4U,
	SYS_REG_MPU_DPA5L,
	SYS_REG_MPU_DPA5U,
} SysGrpProcessorProtectSettingsBnkRegisterType;


typedef enum {
	SYS_REG_MPM = 0,
	SYS_REG_MPC,
	SYS_REG_TID,
	SYS_REG_VMECR,
	SYS_REG_VMTID,
	SYS_REG_VMADR,
	SYS_REG_IPA0L,
	SYS_REG_IPA0U,
	SYS_REG_IPA1L,
	SYS_REG_IPA1U,
	SYS_REG_IPA2L,
	SYS_REG_IPA2U,
	SYS_REG_IPA3L,
	SYS_REG_IPA3U,
	SYS_REG_IPA4L,
	SYS_REG_IPA4U,
	SYS_REG_DPA0L,
	SYS_REG_DPA0U,
	SYS_REG_DPA1L,
	SYS_REG_DPA1U,
	SYS_REG_DPA2L,
	SYS_REG_DPA2U,
	SYS_REG_DPA3L,
	SYS_REG_DPA3U,
	SYS_REG_DPA4L,
	SYS_REG_DPA4U,
	SYS_REG_DPA5L,
	SYS_REG_DPA5U,
} SysGrpProcessorProtectPagingBnkRegisterType;

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

static inline uint32 cpu_get_psw(CpuSystemRegisterType *sys) {
	return sys->grp[SYS_GRP_CPU][SYS_GRP_CPU_BNK_0].r[SYS_REG_PSW];
}


#define PSW_PP_BIT			19U		//周辺装置保護
#define PSW_NPV_BIT			18U		//システム・レジスタ保護
#define PSW_DMP_BIT			17U		//データ・アクセス
#define PSW_IMP_BIT			16U		//プログラム領域に対するメモリ保護
#define IS_TRUSTED_PP(psw)		(((psw) & (1U << PSW_PP_BIT)) == 0x0)
#define IS_TRUSTED_NPV(psw)		(((psw) & (1U << PSW_NPV_BIT)) == 0x0)
#define IS_TRUSTED_DMP(psw)		(((psw) & (1U << PSW_DMP_BIT)) == 0x0)
#define IS_TRUSTED_IMP(psw)		(((psw) & (1U << PSW_IMP_BIT)) == 0x0)

static inline uint32 *cpu_get_mpu_settign_sysreg(CpuSystemRegisterType *sys) {
	return sys->grp[SYS_GRP_PROSESSOR][SYS_GRP_CPU_BNK_1].r;
}


static inline CpuSystemRegisterDataType *sys_get_cpu_base(CpuRegisterType *reg) {
	return &reg->sys.grp[SYS_GRP_CPU][SYS_GRP_CPU_BNK_0];
}

typedef enum {
	CpuExceptionError_None = 0,
	CpuExceptionError_MIP,
	CpuExceptionError_MDP,
	CpuExceptionError_PPI,
} CpuExceptionErrorCodeType;

#define TARGET_CORE_MPU_CONFIG_EXEC_MAXNUM		5U
#define TARGET_CORE_MPU_CONFIG_DATA_MAXNUM		6U

typedef struct {
	bool								enable_protection;
	bool								is_mask_method;
	/*
	 * is_mask_method == FALSE
	 *  => au: upper address
	 *  => al: lower address
	 *
	 * is_mask_method == TRUE
	 *  => au: mask
	 *  => al: base address
	 *
	 *  マスク値を指定する場合は,必ず下位側から 1 を連続させた値を設定してください
	 *  (000050FFHなどのように,1/0が交互に配置された場合の動作は保証しません)。
	 */
	uint32								au;
	uint32								al;
} TargetCoreMpuConfigType;

typedef struct {
	TargetCoreMpuConfigType				common;
	bool								enable_read;
	bool								enable_exec;
} TargetCoreMpuExecConfigType;

typedef struct {
	TargetCoreMpuConfigType				common;
	bool								enable_read;
	bool								enable_write;
} TargetCoreMpuDataConfigType;

typedef struct {
	ObjectContainerType					*region_permissions;
} TargetCoreMpuConfigContainerType;

typedef struct {
	CpuExceptionErrorCodeType			exception_error_code;
	TargetCoreMpuConfigContainerType	data_configs;
	TargetCoreMpuConfigContainerType	exec_configs;
} TargetCoreMpuType;

typedef struct {
	CoreIdType					core_id;
	CpuRegisterType 			reg;
	bool						is_halt;
	uint16 						*current_code;
	OpDecodedCodeType			*decoded_code;
	TargetCoreMpuType			mpu;
} TargetCoreType;


#endif /* _CPU_REGISTER_H_ */
