#ifndef _OP_EXEC_H_
#define _OP_EXEC_H_

#include "target_cpu.h"
#include "cpu_dec/op_codeid.h"
#include "cpu_common/cpu_ops.h"
#include "cpu_exec/op_exec_ops.h"

#define OP_EXEC_TABLE_NUM		OpCodeId_Num

typedef struct {
	int clocks;
	int (*exec) (TargetCoreType *cpu);
} OpExecType;

extern OpExecType op_exec_table[OP_EXEC_TABLE_NUM];

#ifdef CONFIG_STAT_PERF
#include "std_device_ops.h"
typedef struct {
	char *code_name;
} OpExecStringType;
extern ProfStatType op_exec_stat_table[OP_EXEC_TABLE_NUM];
extern OpExecStringType op_exec_string_table[OP_EXEC_TABLE_NUM];
#endif /* CONFIG_CALC_PERFORMANCE */

//#define op_exec_add_1 op_exec_add_1
//#define op_exec_add_2 op_exec_add_2
#define op_exec_addi_6 op_exec_addi
#define op_exec_adf_11 op_exec_adf_11
#define op_exec_and_1 op_exec_and
#define op_exec_andi_6 op_exec_andi
#define op_exec_bsh_12 op_exec_bsh_12
#define op_exec_bsw_12 op_exec_bsw_12
#define op_exec_bcond_3 op_exec_bcond_3
#define op_exec_callt_2 NULL /* not supported */
#define op_exec_caxi_11 op_exec_caxi
//#define op_exec_clr1_8 op_exec_clr1_8
//#define op_exec_clr1_9 op_exec_clr1_9
//#define op_exec_cmov_12 op_exec_cmov_12
//#define op_exec_cmov_11 op_exec_cmov_11
//#define op_exec_cmp_1 op_exec_cmp_1
//#define op_exec_cmp_2 op_exec_cmp_2
#define op_exec_ctret_10 NULL /* not supported */
#define op_exec_di_10 op_exec_diei
#define op_exec_dispose_13 op_exec_dispose
#define op_exec_div_11 op_exec_div
//#define op_exec_divh_1 op_exec_divh_1
//#define op_exec_divh_11 op_exec_divh_11
#define op_exec_divhu_11 op_exec_divhu
#define op_exec_divq_11 op_exec_divq
#define op_exec_divqu_11 op_exec_divqu_11
#define op_exec_divu_11 op_exec_divu
#define op_exec_ei_10 op_exec_diei
#define op_exec_eiret_10 op_exec_eiret_10
#define op_exec_feret_10 op_exec_feret_10
#define op_exec_fetrap_1 op_exec_fetrap_1
#define op_exec_halt_10 op_exec_halt
#define op_exec_hsh_12 op_exec_hsh_12
#define op_exec_hsw_12 op_exec_hsw_12
#define op_exec_jarl_6 op_exec_jarl_6
#define op_exec_jarl_5 op_exec_jr
#define op_exec_jmp_1 op_exec_jmp
#define op_exec_jmp_6 op_exec_jmp_6
#define op_exec_jr_6 op_exec_jr_6
#define op_exec_jr_5 op_exec_jr
#define op_exec_ldsr_9 op_exec_ldsr
#define op_exec_ld_b_7 op_exec_ldb
#define op_exec_ld_b_14 op_exec_ld_b_14
#define op_exec_ld_bu_7 op_exec_ldbu
#define op_exec_ld_bu_14 op_exec_ld_bu_14
#define op_exec_ld_h_7 op_exec_ldhw
#define op_exec_ld_h_14 op_exec_ld_h_14
#define op_exec_ld_hu_14 op_exec_ld_hu_14
#define op_exec_ld_hu_7 op_exec_ldhu
#define op_exec_ld_w_7 op_exec_ldhw
#define op_exec_ld_w_14 op_exec_ld_w_14

#define op_exec_mac_11 op_exec_mac_11
#define op_exec_macu_11 op_exec_macu_11
//#define op_exec_mov_1 op_exec_mov_1
//#define op_exec_mov_2 op_exec_mov_2
//#define op_exec_mov_6 op_exec_mov_6
#define op_exec_movea_6 op_exec_movea
#define op_exec_movhi_6 op_exec_movhi
#define op_exec_mul_11 op_exec_mul
//#define op_exec_mul_12 op_exec_mul_12
//#define op_exec_mulh_1 op_exec_mulh_1
//#define op_exec_mulh_2 op_exec_mulh_2
#define op_exec_mulhi_6 op_exec_mulhi
#define op_exec_mulu_11 op_exec_mulu
//#define op_exec_mulu_12 op_exec_mulu_12
#define op_exec_nop_1 op_exec_nop
#define op_exec_not_1 op_exec_not
//#define op_exec_not1_8 op_exec_not1_8
//#define op_exec_not1_9 op_exec_not1_9
#define op_exec_or_1 op_exec_or
#define op_exec_ori_6 op_exec_ori
#define op_exec_prepare_13 op_exec_prepare
#define op_exec_reti_10 op_exec_reti
#define op_exec_rie_1 NULL /* TODO */
#define op_exec_rie_10 NULL /* TODO */
//#define op_exec_sar_2 op_exec_sar_2
//#define op_exec_sar_9 op_exec_sar_9
#define op_exec_sar_11 op_exec_sar_11
#define op_exec_sasf_9 op_exec_sasf_9
//#define op_exec_satadd_1 op_exec_satadd_1
//#define op_exec_satadd_2 op_exec_satadd_2
#define op_exec_satadd_11 op_exec_satadd_11
//#define op_exec_satsub_1 op_exec_satsub_1
#define op_exec_satsub_11 op_exec_satsub_11
#define op_exec_satsubi_6 op_exec_satsubi
#define op_exec_satsubr_1 op_exec_satsubr_1
#define op_exec_sbf_11 op_exec_sbf_11
#define op_exec_sch0l_9 op_exec_sch0l_9
#define op_exec_sch0r_9 op_exec_sch0r_9
#define op_exec_sch1l_9 op_exec_sch1l_9
#define op_exec_sch1r_9 op_exec_sch1r_9
//#define op_exec_set1_8 op_exec_set1_8
//#define op_exec_set1_9 op_exec_set1_9
#define op_exec_setf_9 op_exec_setf
//#define op_exec_shl_2 op_exec_shl_2
//#define op_exec_shl_9 op_exec_shl_9
#define op_exec_shl_11 op_exec_shl_11
//#define op_exec_shr_2 op_exec_shr_2
//#define op_exec_shr_9 op_exec_shr_9
#define op_exec_shr_11 op_exec_shr_11
#define op_exec_sld_b_4 op_exec_sldb
#define op_exec_sld_bu_4 op_exec_sldbu
#define op_exec_sld_h_4 op_exec_sldh
#define op_exec_sld_hu_4 op_exec_sldhu
#define op_exec_sld_w_4 op_exec_sldw
#define op_exec_sst_b_4 op_exec_sstb
#define op_exec_sst_h_4 op_exec_ssth
#define op_exec_sst_w_4 op_exec_sstw
#define op_exec_stsr_9 op_exec_stsr
#define op_exec_st_b_7 op_exec_stb
#define op_exec_st_b_14 op_exec_st_b_14
#define op_exec_st_h_7 op_exec_sthw
#define op_exec_st_h_14 op_exec_st_h_14
#define op_exec_st_w_7 op_exec_sthw
#define op_exec_st_w_14 op_exec_st_w_14
#define op_exec_sub_1 op_exec_sub
#define op_exec_subr_1 op_exec_subr
#define op_exec_switch_1 op_exec_switch
#define op_exec_sxb_1 op_exec_sxb
#define op_exec_sxh_1 op_exec_sxh
#define op_exec_synce_1 NULL /* not supported */
#define op_exec_syncm_1 NULL /* not supported */
#define op_exec_syncp_1 NULL /* not supported */
#define op_exec_syscall_10 op_exec_syscall_10
#define op_exec_trap_10 op_exec_trap
#define op_exec_tst_1 op_exec_tst
//#define op_exec_tst1_8 op_exec_tst1_8
//#define op_exec_tst1_9 op_exec_tst1_9
#define op_exec_xor_1 op_exec_xor
#define op_exec_xori_6 op_exec_xori
#define op_exec_zxb_1 op_exec_zxb
#define op_exec_zxh_1 op_exec_zxh

#endif /* _OP_EXEC_H_ */
