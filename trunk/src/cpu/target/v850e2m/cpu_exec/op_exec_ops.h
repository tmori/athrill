

#ifndef _OP_EXEC_OPS_H_
#define _OP_EXEC_OPS_H_

#include "target_cpu.h"
#include "cpu_common/cpu_ops.h"

/*
 * ロード命令
 */
extern int op_exec_ldb(TargetCoreType *cpu);
extern int op_exec_ldbu(TargetCoreType *cpu);
extern int op_exec_ldhu(TargetCoreType *cpu);
extern int op_exec_ldhw(TargetCoreType *cpu);
extern int op_exec_sldb(TargetCoreType *cpu);
extern int op_exec_sldbu(TargetCoreType *cpu);
extern int op_exec_sldh(TargetCoreType *cpu);
extern int op_exec_sldhu(TargetCoreType *cpu);
extern int op_exec_sldw(TargetCoreType *cpu);

/*
 * ストア命令
 */
extern int op_exec_sstb(TargetCoreType *cpu);
extern int op_exec_ssth(TargetCoreType *cpu);
extern int op_exec_stb(TargetCoreType *cpu);
extern int op_exec_sthw(TargetCoreType *cpu);
extern int op_exec_sstw(TargetCoreType *cpu);

/*
 * 乗算命令
 */
extern int op_exec_mulhi(TargetCoreType *cpu);
extern int op_exec_mulh_1(TargetCoreType *cpu);
extern int op_exec_mulh_2(TargetCoreType *cpu);
extern int op_exec_mulu_12(TargetCoreType *cpu);
extern int op_exec_mul_12(TargetCoreType *cpu);

/*
 * 除算命令
 */

/*
 * 算術演算命令
 */
extern int op_exec_addi(TargetCoreType *cpu);
extern int op_exec_movea(TargetCoreType *cpu);
extern int op_exec_mov_6(TargetCoreType *cpu);
extern int op_exec_movhi(TargetCoreType *cpu);
extern int op_exec_mov_1(TargetCoreType *cpu);
extern int op_exec_mov_2(TargetCoreType *cpu);

extern int op_exec_add_1(TargetCoreType *cpu);
extern int op_exec_add_2(TargetCoreType *cpu);

extern int op_exec_sub(TargetCoreType *cpu);
extern int op_exec_subr(TargetCoreType *cpu);
extern int op_exec_setf(TargetCoreType *cpu);

extern int op_exec_div(TargetCoreType *cpu);
extern int op_exec_divu(TargetCoreType *cpu);
extern int op_exec_divhu(TargetCoreType *cpu);
extern int op_exec_divh_11(TargetCoreType *cpu);
extern int op_exec_divh_1(TargetCoreType *cpu);
extern int op_exec_mul(TargetCoreType *cpu);
extern int op_exec_mulu(TargetCoreType *cpu);

extern int op_exec_cmov_11(TargetCoreType *cpu);
extern int op_exec_cmov_12(TargetCoreType *cpu);

/*
 * 飽和演算命令
 */
extern int op_exec_satsubi(TargetCoreType *cpu);
extern int op_exec_satadd_1(TargetCoreType *cpu);
extern int op_exec_satsub_1(TargetCoreType *cpu);
extern int op_exec_satadd_2(TargetCoreType *cpu);

/*
 * 論理演算命令
 */
extern int op_exec_and(TargetCoreType *cpu);
extern int op_exec_andi(TargetCoreType *cpu);
extern int op_exec_ori(TargetCoreType *cpu);
extern int op_exec_or(TargetCoreType *cpu);
extern int op_exec_xori(TargetCoreType *cpu);
extern int op_exec_xor(TargetCoreType *cpu);
extern int op_exec_not(TargetCoreType *cpu);
extern int op_exec_sxh(TargetCoreType *cpu);
extern int op_exec_sxb(TargetCoreType *cpu);
extern int op_exec_shl_2(TargetCoreType *cpu);
extern int op_exec_shr_2(TargetCoreType *cpu);
extern int op_exec_sar_2(TargetCoreType *cpu);
extern int op_exec_shl_9(TargetCoreType *cpu);
extern int op_exec_shr_9(TargetCoreType *cpu);
extern int op_exec_sar_9(TargetCoreType *cpu);
extern int op_exec_tst(TargetCoreType *cpu);
extern int op_exec_zxb(TargetCoreType *cpu);
extern int op_exec_zxh(TargetCoreType *cpu);

/*
 * 分岐命令
 */
extern int op_exec_jr(TargetCoreType *cpu);
extern int op_exec_jmp(TargetCoreType *cpu);
extern int op_exec_cmp_1(TargetCoreType *cpu);
extern int op_exec_cmp_2(TargetCoreType *cpu);
extern int op_exec_bcond(TargetCoreType *cpu);

/*
 * ビット命令
 */
extern int op_exec_tst1_8(TargetCoreType *cpu);
extern int op_exec_set1_8(TargetCoreType *cpu);
extern int op_exec_clr1_8(TargetCoreType *cpu);
extern int op_exec_not1_8(TargetCoreType *cpu);


extern int op_exec_tst1_9(TargetCoreType *cpu);
extern int op_exec_set1_9(TargetCoreType *cpu);
extern int op_exec_clr1_9(TargetCoreType *cpu);
extern int op_exec_not1_9(TargetCoreType *cpu);

/*
 * 特殊命令
 */
extern int op_exec_diei(TargetCoreType *cpu);
extern int op_exec_ldsr(TargetCoreType *cpu);
extern int op_exec_stsr(TargetCoreType *cpu);
extern int op_exec_nop(TargetCoreType *cpu);
extern int op_exec_reti(TargetCoreType *cpu);
extern int op_exec_halt(TargetCoreType *cpu);
extern int op_exec_trap(TargetCoreType *cpu);
extern int op_exec_switch(TargetCoreType *cpu);
extern int op_exec_prepare(TargetCoreType *cpu);
extern int op_exec_dispose(TargetCoreType *cpu);



/*
 * ディバッグ機能用命令
 */

#include "../../../../lib/dbg_log.h"


#endif /* _OP_EXEC_OPS_H_ */
