/* This file is generated from prc_rename.def by genrename. */

/* This file is included only when prc_rename.h has been included. */
#ifdef TOPPERS_PRC_RENAME_H
#undef TOPPERS_PRC_RENAME_H

/*
 *  prc_config.c
 */
#undef v850_cpu_exp_no_table
#undef v850_cpu_exp_pc_table
#undef v850_cpu_exp_sp_table
#undef core_state_table
#undef pmr_setting_tbl
#undef prc_hardware_initialize
#undef prc_initialize
#undef prc_terminate
#undef x_config_int
#undef default_int_handler
#undef infinite_loop
#undef target_is_int_controllable

/*
 *  Os_Lcfg.c
 */
#undef intbp_table
#undef pmr_isr2_mask
#undef pmr_isr1_mask
#undef ostkpt_table

/*
 *  prc_support.S
 */
#undef fe_exception_entry
#undef ei_exception_entry
#undef interrupt
#undef dispatch
#undef start_dispatch
#undef exit_and_dispatch
#undef start_r
#undef stack_change_and_call_func_1
#undef stack_change_and_call_func_2
#undef return_main

#ifdef TOPPERS_LABEL_ASM

/*
 *  prc_config.c
 */
#undef _v850_cpu_exp_no_table
#undef _v850_cpu_exp_pc_table
#undef _v850_cpu_exp_sp_table
#undef _core_state_table
#undef _pmr_setting_tbl
#undef _prc_hardware_initialize
#undef _prc_initialize
#undef _prc_terminate
#undef _x_config_int
#undef _default_int_handler
#undef _infinite_loop
#undef _target_is_int_controllable

/*
 *  Os_Lcfg.c
 */
#undef _intbp_table
#undef _pmr_isr2_mask
#undef _pmr_isr1_mask
#undef _ostkpt_table

/*
 *  prc_support.S
 */
#undef _fe_exception_entry
#undef _ei_exception_entry
#undef _interrupt
#undef _dispatch
#undef _start_dispatch
#undef _exit_and_dispatch
#undef _start_r
#undef _stack_change_and_call_func_1
#undef _stack_change_and_call_func_2
#undef _return_main

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_PRC_RENAME_H */
