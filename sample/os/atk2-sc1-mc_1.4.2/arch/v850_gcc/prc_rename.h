/* This file is generated from prc_rename.def by genrename. */

#ifndef TOPPERS_PRC_RENAME_H
#define TOPPERS_PRC_RENAME_H

/*
 *  prc_config.c
 */
#define v850_cpu_exp_no_table		kernel_v850_cpu_exp_no_table
#define v850_cpu_exp_pc_table		kernel_v850_cpu_exp_pc_table
#define v850_cpu_exp_sp_table		kernel_v850_cpu_exp_sp_table
#define core_state_table			kernel_core_state_table
#define pmr_setting_tbl				kernel_pmr_setting_tbl
#define prc_hardware_initialize		kernel_prc_hardware_initialize
#define prc_initialize				kernel_prc_initialize
#define prc_terminate				kernel_prc_terminate
#define x_config_int				kernel_x_config_int
#define default_int_handler			kernel_default_int_handler
#define infinite_loop				kernel_infinite_loop
#define target_is_int_controllable	kernel_target_is_int_controllable

/*
 *  Os_Lcfg.c
 */
#define intbp_table					kernel_intbp_table
#define pmr_isr2_mask				kernel_pmr_isr2_mask
#define pmr_isr1_mask				kernel_pmr_isr1_mask
#define ostkpt_table				kernel_ostkpt_table

/*
 *  prc_support.S
 */
#define fe_exception_entry			kernel_fe_exception_entry
#define ei_exception_entry			kernel_ei_exception_entry
#define interrupt					kernel_interrupt
#define dispatch					kernel_dispatch
#define start_dispatch				kernel_start_dispatch
#define exit_and_dispatch			kernel_exit_and_dispatch
#define start_r						kernel_start_r
#define stack_change_and_call_func_1	kernel_stack_change_and_call_func_1
#define stack_change_and_call_func_2	kernel_stack_change_and_call_func_2
#define return_main					kernel_return_main

#ifdef TOPPERS_LABEL_ASM

/*
 *  prc_config.c
 */
#define _v850_cpu_exp_no_table		_kernel_v850_cpu_exp_no_table
#define _v850_cpu_exp_pc_table		_kernel_v850_cpu_exp_pc_table
#define _v850_cpu_exp_sp_table		_kernel_v850_cpu_exp_sp_table
#define _core_state_table			_kernel_core_state_table
#define _pmr_setting_tbl			_kernel_pmr_setting_tbl
#define _prc_hardware_initialize	_kernel_prc_hardware_initialize
#define _prc_initialize				_kernel_prc_initialize
#define _prc_terminate				_kernel_prc_terminate
#define _x_config_int				_kernel_x_config_int
#define _default_int_handler		_kernel_default_int_handler
#define _infinite_loop				_kernel_infinite_loop
#define _target_is_int_controllable	_kernel_target_is_int_controllable

/*
 *  Os_Lcfg.c
 */
#define _intbp_table				_kernel_intbp_table
#define _pmr_isr2_mask				_kernel_pmr_isr2_mask
#define _pmr_isr1_mask				_kernel_pmr_isr1_mask
#define _ostkpt_table				_kernel_ostkpt_table

/*
 *  prc_support.S
 */
#define _fe_exception_entry			_kernel_fe_exception_entry
#define _ei_exception_entry			_kernel_ei_exception_entry
#define _interrupt					_kernel_interrupt
#define _dispatch					_kernel_dispatch
#define _start_dispatch				_kernel_start_dispatch
#define _exit_and_dispatch			_kernel_exit_and_dispatch
#define _start_r					_kernel_start_r
#define _stack_change_and_call_func_1	_kernel_stack_change_and_call_func_1
#define _stack_change_and_call_func_2	_kernel_stack_change_and_call_func_2
#define _return_main				_kernel_return_main

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_PRC_RENAME_H */
