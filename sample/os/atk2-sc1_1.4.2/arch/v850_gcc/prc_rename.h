/* This file is generated from prc_rename.def by genrename. */

#ifndef TOPPERS_PRC_RENAME_H
#define TOPPERS_PRC_RENAME_H

/*
 *  prc_config.c
 */
#define except_nest_cnt				kernel_except_nest_cnt
#define v850_cpu_exp_no				kernel_v850_cpu_exp_no
#define v850_cpu_exp_pc				kernel_v850_cpu_exp_pc
#define v850_cpu_exp_sp				kernel_v850_cpu_exp_sp
#define current_iintpri				kernel_current_iintpri
#define pmr_setting_tbl				kernel_pmr_setting_tbl
#define nested_lock_os_int_cnt		kernel_nested_lock_os_int_cnt
#define current_intpri				kernel_current_intpri
#define prc_hardware_initialize		kernel_prc_hardware_initialize
#define prc_initialize				kernel_prc_initialize
#define prc_terminate				kernel_prc_terminate
#define x_config_int				kernel_x_config_int
#define default_int_handler			kernel_default_int_handler
#define target_is_int_controllable	kernel_target_is_int_controllable

/*
 *  Os_Lcfg.c
 */
#define pmr_isr2_mask				kernel_pmr_isr2_mask
#define pmr_isr1_mask				kernel_pmr_isr1_mask
#define isr_tbl						kernel_isr_tbl
#define isr_p_isrcb_tbl				kernel_isr_p_isrcb_tbl
#define intbp_tbl					kernel_intbp_tbl

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
#define _except_nest_cnt			_kernel_except_nest_cnt
#define _v850_cpu_exp_no			_kernel_v850_cpu_exp_no
#define _v850_cpu_exp_pc			_kernel_v850_cpu_exp_pc
#define _v850_cpu_exp_sp			_kernel_v850_cpu_exp_sp
#define _current_iintpri			_kernel_current_iintpri
#define _pmr_setting_tbl			_kernel_pmr_setting_tbl
#define _nested_lock_os_int_cnt		_kernel_nested_lock_os_int_cnt
#define _current_intpri				_kernel_current_intpri
#define _prc_hardware_initialize	_kernel_prc_hardware_initialize
#define _prc_initialize				_kernel_prc_initialize
#define _prc_terminate				_kernel_prc_terminate
#define _x_config_int				_kernel_x_config_int
#define _default_int_handler		_kernel_default_int_handler
#define _target_is_int_controllable	_kernel_target_is_int_controllable

/*
 *  Os_Lcfg.c
 */
#define _pmr_isr2_mask				_kernel_pmr_isr2_mask
#define _pmr_isr1_mask				_kernel_pmr_isr1_mask
#define _isr_tbl					_kernel_isr_tbl
#define _isr_p_isrcb_tbl			_kernel_isr_p_isrcb_tbl
#define _intbp_tbl					_kernel_intbp_tbl

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
