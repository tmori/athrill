/* This file is generated from kernel_rename.def by genrename. */

/* This file is included only when kernel_rename.h has been included. */
#ifdef TOPPERS_KERNEL_RENAME_H
#undef TOPPERS_KERNEL_RENAME_H

/*
 *  alarm.c
 */
#undef alarm_initialize
#undef alarm_expire

/*
 *  counter.c
 */
#undef insert_cnt_expr_que
#undef delete_cnt_expr_que
#undef counter_initialize
#undef counter_terminate
#undef get_reltick
#undef get_abstick
#undef expire_process

/*
 *  counter_manage.c
 */
#undef notify_hardware_counter
#undef incr_counter_action

/*
 *  event.c
 */
#undef set_event_action

/*
 *  interrupt.c
 */
#undef interrupt_initialize
#undef release_interrupts
#undef exit_isr2

/*
 *  ioc_manage.c
 */
#undef ioc_initialize

/*
 *  mc.c
 */
#undef giant_lock
#undef spn_lock
#undef ioc_lock
#undef shutdown_reqflg
#undef activated_cores
#undef ccb_initialize
#undef ici_handler_main
#undef dispatch_request
#undef shutdown_request
#undef barrier_sync
#undef internal_shutdownallcores
#undef acquire_tsk_lock
#undef release_tsk_lock
#undef release_tsk_lock_and_dispatch
#undef acquire_cnt_lock
#undef release_cnt_lock
#undef acquire_spn_lock
#undef release_spn_lock
#undef try_spn_lock
#undef acquire_ioc_lock
#undef release_ioc_lock



/*
 *  osctl.c
 */
#undef p_inib_initialize
#undef internal_call_errorhook
#undef call_posttaskhook
#undef call_pretaskhook
#undef init_stack_magic_region
#undef cancel_shutdown_hook
#undef callevel_chk_shutdown
#undef call_protectionhk_main
#undef internal_call_shtdwnhk

/*
 *  osctl_manage.c
 */
#undef appmodeid
#undef fatal_file_name
#undef fatal_line_num

/*
 *  resource.c
 */
#undef resource_initialize

/*
 *  scheduletable.c
 */
#undef schtbl_initialize
#undef schtbl_expire
#undef schtbl_expiry_process
#undef schtbl_head
#undef schtbl_exppoint_process
#undef schtbl_tail

/*
 *  spinlock.c
 */
#undef spinlock_initialize
#undef wrap_sus_all_int
#undef wrap_res_all_int
#undef wrap_sus_os_int
#undef wrap_res_os_int
#undef force_release_spinlocks

/*
 *  task.c
 */
#undef task_initialize
#undef search_schedtsk
#undef make_runnable
#undef make_non_runnable
#undef make_active
#undef preempt
#undef suspend
#undef release_taskresources
#undef exit_task

/*
 *  task_manage.c
 */
#undef activate_task_action

/*
 *  Os_Lcfg.c 
 */
#undef tnum_alarm
#undef tnum_counter
#undef tnum_hardcounter
#undef tnum_isr2
#undef tnum_ici
#undef tnum_stdresource
#undef tnum_task
#undef tnum_exttask
#undef tnum_appmode
#undef tnum_scheduletable
#undef tnum_implscheduletable
#undef tnum_osap
#undef tnum_spinlock
#undef p_ccb_table
#undef tinib_table
#undef p_tcb_table
#undef cntinib_table
#undef p_cntcb_table
#undef hwcntinib_table
#undef alminib_table
#undef p_almcb_table
#undef schtblinib_table
#undef p_schtblcb_table
#undef resinib_table
#undef p_rescb_table
#undef isrinib_table
#undef p_isrcb_table
#undef iciinib_table
#undef p_iciinb_table
#undef tnum_ici_of_core
#undef ici_remain_stksz
#undef osapinib_table
#undef spninib_table
#undef spncb_table
#undef object_initialize
#undef object_terminate
#undef tnum_intno
#undef intinib_table
#undef ostksz_table
#undef ostk_table
#undef tnum_ioc
#undef tnum_queueioc
#undef tnum_ioc_wrapper_send
#undef tnum_ioc_wrapper
#undef ioc_inival_table
#undef ioccb_table
#undef iocinib_table
#undef iocwrpinib_table
#undef appid_str
#undef tskid_str
#undef isrid_str
#undef cntid_str
#undef almid_str
#undef resid_str
#undef schtblid_str
#undef evtid_str
#undef osapid_str
#undef iocid_str
#undef isr_table
#undef isr_p_isrcb_table


#ifdef TOPPERS_LABEL_ASM

/*
 *  alarm.c
 */
#undef _alarm_initialize
#undef _alarm_expire

/*
 *  counter.c
 */
#undef _insert_cnt_expr_que
#undef _delete_cnt_expr_que
#undef _counter_initialize
#undef _counter_terminate
#undef _get_reltick
#undef _get_abstick
#undef _expire_process

/*
 *  counter_manage.c
 */
#undef _notify_hardware_counter
#undef _incr_counter_action

/*
 *  event.c
 */
#undef _set_event_action

/*
 *  interrupt.c
 */
#undef _interrupt_initialize
#undef _release_interrupts
#undef _exit_isr2

/*
 *  ioc_manage.c
 */
#undef _ioc_initialize

/*
 *  mc.c
 */
#undef _giant_lock
#undef _spn_lock
#undef _ioc_lock
#undef _shutdown_reqflg
#undef _activated_cores
#undef _ccb_initialize
#undef _ici_handler_main
#undef _dispatch_request
#undef _shutdown_request
#undef _barrier_sync
#undef _internal_shutdownallcores
#undef _acquire_tsk_lock
#undef _release_tsk_lock
#undef _release_tsk_lock_and_dispatch
#undef _acquire_cnt_lock
#undef _release_cnt_lock
#undef _acquire_spn_lock
#undef _release_spn_lock
#undef _try_spn_lock
#undef _acquire_ioc_lock
#undef _release_ioc_lock



/*
 *  osctl.c
 */
#undef _p_inib_initialize
#undef _internal_call_errorhook
#undef _call_posttaskhook
#undef _call_pretaskhook
#undef _init_stack_magic_region
#undef _cancel_shutdown_hook
#undef _callevel_chk_shutdown
#undef _call_protectionhk_main
#undef _internal_call_shtdwnhk

/*
 *  osctl_manage.c
 */
#undef _appmodeid
#undef _fatal_file_name
#undef _fatal_line_num

/*
 *  resource.c
 */
#undef _resource_initialize

/*
 *  scheduletable.c
 */
#undef _schtbl_initialize
#undef _schtbl_expire
#undef _schtbl_expiry_process
#undef _schtbl_head
#undef _schtbl_exppoint_process
#undef _schtbl_tail

/*
 *  spinlock.c
 */
#undef _spinlock_initialize
#undef _wrap_sus_all_int
#undef _wrap_res_all_int
#undef _wrap_sus_os_int
#undef _wrap_res_os_int
#undef _force_release_spinlocks

/*
 *  task.c
 */
#undef _task_initialize
#undef _search_schedtsk
#undef _make_runnable
#undef _make_non_runnable
#undef _make_active
#undef _preempt
#undef _suspend
#undef _release_taskresources
#undef _exit_task

/*
 *  task_manage.c
 */
#undef _activate_task_action

/*
 *  Os_Lcfg.c 
 */
#undef _tnum_alarm
#undef _tnum_counter
#undef _tnum_hardcounter
#undef _tnum_isr2
#undef _tnum_ici
#undef _tnum_stdresource
#undef _tnum_task
#undef _tnum_exttask
#undef _tnum_appmode
#undef _tnum_scheduletable
#undef _tnum_implscheduletable
#undef _tnum_osap
#undef _tnum_spinlock
#undef _p_ccb_table
#undef _tinib_table
#undef _p_tcb_table
#undef _cntinib_table
#undef _p_cntcb_table
#undef _hwcntinib_table
#undef _alminib_table
#undef _p_almcb_table
#undef _schtblinib_table
#undef _p_schtblcb_table
#undef _resinib_table
#undef _p_rescb_table
#undef _isrinib_table
#undef _p_isrcb_table
#undef _iciinib_table
#undef _p_iciinb_table
#undef _tnum_ici_of_core
#undef _ici_remain_stksz
#undef _osapinib_table
#undef _spninib_table
#undef _spncb_table
#undef _object_initialize
#undef _object_terminate
#undef _tnum_intno
#undef _intinib_table
#undef _ostksz_table
#undef _ostk_table
#undef _tnum_ioc
#undef _tnum_queueioc
#undef _tnum_ioc_wrapper_send
#undef _tnum_ioc_wrapper
#undef _ioc_inival_table
#undef _ioccb_table
#undef _iocinib_table
#undef _iocwrpinib_table
#undef _appid_str
#undef _tskid_str
#undef _isrid_str
#undef _cntid_str
#undef _almid_str
#undef _resid_str
#undef _schtblid_str
#undef _evtid_str
#undef _osapid_str
#undef _iocid_str
#undef _isr_table
#undef _isr_p_isrcb_table


#endif /* TOPPERS_LABEL_ASM */

#include "target_unrename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
