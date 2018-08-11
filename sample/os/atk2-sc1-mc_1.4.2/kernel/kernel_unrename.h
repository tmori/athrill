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


#include "target_unrename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
