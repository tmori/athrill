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
#undef p_runisr
#undef sus_os_cnt
#undef sus_all_cnt

/*
 *  osctl.c
 */
#undef internal_call_errorhook
#undef call_posttaskhook
#undef call_pretaskhook
#undef init_stack_magic_region
#undef call_protectionhk_main
#undef internal_shutdownos
#undef internal_call_shtdwnhk

/*
 *  osctl_manage.c
 */
#undef callevel_stat
#undef appmodeid
#undef kerflg
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
 *  task.c
 */
#undef p_runtsk
#undef p_schedtsk
#undef nextpri
#undef ready_queue
#undef ready_primap
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
#undef tnum_stdresource
#undef tnum_task
#undef tnum_exttask
#undef tnum_appmode
#undef tnum_scheduletable
#undef tnum_implscheduletable
#undef tinib_table
#undef tcb_table
#undef cntinib_table
#undef cntcb_table
#undef hwcntinib_table
#undef alminib_table
#undef almcb_table
#undef schtblinib_table
#undef schtblcb_table
#undef resinib_table
#undef rescb_table
#undef object_initialize
#undef object_terminate
#undef tnum_intno
#undef intinib_table
#undef isrcb_table
#undef ostk
#undef ostksz
#undef ostkpt
#undef appid_str
#undef tskid_str
#undef isrid_str
#undef cntid_str
#undef almid_str
#undef resid_str
#undef schtblid_str
#undef evtid_str


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
#undef _p_runisr
#undef _sus_os_cnt
#undef _sus_all_cnt

/*
 *  osctl.c
 */
#undef _internal_call_errorhook
#undef _call_posttaskhook
#undef _call_pretaskhook
#undef _init_stack_magic_region
#undef _call_protectionhk_main
#undef _internal_shutdownos
#undef _internal_call_shtdwnhk

/*
 *  osctl_manage.c
 */
#undef _callevel_stat
#undef _appmodeid
#undef _kerflg
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
 *  task.c
 */
#undef _p_runtsk
#undef _p_schedtsk
#undef _nextpri
#undef _ready_queue
#undef _ready_primap
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
#undef _tnum_stdresource
#undef _tnum_task
#undef _tnum_exttask
#undef _tnum_appmode
#undef _tnum_scheduletable
#undef _tnum_implscheduletable
#undef _tinib_table
#undef _tcb_table
#undef _cntinib_table
#undef _cntcb_table
#undef _hwcntinib_table
#undef _alminib_table
#undef _almcb_table
#undef _schtblinib_table
#undef _schtblcb_table
#undef _resinib_table
#undef _rescb_table
#undef _object_initialize
#undef _object_terminate
#undef _tnum_intno
#undef _intinib_table
#undef _isrcb_table
#undef _ostk
#undef _ostksz
#undef _ostkpt
#undef _appid_str
#undef _tskid_str
#undef _isrid_str
#undef _cntid_str
#undef _almid_str
#undef _resid_str
#undef _schtblid_str
#undef _evtid_str


#endif /* TOPPERS_LABEL_ASM */

#include "target_unrename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
