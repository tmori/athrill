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


#include "target_unrename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
