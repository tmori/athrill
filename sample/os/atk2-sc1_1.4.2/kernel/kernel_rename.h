/* This file is generated from kernel_rename.def by genrename. */

#ifndef TOPPERS_KERNEL_RENAME_H
#define TOPPERS_KERNEL_RENAME_H

/*
 *  alarm.c
 */
#define alarm_initialize			kernel_alarm_initialize
#define alarm_expire				kernel_alarm_expire

/*
 *  counter.c
 */
#define insert_cnt_expr_que			kernel_insert_cnt_expr_que
#define delete_cnt_expr_que			kernel_delete_cnt_expr_que
#define counter_initialize			kernel_counter_initialize
#define counter_terminate			kernel_counter_terminate
#define get_reltick					kernel_get_reltick
#define get_abstick					kernel_get_abstick
#define expire_process				kernel_expire_process

/*
 *  counter_manage.c
 */
#define notify_hardware_counter		kernel_notify_hardware_counter
#define incr_counter_action			kernel_incr_counter_action

/*
 *  event.c
 */
#define set_event_action			kernel_set_event_action

/*
 *  interrupt.c
 */
#define interrupt_initialize		kernel_interrupt_initialize
#define release_interrupts			kernel_release_interrupts
#define exit_isr2					kernel_exit_isr2


/*
 *  osctl.c
 */
#define internal_call_errorhook		kernel_internal_call_errorhook
#define call_posttaskhook			kernel_call_posttaskhook
#define call_pretaskhook			kernel_call_pretaskhook
#define init_stack_magic_region		kernel_init_stack_magic_region
#define call_protectionhk_main		kernel_call_protectionhk_main
#define internal_shutdownos			kernel_internal_shutdownos
#define internal_call_shtdwnhk		kernel_internal_call_shtdwnhk

/*
 *  osctl_manage.c
 */
#define callevel_stat				kernel_callevel_stat
#define appmodeid					kernel_appmodeid
#define kerflg						kernel_kerflg
#define fatal_file_name				kernel_fatal_file_name
#define fatal_line_num				kernel_fatal_line_num

/*
 *  resource.c
 */
#define resource_initialize			kernel_resource_initialize

/*
 *  scheduletable.c
 */
#define schtbl_initialize			kernel_schtbl_initialize
#define schtbl_expire				kernel_schtbl_expire
#define schtbl_expiry_process		kernel_schtbl_expiry_process
#define schtbl_head					kernel_schtbl_head
#define schtbl_exppoint_process		kernel_schtbl_exppoint_process
#define schtbl_tail					kernel_schtbl_tail

/*
 *  task.c
 */
#define p_runtsk					kernel_p_runtsk
#define p_schedtsk					kernel_p_schedtsk
#define nextpri						kernel_nextpri
#define ready_queue					kernel_ready_queue
#define ready_primap				kernel_ready_primap
#define task_initialize				kernel_task_initialize
#define search_schedtsk				kernel_search_schedtsk
#define make_runnable				kernel_make_runnable
#define make_non_runnable			kernel_make_non_runnable
#define make_active					kernel_make_active
#define preempt						kernel_preempt
#define suspend						kernel_suspend
#define release_taskresources		kernel_release_taskresources
#define exit_task					kernel_exit_task

/*
 *  task_manage.c
 */
#define activate_task_action		kernel_activate_task_action

/*
 *  Os_Lcfg.c
 */
#define tnum_alarm					kernel_tnum_alarm
#define tnum_counter				kernel_tnum_counter
#define tnum_hardcounter			kernel_tnum_hardcounter
#define tnum_isr2					kernel_tnum_isr2
#define tnum_stdresource			kernel_tnum_stdresource
#define tnum_task					kernel_tnum_task
#define tnum_exttask				kernel_tnum_exttask
#define tnum_appmode				kernel_tnum_appmode
#define tnum_scheduletable			kernel_tnum_scheduletable
#define tnum_implscheduletable		kernel_tnum_implscheduletable
#define tinib_table					kernel_tinib_table
#define tcb_table					kernel_tcb_table
#define cntinib_table				kernel_cntinib_table
#define cntcb_table					kernel_cntcb_table
#define hwcntinib_table				kernel_hwcntinib_table
#define alminib_table				kernel_alminib_table
#define almcb_table					kernel_almcb_table
#define schtblinib_table			kernel_schtblinib_table
#define schtblcb_table				kernel_schtblcb_table
#define resinib_table				kernel_resinib_table
#define rescb_table					kernel_rescb_table
#define object_initialize			kernel_object_initialize
#define object_terminate			kernel_object_terminate
#define tnum_intno					kernel_tnum_intno
#define intinib_table				kernel_intinib_table
#define isrcb_table					kernel_isrcb_table
#define ostk						kernel_ostk
#define ostksz						kernel_ostksz
#define ostkpt						kernel_ostkpt
#define appid_str					kernel_appid_str
#define tskid_str					kernel_tskid_str
#define isrid_str					kernel_isrid_str
#define cntid_str					kernel_cntid_str
#define almid_str					kernel_almid_str
#define resid_str					kernel_resid_str
#define schtblid_str				kernel_schtblid_str
#define evtid_str					kernel_evtid_str


#include "target_rename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
