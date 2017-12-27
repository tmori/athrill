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
#define p_runisr					kernel_p_runisr
#define sus_os_cnt					kernel_sus_os_cnt
#define sus_all_cnt					kernel_sus_all_cnt

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


#ifdef TOPPERS_LABEL_ASM

/*
 *  alarm.c
 */
#define _alarm_initialize			_kernel_alarm_initialize
#define _alarm_expire				_kernel_alarm_expire

/*
 *  counter.c
 */
#define _insert_cnt_expr_que		_kernel_insert_cnt_expr_que
#define _delete_cnt_expr_que		_kernel_delete_cnt_expr_que
#define _counter_initialize			_kernel_counter_initialize
#define _counter_terminate			_kernel_counter_terminate
#define _get_reltick				_kernel_get_reltick
#define _get_abstick				_kernel_get_abstick
#define _expire_process				_kernel_expire_process

/*
 *  counter_manage.c
 */
#define _notify_hardware_counter	_kernel_notify_hardware_counter
#define _incr_counter_action		_kernel_incr_counter_action

/*
 *  event.c
 */
#define _set_event_action			_kernel_set_event_action

/*
 *  interrupt.c
 */
#define _interrupt_initialize		_kernel_interrupt_initialize
#define _release_interrupts			_kernel_release_interrupts
#define _exit_isr2					_kernel_exit_isr2
#define _p_runisr					_kernel_p_runisr
#define _sus_os_cnt					_kernel_sus_os_cnt
#define _sus_all_cnt				_kernel_sus_all_cnt

/*
 *  osctl.c
 */
#define _internal_call_errorhook	_kernel_internal_call_errorhook
#define _call_posttaskhook			_kernel_call_posttaskhook
#define _call_pretaskhook			_kernel_call_pretaskhook
#define _init_stack_magic_region	_kernel_init_stack_magic_region
#define _call_protectionhk_main		_kernel_call_protectionhk_main
#define _internal_shutdownos		_kernel_internal_shutdownos
#define _internal_call_shtdwnhk		_kernel_internal_call_shtdwnhk

/*
 *  osctl_manage.c
 */
#define _callevel_stat				_kernel_callevel_stat
#define _appmodeid					_kernel_appmodeid
#define _kerflg						_kernel_kerflg
#define _fatal_file_name			_kernel_fatal_file_name
#define _fatal_line_num				_kernel_fatal_line_num

/*
 *  resource.c
 */
#define _resource_initialize		_kernel_resource_initialize

/*
 *  scheduletable.c
 */
#define _schtbl_initialize			_kernel_schtbl_initialize
#define _schtbl_expire				_kernel_schtbl_expire
#define _schtbl_expiry_process		_kernel_schtbl_expiry_process
#define _schtbl_head				_kernel_schtbl_head
#define _schtbl_exppoint_process	_kernel_schtbl_exppoint_process
#define _schtbl_tail				_kernel_schtbl_tail

/*
 *  task.c
 */
#define _p_runtsk					_kernel_p_runtsk
#define _p_schedtsk					_kernel_p_schedtsk
#define _nextpri					_kernel_nextpri
#define _ready_queue				_kernel_ready_queue
#define _ready_primap				_kernel_ready_primap
#define _task_initialize			_kernel_task_initialize
#define _search_schedtsk			_kernel_search_schedtsk
#define _make_runnable				_kernel_make_runnable
#define _make_non_runnable			_kernel_make_non_runnable
#define _make_active				_kernel_make_active
#define _preempt					_kernel_preempt
#define _suspend					_kernel_suspend
#define _release_taskresources		_kernel_release_taskresources
#define _exit_task					_kernel_exit_task

/*
 *  task_manage.c
 */
#define _activate_task_action		_kernel_activate_task_action

/*
 *  Os_Lcfg.c
 */
#define _tnum_alarm					_kernel_tnum_alarm
#define _tnum_counter				_kernel_tnum_counter
#define _tnum_hardcounter			_kernel_tnum_hardcounter
#define _tnum_isr2					_kernel_tnum_isr2
#define _tnum_stdresource			_kernel_tnum_stdresource
#define _tnum_task					_kernel_tnum_task
#define _tnum_exttask				_kernel_tnum_exttask
#define _tnum_appmode				_kernel_tnum_appmode
#define _tnum_scheduletable			_kernel_tnum_scheduletable
#define _tnum_implscheduletable		_kernel_tnum_implscheduletable
#define _tinib_table				_kernel_tinib_table
#define _tcb_table					_kernel_tcb_table
#define _cntinib_table				_kernel_cntinib_table
#define _cntcb_table				_kernel_cntcb_table
#define _hwcntinib_table			_kernel_hwcntinib_table
#define _alminib_table				_kernel_alminib_table
#define _almcb_table				_kernel_almcb_table
#define _schtblinib_table			_kernel_schtblinib_table
#define _schtblcb_table				_kernel_schtblcb_table
#define _resinib_table				_kernel_resinib_table
#define _rescb_table				_kernel_rescb_table
#define _object_initialize			_kernel_object_initialize
#define _object_terminate			_kernel_object_terminate
#define _tnum_intno					_kernel_tnum_intno
#define _intinib_table				_kernel_intinib_table
#define _isrcb_table				_kernel_isrcb_table
#define _ostk						_kernel_ostk
#define _ostksz						_kernel_ostksz
#define _ostkpt						_kernel_ostkpt
#define _appid_str					_kernel_appid_str
#define _tskid_str					_kernel_tskid_str
#define _isrid_str					_kernel_isrid_str
#define _cntid_str					_kernel_cntid_str
#define _almid_str					_kernel_almid_str
#define _resid_str					_kernel_resid_str
#define _schtblid_str				_kernel_schtblid_str
#define _evtid_str					_kernel_evtid_str


#endif /* TOPPERS_LABEL_ASM */

#include "target_rename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
