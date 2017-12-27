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
 *  ioc_manage.c
 */
#define ioc_initialize				kernel_ioc_initialize

/*
 *  mc.c
 */
#define giant_lock					kernel_giant_lock
#define spn_lock					kernel_spn_lock
#define ioc_lock					kernel_ioc_lock
#define shutdown_reqflg				kernel_shutdown_reqflg
#define activated_cores				kernel_activated_cores
#define ccb_initialize				kernel_ccb_initialize
#define ici_handler_main			kernel_ici_handler_main
#define dispatch_request			kernel_dispatch_request
#define shutdown_request			kernel_shutdown_request
#define barrier_sync				kernel_barrier_sync
#define internal_shutdownallcores	kernel_internal_shutdownallcores
#define acquire_tsk_lock			kernel_acquire_tsk_lock
#define release_tsk_lock			kernel_release_tsk_lock
#define release_tsk_lock_and_dispatch	kernel_release_tsk_lock_and_dispatch
#define acquire_cnt_lock			kernel_acquire_cnt_lock
#define release_cnt_lock			kernel_release_cnt_lock
#define acquire_spn_lock			kernel_acquire_spn_lock
#define release_spn_lock			kernel_release_spn_lock
#define try_spn_lock				kernel_try_spn_lock
#define acquire_ioc_lock			kernel_acquire_ioc_lock
#define release_ioc_lock			kernel_release_ioc_lock



/*
 *  osctl.c
 */
#define p_inib_initialize			kernel_p_inib_initialize
#define internal_call_errorhook		kernel_internal_call_errorhook
#define call_posttaskhook			kernel_call_posttaskhook
#define call_pretaskhook			kernel_call_pretaskhook
#define init_stack_magic_region		kernel_init_stack_magic_region
#define cancel_shutdown_hook		kernel_cancel_shutdown_hook
#define callevel_chk_shutdown		kernel_callevel_chk_shutdown
#define call_protectionhk_main		kernel_call_protectionhk_main
#define internal_call_shtdwnhk		kernel_internal_call_shtdwnhk

/*
 *  osctl_manage.c
 */
#define appmodeid					kernel_appmodeid
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
 *  spinlock.c
 */
#define spinlock_initialize			kernel_spinlock_initialize
#define wrap_sus_all_int			kernel_wrap_sus_all_int
#define wrap_res_all_int			kernel_wrap_res_all_int
#define wrap_sus_os_int				kernel_wrap_sus_os_int
#define wrap_res_os_int				kernel_wrap_res_os_int
#define force_release_spinlocks		kernel_force_release_spinlocks

/*
 *  task.c
 */
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
#define tnum_ici					kernel_tnum_ici
#define tnum_stdresource			kernel_tnum_stdresource
#define tnum_task					kernel_tnum_task
#define tnum_exttask				kernel_tnum_exttask
#define tnum_appmode				kernel_tnum_appmode
#define tnum_scheduletable			kernel_tnum_scheduletable
#define tnum_implscheduletable		kernel_tnum_implscheduletable
#define tnum_osap					kernel_tnum_osap
#define tnum_spinlock				kernel_tnum_spinlock
#define p_ccb_table					kernel_p_ccb_table
#define tinib_table					kernel_tinib_table
#define p_tcb_table					kernel_p_tcb_table
#define cntinib_table				kernel_cntinib_table
#define p_cntcb_table				kernel_p_cntcb_table
#define hwcntinib_table				kernel_hwcntinib_table
#define alminib_table				kernel_alminib_table
#define p_almcb_table				kernel_p_almcb_table
#define schtblinib_table			kernel_schtblinib_table
#define p_schtblcb_table			kernel_p_schtblcb_table
#define resinib_table				kernel_resinib_table
#define p_rescb_table				kernel_p_rescb_table
#define isrinib_table				kernel_isrinib_table
#define p_isrcb_table				kernel_p_isrcb_table
#define iciinib_table				kernel_iciinib_table
#define p_iciinb_table				kernel_p_iciinb_table
#define tnum_ici_of_core			kernel_tnum_ici_of_core
#define ici_remain_stksz			kernel_ici_remain_stksz
#define osapinib_table				kernel_osapinib_table
#define spninib_table				kernel_spninib_table
#define spncb_table					kernel_spncb_table
#define object_initialize			kernel_object_initialize
#define object_terminate			kernel_object_terminate
#define tnum_intno					kernel_tnum_intno
#define intinib_table				kernel_intinib_table
#define ostksz_table				kernel_ostksz_table
#define ostk_table					kernel_ostk_table
#define tnum_ioc					kernel_tnum_ioc
#define tnum_queueioc				kernel_tnum_queueioc
#define tnum_ioc_wrapper_send		kernel_tnum_ioc_wrapper_send
#define tnum_ioc_wrapper			kernel_tnum_ioc_wrapper
#define ioc_inival_table			kernel_ioc_inival_table
#define ioccb_table					kernel_ioccb_table
#define iocinib_table				kernel_iocinib_table
#define iocwrpinib_table			kernel_iocwrpinib_table
#define appid_str					kernel_appid_str
#define tskid_str					kernel_tskid_str
#define isrid_str					kernel_isrid_str
#define cntid_str					kernel_cntid_str
#define almid_str					kernel_almid_str
#define resid_str					kernel_resid_str
#define schtblid_str				kernel_schtblid_str
#define evtid_str					kernel_evtid_str
#define osapid_str					kernel_osapid_str
#define iocid_str					kernel_iocid_str
#define isr_table					kernel_isr_table
#define isr_p_isrcb_table			kernel_isr_p_isrcb_table


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

/*
 *  ioc_manage.c
 */
#define _ioc_initialize				_kernel_ioc_initialize

/*
 *  mc.c
 */
#define _giant_lock					_kernel_giant_lock
#define _spn_lock					_kernel_spn_lock
#define _ioc_lock					_kernel_ioc_lock
#define _shutdown_reqflg			_kernel_shutdown_reqflg
#define _activated_cores			_kernel_activated_cores
#define _ccb_initialize				_kernel_ccb_initialize
#define _ici_handler_main			_kernel_ici_handler_main
#define _dispatch_request			_kernel_dispatch_request
#define _shutdown_request			_kernel_shutdown_request
#define _barrier_sync				_kernel_barrier_sync
#define _internal_shutdownallcores	_kernel_internal_shutdownallcores
#define _acquire_tsk_lock			_kernel_acquire_tsk_lock
#define _release_tsk_lock			_kernel_release_tsk_lock
#define _release_tsk_lock_and_dispatch	_kernel_release_tsk_lock_and_dispatch
#define _acquire_cnt_lock			_kernel_acquire_cnt_lock
#define _release_cnt_lock			_kernel_release_cnt_lock
#define _acquire_spn_lock			_kernel_acquire_spn_lock
#define _release_spn_lock			_kernel_release_spn_lock
#define _try_spn_lock				_kernel_try_spn_lock
#define _acquire_ioc_lock			_kernel_acquire_ioc_lock
#define _release_ioc_lock			_kernel_release_ioc_lock



/*
 *  osctl.c
 */
#define _p_inib_initialize			_kernel_p_inib_initialize
#define _internal_call_errorhook	_kernel_internal_call_errorhook
#define _call_posttaskhook			_kernel_call_posttaskhook
#define _call_pretaskhook			_kernel_call_pretaskhook
#define _init_stack_magic_region	_kernel_init_stack_magic_region
#define _cancel_shutdown_hook		_kernel_cancel_shutdown_hook
#define _callevel_chk_shutdown		_kernel_callevel_chk_shutdown
#define _call_protectionhk_main		_kernel_call_protectionhk_main
#define _internal_call_shtdwnhk		_kernel_internal_call_shtdwnhk

/*
 *  osctl_manage.c
 */
#define _appmodeid					_kernel_appmodeid
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
 *  spinlock.c
 */
#define _spinlock_initialize		_kernel_spinlock_initialize
#define _wrap_sus_all_int			_kernel_wrap_sus_all_int
#define _wrap_res_all_int			_kernel_wrap_res_all_int
#define _wrap_sus_os_int			_kernel_wrap_sus_os_int
#define _wrap_res_os_int			_kernel_wrap_res_os_int
#define _force_release_spinlocks	_kernel_force_release_spinlocks

/*
 *  task.c
 */
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
#define _tnum_ici					_kernel_tnum_ici
#define _tnum_stdresource			_kernel_tnum_stdresource
#define _tnum_task					_kernel_tnum_task
#define _tnum_exttask				_kernel_tnum_exttask
#define _tnum_appmode				_kernel_tnum_appmode
#define _tnum_scheduletable			_kernel_tnum_scheduletable
#define _tnum_implscheduletable		_kernel_tnum_implscheduletable
#define _tnum_osap					_kernel_tnum_osap
#define _tnum_spinlock				_kernel_tnum_spinlock
#define _p_ccb_table				_kernel_p_ccb_table
#define _tinib_table				_kernel_tinib_table
#define _p_tcb_table				_kernel_p_tcb_table
#define _cntinib_table				_kernel_cntinib_table
#define _p_cntcb_table				_kernel_p_cntcb_table
#define _hwcntinib_table			_kernel_hwcntinib_table
#define _alminib_table				_kernel_alminib_table
#define _p_almcb_table				_kernel_p_almcb_table
#define _schtblinib_table			_kernel_schtblinib_table
#define _p_schtblcb_table			_kernel_p_schtblcb_table
#define _resinib_table				_kernel_resinib_table
#define _p_rescb_table				_kernel_p_rescb_table
#define _isrinib_table				_kernel_isrinib_table
#define _p_isrcb_table				_kernel_p_isrcb_table
#define _iciinib_table				_kernel_iciinib_table
#define _p_iciinb_table				_kernel_p_iciinb_table
#define _tnum_ici_of_core			_kernel_tnum_ici_of_core
#define _ici_remain_stksz			_kernel_ici_remain_stksz
#define _osapinib_table				_kernel_osapinib_table
#define _spninib_table				_kernel_spninib_table
#define _spncb_table				_kernel_spncb_table
#define _object_initialize			_kernel_object_initialize
#define _object_terminate			_kernel_object_terminate
#define _tnum_intno					_kernel_tnum_intno
#define _intinib_table				_kernel_intinib_table
#define _ostksz_table				_kernel_ostksz_table
#define _ostk_table					_kernel_ostk_table
#define _tnum_ioc					_kernel_tnum_ioc
#define _tnum_queueioc				_kernel_tnum_queueioc
#define _tnum_ioc_wrapper_send		_kernel_tnum_ioc_wrapper_send
#define _tnum_ioc_wrapper			_kernel_tnum_ioc_wrapper
#define _ioc_inival_table			_kernel_ioc_inival_table
#define _ioccb_table				_kernel_ioccb_table
#define _iocinib_table				_kernel_iocinib_table
#define _iocwrpinib_table			_kernel_iocwrpinib_table
#define _appid_str					_kernel_appid_str
#define _tskid_str					_kernel_tskid_str
#define _isrid_str					_kernel_isrid_str
#define _cntid_str					_kernel_cntid_str
#define _almid_str					_kernel_almid_str
#define _resid_str					_kernel_resid_str
#define _schtblid_str				_kernel_schtblid_str
#define _evtid_str					_kernel_evtid_str
#define _osapid_str					_kernel_osapid_str
#define _iocid_str					_kernel_iocid_str
#define _isr_table					_kernel_isr_table
#define _isr_p_isrcb_table			_kernel_isr_p_isrcb_table


#endif /* TOPPERS_LABEL_ASM */

#include "target_rename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
