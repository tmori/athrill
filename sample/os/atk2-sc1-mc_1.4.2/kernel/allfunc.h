/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2011-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2017 by Witz Corporation
 *  Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: allfunc.h 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		すべての関数をコンパイルするための定義
 */

#ifndef TOPPERS_ALLFUNC_H
#define TOPPERS_ALLFUNC_H

/* alarm.c */
#define TOPPERS_alarm_initialize
#define TOPPERS_GetAlarmBase
#define TOPPERS_GetAlarm
#define TOPPERS_SetRelAlarm
#define TOPPERS_SetAbsAlarm
#define TOPPERS_CancelAlarm
#define TOPPERS_alarm_expire

/* counter_manage.c */
#define TOPPERS_IncrementCounter
#define TOPPERS_GetCounterValue
#define TOPPERS_GetElapsedValue
#define TOPPERS_notify_hardware_counter
#define TOPPERS_incr_counter_action

/* counter.c */
#define TOPPERS_insert_cnt_expr_que
#define TOPPERS_delete_cnt_expr_que
#define TOPPERS_counter_initialize
#define TOPPERS_counter_terminate
#define TOPPERS_get_reltick
#define TOPPERS_get_abstick
#define TOPPERS_expire_process

/* event.c */
#define TOPPERS_SetEvent
#define TOPPERS_ClearEvent
#define TOPPERS_GetEvent
#define TOPPERS_WaitEvent
#define TOPPERS_set_event_action

/* interrupt.c */
#define TOPPERS_interrupt_initialize
#define TOPPERS_release_interrupts
#define TOPPERS_exit_isr2

/* interrupt_manage.c */
#define TOPPERS_DisableAllInterrupts
#define TOPPERS_EnableAllInterrupts
#define TOPPERS_SuspendAllInterrupts
#define TOPPERS_ResumeAllInterrupts
#define TOPPERS_SuspendOSInterrupts
#define TOPPERS_ResumeOSInterrupts
#define TOPPERS_GetISRID
#define TOPPERS_DisableInterruptSource
#define TOPPERS_EnableInterruptSource

/* osctl.c */
#define TOPPERS_internal_call_errorhook
#define TOPPERS_call_posttaskhook
#define TOPPERS_call_pretaskhook
#define TOPPERS_cancel_shutdown_hook
#define TOPPERS_callevel_chk_shutdown
#define TOPPERS_call_protectionhk_main
#define TOPPERS_init_stack_magic_region
#define TOPPERS_internal_call_shtdwnhk
#define TOPPERS_p_inib_initialize
#define TOPPERS_get_error_svcid
#define TOPPERS_get_error_par

/* osctl_manage.c */
#define TOPPERS_StartOS
#define TOPPERS_GetActiveApplicationMode
#define TOPPERS_GetFaultyContext

/* resource.c */
#define TOPPERS_resource_initialize
#define TOPPERS_GetResource
#define TOPPERS_ReleaseResource

/* scheduletable.c */
#define TOPPERS_schtbl_initialize
#define TOPPERS_StartScheduleTableRel
#define TOPPERS_StartScheduleTableAbs
#define TOPPERS_StopScheduleTable
#define TOPPERS_NextScheduleTable
#define TOPPERS_GetScheduleTableStatus
#define TOPPERS_schtbl_expire
#define TOPPERS_schtbl_expiry_process
#define TOPPERS_schtbl_head
#define TOPPERS_schtbl_exppoint_process
#define TOPPERS_schtbl_tail

/* task.c */
#define TOPPERS_task_initialize
#define TOPPERS_search_schedtsk
#define TOPPERS_make_runnable
#define TOPPERS_make_non_runnable
#define TOPPERS_make_active
#define TOPPERS_preempt
#define TOPPERS_suspend
#define TOPPERS_exit_task

/* task_manage.c */
#define TOPPERS_ActivateTask
#define TOPPERS_TerminateTask
#define TOPPERS_ChainTask
#define TOPPERS_Schedule
#define TOPPERS_GetTaskID
#define TOPPERS_GetTaskState
#define TOPPERS_activate_task_action

/* osap.c */
#define TOPPERS_GetApplicationID
#define TOPPERS_CheckTaskOwnership
#define TOPPERS_CheckISROwnership
#define TOPPERS_CheckAlarmOwnership
#define TOPPERS_CheckCounterOwnership
#define TOPPERS_CheckScheduleTableOwnership

/* mc.c */
#define TOPPERS_ccb_initialize
#define TOPPERS_ici_handler_main
#define TOPPERS_dispatch_request
#define TOPPERS_shutdown_request
#define TOPPERS_barrier_sync
#define TOPPERS_internal_shutdownallcores
#define TOPPERS_acquire_tsk_lock
#define TOPPERS_release_tsk_lock
#define TOPPERS_release_tsk_lock_and_dispatch
#define TOPPERS_acquire_cnt_lock
#define TOPPERS_release_cnt_lock
#define TOPPERS_acquire_spn_lock
#define TOPPERS_release_spn_lock
#define TOPPERS_try_spn_lock
#define TOPPERS_acquire_ioc_lock
#define TOPPERS_release_ioc_lock

/* mc_manage.c */
#define TOPPERS_RaiseInterCoreInterrupt
#define TOPPERS_GetCoreID
#define TOPPERS_StartCore
#define TOPPERS_StartNonAutosarCore
#define TOPPERS_GetNumberOfActivatedCores
#define TOPPERS_ShutdownAllCores

/* spinlock.c */
#define TOPPERS_spinlock_initialize
#define TOPPERS_wrap_sus_all_int
#define TOPPERS_wrap_res_all_int
#define TOPPERS_wrap_sus_os_int
#define TOPPERS_wrap_res_os_int
#define TOPPERS_GetSpinlock
#define TOPPERS_ReleaseSpinlock
#define TOPPERS_TryToGetSpinlock
#define TOPPERS_force_release_spinlocks

#endif /* TOPPERS_ALLFUNC_H */
