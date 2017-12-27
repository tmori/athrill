/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2011-2013 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2013 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by FUJITSU VLSI LIMITED, JAPAN
 *  Copyright (C) 2011-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2013 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2013 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2013 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2013 by Witz Corporation, JAPAN
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
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: tool_svc.h 115 2014-12-10 01:33:17Z t_ishikawa $
 */

/*  This file is generated from svc.def by gensvc_atk. */

#ifndef TOPPERS_TOOL_SVC_H
#define TOPPERS_TOOL_SVC_H

#define SVC_TRAP(system_service)	_trap_ ## system_service

#ifndef TOPPERS_SVC_FUNCCALL
#define GetCoreID					_trap_GetCoreID
#define GetNumberOfActivatedCores	_trap_GetNumberOfActivatedCores
#define StartCore					_trap_StartCore
#define StartNonAutosarCore			_trap_StartNonAutosarCore
#define StartOS						_trap_StartOS
/*#define ShutdownOS					_trap_ShutdownOS*/
#define ShutdownAllCores			_trap_ShutdownAllCores
#define ActivateTask				_trap_ActivateTask
#define TerminateTask				_trap_TerminateTask
#define ChainTask					_trap_ChainTask
#define Schedule					_trap_Schedule
#define GetTaskID					_trap_GetTaskID
#define GetTaskState				_trap_GetTaskState
#define EnableAllInterrupts			_trap_EnableAllInterrupts
#define DisableAllInterrupts		_kernel_DisableAllInterrupts
#define ResumeAllInterrupts			_trap_ResumeAllInterrupts
#define SuspendAllInterrupts		_trap_SuspendAllInterrupts
#define ResumeOSInterrupts			_trap_ResumeOSInterrupts
#define SuspendOSInterrupts			_trap_SuspendOSInterrupts
#define GetISRID					_trap_GetISRID
#define DisableInterruptSource		_trap_DisableInterruptSource
#define EnableInterruptSource		_trap_EnableInterruptSource
#define GetResource					_trap_GetResource
#define ReleaseResource				_trap_ReleaseResource
#define SetEvent					_trap_SetEvent
#define ClearEvent					_trap_ClearEvent
#define GetEvent					_trap_GetEvent
#define WaitEvent					_trap_WaitEvent
#define GetAlarmBase				_trap_GetAlarmBase
#define GetAlarm					_trap_GetAlarm
#define SetRelAlarm					_trap_SetRelAlarm
#define SetAbsAlarm					_trap_SetAbsAlarm
#define CancelAlarm					_trap_CancelAlarm
#define IncrementCounter			_trap_IncrementCounter
#define GetCounterValue				_trap_GetCounterValue
#define GetElapsedValue				_trap_GetElapsedValue
#define CheckISRMemoryAccess		_trap_CheckISRMemoryAccess
#define CheckTaskMemoryAccess		_trap_CheckTaskMemoryAccess
#define CheckTaskAccess				_trap_CheckTaskAccess
#define CheckISRAccess				_trap_CheckISRAccess
#define CheckAlarmAccess			_trap_CheckAlarmAccess
#define CheckResourceAccess			_trap_CheckResourceAccess
#define CheckCounterAccess			_trap_CheckCounterAccess
#define CheckScheduleTableAccess	_trap_CheckScheduleTableAccess
#define CheckSpinlockAccess			_trap_CheckSpinlockAccess
#define CheckTaskOwnership			_trap_CheckTaskOwnership
#define CheckISROwnership			_trap_CheckISROwnership
#define CheckAlarmOwnership			_trap_CheckAlarmOwnership
#define CheckCounterOwnership		_trap_CheckCounterOwnership
#define CheckScheduleTableOwnership	_trap_CheckScheduleTableOwnership
#define GetApplicationID			_trap_GetApplicationID
#define CallTrustedFunction			_trap_CallTrustedFunction
#define GetApplicationState			_trap_GetApplicationState
#define GetActiveApplicationMode	_trap_GetActiveApplicationMode
#define StartScheduleTableRel		_trap_StartScheduleTableRel
#define StartScheduleTableAbs		_trap_StartScheduleTableAbs
#define StopScheduleTable			_trap_StopScheduleTable
#define NextScheduleTable			_trap_NextScheduleTable
#define GetScheduleTableStatus		_trap_GetScheduleTableStatus
#define GetFaultyContext			_trap_GetFaultyContext
#define GetSpinlock					_trap_GetSpinlock
#define ReleaseSpinlock				_trap_ReleaseSpinlock
#define TryToGetSpinlock			_trap_TryToGetSpinlock
#define get_error_svcid				_trap_get_error_svcid
#define get_error_par				_trap_get_error_par
#define ioc_send_generic			_trap_ioc_send_generic
#define ioc_write_generic			_trap_ioc_write_generic
#define ioc_receive_generic			_trap_ioc_receive_generic
#define ioc_read_generic			_trap_ioc_read_generic
#define ioc_empty_queue_generic		_trap_ioc_empty_queue_generic
#define RaiseInterCoreInterrupt		_trap_RaiseInterCoreInterrupt
#define AllowAccess					_trap_AllowAccess
#define TerminateApplication		_trap_TerminateApplication
#endif

#ifndef TOPPERS_MACRO_ONLY

LOCAL_INLINE CoreIdType
_trap_GetCoreID(void)
{
	return CAL_SVC_0M(CoreIdType, TFN_GETCOREID);
}

LOCAL_INLINE uint32
_trap_GetNumberOfActivatedCores(void)
{
	return CAL_SVC_0M(uint32, TFN_GETNUMBEROFACTIVATEDCORES);
}

LOCAL_INLINE void
_trap_StartCore(CoreIdType CoreID, StatusType *Status)
{
	CAL_SVC_2N(void, TFN_STARTCORE, CoreIdType, CoreID, StatusType *, Status);
}

LOCAL_INLINE void
_trap_StartNonAutosarCore(CoreIdType CoreID, StatusType *Status)
{
	CAL_SVC_2N(void, TFN_STARTNONAUTOSARCORE, CoreIdType, CoreID, StatusType *, Status);
}

LOCAL_INLINE void
_trap_StartOS(AppModeType Mode)
{
	CAL_SVC_1N(void, TFN_STARTOS, AppModeType, Mode);
}

#if 0
LOCAL_INLINE void
_trap_ShutdownOS(StatusType Error)
{
	CAL_SVC_1N(void, TFN_SHUTDOWNOS, StatusType, Error);
}
#endif

LOCAL_INLINE void
_trap_ShutdownAllCores(StatusType Error)
{
	CAL_SVC_1N(void, TFN_SHUTDOWNALLCORES, StatusType, Error);
}

LOCAL_INLINE StatusType
_trap_ActivateTask(TaskType TaskID)
{
	return CAL_SVC_1M(StatusType, TFN_ACTIVATETASK, TaskType, TaskID);
}

LOCAL_INLINE StatusType
_trap_TerminateTask(void)
{
	return CAL_SVC_0M(StatusType, TFN_TERMINATETASK);
}

LOCAL_INLINE StatusType
_trap_ChainTask(TaskType TaskID)
{
	return CAL_SVC_1M(StatusType, TFN_CHAINTASK, TaskType, TaskID);
}

LOCAL_INLINE StatusType
_trap_Schedule(void)
{
	return CAL_SVC_0M(StatusType, TFN_SCHEDULE);
}

LOCAL_INLINE StatusType
_trap_GetTaskID(TaskRefType TaskID)
{
	return CAL_SVC_1M(StatusType, TFN_GETTASKID, TaskRefType, TaskID);
}

LOCAL_INLINE StatusType
_trap_GetTaskState(TaskType TaskID, TaskStateRefType State)
{
	return CAL_SVC_2M(StatusType, TFN_GETTASKSTATE, TaskType, TaskID, TaskStateRefType, State);
}

LOCAL_INLINE void
_trap_EnableAllInterrupts(void)
{
	CAL_SVC_0N(void, TFN_ENABLEALLINTERRUPTS);
}

LOCAL_INLINE void
_trap_DisableAllInterrupts(void)
{
	_kernel_DisableAllInterrupts();
}

LOCAL_INLINE void
_trap_ResumeAllInterrupts(void)
{
	CAL_SVC_0N(void, TFN_RESUMEALLINTERRUPTS);
}

LOCAL_INLINE void
_trap_SuspendAllInterrupts(void)
{
	CAL_SVC_0N(void, TFN_SUSPENDALLINTERRUPTS);
}

LOCAL_INLINE void
_trap_ResumeOSInterrupts(void)
{
	CAL_SVC_0N(void, TFN_RESUMEOSINTERRUPTS);
}

LOCAL_INLINE void
_trap_SuspendOSInterrupts(void)
{
	CAL_SVC_0N(void, TFN_SUSPENDOSINTERRUPTS);
}

LOCAL_INLINE ISRType
_trap_GetISRID(void)
{
	return CAL_SVC_0M(ISRType, TFN_GETISRID);
}

LOCAL_INLINE StatusType
_trap_DisableInterruptSource(ISRType DisableISR)
{
	return CAL_SVC_1M(StatusType, TFN_DISABLEINTERRUPTSOURCE, ISRType, DisableISR);
}

LOCAL_INLINE StatusType
_trap_EnableInterruptSource(ISRType EnableISR)
{
	return CAL_SVC_1M(StatusType, TFN_ENABLEINTERRUPTSOURCE, ISRType, EnableISR);
}

LOCAL_INLINE StatusType
_trap_GetResource(ResourceType ResID)
{
	return CAL_SVC_1M(StatusType, TFN_GETRESOURCE, ResourceType, ResID);
}

LOCAL_INLINE StatusType
_trap_ReleaseResource(ResourceType ResID)
{
	return CAL_SVC_1M(StatusType, TFN_RELEASERESOURCE, ResourceType, ResID);
}

LOCAL_INLINE StatusType
_trap_SetEvent(TaskType TaskID, EventMaskType Mask)
{
	return CAL_SVC_2M(StatusType, TFN_SETEVENT, TaskType, TaskID, EventMaskType, Mask);
}

LOCAL_INLINE StatusType
_trap_ClearEvent(EventMaskType Mask)
{
	return CAL_SVC_1M(StatusType, TFN_CLEAREVENT, EventMaskType, Mask);
}

LOCAL_INLINE StatusType
_trap_GetEvent(TaskType TaskID, EventMaskRefType Event)
{
	return CAL_SVC_2M(StatusType, TFN_GETEVENT, TaskType, TaskID, EventMaskRefType, Event);
}

LOCAL_INLINE StatusType
_trap_WaitEvent(EventMaskType Mask)
{
	return CAL_SVC_1M(StatusType, TFN_WAITEVENT, EventMaskType, Mask);
}

LOCAL_INLINE StatusType
_trap_GetAlarmBase(AlarmType AlarmID, AlarmBaseRefType Info)
{
	return CAL_SVC_2M(StatusType, TFN_GETALARMBASE, AlarmType, AlarmID, AlarmBaseRefType, Info);
}

LOCAL_INLINE StatusType
_trap_GetAlarm(AlarmType AlarmID, TickRefType Tick)
{
	return CAL_SVC_2M(StatusType, TFN_GETALARM, AlarmType, AlarmID, TickRefType, Tick);
}

LOCAL_INLINE StatusType
_trap_SetRelAlarm(AlarmType AlarmID, TickType increment, TickType cycle)
{
	return CAL_SVC_3M(StatusType, TFN_SETRELALARM, AlarmType, AlarmID, TickType, increment, TickType, cycle);
}

LOCAL_INLINE StatusType
_trap_SetAbsAlarm(AlarmType AlarmID, TickType start, TickType cycle)
{
	return CAL_SVC_3M(StatusType, TFN_SETABSALARM, AlarmType, AlarmID, TickType, start, TickType, cycle);
}

LOCAL_INLINE StatusType
_trap_CancelAlarm(AlarmType AlarmID)
{
	return CAL_SVC_1M(StatusType, TFN_CANCELALARM, AlarmType, AlarmID);
}

LOCAL_INLINE StatusType
_trap_IncrementCounter(CounterType CounterID)
{
	return CAL_SVC_1M(StatusType, TFN_INCREMENTCOUNTER, CounterType, CounterID);
}

LOCAL_INLINE StatusType
_trap_GetCounterValue(CounterType CounterID, TickRefType Value)
{
	return CAL_SVC_2M(StatusType, TFN_GETCOUNTERVALUE, CounterType, CounterID, TickRefType, Value);
}

LOCAL_INLINE StatusType
_trap_GetElapsedValue(CounterType CounterID, TickRefType Value, TickRefType ElapsedValue)
{
	return CAL_SVC_3M(StatusType, TFN_GETELAPSEDVALUE, CounterType, CounterID, TickRefType, Value, TickRefType, ElapsedValue);
}

LOCAL_INLINE AccessType
_trap_CheckISRMemoryAccess(ISRType ISRID, MemoryStartAddressType Address, MemorySizeType Size)
{
	return CAL_SVC_3M(AccessType, TFN_CHECKISRMEMORYACCESS, ISRType, ISRID, MemoryStartAddressType, Address, MemorySizeType, Size);
}

LOCAL_INLINE AccessType
_trap_CheckTaskMemoryAccess(TaskType TaskID, MemoryStartAddressType Address, MemorySizeType Size)
{
	return CAL_SVC_3M(AccessType, TFN_CHECKTASKMEMORYACCESS, TaskType, TaskID, MemoryStartAddressType, Address, MemorySizeType, Size);
}

LOCAL_INLINE ObjectAccessType
_trap_CheckTaskAccess(ApplicationType ApplID, TaskType TaskID)
{
	return CAL_SVC_2M(ObjectAccessType, TFN_CHECKTASKACCESS, ApplicationType, ApplID, TaskType, TaskID);
}

LOCAL_INLINE ObjectAccessType
_trap_CheckISRAccess(ApplicationType ApplID, ISRType ISRID)
{
	return CAL_SVC_2M(ObjectAccessType, TFN_CHECKISRACCESS, ApplicationType, ApplID, ISRType, ISRID);
}

LOCAL_INLINE ObjectAccessType
_trap_CheckAlarmAccess(ApplicationType ApplID, AlarmType AlarmID)
{
	return CAL_SVC_2M(ObjectAccessType, TFN_CHECKALARMACCESS, ApplicationType, ApplID, AlarmType, AlarmID);
}

LOCAL_INLINE ObjectAccessType
_trap_CheckResourceAccess(ApplicationType ApplID, ResourceType ResID)
{
	return CAL_SVC_2M(ObjectAccessType, TFN_CHECKRESOURCEACCESS, ApplicationType, ApplID, ResourceType, ResID);
}

LOCAL_INLINE ObjectAccessType
_trap_CheckCounterAccess(ApplicationType ApplID, CounterType CounterID)
{
	return CAL_SVC_2M(ObjectAccessType, TFN_CHECKCOUNTERACCESS, ApplicationType, ApplID, CounterType, CounterID);
}

LOCAL_INLINE ObjectAccessType
_trap_CheckScheduleTableAccess(ApplicationType ApplID, ScheduleTableType ScheduleTableID)
{
	return CAL_SVC_2M(ObjectAccessType, TFN_CHECKSCHEDULETABLEACCESS, ApplicationType, ApplID, ScheduleTableType, ScheduleTableID);
}

LOCAL_INLINE ObjectAccessType
_trap_CheckSpinlockAccess(ApplicationType ApplID, SpinlockIdType SpinlockId)
{
	return CAL_SVC_2M(ObjectAccessType, TFN_CHECKSPINLOCKACCESS, ApplicationType, ApplID, SpinlockIdType, SpinlockId);
}

LOCAL_INLINE ApplicationType
_trap_CheckTaskOwnership(TaskType TaskID)
{
	return CAL_SVC_1M(ApplicationType, TFN_CHECKTASKOWNERSHIP, TaskType, TaskID);
}

LOCAL_INLINE ApplicationType
_trap_CheckISROwnership(ISRType ISRID)
{
	return CAL_SVC_1M(ApplicationType, TFN_CHECKISROWNERSHIP, ISRType, ISRID);
}

LOCAL_INLINE ApplicationType
_trap_CheckAlarmOwnership(AlarmType AlarmID)
{
	return CAL_SVC_1M(ApplicationType, TFN_CHECKALARMOWNERSHIP, AlarmType, AlarmID);
}

LOCAL_INLINE ApplicationType
_trap_CheckCounterOwnership(CounterType CounterID)
{
	return CAL_SVC_1M(ApplicationType, TFN_CHECKCOUNTEROWNERSHIP, CounterType, CounterID);
}

LOCAL_INLINE ApplicationType
_trap_CheckScheduleTableOwnership(ScheduleTableType ScheduleTableID)
{
	return CAL_SVC_1M(ApplicationType, TFN_CHECKSCHEDULETABLEOWNERSHIP, ScheduleTableType, ScheduleTableID);
}

LOCAL_INLINE ApplicationType
_trap_GetApplicationID(void)
{
	return CAL_SVC_0M(ApplicationType, TFN_GETAPPLICATIONID);
}

LOCAL_INLINE StatusType
_trap_CallTrustedFunction(TrustedFunctionIndexType FunctionIndex, TrustedFunctionParameterRefType FunctionParams)
{
	return CAL_SVC_2M(StatusType, TFN_CALLTRUSTEDFUNCTION, TrustedFunctionIndexType, FunctionIndex, TrustedFunctionParameterRefType, FunctionParams);
}

LOCAL_INLINE StatusType
_trap_GetApplicationState(ApplicationType Application, ApplicationStateRefType Value)
{
	return CAL_SVC_2M(StatusType, TFN_GETAPPLICATIONSTATE, ApplicationType, Application, ApplicationStateRefType, Value);
}

LOCAL_INLINE AppModeType
_trap_GetActiveApplicationMode(void)
{
	return CAL_SVC_0M(AppModeType, TFN_GETACTIVEAPPLICATIONMODE);
}

LOCAL_INLINE StatusType
_trap_StartScheduleTableRel(ScheduleTableType ScheduleTableID, TickType Offset)
{
	return CAL_SVC_2M(StatusType, TFN_STARTSCHEDULETABLEREL, ScheduleTableType, ScheduleTableID, TickType, Offset);
}

LOCAL_INLINE StatusType
_trap_StartScheduleTableAbs(ScheduleTableType ScheduleTableID, TickType Start)
{
	return CAL_SVC_2M(StatusType, TFN_STARTSCHEDULETABLEABS, ScheduleTableType, ScheduleTableID, TickType, Start);
}

LOCAL_INLINE StatusType
_trap_StopScheduleTable(ScheduleTableType ScheduleTableID)
{
	return CAL_SVC_1M(StatusType, TFN_STOPSCHEDULETABLE, ScheduleTableType, ScheduleTableID);
}

LOCAL_INLINE StatusType
_trap_NextScheduleTable(ScheduleTableType ScheduleTableID_From, ScheduleTableType ScheduleTableID_To)
{
	return CAL_SVC_2M(StatusType, TFN_NEXTSCHEDULETABLE, ScheduleTableType, ScheduleTableID_From, ScheduleTableType, ScheduleTableID_To);
}

LOCAL_INLINE StatusType
_trap_GetScheduleTableStatus(ScheduleTableType ScheduleTableID, ScheduleTableStatusRefType ScheduleStatus)
{
	return CAL_SVC_2M(StatusType, TFN_GETSCHEDULETABLESTATUS, ScheduleTableType, ScheduleTableID, ScheduleTableStatusRefType, ScheduleStatus);
}

LOCAL_INLINE FaultyContextType
_trap_GetFaultyContext(void)
{
	return CAL_SVC_0M(FaultyContextType, TFN_GETFAULTYCONTEXT);
}

LOCAL_INLINE StatusType
_trap_GetSpinlock(SpinlockIdType SpinlockId)
{
	return CAL_SVC_1M(StatusType, TFN_GETSPINLOCK, SpinlockIdType, SpinlockId);
}

LOCAL_INLINE StatusType
_trap_ReleaseSpinlock(SpinlockIdType SpinlockId)
{
	return CAL_SVC_1M(StatusType, TFN_RELEASESPINLOCK, SpinlockIdType, SpinlockId);
}

LOCAL_INLINE StatusType
_trap_TryToGetSpinlock(SpinlockIdType SpinlockId, TryToGetSpinlockType *Success)
{
	return CAL_SVC_2M(StatusType, TFN_TRYTOGETSPINLOCK, SpinlockIdType, SpinlockId, TryToGetSpinlockType *, Success);
}

LOCAL_INLINE OSServiceIdType
_trap_get_error_svcid(void)
{
	return CAL_SVC_0M(OSServiceIdType, TFN_GET_ERROR_SVCID);
}

LOCAL_INLINE void
_trap_get_error_par(_ErrorHook_Par *p_errorhook_par, uint8 par_num)
{
	CAL_SVC_2N(void, TFN_GET_ERROR_PAR, _ErrorHook_Par *, p_errorhook_par, uint8, par_num);
}

LOCAL_INLINE Std_ReturnType
_trap_ioc_send_generic(IocType IocWrapperId, const void *in)
{
	return CAL_SVC_2M(Std_ReturnType, TFN_IOC_SEND_GENERIC, IocType, IocWrapperId, const void *, in);
}

LOCAL_INLINE Std_ReturnType
_trap_ioc_write_generic(IocType IocWrapperId, const void *in)
{
	return CAL_SVC_2M(Std_ReturnType, TFN_IOC_WRITE_GENERIC, IocType, IocWrapperId, const void *, in);
}

LOCAL_INLINE Std_ReturnType
_trap_ioc_receive_generic(IocType IocId, void *out)
{
	return CAL_SVC_2M(Std_ReturnType, TFN_IOC_RECEIVE_GENERIC, IocType, IocId, void *, out);
}

LOCAL_INLINE Std_ReturnType
_trap_ioc_read_generic(IocType IocId, void *out)
{
	return CAL_SVC_2M(Std_ReturnType, TFN_IOC_READ_GENERIC, IocType, IocId, void *, out);
}

LOCAL_INLINE Std_ReturnType
_trap_ioc_empty_queue_generic(IocType IocId)
{
	return CAL_SVC_1M(Std_ReturnType, TFN_IOC_EMPTY_QUEUE_GENERIC, IocType, IocId);
}

LOCAL_INLINE StatusType
_trap_RaiseInterCoreInterrupt(ISRType ISRID)
{
	return CAL_SVC_1M(StatusType, TFN_RAISEINTERCOREINTERRUPT, ISRType, ISRID);
}

LOCAL_INLINE StatusType
_trap_AllowAccess(void)
{
	return CAL_SVC_0M(StatusType, TFN_ALLOWACCESS);
}

LOCAL_INLINE StatusType
_trap_TerminateApplication(ApplicationType Application, RestartType RestartOption)
{
	return CAL_SVC_2M(StatusType, TFN_TERMINATEAPPLICATION, ApplicationType, Application, RestartType, RestartOption);
}

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_TOOL_SVC_H */
