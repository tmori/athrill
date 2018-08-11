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
 *  $Id: Os.h 739 2017-01-24 10:05:05Z nces-hibino $
 */

/*
 *		ATK2 OSヘッダファイル
 *
 *  ATK2がサポートするシステムサービスの宣言と，必要なデー
 *  タ型，定数，マクロの定義を含むヘッダファイル
 *
 *  アセンブリ言語のソースファイルからこのファイルをインクルードする時
 *  は，TOPPERS_MACRO_ONLYを定義しておく
 *  これにより，マクロ定義以外を除くようになっている
 *
 *  このファイルをインクルードする前にインクルードしておくべきファイル
 *  はない
 */

#ifndef TOPPERS_OS_H
#define TOPPERS_OS_H

/*
 *  共通のデータ型・定数・マクロ
 */
#include "Std_Types.h"
#include "MemMap.h"
#include "Rte_Os_Type.h"

#if !defined(TOPPERS_CFG1_OUT) && !defined(OMIT_INCLUDE_OS_CFG)
#include "Os_Cfg.h"
#endif

/*
 *  ターゲット依存部
 */
#include "target_kernel.h"

#ifndef TOPPERS_MACRO_ONLY

/*
 *  データ型の定義
 */

/*
 *  オブジェクト番号の型定義
 */
typedef uint8	TaskStateType;              /* タスク状態 */
typedef uint32	EventMaskType;              /* イベントマスク */
typedef uint32	TickType;                   /* カウンタ値（ティック）*/
typedef uint32	AppModeType;                /* アプリケーションモード */
typedef uint8	OSServiceIdType;            /* システムサービスID */
typedef uint8	ScheduleTableStatusType;    /* スケジュールテーブル状態 */
typedef uint8	ProtectionReturnType;       /* プロテクションフックからの返り値 */
typedef uintptr	MemorySizeType;             /* メモリ領域サイズ */

typedef struct {
	TickType	maxallowedvalue;            /* カウンタ指定の最大値 */
	TickType	ticksperbase;               /* OSでは使用せず，ユーザが自由に使用する値 */
	TickType	mincycle;                   /* サイクル指定の最小値 */
} AlarmBaseType;

/*
 *  最適化するため，依存部再定義できる型
 */
#ifndef OMIT_DATA_TYPE
typedef uint32	TimeType;                   /* 時間 */
typedef uint32	AlarmType;                  /* アラームID */
typedef uint32	ResourceType;               /* リソースID */
typedef uint32	TaskType;                   /* タスクID */
typedef uint32	ISRType;                    /* ISR ID */
typedef uint32	CounterType;                /* カウンタID */
typedef uint32	ScheduleTableType;          /* スケジュールテーブルID */
typedef float32	PhysicalTimeType;           /* （ティックから時間に換算用）時間 */
#endif /* OMIT_DATA_TYPE */

typedef AlarmBaseType *				AlarmBaseRefType;
typedef TaskType *					TaskRefType;
typedef TaskStateType *				TaskStateRefType;
typedef EventMaskType *				EventMaskRefType;
typedef TickType *					TickRefType;
typedef ScheduleTableStatusType *	ScheduleTableStatusRefType;

/*
 *  保護違反を起こした処理単位の型
 */
typedef uint8 FaultyContextType;

/*
 *  OSオブジェクト宣言用のマクロ
 */
#define DeclareTask(TaskIdentifier)
#define DeclareResource(ResourceIdentifier)
#define DeclareEvent(EventIdentifier)
#define DeclareAlarm(AlarmIdentifier)

/*
 *  メインルーチン定義用のマクロ
 */
#define TASK(TaskName)		void TaskMain ## TaskName(void)
#define ISR(ISRName)		void ISRMain ## ISRName(void)
#ifndef C1ISR
#define C1ISR(ISRName)		void C1ISRMain ## ISRName(void)
#endif /* C1ISR */
#define ALARMCALLBACK(AlarmCallBackName) \
	void AlarmMain ## AlarmCallBackName(void)

/*
 *  メモリ領域確保のための型定義
 */
#ifndef TOPPERS_STK_T
#define TOPPERS_STK_T	sintptr
#endif /* TOPPERS_STK_T */
typedef	TOPPERS_STK_T StackType;    /* スタック領域を確保するための型 */

/*
 *  システムサービスパラメータ取得のための定義
 */
typedef union {
	TaskType					tskid;
	TaskRefType					p_tskid;
	TaskStateRefType			p_stat;
	ResourceType				resid;
	EventMaskType				mask;
	EventMaskRefType			p_mask;
	AlarmType					almid;
	AlarmBaseRefType			p_info;
	TickRefType					p_tick;
	TickRefType					p_val;
	TickRefType					p_eval;
	TickType					incr;
	TickType					cycle;
	TickType					start;
	AppModeType					mode;
	CounterType					cntid;
	ScheduleTableType			schtblid;
	TickType					offset;
	ScheduleTableType			schtblid_from;
	ScheduleTableType			schtblid_to;
	ScheduleTableStatusRefType	p_schtblstate;
	ISRType						isrid;
} ErrorHook_Par;

/*
 *  メモリ領域確保のためのマクロ
 *
 *  以下のTOPPERS_COUNT_SZとTOPPERS_ROUND_SZの定義は，unitが2の巾乗であ
 *  ることを仮定している．
 */
#ifndef TOPPERS_COUNT_SZ
#define TOPPERS_COUNT_SZ(sz, unit)	((((sz) + (unit)) - (1U)) / (unit))
#endif /* TOPPERS_COUNT_SZ */
#ifndef TOPPERS_ROUND_SZ
#define TOPPERS_ROUND_SZ(sz, unit)	((((sz) + (unit)) - (1U)) & (~((unit) - (1U))))
#endif /* TOPPERS_ROUND_SZ */

#define COUNT_STK_T(sz)		(TOPPERS_COUNT_SZ((sz), sizeof(StackType)))
#define ROUND_STK_T(sz)		(TOPPERS_ROUND_SZ((sz), sizeof(StackType)))

/*
 *  システムサービスAPIの宣言
 */

/*
 *  OS管理
 */
extern AppModeType GetActiveApplicationMode(void);
extern void StartOS(AppModeType Mode);
extern void ShutdownOS(StatusType Error);
extern FaultyContextType GetFaultyContext(void);

/*
 *  タスク管理
 */
extern StatusType ActivateTask(TaskType TaskID);
extern StatusType TerminateTask(void);
extern StatusType ChainTask(TaskType TaskID);
extern StatusType Schedule(void);
extern StatusType GetTaskID(TaskRefType TaskID);
extern StatusType GetTaskState(TaskType TaskID, TaskStateRefType State);

/*
 *  割込み管理
 */
extern void EnableAllInterrupts(void);
extern void DisableAllInterrupts(void);
extern void ResumeAllInterrupts(void);
extern void SuspendAllInterrupts(void);
extern void ResumeOSInterrupts(void);
extern void SuspendOSInterrupts(void);
extern ISRType GetISRID(void);
extern StatusType DisableInterruptSource(ISRType DisableISR);
extern StatusType EnableInterruptSource(ISRType EnableISR);

/*
 *  イベント管理
 */
extern StatusType SetEvent(TaskType TaskID, EventMaskType Mask);
extern StatusType ClearEvent(EventMaskType Mask);
extern StatusType GetEvent(TaskType TaskID, EventMaskRefType Event);
extern StatusType WaitEvent(EventMaskType Mask);

/*
 *  リソース管理
 */
extern StatusType GetResource(ResourceType ResID);
extern StatusType ReleaseResource(ResourceType ResID);

/*
 *  カウンタ制御
 */
extern StatusType IncrementCounter(CounterType CounterID);

/*
 *  ソフトウェアフリーランタイマ制御
 */
extern StatusType GetCounterValue(CounterType CounterID, TickRefType Value);
extern StatusType GetElapsedValue(CounterType CounterID, TickRefType Value, TickRefType ElapsedValue);

/*
 *  アラーム制御
 */
extern StatusType GetAlarmBase(AlarmType AlarmID, AlarmBaseRefType Info);
extern StatusType GetAlarm(AlarmType AlarmID, TickRefType Tick);
extern StatusType SetRelAlarm(AlarmType AlarmID, TickType increment, TickType cycle);
extern StatusType SetAbsAlarm(AlarmType AlarmID, TickType start, TickType cycle);
extern StatusType CancelAlarm(AlarmType AlarmID);

/*
 *  スケジュールテーブル制御
 */
extern StatusType StartScheduleTableRel(ScheduleTableType ScheduleTableID, TickType Offset);
extern StatusType StartScheduleTableAbs(ScheduleTableType ScheduleTableID, TickType Start);
extern StatusType StopScheduleTable(ScheduleTableType ScheduleTableID);
extern StatusType NextScheduleTable(ScheduleTableType ScheduleTableID_From, ScheduleTableType ScheduleTableID_To);
extern StatusType GetScheduleTableStatus(ScheduleTableType ScheduleTableID, ScheduleTableStatusRefType ScheduleStatus);

#if !defined(TOPPERS_CFG1_OUT) && !defined(OMIT_INCLUDE_OS_CFG) && !defined(OMIT_INCLUDE_OS_LCFG)
#include "Os_Lcfg.h"
#endif

/*
 *  フックルーチン
 */
#ifdef CFG_USE_ERRORHOOK
extern void ErrorHook(StatusType Error);
#endif /* CFG_USE_ERRORHOOK */

#ifdef CFG_USE_PRETASKHOOK
extern void PreTaskHook(void);
#endif /* CFG_USE_PRETASKHOOK */

#ifdef CFG_USE_POSTTASKHOOK
extern void PostTaskHook(void);
#endif /* CFG_USE_POSTTASKHOOK */

#ifdef CFG_USE_STARTUPHOOK
extern void StartupHook(void);
#endif /* CFG_USE_STARTUPHOOK */

#ifdef CFG_USE_SHUTDOWNHOOK
extern void ShutdownHook(StatusType Error);
#endif /* CFG_USE_SHUTDOWNHOOK */

#ifdef CFG_USE_PROTECTIONHOOK
extern ProtectionReturnType ProtectionHook(StatusType FatalError);
#endif /* CFG_USE_PROTECTIONHOOK */

/*
 *  ファイル名，行番号の参照用の変数
 */
extern const char8	*kernel_fatal_file_name;    /* ファイル名 */
extern sint32		kernel_fatal_line_num;      /* 行番号 */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  OSのエラーコード
 */
#define E_OS_ACCESS							UINT_C(1)
#define E_OS_CALLEVEL						UINT_C(2)
#define E_OS_ID								UINT_C(3)
#define E_OS_LIMIT							UINT_C(4)
#define E_OS_NOFUNC							UINT_C(5)
#define E_OS_RESOURCE						UINT_C(6)
#define E_OS_STATE							UINT_C(7)
#define E_OS_VALUE							UINT_C(8)
#define E_OS_SERVICEID						UINT_C(9)
#define E_OS_ILLEGAL_ADDRESS				UINT_C(10)
#define E_OS_MISSINGEND						UINT_C(11)
#define E_OS_DISABLEDINT					UINT_C(12)
#define E_OS_STACKFAULT						UINT_C(13)
#define E_OS_PROTECTION_MEMORY				UINT_C(14)
#define E_OS_PROTECTION_TIME_TASK			UINT_C(15)
#define E_OS_PROTECTION_TIME_ISR			UINT_C(16)
#define E_OS_PROTECTION_ARRIVAL_TASK		UINT_C(17)
#define E_OS_PROTECTION_ARRIVAL_ISR			UINT_C(18)
#define E_OS_PROTECTION_LOCKED_RESOURCE		UINT_C(19)
#define E_OS_PROTECTION_LOCKED_OSINT		UINT_C(20)
#define E_OS_PROTECTION_LOCKED_ALLINT		UINT_C(21)
#define E_OS_PROTECTION_EXCEPTION			UINT_C(22)
#define E_OS_PROTECTION_FATAL				UINT_C(23)
#define E_OS_MODE							UINT_C(24)
#define E_OS_SHUTDOWN_FATAL					UINT_C(25)
#define E_OS_PARAM_POINTER					UINT_C(26)
#define E_OS_SYS_ASSERT_FATAL				UINT_C(27)
#define E_OS_STACKINSUFFICIENT				UINT_C(28)
#define E_OS_CORE							UINT_C(29)
#define E_OS_SPINLOCK						UINT_C(30)
#define E_OS_INTERFERENCE_DEADLOCK			UINT_C(31)
#define E_OS_NESTING_DEADLOCK				UINT_C(32)
#define E_OS_SHUTDOWN_OTHER_CORE			UINT_C(33)
#define E_OS_TIMEINSUFFICIENT				UINT_C(34)
#define E_OS_PROTECTION_TIMEWINDOW			UINT_C(35)
#define E_OS_PROTECTION_COUNT_ISR			UINT_C(36)

/* AUTOSAR仕様R4.0.3との互換性考慮 */
#define OS_E_PARAM_POINTER					E_OS_PARAM_POINTER

#define ERRCODE_NUM							UINT_C(36) /* エラーコード数 */

/*
 *  その他の定数値
 */
#define UINT32_INVALID		UINT_C(0xffffffff)
#define UINT8_INVALID		UINT_C(0xff)

#define SUSPENDED			((TaskStateType) 0) /* 休止状態 */
#define RUNNING				((TaskStateType) 1) /* 実行状態 */
#define READY				((TaskStateType) 2) /* 実行可能状態 */
#define WAITING				((TaskStateType) 3) /* 待ち状態 */

/*
 *  最適化するため，依存部での再定義が必要
 */
#ifndef OMIT_DATA_TYPE
#define INVALID_TASK			((TaskType) UINT32_INVALID)
#define INVALID_ISR				((ISRType) UINT32_INVALID)
#endif /* OMIT_DATA_TYPE */
#define INVALID_APPMODETYPE	((AppModeType) UINT32_INVALID)

/*
 *  スケジュールテーブルのステータス定義
 */
#define SCHEDULETABLE_STOPPED					((ScheduleTableStatusType) 0x01)
#define SCHEDULETABLE_NEXT						((ScheduleTableStatusType) 0x02)
#define SCHEDULETABLE_WAITING					((ScheduleTableStatusType) 0x04)
#define SCHEDULETABLE_RUNNING					((ScheduleTableStatusType) 0x08)
#define SCHEDULETABLE_RUNNING_AND_SYNCHRONOUS	((ScheduleTableStatusType) 0x10)

/*
 *  システムサービスID
 */
#define OSServiceId_GetISRID					((OSServiceIdType) 0x01)
#define OSServiceId_StartScheduleTableRel		((OSServiceIdType) 0x07)
#define OSServiceId_StartScheduleTableAbs		((OSServiceIdType) 0x08)
#define OSServiceId_StopScheduleTable			((OSServiceIdType) 0x09)
#define OSServiceId_NextScheduleTable			((OSServiceIdType) 0x0a)
#define OSServiceId_GetScheduleTableStatus		((OSServiceIdType) 0x0e)
#define OSServiceId_IncrementCounter			((OSServiceIdType) 0x0f)
#define OSServiceId_GetCounterValue				((OSServiceIdType) 0x10)
#define OSServiceId_GetElapsedValue				((OSServiceIdType) 0x11)

#define OSServiceId_EnableInterruptSource		((OSServiceIdType) 0xa0)
#define OSServiceId_DisableInterruptSource		((OSServiceIdType) 0xa1)
#define OSServiceId_TaskMissingEnd				((OSServiceIdType) 0xaf)
#define OSServiceId_ISRMissingEnd				((OSServiceIdType) 0xb0)
#define OSServiceId_HookMissingEnd				((OSServiceIdType) 0xb1)

#define OSServiceId_ActivateTask				((OSServiceIdType) 0xe0)
#define OSServiceId_TerminateTask				((OSServiceIdType) 0xe1)
#define OSServiceId_ChainTask					((OSServiceIdType) 0xe2)
#define OSServiceId_Schedule					((OSServiceIdType) 0xe3)
#define OSServiceId_GetTaskID					((OSServiceIdType) 0xe4)
#define OSServiceId_GetTaskState				((OSServiceIdType) 0xe5)
#define OSServiceId_EnableAllInterrupts			((OSServiceIdType) 0xe6)
#define OSServiceId_DisableAllInterrupts		((OSServiceIdType) 0xe7)
#define OSServiceId_ResumeAllInterrupts			((OSServiceIdType) 0xe8)
#define OSServiceId_SuspendAllInterrupts		((OSServiceIdType) 0xe9)
#define OSServiceId_ResumeOSInterrupts			((OSServiceIdType) 0xea)
#define OSServiceId_SuspendOSInterrupts			((OSServiceIdType) 0xeb)
#define OSServiceId_GetResource					((OSServiceIdType) 0xec)
#define OSServiceId_ReleaseResource				((OSServiceIdType) 0xed)
#define OSServiceId_SetEvent					((OSServiceIdType) 0xee)
#define OSServiceId_ClearEvent					((OSServiceIdType) 0xef)
#define OSServiceId_GetEvent					((OSServiceIdType) 0xf0)
#define OSServiceId_WaitEvent					((OSServiceIdType) 0xf1)
#define OSServiceId_GetAlarmBase				((OSServiceIdType) 0xf2)
#define OSServiceId_GetAlarm					((OSServiceIdType) 0xf3)
#define OSServiceId_SetRelAlarm					((OSServiceIdType) 0xf4)
#define OSServiceId_SetAbsAlarm					((OSServiceIdType) 0xf5)
#define OSServiceId_CancelAlarm					((OSServiceIdType) 0xf6)
#define OSServiceId_GetActiveApplicationMode	((OSServiceIdType) 0xf7)
#define OSServiceId_StartOS						((OSServiceIdType) 0xf8)
#define OSServiceId_ShutdownOS					((OSServiceIdType) 0xf9)

/*
 *  保護違反を起こした処理単位の定義
 */
#define FC_INVALID			UINT_C(0x00)        /* 保護違反を起こした処理単位が特定できない */
#define FC_TASK				UINT_C(0x01)        /* 保護違反を起こした処理単位がタスク */
#define FC_C2ISR			UINT_C(0x02)        /* 保護違反を起こした処理単位がC2ISR */
#define FC_SYSTEM_HOOK		UINT_C(0x03)        /* 保護違反を起こした処理単位がシステム定義のフック */

/*
 *  システムサービスパラメータ取得のための定義
 */
#ifndef TOPPERS_MACRO_ONLY

/*
 *  エラーフックOFF時，サービスID取得とパラメータ取得もOFFになる
 */
#ifdef CFG_USE_ERRORHOOK

#ifdef CFG_USE_GETSERVICEID
extern OSServiceIdType	errorhook_svcid;
#endif /* CFG_USE_GETSERVICEID */

#ifdef CFG_USE_PARAMETERACCESS
extern ErrorHook_Par	errorhook_par1;
extern ErrorHook_Par	errorhook_par2;
extern ErrorHook_Par	errorhook_par3;
#endif /* CFG_USE_PARAMETERACCESS */

#endif /* CFG_USE_ERRORHOOK */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  エラーフックOFF時，サービスID取得とパラメータ取得もOFFになる
 */
#ifdef CFG_USE_ERRORHOOK

/*
 *  マクロの定義
 */
#ifdef CFG_USE_GETSERVICEID
#define OSErrorGetServiceId()				(errorhook_svcid)
#endif /* CFG_USE_GETSERVICEID */

/*
 *  エラーを引き起こしたシステムサービスID
 */
#ifdef CFG_USE_PARAMETERACCESS
#define OSError_StartOS_Mode()									(errorhook_par1.mode)
#define OSError_ActivateTask_TaskID()							(errorhook_par1.tskid)
#define OSError_ChainTask_TaskID()								(errorhook_par1.tskid)
#define OSError_GetTaskID_TaskID()								(errorhook_par1.p_tskid)
#define OSError_GetTaskState_TaskID()							(errorhook_par1.tskid)
#define OSError_GetTaskState_State()							(errorhook_par2.p_stat)
#define OSError_GetResource_ResID()								(errorhook_par1.resid)
#define OSError_ReleaseResource_ResID()							(errorhook_par1.resid)
#define OSError_SetEvent_TaskID()								(errorhook_par1.tskid)
#define OSError_SetEvent_Mask()									(errorhook_par2.mask)
#define OSError_ClearEvent_Mask()								(errorhook_par1.mask)
#define OSError_GetEvent_TaskID()								(errorhook_par1.tskid)
#define OSError_GetEvent_Event()								(errorhook_par2.p_mask)
#define OSError_WaitEvent_Mask()								(errorhook_par1.mask)
#define OSError_GetAlarmBase_AlarmID()							(errorhook_par1.almid)
#define OSError_GetAlarmBase_Info()								(errorhook_par2.p_info)
#define OSError_GetAlarm_AlarmID()								(errorhook_par1.almid)
#define OSError_GetAlarm_Tick()									(errorhook_par2.p_tick)
#define OSError_SetRelAlarm_AlarmID()							(errorhook_par1.almid)
#define OSError_SetRelAlarm_increment()							(errorhook_par2.incr)
#define OSError_SetRelAlarm_cycle()								(errorhook_par3.cycle)
#define OSError_SetAbsAlarm_AlarmID()							(errorhook_par1.almid)
#define OSError_SetAbsAlarm_start()								(errorhook_par2.start)
#define OSError_SetAbsAlarm_cycle()								(errorhook_par3.cycle)
#define OSError_CancelAlarm_AlarmID()							(errorhook_par1.almid)
#define OSError_IncrementCounter_CounterID()					(errorhook_par1.cntid)
#define OSError_GetCounterValue_CounterID()						(errorhook_par1.cntid)
#define OSError_GetCounterValue_Value()							(errorhook_par2.p_val)
#define OSError_GetElapsedValue_CounterID()						(errorhook_par1.cntid)
#define OSError_GetElapsedValue_Value()							(errorhook_par2.p_val)
#define OSError_GetElapsedValue_ElapsedValue()					(errorhook_par3.p_eval)
#define OSError_StartScheduleTableRel_ScheduleTableID()			(errorhook_par1.schtblid)
#define OSError_StartScheduleTableRel_Offset()					(errorhook_par2.offset)
#define OSError_StartScheduleTableAbs_ScheduleTableID()			(errorhook_par1.schtblid)
#define OSError_StartScheduleTableAbs_Start()					(errorhook_par2.start)
#define OSError_StopScheduleTable_ScheduleTableID()				(errorhook_par1.schtblid)
#define OSError_NextScheduleTable_ScheduleTableID_From()		(errorhook_par1.schtblid_from)
#define OSError_NextScheduleTable_ScheduleTableID_To()			(errorhook_par2.schtblid_to)
#define OSError_GetScheduleTableStatus_ScheduleTableID()		(errorhook_par1.schtblid)
#define OSError_GetScheduleTableStatus_ScheduleStatus()			(errorhook_par2.p_schtblstate)
#define OSError_DisableInterruptSource_DisableISR()				(errorhook_par1.isrid)
#define OSError_EnableInterruptSource_EnableISR()				(errorhook_par1.isrid)

#endif /* CFG_USE_PARAMETERACCESS */

#endif /* CFG_USE_ERRORHOOK */

/*
 *  プロテクションフック関係のマクロ
 */
#define PRO_IGNORE					UINT_C(0x00)
#define PRO_SHUTDOWN				UINT_C(0x01)

/*
 *  バージョン情報
 */
#define OS_SW_MAJOR_VERSION				UINT_C(1)   /* サプライヤーバージョン */
#define OS_SW_MINOR_VERSION				UINT_C(4)
#define OS_SW_PATCH_VERSION				UINT_C(2)

#define OS_AR_RELEASE_MAJOR_VERSION		UINT_C(4)   /* AUTOSARリリースバージョン */
#define OS_AR_RELEASE_MINOR_VERSION		UINT_C(0)
#define OS_AR_RELEASE_REVISION_VERSION	UINT_C(3)

#define TKERNEL_NAME	"TOPPERS/ATK2-SC1"  /* カーネル名称（独自仕様） */

#endif /* TOPPERS_OS_H */
