/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2011-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2015 by Witz Corporation
 *  Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015 by SUZUKI MOTOR CORPORATION
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
 *  $Id: svc_table.c 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*  This file is generated from svc.def by gensvc_atk. */

#include "kernel_int.h"

void * const svc_table[TMAX_SVCID + 1] = {
	(void *) &StartOS,
	(void *) &ShutdownOS,
	(void *) &ActivateTask,
	(void *) &TerminateTask,
	(void *) &ChainTask,
	(void *) &Schedule,
	(void *) &GetTaskID,
	(void *) &GetTaskState,
	(void *) &EnableAllInterrupts,
	(void *) &DisableAllInterrupts,
	(void *) &ResumeAllInterrupts,
	(void *) &SuspendAllInterrupts,
	(void *) &ResumeOSInterrupts,
	(void *) &SuspendOSInterrupts,
	(void *) &GetISRID,
	(void *) &DisableInterruptSource,
	(void *) &EnableInterruptSource,
	(void *) &GetResource,
	(void *) &ReleaseResource,
	(void *) &SetEvent,
	(void *) &ClearEvent,
	(void *) &GetEvent,
	(void *) &WaitEvent,
	(void *) &GetAlarmBase,
	(void *) &GetAlarm,
	(void *) &SetRelAlarm,
	(void *) &SetAbsAlarm,
	(void *) &CancelAlarm,
	(void *) &IncrementCounter,
	(void *) &GetCounterValue,
	(void *) &GetElapsedValue,
	(void *) &CheckISRMemoryAccess,
	(void *) &CheckTaskMemoryAccess,
	(void *) &CheckTaskAccess,
	(void *) &CheckISRAccess,
	(void *) &CheckAlarmAccess,
	(void *) &CheckResourceAccess,
	(void *) &CheckCounterAccess,
	(void *) &CheckScheduleTableAccess,
	(void *) &CheckTaskOwnership,
	(void *) &CheckISROwnership,
	(void *) &CheckAlarmOwnership,
	(void *) &CheckCounterOwnership,
	(void *) &CheckScheduleTableOwnership,
	(void *) &GetApplicationID,
	(void *) &CallTrustedFunction,
	(void *) &GetApplicationState,
	(void *) &GetActiveApplicationMode,
	(void *) &StartScheduleTableRel,
	(void *) &StartScheduleTableAbs,
	(void *) &StopScheduleTable,
	(void *) &NextScheduleTable,
	(void *) &GetScheduleTableStatus,
	(void *) &GetFaultyContext,
	(void *) &ioc_send_generic,
	(void *) &ioc_write_generic,
	(void *) &ioc_receive_generic,
	(void *) &ioc_read_generic,
	(void *) &ioc_empty_queue_generic,
	(void *) &AllowAccess,
	(void *) &TerminateApplication,
	(void *) &exit_task,
	TARGET_SVC_TABLE
};
