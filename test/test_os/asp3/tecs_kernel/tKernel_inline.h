/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2015 by Ushio Laboratory
 *              Graduate School of Engineering Science, Osaka Univ., JAPAN
 *  Copyright (C) 2015 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  $Id: tKernel_inline.h 509 2016-01-12 06:06:14Z ertl-hiro $
 */

#ifndef TOPPERS_TKERNEL_INLINE_H
#define TOPPERS_TKERNEL_INLINE_H

/*
 *  自タスクの拡張情報の参照
 */
Inline ER
eKernel_getExtendedInformation(intptr_t* p_exinf)
{
	return(get_inf(p_exinf));
}

/*
 *  起床待ち
 */
Inline ER
eKernel_sleep(void)
{
	return(slp_tsk());
}

/*
 *  起床待ち（タイムアウトあり）
 */
Inline ER
eKernel_sleepTimeout(TMO timeout)
{
	return(tslp_tsk(timeout));
}

/*
 *  自タスクの遅延
 */
Inline ER
eKernel_delay(RELTIM delayTime)
{
	return(dly_tsk(delayTime));
}

/*
 *  自タスクの終了
 */
Inline ER
eKernel_exit(void)
{
	return(ext_tsk());
}

/*
 *  タスク終了の禁止
 */
Inline ER
eKernel_disableTerminate(void)
{
	return(dis_ter());
}

/*
 *  タスク終了の許可
 */
Inline ER
eKernel_enableTerminate(void)
{
	return(ena_ter());
}

/*
 *  タスク終了禁止状態の参照
 */
Inline bool_t
eKernel_senseTerminate(void)
{
	return(sns_ter());
}

/*
 *  システム時刻の設定
 */
Inline ER
eKernel_setTime(SYSTIM systemTime)
{
	return(set_tim(systemTime));
}

/*
 *  システム時刻の参照
 */
Inline ER
eKernel_getTime(SYSTIM* p_systemTime)
{
	return(get_tim(p_systemTime));
}

/*
 *  システム時刻の調整
 */
Inline ER
eKernel_adjustTime(int32_t adjustTime)
{
	return(adj_tim(adjustTime));
}

/*
 *  高分解能タイマの参照
 */
Inline HRTCNT
eKernel_fetchHighResolutionTimer(void)
{
	return(fch_hrt());
}

/*
 *  タスクの優先順位の回転
 */
Inline ER
eKernel_rotateReadyQueue(PRI taskPriority)
{
	return(rot_rdq(taskPriority));
}

/*
 *  実行状態のタスクIDの参照
 */
Inline ER
eKernel_getTaskId(ID *p_taskId)
{
	return(get_tid(p_taskId));
}

/*
 *  実行できるタスクの数の参照
 */
Inline ER
eKernel_getLoad(PRI taskPriority, uint_t* p_load)
{
	return(get_lod(taskPriority, p_load));
}

/*
 *  指定した優先順位のタスクIDの参照
 */
Inline ER
eKernel_getNthTask(PRI taskPriority, uint_t nth, ID* p_taskID)
{
	return (get_nth(taskPriority, nth, p_taskID));
}
/*
 *  CPUロック状態への遷移
 */
Inline ER
eKernel_lockCpu()
{
	return(loc_cpu());
}

/*
 *  CPUロック状態の解除
 */
Inline ER
eKernel_unlockCpu()
{
	return(unl_cpu());
}

/*
 *  ディスパッチの禁止
 */
Inline ER
eKernel_disableDispatch()
{
	return(dis_dsp());
}

/*
 *  ディスパッチの許可
 */
Inline ER
eKernel_enableDispatch()
{
	return(ena_dsp());
}

/*
 *  コンテキストの参照
 */
Inline bool_t
eKernel_senseContext()
{
	return(sns_ctx());
}

/*
 *  CPUロック状態の参照
 */
Inline bool_t
eKernel_senseLock()
{
	return(sns_loc());
}

/*
 *  ディスパッチ禁止状態の参照
 */
Inline bool_t
eKernel_senseDispatch()
{
	return(sns_dsp());
}

/*
 *  ディスパッチ保留状態の参照
 */
Inline bool_t
eKernel_senseDispatchPendingState()
{
	return(sns_dpn());
}

/*
 *  カーネル非動作状態の参照
 */
Inline bool_t
eKernel_senseKernel()
{
	return(sns_ker());
}

/*
 *  カーネルの終了
 */
Inline ER
eKernel_exitKernel()
{
	return(ext_ker());
}

/*
 *  割込み優先度マスクの変更
 */
Inline ER
eKernel_changeInterruptPriorityMask(PRI interruptPriority)
{
	return(chg_ipm(interruptPriority));
}

/*
 *  割込み優先度マスクの参照
 */
Inline ER
eKernel_getInterruptPriorityMask(PRI* p_interruptPriority)
{
	return(get_ipm(p_interruptPriority));
}

/*
 *  高分解能タイマの参照（非タスクコンテキスト用）
 */
Inline HRTCNT
eiKernel_fetchHighResolutionTimer(void)
{
	return(fch_hrt());
}

/*
 *  タスクの優先順位の回転（非タスクコンテキスト用）
 */
Inline ER
eiKernel_rotateReadyQueue(PRI taskPriority)
{
	return(rot_rdq(taskPriority));
}

/*
 *  実行状態のタスクIDの参照（非タスクコンテキスト用）
 */
Inline ER
eiKernel_getTaskId(ID* p_taskId)
{
	return(get_tid(p_taskId));
}

/*
 *  CPUロック状態への遷移（非タスクコンテキスト用）
 */
Inline ER
eiKernel_lockCpu()
{
	return(loc_cpu());
}

/*
 *  CPUロック状態の解除（非タスクコンテキスト用）
 */
Inline ER
eiKernel_unlockCpu()
{
	return(unl_cpu());
}

/*
 *  コンテキストの参照（非タスクコンテキスト用）
 */
Inline bool_t
eiKernel_senseContext()
{
	return(sns_ctx());
}

/*
 *  CPUロック状態の参照（非タスクコンテキスト用）
 */
Inline bool_t
eiKernel_senseLock()
{
	return(sns_loc());
}

/*
 *  ディスパッチ禁止状態の参照（非タスクコンテキスト用）
 */
Inline bool_t
eiKernel_senseDispatch()
{
	return(sns_dsp());
}

/*
 *  ディスパッチ保留状態の参照（非タスクコンテキスト用）
 */
Inline bool_t
eiKernel_senseDispatchPendingState()
{
	return(sns_dpn());
}

/*
 *  カーネル非動作状態の参照（非タスクコンテキスト用）
 */
Inline bool_t
eiKernel_senseKernel()
{
	return(sns_ker());
}

/*
 *  カーネルの終了（非タスクコンテキスト用）
 */
Inline ER
eiKernel_exitKernel()
{
	return(ext_ker());
}

/*
 *  CPU例外発生時のディスパッチ保留状態の参照
 */
Inline bool_t
eiKernel_exceptionSenseDispatchPendingState(const void *p_exceptionInformation)
{
	return(xsns_dpn((void *) p_exceptionInformation));
}

#endif /* TOPPERS_TKERNEL_INLINE_H */
