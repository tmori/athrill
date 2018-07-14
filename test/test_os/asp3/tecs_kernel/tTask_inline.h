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
 *  $Id: tTask_inline.h 509 2016-01-12 06:06:14Z ertl-hiro $
 */

#ifndef TOPPERS_TTASK_INLINE_H
#define TOPPERS_TTASK_INLINE_H

/*
 *  タスクの起動
 */
Inline ER
eTask_activate(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(act_tsk(ATTR_id));
}

/*
 *  タスク起動要求のキャンセル
 */
Inline ER_UINT
eTask_cancelActivate(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(can_act(ATTR_id));
}

/*
 *  タスク状態の参照
 */
Inline ER
eTask_getTaskState(CELLIDX idx, STAT *p_tskstat)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(get_tst(ATTR_id, p_tskstat));
}

/*
 *  タスクのベース優先度の変更
 */
Inline ER
eTask_changePriority(CELLIDX idx, PRI priority)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(chg_pri(ATTR_id, priority));
}

/*
 *  タスク優先度の参照
 */
Inline ER
eTask_getPriority(CELLIDX idx, PRI *p_priority)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(get_pri(ATTR_id, p_priority));
}

/*
 *  タスクの状態参照
 */
Inline ER
eTask_refer(CELLIDX idx, T_RTSK *pk_taskStatus)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(ref_tsk(ATTR_id, pk_taskStatus));
}

/*
 *  タスクの起床
 */
Inline ER
eTask_wakeup(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(wup_tsk(ATTR_id));
}

/*
 *  タスク起床要求のキャンセル
 */
Inline ER_UINT
eTask_cancelWakeup(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(can_wup(ATTR_id));
}

/*
 *  待ち状態の強制解除
 */
Inline ER
eTask_releaseWait(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(rel_wai(ATTR_id));
}

/*
 *  強制待ち状態への移行
 */
Inline ER
eTask_suspend(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(sus_tsk(ATTR_id));
}

/*
 *  強制待ち状態からの再開
 */
Inline ER
eTask_resume(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(rsm_tsk(ATTR_id));
}

/*
 *  タスクの終了要求［NGKI3469］
 */
Inline ER
eTask_raiseTerminate(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(ras_ter(ATTR_id));
}

/*
 *  タスクの強制終了
 */
Inline ER
eTask_terminate(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(ter_tsk(ATTR_id));
}

/*
 *  タスクの起動（非タスクコンテキスト用）
 */
Inline ER
eiTask_activate(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(act_tsk(ATTR_id));
}

/*
 *  タスクの起床（非タスクコンテキスト用）
 */
Inline ER
eiTask_wakeup(CELLIDX idx)
{
	CELLCB	*p_cellcb= GET_CELLCB(idx);
	return(iwup_tsk(ATTR_id));
}

/*
 *  待ち状態の強制解除（非タスクコンテキスト用）
 */
Inline ER
eiTask_releaseWait(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(irel_wai(ATTR_id));
}

#endif /* TOPPERS_TTASK_INLINE_H */
