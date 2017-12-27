/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2004-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2017 by Witz Corporation
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
 *  $Id: scheduletable.c 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		スケジュールテーブル管理モジュール
 */

#include "kernel_impl.h"
#include "check.h"
#include "scheduletable.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_STASCHTBLREL_ENTER
#define LOG_STASCHTBLREL_ENTER(schtblid, offset)
#endif /* LOG_STASCHTBLREL_ENTER */

#ifndef LOG_STASCHTBLREL_LEAVE
#define LOG_STASCHTBLREL_LEAVE(ercd)
#endif /* LOG_STASCHTBLREL_LEAVE */

#ifndef LOG_STASCHTBLABS_ENTER
#define LOG_STASCHTBLABS_ENTER(schtblid, start)
#endif /* LOG_STASCHTBLABS_ENTER */

#ifndef LOG_STASCHTBLABS_LEAVE
#define LOG_STASCHTBLABS_LEAVE(ercd)
#endif /* LOG_STASCHTBLABS_LEAVE */

#ifndef LOG_STPSCHTBL_ENTER
#define LOG_STPSCHTBL_ENTER(schtblid)
#endif /* LOG_STPSCHTBL_ENTER */

#ifndef LOG_STPSCHTBL_LEAVE
#define LOG_STPSCHTBL_LEAVE(ercd)
#endif /* LOG_STPSCHTBL_LEAVE */

#ifndef LOG_NXTSCHTBL_ENTER
#define LOG_NXTSCHTBL_ENTER(from, to)
#endif /* LOG_NXTSCHTBL_ENTER */

#ifndef LOG_NXTSCHTBL_LEAVE
#define LOG_NXTSCHTBL_LEAVE(ercd)
#endif /* LOG_NXTSCHTBL_LEAVE */

#ifndef LOG_GETSCHTBLST_ENTER
#define LOG_GETSCHTBLST_ENTER(schtblid)
#endif /* LOG_GETSCHTBLST_ENTER */

#ifndef LOG_GETSCHTBLST_LEAVE
#define LOG_GETSCHTBLST_LEAVE(ercd, p_status)
#endif /* LOG_GETSCHTBLST_LEAVE */

#ifndef LOG_SCHTBL_ENTER
#define LOG_SCHTBL_ENTER(p_schtblcb)
#endif /* LOG_SCHTBL_ENTER */

#ifndef LOG_SCHTBL_LEAVE
#define LOG_SCHTBL_LEAVE(p_schtblcb)
#endif /* LOG_SCHTBL_LEAVE */

/*
 *  スケジュールテーブルオブジェクトの初期化
 */
#ifdef TOPPERS_schtbl_initialize

void
schtbl_initialize(void)
{
	ScheduleTableType	i;
	SCHTBLCB			*p_schtblcb;
	CNTCB				*p_cntcb;
	TickType			staval;

	for (i = 0U; i < tnum_scheduletable; i++) {
		p_schtblcb = &(schtblcb_table[i]);
		p_schtblcb->p_schtblinib = &(schtblinib_table[i]);

		/*
		 *  STOPPED状態にする時，p_nextschtblcb，p_prevschtblcbを初期化する
		 *  RUNNING，NEXT状態にする時，expptindexを初期化する
		 */
		p_schtblcb->p_nextschtblcb = NULL;
		p_schtblcb->p_prevschtblcb = NULL;
		(p_schtblcb->cntexpinfo).expirefunc = &schtbl_expire;

		if ((p_schtblcb->p_schtblinib->autosta & ((AppModeType) 1 << appmodeid)) != APPMODE_NONE) {
			if (is_implschtbl(i)) {
				p_schtblcb->status = SCHEDULETABLE_RUNNING_AND_SYNCHRONOUS;
			}
			else {
				p_schtblcb->status = SCHEDULETABLE_RUNNING;
			}
			p_schtblcb->expptindex = EXPPTINDEX_INITIAL;

			p_cntcb = p_schtblcb->p_schtblinib->p_cntcb;
			staval = p_schtblcb->p_schtblinib->staval;
			if (p_schtblcb->p_schtblinib->actatr == ABSOLUTE) {
				/*
				 *  絶対時間の起動
				 *  満了時間が0の場合，次の周期の0のタイミングとなる
				 *  （get_abstickに考慮済み）
				 */
				(p_schtblcb->cntexpinfo).expiretick = get_abstick(p_cntcb, staval);
			}
			else {
				/* 相対時間の起動 */
				(p_schtblcb->cntexpinfo).expiretick = get_reltick(p_cntcb, staval);
			}

			insert_cnt_expr_que(&(p_schtblcb->cntexpinfo), p_cntcb);
		}
		else {
			p_schtblcb->status = SCHEDULETABLE_STOPPED;

			queue_initialize(&(p_schtblcb->cntexpinfo.cntexpque));
		}
	}
}

#endif /* TOPPERS_schtbl_initialize */

/*
 *  指定したスケジュールテーブルの開始(相対時間)
 */
#ifdef TOPPERS_StartScheduleTableRel

StatusType
StartScheduleTableRel(ScheduleTableType ScheduleTableID, TickType Offset)
{
	StatusType	ercd = E_OK;
	SCHTBLCB	*p_schtblcb;
	CNTCB		*p_cntcb;

	LOG_STASCHTBLREL_ENTER(ScheduleTableID, Offset);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_STARTSCHEDULETABLEREL);
	CHECK_ID(ScheduleTableID < tnum_scheduletable);
	CHECK_ID(ScheduleTableID >= tnum_implscheduletable);

	p_schtblcb = get_schtblcb(ScheduleTableID);
	p_cntcb = p_schtblcb->p_schtblinib->p_cntcb;

	CHECK_VALUE((Offset != 0U) &&
				((p_cntcb->p_cntinib->maxval - p_schtblcb->p_schtblinib->p_exppt->offset) >= Offset));

	x_nested_lock_os_int();

	S_D_CHECK_STATE(p_schtblcb->status == SCHEDULETABLE_STOPPED);

	p_schtblcb->status = SCHEDULETABLE_RUNNING;
	p_schtblcb->expptindex = EXPPTINDEX_INITIAL;
	p_schtblcb->cntexpinfo.expiretick = get_reltick(p_cntcb, Offset);

	insert_cnt_expr_que(&(p_schtblcb->cntexpinfo), p_cntcb);

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_STASCHTBLREL_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.schtblid = ScheduleTableID;
	temp_errorhook_par2.offset = Offset;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_StartScheduleTableRel);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_StartScheduleTableRel */

/*
 *  指定したスケジュールテーブルの開始(絶対時間)
 */
#ifdef TOPPERS_StartScheduleTableAbs

StatusType
StartScheduleTableAbs(ScheduleTableType ScheduleTableID, TickType Start)
{
	StatusType	ercd = E_OK;
	SCHTBLCB	*p_schtblcb;
	CNTCB		*p_cntcb;

	LOG_STASCHTBLABS_ENTER(ScheduleTableID, Start);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_STARTSCHEDULETABLEABS);
	CHECK_ID(ScheduleTableID < tnum_scheduletable);

	p_schtblcb = get_schtblcb(ScheduleTableID);
	p_cntcb = p_schtblcb->p_schtblinib->p_cntcb;

	CHECK_VALUE(p_cntcb->p_cntinib->maxval >= Start);

	x_nested_lock_os_int();

	S_D_CHECK_STATE(p_schtblcb->status == SCHEDULETABLE_STOPPED);

	/* 暗黙同期の場合，同期動作状態で動作する */
	if (is_implschtbl(ScheduleTableID)) {
		p_schtblcb->status = SCHEDULETABLE_RUNNING_AND_SYNCHRONOUS;
	}
	else {
		p_schtblcb->status = SCHEDULETABLE_RUNNING;
	}
	p_schtblcb->expptindex = EXPPTINDEX_INITIAL;
	p_schtblcb->cntexpinfo.expiretick = get_abstick(p_cntcb, Start);

	insert_cnt_expr_que(&(p_schtblcb->cntexpinfo), p_cntcb);

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_STASCHTBLABS_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.schtblid = ScheduleTableID;
	temp_errorhook_par2.start = Start;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_StartScheduleTableAbs);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_StartScheduleTableAbs */

/*
 *  指定したスケジュールテーブルの停止
 */
#ifdef TOPPERS_StopScheduleTable

StatusType
StopScheduleTable(ScheduleTableType ScheduleTableID)
{
	StatusType	ercd = E_OK;
	SCHTBLCB	*p_schtblcb, *p_nextcb;

	LOG_STPSCHTBL_ENTER(ScheduleTableID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_STOPSCHEDULETABLE);
	CHECK_ID(ScheduleTableID < tnum_scheduletable);

	p_schtblcb = get_schtblcb(ScheduleTableID);

	x_nested_lock_os_int();

	S_D_CHECK_NOFUNC(p_schtblcb->status != SCHEDULETABLE_STOPPED);

	/*
	 *  指定されたスケジュールテーブルがSCHEDULETABLE_NEXTの場合,
	 *  自分をNextにしたスケジュールテーブルから，自分を外す
	 */
	if (p_schtblcb->status == SCHEDULETABLE_NEXT) {
		p_schtblcb->p_prevschtblcb->p_nextschtblcb = NULL;
		p_schtblcb->p_prevschtblcb = NULL;
	}
	else {
		/*
		 *  Nextスケジュールテーブルが存在した場合,
		 *  Nextスケジュールテーブルの予約をキャンセルする
		 */
		p_nextcb = p_schtblcb->p_nextschtblcb;
		if (p_nextcb != NULL) {
			p_nextcb->status = SCHEDULETABLE_STOPPED;
			p_nextcb->p_prevschtblcb = NULL;
			p_schtblcb->p_nextschtblcb = NULL;
		}

		/* カウンタ満了キューから既に登録した満了処理を削除 */
		delete_cnt_expr_que(&(p_schtblcb->cntexpinfo),
							p_schtblcb->p_schtblinib->p_cntcb);
	}

	p_schtblcb->status = SCHEDULETABLE_STOPPED;

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_STPSCHTBL_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.schtblid = ScheduleTableID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_StopScheduleTable);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}
#endif /* TOPPERS_StopScheduleTable */

/*
 *  スケジュールテーブルの切替え
 */
#ifdef TOPPERS_NextScheduleTable

StatusType
NextScheduleTable(ScheduleTableType ScheduleTableID_From,
				  ScheduleTableType ScheduleTableID_To)
{
	StatusType	ercd = E_OK;
	SCHTBLCB	*p_curcb, *p_nextcb;

	LOG_NXTSCHTBL_ENTER(ScheduleTableID_From, ScheduleTableID_To);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_NEXTSCHEDULETABLE);
	CHECK_ID(ScheduleTableID_From < tnum_scheduletable);
	CHECK_ID(ScheduleTableID_To < tnum_scheduletable);
	CHECK_ID((ScheduleTableID_From < tnum_implscheduletable) ==
			 (ScheduleTableID_To < tnum_implscheduletable));

	p_curcb = get_schtblcb(ScheduleTableID_From);
	p_nextcb = get_schtblcb(ScheduleTableID_To);

	CHECK_ID(p_curcb->p_schtblinib->p_cntcb == p_nextcb->p_schtblinib->p_cntcb);

	x_nested_lock_os_int();

	/* ScheduleTableID_Fromの状態チェック */
	S_D_CHECK_NOFUNC((p_curcb->status & (SCHEDULETABLE_STOPPED | SCHEDULETABLE_NEXT)) == 0U);

	/* ScheduleTableID_Toの状態チェック */
	S_D_CHECK_STATE(p_nextcb->status == SCHEDULETABLE_STOPPED);

	/*
	 *  Currentに対して既にNextが存在した場合,
	 *  これまでのNextに対してキャンセルする
	 */
	if (p_curcb->p_nextschtblcb != NULL) {
		p_curcb->p_nextschtblcb->status = SCHEDULETABLE_STOPPED;
		p_curcb->p_nextschtblcb->p_prevschtblcb = NULL;
	}

	p_curcb->p_nextschtblcb = p_nextcb;
	p_nextcb->status = SCHEDULETABLE_NEXT;
	p_nextcb->p_prevschtblcb = p_curcb;

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_NXTSCHTBL_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.schtblid_from = ScheduleTableID_From;
	temp_errorhook_par2.schtblid_to = ScheduleTableID_To;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_NextScheduleTable);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_NextScheduleTable */

/*
 *  スケジュールテーブル状態の取得
 */
#ifdef TOPPERS_GetScheduleTableStatus

StatusType
GetScheduleTableStatus(ScheduleTableType ScheduleTableID,
					   ScheduleTableStatusRefType ScheduleStatus)
{
	StatusType	ercd = E_OK;
	SCHTBLCB	*p_schtblcb;

	LOG_GETSCHTBLST_ENTER(ScheduleTableID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_GETSCHEDULETABLESTATUS);
	CHECK_ID(ScheduleTableID < tnum_scheduletable);
	CHECK_PARAM_POINTER(ScheduleStatus);

	p_schtblcb = get_schtblcb(ScheduleTableID);

	*ScheduleStatus = p_schtblcb->status;

  exit_no_errorhook:
	LOG_GETSCHTBLST_LEAVE(ercd, ScheduleStatus);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.schtblid = ScheduleTableID;
	temp_errorhook_par2.p_schtblstate = ScheduleStatus;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetScheduleTableStatus);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetScheduleTableStatus */

/*
 *  スケジュール満了処理関数
 */
#ifdef TOPPERS_schtbl_expire

void
schtbl_expire(CNTEXPINFO *p_cntexpinfo, CNTCB *p_cntcb)
{
	SCHTBLEXPINFO schtblexpinfo;

	schtblexpinfo.p_schtblcb = (SCHTBLCB *) p_cntexpinfo;

	schtbl_expiry_process(&schtblexpinfo, p_cntcb);

	if (schtblexpinfo.p_schtblcb != NULL) {
		insert_cnt_expr_que(&(schtblexpinfo.p_schtblcb->cntexpinfo), (CNTCB *) p_cntcb);
	}
}

#endif /* TOPPERS_schtbl_expire */

/*
 *  満了処理関数から各タイミング処理の実行
 */
#ifdef TOPPERS_schtbl_expiry_process

void
schtbl_expiry_process(SCHTBLEXPINFO *p_schtblexpinfo, const CNTCB *p_cntcb)
{
	boolean		loopcont = FALSE;
	SCHTBLCB	*p_schtblcb;

	/*
	 *  設定した満了点は，すぐ満了する可能性があるので，
	 *  繰り返し情報によってループ処理
	 */
	do {
		p_schtblcb = p_schtblexpinfo->p_schtblcb;

		if (p_schtblcb->expptindex < p_schtblcb->p_schtblinib->tnum_exppt) {
			/* 各満了点時 */
			loopcont = schtbl_exppoint_process(p_schtblcb, p_cntcb);
		}
		else if (p_schtblcb->expptindex == p_schtblcb->p_schtblinib->tnum_exppt) {
			/* 最終遅延処理 */
			loopcont = schtbl_tail(p_schtblcb, p_schtblexpinfo, p_cntcb);
		}
		else {
			/*
			 *  次スケジュールテーブルの開始時
			 *  p_schtblcb->expptindex == EXPPTINDEX_INITIALしかあり得ない
			 */
			loopcont = schtbl_head(p_schtblcb, p_cntcb);
		}
	} while (loopcont != FALSE);
}

#endif /* TOPPERS_schtbl_expiry_process */

/*
 *  スケジュールテーブルの開始処理
 */
#ifdef TOPPERS_schtbl_head

boolean
schtbl_head(SCHTBLCB *p_schtblcb, const CNTCB *p_cntcb)
{
	boolean				loopcont;
	const SCHTBLEXPPTCB	*p_exppoint;

	p_exppoint = &(p_schtblcb->p_schtblinib->p_exppt[EXPPTINDEX_TOP]);

	if (p_exppoint->offset == 0U) {
		/* 初期オフセット0の場合，今回満了処理内で1個目の満了点処理を行う */
		loopcont = TRUE;
	}
	else {
		loopcont = FALSE;
		/* 次に起動すべき時間の選定 */
		p_schtblcb->cntexpinfo.expiretick = add_tick(p_schtblcb->cntexpinfo.expiretick,
													 p_exppoint->offset, p_cntcb->p_cntinib->maxval2);
	}

	p_schtblcb->expptindex = EXPPTINDEX_TOP;

	return(loopcont);
}

#endif /* TOPPERS_schtbl_head */

/*
 *  スケジュールテーブルの各満了点処理
 */
#ifdef TOPPERS_schtbl_exppoint_process

boolean
schtbl_exppoint_process(SCHTBLCB *p_schtblcb, const CNTCB *p_cntcb)
{
	boolean				loopcont = FALSE;
	const SCHTBLEXPPTCB	*p_exppoint;
	const SCHTBLEXPPTCB *pp_exppoint;
	uint8				index;
	TickType			currtime;

	pp_exppoint = p_schtblcb->p_schtblinib->p_exppt;
	index = p_schtblcb->expptindex;
	p_exppoint = &(pp_exppoint[index]);

	/* 満了処理の実行 */
	LOG_SCHTBL_ENTER(p_schtblcb);
	(p_exppoint->expptfnt)();
	LOG_SCHTBL_LEAVE(p_schtblcb);

	/* 現在時間の退避 */
	currtime = p_exppoint->offset;
	/* 次の満了点へ */
	index++;
	p_schtblcb->expptindex = index;

	if (p_schtblcb->expptindex == p_schtblcb->p_schtblinib->tnum_exppt) {
		/* 現在が周期の最後の満了点の場合 */
		if (p_schtblcb->p_schtblinib->length == currtime) {
			/*
			 *  単発スケジュールテーブル最終遅延値が0の場合，Nextが存在するかもしれないため，
			 *  スケジュールテーブルの最後タイミング処理をする
			 */
			loopcont = TRUE;
		}
		else {
			/* 最終遅延処理のため，満了時間の設定 */
			p_schtblcb->cntexpinfo.expiretick = add_tick(p_schtblcb->cntexpinfo.expiretick,
														 (p_schtblcb->p_schtblinib->length - currtime), p_cntcb->p_cntinib->maxval2);
		}
	}
	else {
		p_exppoint = &(pp_exppoint[index]);
		/* 次の満了点の満了時間の設定 */
		p_schtblcb->cntexpinfo.expiretick = add_tick(p_schtblcb->cntexpinfo.expiretick,
													 (p_exppoint->offset - currtime), p_cntcb->p_cntinib->maxval2);
	}

	return(loopcont);
}

#endif /* TOPPERS_schtbl_exppoint_process */

/*
 *  スケジュールテーブルの終端処理
 */
#ifdef TOPPERS_schtbl_tail

boolean
schtbl_tail(SCHTBLCB *p_schtblcb, SCHTBLEXPINFO *p_schtblexpinfo, const CNTCB *p_cntcb)
{
	boolean				loopcont = FALSE;
	SCHTBLCB			*p_nextcb;
	const SCHTBLEXPPTCB	*p_exppoint;

	/* 周期の最後にてNextが存在するかチェック */
	if (p_schtblcb->p_nextschtblcb != NULL) {
		p_nextcb = p_schtblcb->p_nextschtblcb;

		/*
		 *  スケジュールテーブルの切り替え
		 *
		 *  暗黙同期同士の切替を考慮し，状態の引継ぎが必要
		 *  NextScheduleTableで同期方法チェックしているので，
		 *  同期方法の不整合がない
		 */
		p_nextcb->status = p_schtblcb->status;
		p_nextcb->expptindex = EXPPTINDEX_INITIAL;

		/* Nextの満了点設定基準は，Prevの満了時刻となる */
		p_nextcb->cntexpinfo.expiretick = p_schtblcb->cntexpinfo.expiretick;
		p_nextcb->p_prevschtblcb = NULL;

		/* 今まで実行状態のスケジュールテーブルに対して終了処理 */
		p_schtblcb->status = SCHEDULETABLE_STOPPED;
		p_schtblcb->p_nextschtblcb = NULL;

		/*
		 *  上流側よりNextの初期満了点をカウンタ満了キューに追加する時，
		 *  使用される
		 */
		p_schtblexpinfo->p_schtblcb = p_nextcb;

		loopcont = TRUE;
	}
	else {
		/* 周期制御の有無チェック */
		if (p_schtblcb->p_schtblinib->repeat != FALSE) {

			p_schtblcb->expptindex = EXPPTINDEX_TOP;
			p_exppoint = &(p_schtblcb->p_schtblinib->p_exppt[EXPPTINDEX_TOP]);

			if (p_exppoint->offset == 0U) {
				/* 初期オフセット0の場合，今回満了処理内で1個目の満了点処理を行う */
				loopcont = TRUE;
			}
			else {
				/* 最終遅延処理のため，満了時間の設定 */
				p_schtblcb->cntexpinfo.expiretick = add_tick(p_schtblcb->cntexpinfo.expiretick,
															 p_exppoint->offset, p_cntcb->p_cntinib->maxval2);
			}
		}
		else {
			/* 周期起動しないので，終了処理 */
			p_schtblcb->status = SCHEDULETABLE_STOPPED;
			p_schtblexpinfo->p_schtblcb = NULL;
			p_schtblcb->p_prevschtblcb = NULL;
			p_schtblcb->p_nextschtblcb = NULL;
		}
	}

	return(loopcont);
}

#endif /* TOPPERS_schtbl_tail */
