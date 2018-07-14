/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: task_manage.c 470 2015-12-30 09:50:36Z ertl-hiro $
 */

/*
 *		タスク管理機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "taskhook.h"
#include "wait.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_ACT_TSK_ENTER
#define LOG_ACT_TSK_ENTER(tskid)
#endif /* LOG_ACT_TSK_ENTER */

#ifndef LOG_ACT_TSK_LEAVE
#define LOG_ACT_TSK_LEAVE(ercd)
#endif /* LOG_ACT_TSK_LEAVE */

#ifndef LOG_CAN_ACT_ENTER
#define LOG_CAN_ACT_ENTER(tskid)
#endif /* LOG_CAN_ACT_ENTER */

#ifndef LOG_CAN_ACT_LEAVE
#define LOG_CAN_ACT_LEAVE(ercd)
#endif /* LOG_CAN_ACT_LEAVE */

#ifndef LOG_GET_TST_ENTER
#define LOG_GET_TST_ENTER(tskid, p_tskstat)
#endif /* LOG_GET_TST_ENTER */

#ifndef LOG_GET_TST_LEAVE
#define LOG_GET_TST_LEAVE(ercd, p_tskstat)
#endif /* LOG_GET_TST_LEAVE */

#ifndef LOG_CHG_PRI_ENTER
#define LOG_CHG_PRI_ENTER(tskid, tskpri)
#endif /* LOG_CHG_PRI_ENTER */

#ifndef LOG_CHG_PRI_LEAVE
#define LOG_CHG_PRI_LEAVE(ercd)
#endif /* LOG_CHG_PRI_LEAVE */

#ifndef LOG_GET_PRI_ENTER
#define LOG_GET_PRI_ENTER(tskid, p_tskpri)
#endif /* LOG_GET_PRI_ENTER */

#ifndef LOG_GET_PRI_LEAVE
#define LOG_GET_PRI_LEAVE(ercd, p_tskpri)
#endif /* LOG_GET_PRI_LEAVE */

#ifndef LOG_GET_INF_ENTER
#define LOG_GET_INF_ENTER(p_exinf)
#endif /* LOG_GET_INF_ENTER */

#ifndef LOG_GET_INF_LEAVE
#define LOG_GET_INF_LEAVE(ercd, p_exinf)
#endif /* LOG_GET_INF_LEAVE */

/*
 *  タスクの起動［NGKI3529］
 */
#ifdef TOPPERS_act_tsk

ER
act_tsk(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_ACT_TSK_ENTER(tskid);
	CHECK_UNL();								/*［NGKI1114］*/
	if (tskid == TSK_SELF && !sense_context()) {
		p_tcb = p_runtsk;						/*［NGKI1121］*/
	}
	else {
		CHECK_ID(VALID_TSKID(tskid));			/*［NGKI1115］*/
		p_tcb = get_tcb(tskid);
	}

	lock_cpu();
	if (TSTAT_DORMANT(p_tcb->tstat)) {
		make_active(p_tcb);						/*［NGKI1118］*/
		if (p_runtsk != p_schedtsk) {
			if (!sense_context()) {
				dispatch();
			}
			else {
				request_dispatch();
			}
		}
		ercd = E_OK;
	}
	else if ((p_tcb->p_tinib->tskatr & TA_NOACTQUE) != 0U || p_tcb->actque) {
		ercd = E_QOVR;			   				/*［NGKI3528］*/
	}
	else {
		p_tcb->actque = true;					/*［NGKI3527］*/
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_ACT_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_act_tsk */

/*
 *  タスク起動要求のキャンセル［NGKI1138］
 */
#ifdef TOPPERS_can_act

ER_UINT
can_act(ID tskid)
{
	TCB		*p_tcb;
	ER_UINT	ercd;

	LOG_CAN_ACT_ENTER(tskid);
	CHECK_TSKCTX_UNL();							/*［NGKI1139］［NGKI1140］*/
	if (tskid == TSK_SELF) {
		p_tcb = p_runtsk;						/*［NGKI1146］*/
	}
	else {
		CHECK_ID(VALID_TSKID(tskid));			/*［NGKI1141］*/
		p_tcb = get_tcb(tskid);
	}

	lock_cpu();
	ercd = p_tcb->actque ? 1 : 0;				/*［NGKI1144］*/
	p_tcb->actque = false;						/*［NGKI1144］*/
	unlock_cpu();

  error_exit:
	LOG_CAN_ACT_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_can_act */

/*
 *  タスク状態の参照［NGKI3613］
 */
#ifdef TOPPERS_get_tst

ER
get_tst(ID tskid, STAT *p_tskstat)
{
	TCB		*p_tcb;
	uint_t	tstat;
	ER		ercd;

	LOG_GET_TST_ENTER(tskid, p_tskstat);
	CHECK_TSKCTX_UNL();							/*［NGKI3614］［NGKI3615］*/
	if (tskid == TSK_SELF) {
		p_tcb = p_runtsk;						/*［NGKI3621］*/
	}
	else {
		CHECK_ID(VALID_TSKID(tskid));			/*［NGKI3616］*/
		p_tcb = get_tcb(tskid);
	}

	lock_cpu();
	tstat = p_tcb->tstat;
	if (TSTAT_DORMANT(tstat)) {					/*［NGKI3620］*/
		*p_tskstat = TTS_DMT;
	}
	else if (TSTAT_SUSPENDED(tstat)) {
		if (TSTAT_WAITING(tstat)) {
			*p_tskstat = TTS_WAS;
		}
		else {
			*p_tskstat = TTS_SUS;
		}
	}
	else if (TSTAT_WAITING(tstat)) {
		*p_tskstat = TTS_WAI;
	}
	else if (p_tcb == p_runtsk) {
		*p_tskstat = TTS_RUN;
	}
	else {
		*p_tskstat = TTS_RDY;
	}
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_GET_TST_LEAVE(ercd, p_tskstat);
	return(ercd);
}

#endif /* TOPPERS_get_tst */

/*
 *  タスクのベース優先度の変更［NGKI1183］
 */
#ifdef TOPPERS_chg_pri

ER
chg_pri(ID tskid, PRI tskpri)
{
	TCB		*p_tcb;
	uint_t	newbpri;
	ER		ercd;

	LOG_CHG_PRI_ENTER(tskid, tskpri);
	CHECK_TSKCTX_UNL();							/*［NGKI1184］［NGKI1185］*/
	if (tskid == TSK_SELF) {
		p_tcb = p_runtsk;						/*［NGKI1198］*/
	}
	else {
		CHECK_ID(VALID_TSKID(tskid));			/*［NGKI1187］*/
		p_tcb = get_tcb(tskid);
	}
	if (tskpri == TPRI_INI) {
		newbpri = p_tcb->p_tinib->ipriority;	/*［NGKI1199］*/
	}
	else {
		CHECK_PAR(VALID_TPRI(tskpri));			/*［NGKI1188］*/
		newbpri = INT_PRIORITY(tskpri);
	}
	CHECK_NOSPT((p_tcb->p_tinib->tskatr & TA_RSTR) == 0U);	/*［NGKI1186］*/

	lock_cpu();
	if (TSTAT_DORMANT(p_tcb->tstat)) {
		ercd = E_OBJ;							/*［NGKI1191］*/
	}
	else if ((p_tcb->p_lastmtx != NULL || TSTAT_WAIT_MTX(p_tcb->tstat))
						&& !((*mtxhook_check_ceilpri)(p_tcb, newbpri))) {
		ercd = E_ILUSE;							/*［NGKI1201］*/
	}
	else {
		p_tcb->bpriority = newbpri;				/*［NGKI1192］*/
		if (p_tcb->p_lastmtx == NULL || !((*mtxhook_scan_ceilmtx)(p_tcb))) {
			change_priority(p_tcb, newbpri, false);		/*［NGKI1193］*/
			if (p_runtsk != p_schedtsk) {
				dispatch();
			}									/*［NGKI1197］*/
		}
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_CHG_PRI_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_chg_pri */

/*
 *  タスク優先度の参照［NGKI1202］
 */
#ifdef TOPPERS_get_pri

ER
get_pri(ID tskid, PRI *p_tskpri)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_GET_PRI_ENTER(tskid, p_tskpri);
	CHECK_TSKCTX_UNL();							/*［NGKI1203］［NGKI1204］*/
	if (tskid == TSK_SELF) {
		p_tcb = p_runtsk;						/*［NGKI1211］*/
	}
	else {
		CHECK_ID(VALID_TSKID(tskid));			/*［NGKI1205］*/
		p_tcb = get_tcb(tskid);
	}

	lock_cpu();
	if (TSTAT_DORMANT(p_tcb->tstat)) {
		ercd = E_OBJ;							/*［NGKI1209］*/
	}
	else {
		*p_tskpri = EXT_TSKPRI(p_tcb->priority);	/*［NGKI1210］*/
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_GET_PRI_LEAVE(ercd, p_tskpri);
	return(ercd);
}

#endif /* TOPPERS_get_pri */

/*
 *  自タスクの拡張情報の参照［NGKI1212］
 */
#ifdef TOPPERS_get_inf

ER
get_inf(intptr_t *p_exinf)
{
	ER		ercd;

	LOG_GET_INF_ENTER(p_exinf);
	CHECK_TSKCTX_UNL();							/*［NGKI1213］［NGKI1214］*/

	lock_cpu();
	*p_exinf = p_runtsk->p_tinib->exinf;		/*［NGKI1216］*/
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_GET_INF_LEAVE(ercd, p_exinf);
	return(ercd);
}

#endif /* TOPPERS_get_inf */
