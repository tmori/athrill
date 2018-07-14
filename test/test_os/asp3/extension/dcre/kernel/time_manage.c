/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2017 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: time_manage.c 782 2017-03-10 23:33:54Z ertl-hiro $
 */

/*
 *		システム時刻管理機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "semaphore.h"
#include "eventflag.h"
#include "dataqueue.h"
#include "time_event.h"
#include "target_timer.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_SET_TIM_ENTER
#define LOG_SET_TIM_ENTER(systim)
#endif /* LOG_SET_TIM_ENTER */

#ifndef LOG_SET_TIM_LEAVE
#define LOG_SET_TIM_LEAVE(ercd)
#endif /* LOG_SET_TIM_LEAVE */

#ifndef LOG_GET_TIM_ENTER
#define LOG_GET_TIM_ENTER(p_systim)
#endif /* LOG_GET_TIM_ENTER */

#ifndef LOG_GET_TIM_LEAVE
#define LOG_GET_TIM_LEAVE(ercd, p_systim)
#endif /* LOG_GET_TIM_LEAVE */

#ifndef LOG_ADJ_TIM_ENTER
#define LOG_ADJ_TIM_ENTER(adjtim)
#endif /* LOG_ADJ_TIM_ENTER */

#ifndef LOG_ADJ_TIM_LEAVE
#define LOG_ADJ_TIM_LEAVE(ercd)
#endif /* LOG_ADJ_TIM_LEAVE */

#ifndef LOG_FCH_HRT_ENTER
#define LOG_FCH_HRT_ENTER()
#endif /* LOG_FCH_HRT_ENTER */

#ifndef LOG_FCH_HRT_LEAVE
#define LOG_FCH_HRT_LEAVE(hrtcnt)
#endif /* LOG_FCH_HRT_LEAVE */

/*
 *  システム時刻の設定［NGKI3563］
 */
#ifdef TOPPERS_set_tim

ER
set_tim(SYSTIM systim)
{
	ER		ercd;

	LOG_SET_TIM_ENTER(systim);
	CHECK_TSKCTX_UNL();							/*［NGKI3564］［NGKI3565］*/

	lock_cpu();
	update_current_evttim();					/*［ASPD1059］*/
	systim_offset = systim - monotonic_evttim;	/*［ASPD1060］*/
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_SET_TIM_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_set_tim */

/*
 *  システム時刻の参照［NGKI2349］
 */
#ifdef TOPPERS_get_tim

ER
get_tim(SYSTIM *p_systim)
{
	ER		ercd;

	LOG_GET_TIM_ENTER(p_systim);
	CHECK_TSKCTX_UNL();							/*［NGKI2350］［NGKI2351］*/

	lock_cpu();
	update_current_evttim();					/*［ASPD1057］*/
	*p_systim = systim_offset + monotonic_evttim;	/*［ASPD1058］*/
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_GET_TIM_LEAVE(ercd, p_systim);
	return(ercd);
}

#endif /* TOPPERS_get_tim */

/*
 *  システム時刻の調整［NGKI3581］
 */
#ifdef TOPPERS_adj_tim

ER
adj_tim(int32_t adjtim)
{
	EVTTIM	previous_evttim;
	ER		ercd;

	LOG_ADJ_TIM_ENTER(adjtim);
	CHECK_UNL();								/*［NGKI3583］*/
	CHECK_PAR(TMIN_ADJTIM <= adjtim && adjtim <= TMAX_ADJTIM);
												/*［NGKI3584］*/
	lock_cpu();
	update_current_evttim();					/*［ASPD1051］*/
	if (check_adjtim(adjtim)) {					/*［ASPD1052］*/
		ercd = E_OBJ;
	}
	else {
		previous_evttim = current_evttim;
		current_evttim += adjtim;				/*［ASPD1053］*/
		boundary_evttim = current_evttim - BOUNDARY_MARGIN;	/*［ASPD1055］*/

		if (adjtim > 0
				&& monotonic_evttim - previous_evttim < (EVTTIM) adjtim) {
#ifdef UINT64_MAX
			if (current_evttim < monotonic_evttim) {
				systim_offset += 1LLU << 32;
			}
#endif /* UINT64_MAX */
			monotonic_evttim = current_evttim;	/*［ASPD1054］*/
		}

		set_hrt_event();						/*［ASPD1056］*/
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_ADJ_TIM_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_adj_tim */

/*
 *  高分解能タイマの参照［NGKI3569］
 *
 *  任意の状態から呼び出せるようにするために，SILの全割込みロック状態の
 *  制御機能を用いて，排他制御を実現している［NGKI3572］．
 */
#ifdef TOPPERS_fch_hrt

HRTCNT
fch_hrt(void)
{
	HRTCNT	hrtcnt;
	SIL_PRE_LOC;

	LOG_FCH_HRT_ENTER();

	SIL_LOC_INT();
	hrtcnt = target_hrt_get_current();			/*［NGKI3571］*/
	SIL_UNL_INT();

	LOG_FCH_HRT_LEAVE(hrtcnt);
	return(hrtcnt);
}

#endif /* TOPPERS_fch_hrt */

/*
 *  通知方法のエラーチェック
 */
#ifdef TOPPERS_chknfy

ER
check_nfyinfo(const T_NFYINFO *p_nfyinfo)
{
	ER		ercd;

	if (p_nfyinfo->nfymode == TNFY_HANDLER) {
		CHECK_PAR(FUNC_ALIGN(p_nfyinfo->nfy.handler.tmehdr));
		CHECK_PAR(FUNC_NONNULL(p_nfyinfo->nfy.handler.tmehdr));
	}
	else {
		switch (p_nfyinfo->nfymode & 0x0fU) {
		case TNFY_SETVAR:
			CHECK_PAR((p_nfyinfo->nfymode & ~0x0fU) == 0);
			CHECK_PAR(INTPTR_ALIGN(p_nfyinfo->nfy.setvar.p_var));
			CHECK_PAR(INTPTR_NONNULL(p_nfyinfo->nfy.setvar.p_var));
			break;
		case TNFY_INCVAR:
			CHECK_PAR((p_nfyinfo->nfymode & ~0x0fU) == 0);
			CHECK_PAR(INTPTR_ALIGN(p_nfyinfo->nfy.incvar.p_var));
			CHECK_PAR(INTPTR_NONNULL(p_nfyinfo->nfy.incvar.p_var));
			break;
		case TNFY_ACTTSK:
			CHECK_ID(VALID_TSKID(p_nfyinfo->nfy.acttsk.tskid));
			break;
		case TNFY_WUPTSK:
			CHECK_ID(VALID_TSKID(p_nfyinfo->nfy.wuptsk.tskid));
			break;
		case TNFY_SIGSEM:
			CHECK_ID(VALID_SEMID(p_nfyinfo->nfy.sigsem.semid));
			break;
		case TNFY_SETFLG:
			CHECK_ID(VALID_FLGID(p_nfyinfo->nfy.setflg.flgid));
			break;
		case TNFY_SNDDTQ:
			CHECK_ID(VALID_DTQID(p_nfyinfo->nfy.snddtq.dtqid));
			break;
		default:
			CHECK_PAR(false);
			break;
		}
		switch (p_nfyinfo->nfymode & ~0x0fU) {
		case 0:
			break;
		case TENFY_SETVAR:
			CHECK_PAR(INTPTR_ALIGN(p_nfyinfo->enfy.setvar.p_var));
			CHECK_PAR(INTPTR_NONNULL(p_nfyinfo->enfy.setvar.p_var));
			break;
		case TENFY_INCVAR:
			CHECK_PAR(INTPTR_ALIGN(p_nfyinfo->enfy.incvar.p_var));
			CHECK_PAR(INTPTR_NONNULL(p_nfyinfo->enfy.incvar.p_var));
			break;
		case TENFY_ACTTSK:
			CHECK_ID(VALID_TSKID(p_nfyinfo->enfy.acttsk.tskid));
			break;
		case TENFY_WUPTSK:
			CHECK_ID(VALID_TSKID(p_nfyinfo->enfy.wuptsk.tskid));
			break;
		case TENFY_SIGSEM:
			CHECK_ID(VALID_SEMID(p_nfyinfo->enfy.sigsem.semid));
			break;
		case TENFY_SETFLG:
			CHECK_ID(VALID_FLGID(p_nfyinfo->enfy.setflg.flgid));
			break;
		case TENFY_SNDDTQ:
			CHECK_ID(VALID_DTQID(p_nfyinfo->enfy.snddtq.dtqid));
			break;
		default:
			CHECK_PAR(false);
			break;
		}
	}
	ercd = E_OK;

  error_exit:
	return(ercd);
}

#endif /* TOPPERS_chknfy */

/*
 *  通知ハンドラ
 */
#ifdef TOPPERS_nfyhdr

void
notify_handler(intptr_t exinf)
{
	T_NFYINFO	*p_nfyinfo = (T_NFYINFO *) exinf;
	ER			ercd;

	switch (p_nfyinfo->nfymode & 0x0fU) {
	case TNFY_SETVAR:
		*(p_nfyinfo->nfy.setvar.p_var) = p_nfyinfo->nfy.setvar.value;
		ercd = E_OK;
		break;
	case TNFY_INCVAR:
		(void) loc_cpu();
		*(p_nfyinfo->nfy.incvar.p_var) += 1;
		(void) unl_cpu();
		ercd = E_OK;
		break;
	case TNFY_ACTTSK:
		ercd = act_tsk(p_nfyinfo->nfy.acttsk.tskid);
		break;
	case TNFY_WUPTSK:
		ercd = wup_tsk(p_nfyinfo->nfy.wuptsk.tskid);
		break;
	case TNFY_SIGSEM:
		ercd = sig_sem(p_nfyinfo->nfy.sigsem.semid);
		break;
	case TNFY_SETFLG:
		ercd = set_flg(p_nfyinfo->nfy.setflg.flgid,
							p_nfyinfo->nfy.setflg.flgptn);
		break;
	case TNFY_SNDDTQ:
		ercd = psnd_dtq(p_nfyinfo->nfy.snddtq.dtqid,
							p_nfyinfo->nfy.snddtq.data);
		break;
	default:
		ercd = E_SYS;
		break;
	}

	if (ercd != E_OK) {
		switch (p_nfyinfo->nfymode & ~0x0fU) {
		case TENFY_SETVAR:
			*(p_nfyinfo->enfy.setvar.p_var) = (intptr_t) ercd;
			break;
		case TENFY_INCVAR:
			(void) loc_cpu();
			*(p_nfyinfo->enfy.incvar.p_var) += 1;
			(void) unl_cpu();
			break;
		case TENFY_ACTTSK:
			(void) act_tsk(p_nfyinfo->enfy.acttsk.tskid);
			break;
		case TENFY_WUPTSK:
			(void) wup_tsk(p_nfyinfo->enfy.wuptsk.tskid);
			break;
		case TENFY_SIGSEM:
			(void) sig_sem(p_nfyinfo->enfy.sigsem.semid);
			break;
		case TENFY_SETFLG:
			(void) set_flg(p_nfyinfo->enfy.setflg.flgid,
							p_nfyinfo->enfy.setflg.flgptn);
			break;
		case TENFY_SNDDTQ:
			(void) psnd_dtq(p_nfyinfo->enfy.snddtq.dtqid, (intptr_t) ercd);
			break;
		default:
			break;
		}
	}
}

#endif /* TOPPERS_nfyhdr */
