/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
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
 *  $Id: resource.c 2401 2017-03-14 09:09:24Z witz-itoyo $
 */

/*
 *		リソース管理モジュール
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "interrupt.h"
#include "resource.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_GETRES_ENTER
#define LOG_GETRES_ENTER(resid)
#endif /* LOG_GETRES_ENTER */

#ifndef LOG_GETRES_LEAVE
#define LOG_GETRES_LEAVE(ercd)
#endif /* LOG_GETRES_LEAVE */

#ifndef LOG_RELRES_ENTER
#define LOG_RELRES_ENTER(resid)
#endif /* LOG_RELRES_ENTER */

#ifndef LOG_RELRES_LEAVE
#define LOG_RELRES_LEAVE(ercd)
#endif /* LOG_RELRES_LEAVE */

/*
 *  リソース管理機能の初期化
 */
#ifdef TOPPERS_resource_initialize

void
resource_initialize(void)
{
	ResourceType	i;
	RESCB			*p_rescb;

	for (i = 0U; i < tnum_stdresource; i++) {
		p_rescb = &(rescb_table[i]);
		p_rescb->p_resinib = &(resinib_table[i]);
		p_rescb->lockflg = FALSE;
	}
}

#endif /* TOPPERS_resource_initialize */

/*
 *  リソースの獲得
 */
#ifdef TOPPERS_GetResource

StatusType
GetResource(ResourceType ResID)
{
	StatusType		ercd = E_OK;
	PriorityType	ceilpri, curpri;
	RESCB			*p_rescb;

	LOG_GETRES_ENTER(ResID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_GETRESOURCE);
	CHECK_ID(ResID < tnum_stdresource);
	p_rescb = get_rescb(ResID);

	ceilpri = p_rescb->p_resinib->ceilpri;
	if (callevel_stat == TCL_TASK) {
		CHECK_ACCESS(p_runtsk->p_tinib->inipri >= ceilpri);

		x_nested_lock_os_int();
		D_CHECK_ACCESS(p_rescb->lockflg == FALSE);
		curpri = p_runtsk->curpri;
		p_rescb->prevpri = curpri;
		p_rescb->lockflg = TRUE;
		p_rescb->p_prevrescb = p_runtsk->p_lastrescb;
		p_runtsk->p_lastrescb = p_rescb;
		if (ceilpri < curpri) {
			p_runtsk->curpri = ceilpri;
			if (ceilpri <= TPRI_MINISR) {
				x_set_ipm(ceilpri);
			}
		}
	}
	else {
		CHECK_ACCESS(GET_INTPRI(p_runisr) >= ceilpri);

		x_nested_lock_os_int();
		D_CHECK_ACCESS(p_rescb->lockflg == FALSE);
		curpri = x_get_ipm();
		p_rescb->prevpri = curpri;
		p_rescb->lockflg = TRUE;
		p_rescb->p_prevrescb = p_runisr->p_lastrescb;
		p_runisr->p_lastrescb = p_rescb;
		if (ceilpri < curpri) {
			x_set_ipm(ceilpri);
		}
	}

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_GETRES_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.resid = ResID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetResource);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetResource */

/*
 *  リソースの返却
 */
#ifdef TOPPERS_ReleaseResource

StatusType
ReleaseResource(ResourceType ResID)
{
	StatusType	ercd = E_OK;
	RESCB		*p_rescb;

	LOG_RELRES_ENTER(ResID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_RELEASERESOURCE);
	CHECK_ID(ResID < tnum_stdresource);
	p_rescb = get_rescb(ResID);

	if (callevel_stat == TCL_TASK) {
		CHECK_NOFUNC(p_runtsk->p_lastrescb == p_rescb);

		x_nested_lock_os_int();
		if (p_rescb->prevpri <= TPRI_MINISR) {
			x_set_ipm(p_rescb->prevpri);
		}
		else {
			if (p_runtsk->curpri <= TPRI_MINISR) {
				x_set_ipm((PriorityType) TIPM_ENAALL);
			}
		}
		p_runtsk->curpri = p_rescb->prevpri;
		p_runtsk->p_lastrescb = p_rescb->p_prevrescb;
		p_rescb->lockflg = FALSE;
		if (p_runtsk->curpri > nextpri) {
			preempt();
			dispatch();
		}
	}
	else {
		CHECK_NOFUNC(p_runisr->p_lastrescb == p_rescb);

		x_nested_lock_os_int();
		x_set_ipm(p_rescb->prevpri);
		p_runisr->p_lastrescb = p_rescb->p_prevrescb;
		p_rescb->lockflg = FALSE;
	}

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_RELRES_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.resid = ResID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ReleaseResource);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_ReleaseResource */
