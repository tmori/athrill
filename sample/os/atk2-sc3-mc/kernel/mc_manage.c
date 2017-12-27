/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2008-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2015 by Witz Corporation
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
 *  $Id: mc_manage.c 630 2016-03-17 07:42:55Z ertl-ishikawa $
 */

/*
 *		マルチコア管理モジュール
 */
#include "kernel_impl.h"
#include "mc.h"
#include "target_ici.h"
#include "task.h"
#include "interrupt.h"
#include "check.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_RASICI_ENTER
#define LOG_RASICI_ENTER(ISRID)
#endif /* LOG_RASICI_ENTER */

#ifndef LOG_RASICI_LEAVE
#define LOG_RASICI_LEAVE(ercd)
#endif /* LOG_RASICI_LEAVE */

#ifndef LOG_GETCOREID_ENTER
#define LOG_GETCOREID_ENTER()
#endif /* LOG_GETCOREID_ENTER */

#ifndef LOG_GETCOREID_LEAVE
#define LOG_GETCOREID_LEAVE(coreid)
#endif /* LOG_GETCOREID_LEAVE */

#ifndef LOG_STARTCORE_ENTER
#define LOG_STARTCORE_ENTER(CoreID)
#endif /* LOG_STARTCORE_ENTER */

#ifndef LOG_STARTCORE_LEAVE
#define LOG_STARTCORE_LEAVE(ercd, p_status)
#endif /* LOG_STARTCORE_LEAVE */

#ifndef LOG_STARTNONAUTOSARCORE_ENTER
#define LOG_STARTNONAUTOSARCORE_ENTER(CoreID)
#endif /* LOG_STARTNONAUTOSARCORE_ENTER */

#ifndef LOG_STARTNONAUTOSARCORE_LEAVE
#define LOG_STARTNONAUTOSARCORE_LEAVE(ercd, p_status)
#endif /* LOG_STARTNONAUTOSARCORE_LEAVE */

#ifndef LOG_GETNUMOFACTCORES_ENTER
#define LOG_GETNUMOFACTCORES_ENTER()
#endif /* LOG_GETNUMOFACTCORES_ENTER */

#ifndef LOG_GETNUMOFACTCORES_LEAVE
#define LOG_GETNUMOFACTCORES_LEAVE(activated_cores)
#endif /* LOG_GETNUMOFACTCORES_LEAVE */

#ifndef LOG_SHUTDOWNALLCORES_ENTER
#define LOG_SHUTDOWNALLCORES_ENTER(Error)
#endif /* LOG_SHUTDOWNALLCORES_ENTER */

#ifndef LOG_SHUTDOWNALLCORES_LEAVE
#define LOG_SHUTDOWNALLCORES_LEAVE(ercd)
#endif /* LOG_SHUTDOWNALLCORES_LEAVE */


/*
 *  ユーザ定義のコア間割込み要求
 */
#ifdef TOPPERS_RaiseInterCoreInterrupt

StatusType
RaiseInterCoreInterrupt(ISRType ISRID)
{
	StatusType	ercd = E_OK;
	CCB			*p_ccb;
	CoreIdType	coreid;
	ISRCB		*p_isrcb;

	LOG_RASICI_ENTER(ISRID);
	CHECK_CALLEVEL(CALLEVEL_RAISEINTERCOREINTERRUPT);
	CHECK_ID((ISRID >= (tnum_isr2 - tnum_ici)) && (ISRID < tnum_isr2));
	p_isrcb = get_isrcb(ISRID);

	CHECK_CORE(p_isrcb->p_isrinib != NULL);
	CHECK_RIGHT(p_isrcb->p_isrinib->acsbtmp);
	coreid = p_isrcb->p_isrinib->p_osapcb->p_osapinib->coreid;
	p_ccb = get_p_ccb(coreid);

	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);

	/*
	 *  該当コア間割込みが許可されている場合，ビットをセットした上，
	 *  ターゲット依存部にコア間割込みを要求する
	 *  該当コア間割込みが禁止されている場合，ビットをセットして，
	 *  リターンする（個別割込み許可時に，このビットを見て処理する）
	 */
	p_ccb->ici_request_map |= iciinib_table[(ISRID + tnum_ici) - tnum_isr2].ici_bit_ptn;
	if ((p_ccb->ici_bit_mask & iciinib_table[(ISRID + tnum_ici) - tnum_isr2].ici_bit_ptn) != 0U) {
		target_ici_raise(coreid);
	}

	release_tsk_lock(p_ccb);
	x_nested_unlock_os_int();

  exit_no_errorhook:
	LOG_RASICI_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.isrid = ISRID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_RaiseInterCoreInterrupt);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_RaiseInterCoreInterrupt */

/*
 *  CoreID取得
 */
#ifdef TOPPERS_GetCoreID

CoreIdType
GetCoreID(void)
{
	CoreIdType coreid;

	LOG_GETCOREID_ENTER();
	coreid = x_core_id();
	LOG_GETCOREID_LEAVE(coreid);

	return(coreid);
}

#endif /* TOPPERS_GetCoreID */

/*
 *  AUTOSAR OS管理のコアを起動
 */
#ifdef TOPPERS_StartCore

void
StartCore(CoreIdType CoreID, StatusType *Status)
{
	CCB			*p_ccb;
	CCB			*my_p_ccb;
	CoreIdType	my_coreid;
#ifdef CFG_USE_ERRORHOOK
	StatusType	ercd;

	ercd = E_OK;
#endif /* CFG_USE_ERRORHOOK */

	LOG_STARTCORE_ENTER(CoreID);
	p_ccb = get_p_ccb(OS_CORE_ID_MASTER);
	my_coreid = x_core_id();

	if (my_coreid >= TotalNumberOfCores) {
		/*
		 *  コアIDが，TotalNumberOfCoresより大きい場合は，
		 *  CCBが存在しないので，StatusがNULL以外なら，
		 *  エラーコードセットしてreturn
		 */

		/* 拡張エラーのチェック */
		if (Status != NULL) {
			if (p_ccb->kerflg != FALSE) {
				/* OS起動後に呼んだ場合 */
				*Status = E_OS_ACCESS;
			}
			else if (CoreID >= TotalNumberOfCores) {
				/* 無効なコアIDを指定した場合 */
				*Status = E_OS_ID;
			}
			else if (is_halt(CoreID) == FALSE) {
				/* 指定したコアが既に起動している場合 */
				*Status = E_OS_STATE;
			}
			else if (x_start_core(CoreID) == FALSE) {
				/* 無効なコアIDを指定した場合 */
				*Status = E_OS_ID;
			}
			else {
				/* 正常にコアを起動した場合 */
				activated_cores++;
				*Status = E_OK;
			}
		}
	}
	else {
		/*
		 *  OS管理内（CCBがある）コアから呼ばれた場合，
		 *  エラーフックを上がる
		 */

		my_p_ccb = get_my_p_ccb();

		/* 拡張エラーのチェック */
		if (Status == NULL) {
#ifdef CFG_USE_ERRORHOOK
			ercd = OS_E_PARAM_POINTER;
#endif /* CFG_USE_ERRORHOOK */
		}
		else if (p_ccb->kerflg != FALSE) {
			/* OS起動後に呼んだ場合 */
			if (my_p_ccb->kerflg == FALSE) {
				/* 自コアは未起動の場合 */
				*Status = E_OS_ACCESS;
#ifdef CFG_USE_ERRORHOOK
				ercd = E_OS_ACCESS;
#endif /* CFG_USE_ERRORHOOK */
			}
			else if (PROBE_MEM_WRITE(Status, StatusType) != FALSE) {
				/* Statusへのアクセス権限がある場合 */
				*Status = E_OS_ACCESS;
#ifdef CFG_USE_ERRORHOOK
				ercd = E_OS_ACCESS;
#endif /* CFG_USE_ERRORHOOK */
			}
			else {
				/* Statusへのアクセス権限がない場合 */
#ifdef CFG_USE_ERRORHOOK
				ercd = E_OS_ILLEGAL_ADDRESS;
#endif /* CFG_USE_ERRORHOOK */
			}
		}
		else {
			/*
			 *  OS起動前の場合，特権モードで呼び出されるので，
			 *  *Statusを変更しても良い
			 */
			if (CoreID >= TotalNumberOfCores) {
				/* 無効なコアIDを指定した場合 */
				*Status = E_OS_ID;
			}
			else if (is_halt(CoreID) == FALSE) {
				/* 指定したコアが既に起動している場合 */
				*Status = E_OS_STATE;
			}
			else if (x_start_core(CoreID) == FALSE) {
				/* 無効なコアIDを指定した場合 */
				*Status = E_OS_ID;
			}
			else {
				/* 正常にコアを起動した場合 */
				activated_cores++;
				*Status = E_OK;
				goto exit_no_errorhook;
			}
#ifdef CFG_USE_ERRORHOOK
			ercd = *Status;
#endif /* CFG_USE_ERRORHOOK */
		}

#ifdef CFG_USE_ERRORHOOK
		x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
		my_p_ccb->_errorhook_par1.coreid = CoreID;
		my_p_ccb->_errorhook_par2.status = Status;
#endif /* CFG_USE_PARAMETERACCESS */
		if (p_ccb->kerflg != FALSE) {
			call_errorhook(ercd, OSServiceId_StartCore);
		}
		x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */
	}

  exit_no_errorhook:
	LOG_STARTCORE_LEAVE(CoreID, Status);
	return;
}

#endif /* TOPPERS_StartCore */

/*
 *  AUTOSAR OS管理外のコアを起動
 */
#ifdef TOPPERS_StartNonAutosarCore

void
StartNonAutosarCore(CoreIdType CoreID, StatusType *Status)
{
	CCB			*p_ccb;
	CCB			*my_p_ccb;
	CoreIdType	my_coreid;
#ifdef CFG_USE_ERRORHOOK
	StatusType	ercd;

	ercd = E_OK;
#endif /* CFG_USE_ERRORHOOK */

	LOG_STARTNONAUTOSARCORE_ENTER(CoreID);
	p_ccb = get_p_ccb(OS_CORE_ID_MASTER);
	my_coreid = x_core_id();

	if (my_coreid >= TotalNumberOfCores) {
		/*
		 *  OS管理外（CCBがない）コアから呼ばれた場合，
		 *  エラーフックを上がらない
		 */

		/* 拡張エラーのチェック */
		if (Status != NULL) {
			if (is_halt(CoreID) == FALSE) {
				/* 指定したコアが既に起動している場合 */
				if (p_ccb->kerflg != FALSE) {
					*Status = E_OS_ACCESS;
				}
				else {
					*Status = E_OS_STATE;
				}
			}
			else if (x_start_core(CoreID) == FALSE) {
				/* 無効なコアIDを指定した場合 */
				if (p_ccb->kerflg != FALSE) {
					*Status = E_OS_ACCESS;
				}
				else {
					*Status = E_OS_ID;
				}
			}
			else {
				/* 正常にコアを起動した場合 */
				*Status = E_OK;
			}
		}
	}
	else {
		/*
		 *  OS管理内（CCBがある）コアから呼ばれた場合，
		 *  エラーフックを上がる
		 */

		my_p_ccb = get_my_p_ccb();

		/* 拡張エラーのチェック */
		if (Status == NULL) {
#ifdef CFG_USE_ERRORHOOK
			ercd = OS_E_PARAM_POINTER;
#endif /* CFG_USE_ERRORHOOK */
		}
		else if (is_halt(CoreID) == FALSE) {
			/* 指定したコアが既に起動している場合 */
			if (p_ccb->kerflg != FALSE) {
				if (my_p_ccb->kerflg == FALSE) {
					/* 自コアは未起動の場合 */
					*Status = E_OS_ACCESS;
#ifdef CFG_USE_ERRORHOOK
					ercd = E_OS_ACCESS;
#endif /* CFG_USE_ERRORHOOK */
				}
				else if (PROBE_MEM_WRITE(Status, StatusType) != FALSE) {
					/* Statusへのアクセス権限がある場合 */
					*Status = E_OS_ACCESS;
#ifdef CFG_USE_ERRORHOOK
					ercd = E_OS_ACCESS;
#endif /* CFG_USE_ERRORHOOK */
				}
				else {
					/* Statusへのアクセス権限がない場合 */
#ifdef CFG_USE_ERRORHOOK
					ercd = E_OS_ILLEGAL_ADDRESS;
#endif /* CFG_USE_ERRORHOOK */
				}
			}
			else {
				*Status = E_OS_STATE;
#ifdef CFG_USE_ERRORHOOK
				ercd = E_OS_STATE;
#endif /* CFG_USE_ERRORHOOK */
			}
		}
		else if (x_start_core(CoreID) == FALSE) {
			/* 指定したコアが既に起動している場合 */
			if (p_ccb->kerflg != FALSE) {
				if (my_p_ccb->kerflg == FALSE) {
					/* 自コアは未起動の場合 */
					*Status = E_OS_ACCESS;
#ifdef CFG_USE_ERRORHOOK
					ercd = E_OS_ACCESS;
#endif /* CFG_USE_ERRORHOOK */
				}
				else if (PROBE_MEM_WRITE(Status, StatusType) != FALSE) {
					/* Statusへのアクセス権限がある場合 */
					*Status = E_OS_ACCESS;
#ifdef CFG_USE_ERRORHOOK
					ercd = E_OS_ACCESS;
#endif /* CFG_USE_ERRORHOOK */
				}
				else {
					/* Statusへのアクセス権限がない場合 */
#ifdef CFG_USE_ERRORHOOK
					ercd = E_OS_ILLEGAL_ADDRESS;
#endif /* CFG_USE_ERRORHOOK */
				}
			}
			else {
				*Status = E_OS_ID;
#ifdef CFG_USE_ERRORHOOK
				ercd = E_OS_ID;
#endif /* CFG_USE_ERRORHOOK */
			}
		}
		else {
			/* 正常にコアを起動した場合 */
			*Status = E_OK;
			goto exit_no_errorhook;
		}

		/* エラーフックを呼ぶ */
#ifdef CFG_USE_ERRORHOOK
		x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
		my_p_ccb->_errorhook_par1.coreid = CoreID;
		my_p_ccb->_errorhook_par2.status = Status;
#endif /* CFG_USE_PARAMETERACCESS */
		if (p_ccb->kerflg != FALSE) {
			call_errorhook(ercd, OSServiceId_StartNonAutosarCore);
		}
		x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */
	}

  exit_no_errorhook:
	LOG_STARTNONAUTOSARCORE_LEAVE(CoreID, Status);
	return;
}

#endif /* TOPPERS_StartNonAutosarCore */

/*
 *  AUTOSAR OS管理のコアが起動している数
 */
#ifdef TOPPERS_GetNumberOfActivatedCores

uint32
GetNumberOfActivatedCores(void)
{
	LOG_GETNUMOFACTCORES_ENTER();
	LOG_GETNUMOFACTCORES_LEAVE(activated_cores);

	return(activated_cores);
}

#endif /* GetNumberOfActivatedCores */

/*
 *  OSの終了
 */
#ifdef TOPPERS_ShutdownAllCores

void
ShutdownAllCores(StatusType Error)
{
	StatusType	ercd = Error;
	CCB			*p_ccb = get_my_p_ccb();

	LOG_SHUTDOWNALLCORES_ENTER(Error);
	/*
	 *  呼出し元所属 OSアプリケーションの信頼/非信頼のチェック
	 *  呼出し元が信頼関数，システム定義フックである場合は
	 *  ShutdownAllCores() を実行する
	 */
	if (p_ccb->run_trusted == FALSE) {
		/* 非信頼から呼ばれた場合はエラーフックを呼ぶ */
#ifdef CFG_USE_ERRORHOOK
		x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
		p_ccb->_errorhook_par1.error = Error;
#endif /* CFG_USE_PARAMETERACCESS */
		call_errorhook(E_OS_ACCESS, OSServiceId_ShutdownAllCores);
		x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */
	}
	else {
		/*
		 *  不正な処理単位から呼び出した場合も，ErrorをE_OS_SHUTDOWN_FATALとして
		 *  ShutdownAllCoresを呼び出したものとして，シャットダウン処理を行う
		 */
		if (((p_ccb->callevel_stat & TCLMASK) | (CALLEVEL_SHUTDOWNALLCORES)) != (CALLEVEL_SHUTDOWNALLCORES)) {
			ercd = E_OS_SHUTDOWN_FATAL;
		}

		/*
		 *  OSで定義されていないエラーコードが指定された場合，ErrorをE_OS_SHUTDOWN_FATALとして
		 *  ShutdownAllCoresを呼び出したものとして，シャットダウン処理を行う
		 */
		if (ercd > ERRCODE_NUM) {
			ercd = E_OS_SHUTDOWN_FATAL;
		}

		internal_shutdownallcores(ercd);
	}
	LOG_SHUTDOWNALLCORES_LEAVE(ercd);
}
#endif /* TOPPERS_ShutdownAllCores */
