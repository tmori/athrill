/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: syslog.h 837 2012-12-26 15:09:59Z ertl-hiro $
 */

/*
 *		システムログ機能
 */

#ifndef TOPPERS_SYSLOG_H
#define TOPPERS_SYSLOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  システムログ出力を行うための定義
 */
#include <t_syslog.h>
#include <extsvc_fncode.h>
#include "target_syssvc.h"

/*
 *  ログバッファとそれにアクセスするためのポインタ
 */
#ifndef TCNT_SYSLOG_BUFFER
#define TCNT_SYSLOG_BUFFER	32		/* ログバッファのサイズ */
#endif /* TCNT_SYSLOG_BUFFER */

/*
 *  システムログ機能の拡張サービスコールのスタックサイズ
 */ 
#ifndef SSZ_SYSLOG_WRI_LOG
#define SSZ_SYSLOG_WRI_LOG	1024
#endif /* SSZ_SYSLOG_WRI_LOG */

#ifndef SSZ_SYSLOG_REA_LOG
#define SSZ_SYSLOG_REA_LOG	1024
#endif /* SSZ_SYSLOG_REA_LOG */

#ifndef SSZ_SYSLOG_MSK_LOG
#define SSZ_SYSLOG_MSK_LOG	1024
#endif /* SSZ_SYSLOG_MSK_LOG */

#ifndef SSZ_SYSLOG_REF_LOG
#define SSZ_SYSLOG_REF_LOG	1024
#endif /* SSZ_SYSLOG_REF_LOG */

/*
 *  システムログ機能の初期化
 */
extern void	syslog_initialize(intptr_t exinf) throw();

/*
 *  システムログ機能の拡張サービスコールによる呼出しインタフェース
 */
#ifndef TOPPERS_SVC_CALL

/*
 *  ログ情報の出力
 *
 *  TOPPERS_OMIT_SYSLOGが定義されていない場合，syslog_wri_logと
 *  syslog_fwri_logは，t_syslog.hで定義されるため，ここでは定義しない．
 */
#ifdef TOPPERS_OMIT_SYSLOG

Inline ER
syslog_wri_log(uint_t prio, const SYSLOG *p_syslog)
{
	return((ER) cal_svc(TFN_SYSLOG_WRI_LOG, (intptr_t) prio,
										(intptr_t) p_syslog, 0, 0, 0));
}

Inline ER
syslog_fwri_log(ER ercd, const SYSLOG *p_syslog)
{
	return((ER) cal_svc(TFN_SYSLOG_WRI_LOG, (intptr_t) ercd,
										(intptr_t) p_syslog, 0, 0, 0));
}

#endif /* TOPPERS_OMIT_SYSLOG */

/*
 *  ログバッファからのログ情報の読出し
 */
Inline ER_UINT
syslog_rea_log(SYSLOG *p_syslog)
{
	return(cal_svc(TFN_SYSLOG_REA_LOG, (intptr_t) p_syslog, 0, 0, 0, 0));
}

/* 
 *  出力すべきログ情報の重要度の設定
 */
Inline ER_UINT
syslog_msk_log(intptr_t logmask, intptr_t lowmask)
{
	return(cal_svc(TFN_SYSLOG_MSK_LOG, (intptr_t) logmask,
											(intptr_t) lowmask, 0, 0, 0));
}

/* 
 *  ログバッファの状態参照
 */
Inline ER_UINT
syslog_ref_log(intptr_t pk_rlog)
{
	return(cal_svc(TFN_SYSLOG_REF_LOG, (intptr_t) pk_rlog, 0, 0, 0, 0));
}

#endif /* TOPPERS_SVC_CALL */

/*
 *  システムログ機能の関数呼出しによる呼出しインタフェース
 */
extern ER		_syslog_syslog_wri_log(uint_t prio,
										const SYSLOG *p_syslog) throw();
extern ER		_syslog_syslog_fwri_log(ER ercd,
										const SYSLOG *p_syslog) throw();
extern ER_UINT	_syslog_syslog_rea_log(SYSLOG *p_syslog) throw();
extern ER		_syslog_syslog_msk_log(uint_t logmask, uint_t lowmask) throw();
extern ER		_syslog_syslog_ref_log(T_SYSLOG_RLOG *pk_rlog) throw();

#ifdef TOPPERS_SVC_CALL
#define syslog_wri_log	_syslog_syslog_wri_log
#define syslog_fwri_log	_syslog_syslog_fwri_log
#define syslog_rea_log	_syslog_syslog_rea_log
#define syslog_msk_log	_syslog_syslog_msk_log
#define syslog_ref_log	_syslog_syslog_ref_log
#endif /* TOPPERS_SVC_CALL */

/*
 *  システムログ機能を拡張サービスコールとして登録するための定義
 */
extern ER_UINT	extsvc_syslog_wri_log(intptr_t prio, intptr_t p_syslog,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_syslog_fwri_log(intptr_t ercd, intptr_t p_syslog,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_syslog_rea_log(intptr_t p_syslog, intptr_t par2,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_syslog_msk_log(intptr_t logmask, intptr_t lowmask,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_syslog_ref_log(intptr_t pk_rlog, intptr_t par2,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_SYSLOG_H */
