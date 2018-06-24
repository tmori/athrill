/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: serial.h 837 2012-12-26 15:09:59Z ertl-hiro $
 */

/*
 *		シリアルインタフェースドライバ
 */

#ifndef TOPPERS_SERIAL_H
#define TOPPERS_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <extsvc_fncode.h>
#include "target_syssvc.h"

/*
 *  シリアルインタフェースドライバの拡張サービスコールのスタックサイズ
 */ 
#ifndef SSZ_SERIAL_OPN_POR
#define SSZ_SERIAL_OPN_POR	1024
#endif /* SSZ_SERIAL_OPN_POR */

#ifndef SSZ_SERIAL_CLS_POR
#define SSZ_SERIAL_CLS_POR	1024
#endif /* SSZ_SERIAL_CLS_POR */

#ifndef SSZ_SERIAL_REA_DAT
#define SSZ_SERIAL_REA_DAT	1024
#endif /* SSZ_SERIAL_REA_DAT */

#ifndef SSZ_SERIAL_WRI_DAT
#define SSZ_SERIAL_WRI_DAT	1024
#endif /* SSZ_SERIAL_WRI_DAT */

#ifndef SSZ_SERIAL_CTL_POR
#define SSZ_SERIAL_CTL_POR	1024
#endif /* SSZ_SERIAL_CTL_POR */

#ifndef SSZ_SERIAL_REF_POR
#define SSZ_SERIAL_REF_POR	1024
#endif /* SSZ_SERIAL_REF_POR */

/*
 *  シリアルインタフェースドライバの用いるパケット
 */
typedef struct {
	uint_t		reacnt;			/* 受信バッファ中の文字数 */
	uint_t		wricnt;			/* 送信バッファ中の文字数 */
} T_SERIAL_RPOR;

/*
 *  シリアルインタフェースドライバの初期化ルーチン
 */
extern void		serial_initialize(intptr_t exinf) throw();

/*
 *  シリアルインタフェースドライバからの未送信文字の取出し
 */
extern bool_t	serial_get_chr(ID portid, char *p_c) throw();

/*
 *  シリアルインタフェースドライバの拡張サービスコールによる呼出しイン
 *  タフェース
 */
#ifndef TOPPERS_SVC_CALL

Inline ER
serial_opn_por(ID portid)
{
	return((ER) cal_svc(TFN_SERIAL_OPN_POR, (intptr_t) portid, 0, 0, 0, 0));
}

Inline ER
serial_cls_por(ID portid)
{
	return((ER) cal_svc(TFN_SERIAL_CLS_POR, (intptr_t) portid, 0, 0, 0, 0));
}

Inline ER_UINT
serial_rea_dat(ID portid, char *buf, uint_t len)
{
	return(cal_svc(TFN_SERIAL_REA_DAT, (intptr_t) portid, (intptr_t) buf,
													(intptr_t) len, 0, 0));
}

Inline ER_UINT
serial_wri_dat(ID portid, const char *buf, uint_t len)
{
	return(cal_svc(TFN_SERIAL_WRI_DAT, (intptr_t) portid, (intptr_t) buf,
													(intptr_t) len, 0, 0));
}

Inline ER
serial_ctl_por(ID portid, uint_t ioctl)
{
	return((ER) cal_svc(TFN_SERIAL_CTL_POR, (intptr_t) portid,
												(intptr_t) ioctl, 0, 0, 0));
}

Inline ER
serial_ref_por(ID portid, T_SERIAL_RPOR *pk_rpor)
{
	return((ER) cal_svc(TFN_SERIAL_REF_POR, (intptr_t) portid,
											(intptr_t) pk_rpor, 0, 0, 0));
}

#endif /* TOPPERS_SVC_CALL */

/*
 *  シリアルインタフェースドライバの関数呼出しによる呼出しインタフェー
 *  ス
 */
extern ER		_serial_serial_opn_por(ID portid) throw();
extern ER		_serial_serial_cls_por(ID portid) throw();
extern ER_UINT	_serial_serial_rea_dat(ID portid, char *buf,
													uint_t len) throw();
extern ER_UINT	_serial_serial_wri_dat(ID portid, const char *buf,
													uint_t len) throw();
extern ER		_serial_serial_ctl_por(ID portid, uint_t ioctl) throw();
extern ER		_serial_serial_ref_por(ID portid,
										T_SERIAL_RPOR *pk_rpor) throw();

#ifdef TOPPERS_SVC_CALL
#define serial_opn_por	_serial_serial_opn_por
#define serial_cls_por	_serial_serial_cls_por
#define serial_rea_dat	_serial_serial_rea_dat
#define serial_wri_dat	_serial_serial_wri_dat
#define serial_ctl_por	_serial_serial_ctl_por
#define serial_ref_por	_serial_serial_ref_por
#endif /* TOPPERS_SVC_CALL */

/*
 *  シリアルインタフェースドライバを拡張サービスコールとして登録するた
 *  めの定義
 */
extern ER_UINT	extsvc_serial_opn_por(intptr_t portid, intptr_t par2,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_serial_cls_por(intptr_t portid, intptr_t par2,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_serial_rea_dat(intptr_t portid, intptr_t buf,
									intptr_t len, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_serial_wri_dat(intptr_t portid, intptr_t buf,
									intptr_t len, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_serial_ctl_por(intptr_t portid, intptr_t ioctl,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();
extern ER_UINT	extsvc_serial_ref_por(intptr_t portid, intptr_t pk_rpor,
									intptr_t par3, intptr_t par4,
									intptr_t par5, ID cdmid) throw();

/*
 *  シリアルインタフェースドライバの動作制御用のための定数
 *
 *  以下の定数は，ビット毎に論理和をとって用いる．
 */
#define	IOCTL_NULL	0U			/* 指定なし */
#define	IOCTL_ECHO	0x0001U		/* 受信した文字をエコーバック */
#define	IOCTL_CRLF	0x0010U		/* LFを送信する前にCRを付加 */
#define	IOCTL_FCSND	0x0100U		/* 送信に対してフロー制御を行う */
#define	IOCTL_FCANY	0x0200U		/* どのような文字でも送信再開 */
#define	IOCTL_FCRCV	0x0400U		/* 受信に対してフロー制御を行う */

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_SERIAL_H */
