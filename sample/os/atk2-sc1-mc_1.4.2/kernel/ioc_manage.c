/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2017 by Witz Corporation
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
 *  $Id: ioc_manage.c 739 2017-01-24 10:05:05Z nces-hibino $
 */

/*
 *		IOCモジュール
 */
#include "ioc_impl.h"
#include "mc.h"
#include "check.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_IOCSEND_ENTER
#define LOG_IOCSEND_ENTER(senderid)
#endif /* LOG_IOCSEND_ENTER */

#ifndef LOG_IOCSEND_LEAVE
#define LOG_IOCSEND_LEAVE(ercd)
#endif /* LOG_IOCSEND_LEAVE */

#ifndef LOG_IOCWRITE_ENTER
#define LOG_IOCWRITE_ENTER(senderid)
#endif /* LOG_IOCWRITE_ENTER */

#ifndef LOG_IOCWRITE_LEAVE
#define LOG_IOCWRITE_LEAVE(ercd)
#endif /* LOG_IOCWRITE_LEAVE */

#ifndef LOG_IOCRECEIVE_ENTER
#define LOG_IOCRECEIVE_ENTER(iocid)
#endif /* LOG_IOCRECEIVE_ENTER */

#ifndef LOG_IOCRECEIVE_LEAVE
#define LOG_IOCRECEIVE_LEAVE(ercd)
#endif /* LOG_IOCRECEIVE_LEAVE */

#ifndef LOG_IOCREAD_ENTER
#define LOG_IOCREAD_ENTER(iocid)
#endif /* LOG_IOCREAD_ENTER */

#ifndef LOG_IOCREAD_LEAVE
#define LOG_IOCREAD_LEAVE(ercd)
#endif /* LOG_IOCREAD_LEAVE */

#ifndef LOG_IOCEMPTYQUEUE_ENTER
#define LOG_IOCEMPTYQUEUE_ENTER(iocid)
#endif /* LOG_IOCEMPTYQUEUE_ENTER */

#ifndef LOG_IOCEMPTUQUEUE_LEAVE
#define LOG_IOCEMPTUQUEUE_LEAVE(ercd)
#endif /* LOG_IOCEMPTUQUEUE_LEAVE */


/* 内部関数のプロトタイプ宣言 */
LOCAL_INLINE void ioc_memcpy(void *p_dst, const void *p_src, uint32 sz);


/*
 *  初期化関数
 */
void
ioc_initialize(void)
{
	IocType			i;
	IOCCB			*p_ioccb;
	const IOCINIB	*p_iocinib;

	if (x_sense_mcore() != FALSE) {
		/* キューの初期化 */
		for (i = 0U; i < tnum_queueioc; i++) {
			p_ioccb = &ioccb_table[i];
			p_ioccb->quecnt = 0U;
			p_ioccb->head = 0U;
			p_ioccb->tail = 0U;
			p_ioccb->lostflg = FALSE;
		}

		/* 各IOCのバッファ初期化 */
		for (i = 0U; i < tnum_ioc; i++) {
			if (ioc_inival_table[i] != NULL) {
				p_iocinib = &iocinib_table[i];
				ioc_memcpy((void *) (p_iocinib->p_iocmb), ioc_inival_table[i], p_iocinib->datasz);
			}
		}
	}
}



LOCAL_INLINE void
ioc_memcpy(void *p_dst, const void *p_src, uint32 sz)
{
	uint8		*p_tmp_dst = (uint8 *) p_dst;
	const uint8	*p_tmp_src = (const uint8 *) p_src;
	CounterType	i;

	for (i = 0U; i < sz; i++) {
		p_tmp_dst[i] = p_tmp_src[i];
	}
}


Std_ReturnType
ioc_send_generic(IocType IocWrapperId, const void *in)
{
	const IOCWRPINIB	*p_iocwrpinib;
	const IOCINIB		*p_iocinib;
	IOCCB				*p_ioccb;
	Std_ReturnType		ercd = IOC_E_OK;

	CHECK_IOC_ID(IocWrapperId < tnum_ioc_wrapper_send);
	p_iocwrpinib = get_iocwrpinib(IocWrapperId);
	p_iocinib = p_iocwrpinib->p_iocinib;
	p_ioccb = get_ioccb(IOCID(p_iocwrpinib));

	LOG_IOCSEND_ENTER(p_iocwrpinib->senderid);

	CHECK_IOC_CALLEVEL_DISABLEDINT(CALLEVEL_IOCSEND);
	CHECK_IOC_ACCESS(p_iocwrpinib->p_sndosapinib == get_my_p_ccb()->p_currentosap);

	x_nested_lock_os_int();
	if (p_iocinib->intercoreflg != FALSE) {
		acquire_ioc_lock(GET_IOCLOCK());
	}

	S_D_CHECK_IOC_LIMIT(p_ioccb->quecnt < p_iocinib->maxque);
	ioc_memcpy((void *) ((uint32) p_iocinib->p_iocmb + ((p_iocinib->datasz) * (p_ioccb->tail))), in, p_iocinib->datasz);
	p_ioccb->quecnt++;
	p_ioccb->tail++;
	if (p_ioccb->tail >= p_iocinib->maxque) {
		p_ioccb->tail = 0U;
	}

  d_exit_no_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_IOCSEND_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	goto errorhook_start;

  exit_errorhook:
	x_nested_lock_os_int();
  errorhook_start:
	if (!(IocWrapperId < tnum_ioc_wrapper_send)) {
#ifdef CFG_USE_PARAMETERACCESS
		get_my_p_ccb()->temp_errorhook_par1.wrapperid = IocWrapperId;
#endif /* CFG_USE_PARAMETERACCESS */
		call_errorhook(ercd, IOCServiceId_ioc_send_generic);
	}
	else {
		p_iocwrpinib = get_iocwrpinib(IocWrapperId);
		p_iocinib = p_iocwrpinib->p_iocinib;
#ifdef CFG_USE_PARAMETERACCESS
		get_my_p_ccb()->temp_errorhook_par1.iocid = IOCID(p_iocwrpinib);
		get_my_p_ccb()->temp_errorhook_par2.senderid = p_iocwrpinib->senderid;
#endif /* CFG_USE_PARAMETERACCESS */
		if (p_iocinib->groupflg != FALSE) {
			call_errorhook(ercd, IOCServiceId_IOC_SendGroup);
		}
		else {
			call_errorhook(ercd, IOCServiceId_IOC_Send);
		}
	}
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}


Std_ReturnType
ioc_write_generic(IocType IocWrapperId, const void *in)
{
	const IOCWRPINIB	*p_iocwrpinib;
	const IOCINIB		*p_iocinib;
	Std_ReturnType		ercd = IOC_E_OK;

	CHECK_IOC_ID((tnum_ioc_wrapper_send <= IocWrapperId) && (IocWrapperId < tnum_ioc_wrapper));
	p_iocwrpinib = get_iocwrpinib(IocWrapperId);
	p_iocinib = p_iocwrpinib->p_iocinib;

	LOG_IOCWRITE_ENTER(p_iocwrpinib->senderid);

	CHECK_IOC_CALLEVEL_DISABLEDINT(CALLEVEL_IOCWRITE);
	CHECK_IOC_ACCESS(p_iocwrpinib->p_sndosapinib == get_my_p_ccb()->p_currentosap);

	x_nested_lock_os_int();
	if (p_iocinib->intercoreflg != FALSE) {
		acquire_ioc_lock(GET_IOCLOCK());
	}

	ioc_memcpy((void *) (p_iocinib->p_iocmb), in, p_iocinib->datasz);

  d_exit_no_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_IOCWRITE_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	goto errorhook_start;

  exit_errorhook:
	x_nested_lock_os_int();
  errorhook_start:
	if (!((tnum_ioc_wrapper_send <= IocWrapperId) && (IocWrapperId < tnum_ioc_wrapper))) {
#ifdef CFG_USE_PARAMETERACCESS
		get_my_p_ccb()->temp_errorhook_par1.wrapperid = IocWrapperId;
#endif /* CFG_USE_PARAMETERACCESS */
		call_errorhook(ercd, IOCServiceId_ioc_write_generic);
	}
	else {
		p_iocwrpinib = get_iocwrpinib(IocWrapperId);
		p_iocinib = p_iocwrpinib->p_iocinib;
#ifdef CFG_USE_PARAMETERACCESS
		get_my_p_ccb()->temp_errorhook_par1.iocid = IOCID(p_iocwrpinib);
		get_my_p_ccb()->temp_errorhook_par2.senderid = p_iocwrpinib->senderid;
#endif /* CFG_USE_PARAMETERACCESS */
		if (p_iocinib->groupflg != FALSE) {
			call_errorhook(ercd, IOCServiceId_IOC_WriteGroup);
		}
		else {
			call_errorhook(ercd, IOCServiceId_IOC_Write);
		}
	}
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}


Std_ReturnType
ioc_receive_generic(IocType IocId, void *out)
{
	const IOCINIB	*p_iocinib;
	IOCCB			*p_ioccb;
	Std_ReturnType	ercd = IOC_E_OK;

	LOG_IOCRECEIVE_ENTER(IocId);

	CHECK_IOC_ID(IocId < tnum_ioc);
	p_iocinib = get_iocinib(IocId);
	p_ioccb = get_ioccb(IocId);

	CHECK_IOC_CALLEVEL_DISABLEDINT(CALLEVEL_IOCRECEIVE);
	CHECK_IOC_ACCESS(p_iocinib->p_rcvosapinib == get_my_p_ccb()->p_currentosap);

	x_nested_lock_os_int();
	if (p_iocinib->intercoreflg != FALSE) {
		acquire_ioc_lock(GET_IOCLOCK());
	}

	S_D_CHECK_IOC_NO_DATA(p_ioccb->quecnt > 0U);
	ioc_memcpy(out, (void *) ((uint32) p_iocinib->p_iocmb + ((p_iocinib->datasz) * (p_ioccb->head))), p_iocinib->datasz);

	p_ioccb->quecnt--;
	p_ioccb->head++;
	if (p_ioccb->head >= p_iocinib->maxque) {
		p_ioccb->head = 0U;
	}

	S_D_CHECK_IOC_LOST(p_ioccb->lostflg == FALSE);

  d_exit_no_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_IOCRECEIVE_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	goto errorhook_start;

  exit_errorhook:
	x_nested_lock_os_int();
  errorhook_start:
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.iocid = IocId;
#endif /* CFG_USE_PARAMETERACCESS */
	if (!(IocId < tnum_ioc)) {
		call_errorhook(ercd, IOCServiceId_ioc_receive_generic);
	}
	else {
		p_iocinib = get_iocinib(IocId);
		if (p_iocinib->groupflg != FALSE) {
			call_errorhook(ercd, IOCServiceId_IOC_ReceiveGroup);
		}
		else {
			call_errorhook(ercd, IOCServiceId_IOC_Receive);
		}
	}
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}


Std_ReturnType
ioc_read_generic(IocType IocId, void *out)
{
	const IOCINIB	*p_iocinib;
	Std_ReturnType	ercd = IOC_E_OK;

	LOG_IOCREAD_ENTER(IocId);

	CHECK_IOC_ID(IocId < tnum_ioc);
	p_iocinib = get_iocinib(IocId);

	CHECK_IOC_CALLEVEL_DISABLEDINT(CALLEVEL_IOCREAD);
	CHECK_IOC_ACCESS(p_iocinib->p_rcvosapinib == get_my_p_ccb()->p_currentosap);

	x_nested_lock_os_int();
	if (p_iocinib->intercoreflg != FALSE) {
		acquire_ioc_lock(GET_IOCLOCK());
	}

	ioc_memcpy(out, (void *) (p_iocinib->p_iocmb), p_iocinib->datasz);

  d_exit_no_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_IOCREAD_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	goto errorhook_start;

  exit_errorhook:
	x_nested_lock_os_int();
  errorhook_start:
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.iocid = IocId;
#endif /* CFG_USE_PARAMETERACCESS */
	if (!(IocId < tnum_ioc)) {
		call_errorhook(ercd, IOCServiceId_ioc_read_generic);
	}
	else {
		p_iocinib = get_iocinib(IocId);
		if (p_iocinib->groupflg != FALSE) {
			call_errorhook(ercd, IOCServiceId_IOC_ReadGroup);
		}
		else {
			call_errorhook(ercd, IOCServiceId_IOC_Read);
		}
	}
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}


Std_ReturnType
ioc_empty_queue_generic(IocType IocId)
{
	IOCCB			*p_ioccb;
	const IOCINIB	*p_iocinib;
	Std_ReturnType	ercd = IOC_E_OK;

	CHECK_IOC_ID(IocId < tnum_ioc);
	p_ioccb = get_ioccb(IocId);
	p_iocinib = get_iocinib(IocId);

	CHECK_IOC_CALLEVEL_DISABLEDINT(CALLEVEL_IOCEMPTYQUEUE);
	CHECK_IOC_ACCESS(p_iocinib->p_rcvosapinib == get_my_p_ccb()->p_currentosap);

	x_nested_lock_os_int();
	if (p_iocinib->intercoreflg != FALSE) {
		acquire_ioc_lock(GET_IOCLOCK());
	}

	p_ioccb->quecnt = 0U;
	p_ioccb->head = 0U;
	p_ioccb->tail = 0U;
	p_ioccb->lostflg = FALSE;

  d_exit_no_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_IOCREAD_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	if (p_iocinib->intercoreflg != FALSE) {
		release_ioc_lock(GET_IOCLOCK());
	}
	goto errorhook_start;

  exit_errorhook:
	x_nested_lock_os_int();
  errorhook_start:
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.iocid = IocId;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, IOCServiceId_IOC_EmptyQueue);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}
