/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2015 by Witz Corporation
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
 *  $Id: memory.c 425 2015-12-07 08:06:19Z witz-itoyo $
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "interrupt.h"
#include "memory.h"

/*
 *		メモリオブジェクト管理モジュール
 */

/*
 *  トレースマクロのデフォルト定義
 */

#ifndef LOG_CHKISRMEMACS_ENTER
#define LOG_CHKISRMEMACS_ENTER(ISRID, Address, Size)
#endif /* LOG_CHKISRMEMACS_ENTER */

#ifndef LOG_CHKISRMEMACS_LEAVE
#define LOG_CHKISRMEMACS_LEAVE(ercd)
#endif /* LOG_CHKISRMEMACS_LEAVE */

#ifndef LOG_CHKTSKMEMACS_ENTER
#define LOG_CHKTSKMEMACS_ENTER(TaskID, Address, Size)
#endif /* LOG_CHKTSKMEMACS_ENTER */

#ifndef LOG_CHKTSKMEMACS_LEAVE
#define LOG_CHKTSKMEMACS_LEAVE(ercd)
#endif /* LOG_CHKTSKMEMACS_LEAVE */


/*
 *  内部関数のプロトタイプ宣言
 */
#ifndef USE_TSKINICTXB

LOCAL_INLINE AccessType check_address_sstack(const MemoryStartAddressType base, MemorySizeType size, const TINIB *p_tinib);
LOCAL_INLINE AccessType check_address_ustack(const MemoryStartAddressType base, MemorySizeType size, const TINIB *p_tinib);

#endif /* USE_TSKINICTXB */

/*
 *  タスクメモリアクセスチェックシステムサービス
 *  指定された タスク の 指定されたメモリ領域のアクセス権をチェックする
 */
#ifdef TOPPERS_CheckTaskMemoryAccess

AccessType
CheckTaskMemoryAccess(TaskType TaskID, MemoryStartAddressType Address, MemorySizeType Size)
{
	AccessType	access;
	StatusType	ercd;
	TCB			*p_tcb;
	boolean		over_region;

	LOG_CHKTSKMEMACS_ENTER(TaskID, Address, Size);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKTASKMEMORYACCESS);
	CHECK_ID(TaskID < tnum_task);
	CHECK_VALUE(Size > 0U);

	p_tcb = get_tcb(TaskID);

	access = check_task_memory(p_tcb, Address, Size, &over_region);
	CHECK_ILLEGAL_ADDRESS(over_region == FALSE);

  exit_finish:
	LOG_CHKTSKMEMACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	_errorhook_par1.tskid = TaskID;
	_errorhook_par2.adr = Address;
	_errorhook_par3.sz = Size;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckTaskMemoryAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = AP_NoAccess;
	goto exit_finish;
}

#endif /* TOPPERS_CheckTaskMemoryAccess */

/*
 *  ISRメモリアクセスチェックシステムサービス
 *  指定された ISR の 指定されたメモリ領域のアクセス権をチェックする
 */
#ifdef TOPPERS_CheckISRMemoryAccess

AccessType
CheckISRMemoryAccess(ISRType ISRID, MemoryStartAddressType Address, MemorySizeType Size)
{
	AccessType	access;
	StatusType	ercd;
	ISRCB		*p_isrcb;
	boolean		over_region;

	LOG_CHKISRMEMACS_ENTER(ISRID, Address, Size);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKISRMEMORYACCESS);
	CHECK_ID(ISRID < tnum_isr2);
	CHECK_VALUE(Size > 0U);

	p_isrcb = get_isrcb(ISRID);

	access = check_isr_memory(p_isrcb, Address, Size, &over_region);
	CHECK_ILLEGAL_ADDRESS(over_region == FALSE);

  exit_finish:
	LOG_CHKISRMEMACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	_errorhook_par1.isrid = ISRID;
	_errorhook_par2.adr = Address;
	_errorhook_par3.sz = Size;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckISRMemoryAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = AP_NoAccess;
	goto exit_finish;
}

#endif /* TOPPERS_CheckISRMemoryAccess */

/*
 *  メモリアクセスチェック関数
 */

/*
 *  スタックアクセスチェック本体
 */
#ifdef TOPPERS_check_address_stack

AccessType
check_address_stack(const MemoryStartAddressType base, MemorySizeType size, const MemoryStartAddressType mobase, MemorySizeType mosize)
{
	AccessType result;
	if (within_memobj(base, size, mobase, mosize) != FALSE) {
		result = AP_Readable | AP_Writable | AP_StackSpace;
	}
	else {
		result = NO_ACCESS;
	}
	return(result);
}

#endif /* TOPPERS_check_address_stack */

#ifndef USE_TSKINICTXB

LOCAL_INLINE AccessType
check_address_sstack(const MemoryStartAddressType base, MemorySizeType size, const TINIB *p_tinib)
{
	return(check_address_stack(base, size, p_tinib->sstk, p_tinib->sstksz));
}

LOCAL_INLINE AccessType
check_address_ustack(const MemoryStartAddressType base, MemorySizeType size, const TINIB *p_tinib)
{
	return(check_address_stack(base, size, p_tinib->ustk, p_tinib->ustksz));
}

#endif /* USE_TSKINICTXB */


#ifndef OMIT_STANDARD_MEMINIB
/*
 *  メモリオブジェクト初期化ブロックの検索
 */
#ifdef TOPPERS_search_meminib

uint32
search_meminib(MemoryStartAddressType addr)
{
	uint32 left, right, i;

	left = 0U;
	right = tnum_meminib - 1U;
	while (left < right) {
		i = (left + right + 1U) / 2U;
		if (addr < (MemoryStartAddressType const) memtop_table[i]) {
			right = i - 1U;
		}
		else {
			left = i;
		}
	}
	return(left);
}

#endif /* TOPPERS_search_meminib */
/*
 *  メモリアクセスチェック
 *   メモリプロテクション単位(OSアプリケーション) でのチェックを行う
 *
 *  信頼OSアプリケーションの場合は，バックグラウンドをチェックする
 *    現実装では全領域アクセス可能
 *  非信頼OSアプリケーションの場合は，メモリオブジェクトの属性を返却する
 */
#ifdef TOPPERS_check_osap_memory

AccessType
check_osap_memory(OSAPCB *p_osapcb, const MemoryStartAddressType adr, MemorySizeType size)
{
	AccessType				access = AP_NoAccess;
	const MEMINIB			*p_meminib;
	uint32					pos;
	uint32					btptn;
	MemoryStartAddressType	sadr = adr;
	MemoryStartAddressType	eadr = (MemoryStartAddressType) ((MemorySizeType) sadr + size);

	if (p_osapcb->p_osapinib->osap_trusted != FALSE) {
		access = probe_trusted_osap_mem(sadr, eadr);
	}
	else {

		pos = search_meminib(sadr);
		p_meminib = &(meminib_table[pos]);
		if ((pos + 1U) < tnum_meminib) {
			if (((uint8 *) eadr - 1) < (uint8 * const) memtop_table[pos + 1U]) {
				btptn = p_osapcb->p_osapinib->btptn;
				access |= ((p_meminib->acptnr & btptn) != 0U) ? AP_Readable   : AP_NoAccess;
				access |= ((p_meminib->acptnw & btptn) != 0U) ? AP_Writable   : AP_NoAccess;
				access |= ((p_meminib->acptnx & btptn) != 0U) ? AP_Executable : AP_NoAccess;
			}
		}
	}
	return(access);
}

#endif /* TOPPERS_check_osap_memory */

/*
 *  ISRメモリアクセスチェックシステムサービス(カーネル内部用)
 */
#ifdef TOPPERS_check_isr_memory

AccessType
check_isr_memory(ISRCB *p_isrcb, const MemoryStartAddressType adr, MemorySizeType size, boolean *over_region)
{
	AccessType				access = AP_NoAccess;
	uint32					pos;
	MemoryStartAddressType	sadr = adr;
	MemorySizeType			memsize;

	pos = search_meminib(sadr);
	memsize = (MemorySizeType) (((uint8 *) (((pos + 1U) < tnum_meminib) ?
											((uint8 * const) memtop_table[pos + 1U]) : 0U)) - (const uint8 *) adr);

	/* リージョンを跨ったか判定 */
	if (size > memsize) {
		*over_region = TRUE;
	}
	else {
		*over_region = FALSE;
		access = check_address_stack(adr, size, _ostk, _ostksz);
	}

	/* 機能レベル2ではC2ISRは信頼しかないので，全アクセス可能にする */
	access |= (AP_Readable | AP_Writable | AP_Executable);

	return(access);
}

#endif /* TOPPERS_check_isr_memory */

/*
 *  タスクメモリアクセスチェックシステムサービス(カーネル内部用)
 */
#ifdef TOPPERS_check_task_memory

AccessType
check_task_memory(const TCB *p_tcb, const MemoryStartAddressType adr, MemorySizeType size, boolean *over_region)
{
	AccessType				access = AP_NoAccess;
	const MEMINIB			*p_meminib;
	uint32					pos;
	uint32					btptn;
	MemoryStartAddressType	sadr = adr;
	MemorySizeType			memsize;

	pos = search_meminib(sadr);
	memsize = (MemorySizeType) (((uint8 *) (((pos + 1U) < tnum_meminib) ?
											((uint8 * const) memtop_table[pos + 1U]) : 0U)) - (const uint8 *) adr);

	/* リージョンを跨ったか判定 */
	if (size > memsize) {
		*over_region = TRUE;
	}
	else {
		*over_region = FALSE;
	}

	if (p_tcb->p_tinib->p_osapcb->p_osapinib->osap_trusted != FALSE) {
		access = check_address_sstack(adr, size, p_tcb->p_tinib);

		/* 信頼タスクの場合は全アクセス可能 */
		access |= (AP_Readable | AP_Writable | AP_Executable);
	}
	else if (*over_region == FALSE) {
		p_meminib = &(meminib_table[pos]);
		if ((p_meminib->mematr & TOPPERS_USTACK) == 0U) {
			btptn = p_tcb->p_tinib->p_osapcb->p_osapinib->btptn;
			access |= ((p_meminib->acptnr & btptn) != 0U) ? AP_Readable   : AP_NoAccess;
			access |= ((p_meminib->acptnw & btptn) != 0U) ? AP_Writable   : AP_NoAccess;
			access |= ((p_meminib->acptnx & btptn) != 0U) ? AP_Executable : AP_NoAccess;
		}
		else {
			access = check_address_ustack(adr, size, p_tcb->p_tinib);
		}
	}
	else {
		/* 上記以外の場合，処理は行わない(戻り値：AP_NoAccess) */
	}

	return(access);
}

#endif /* TOPPERS_check_task_memory */
#endif /* OMIT_STANDARD_MEMINIB */

/*
 *  メモリアクセス権のチェック
 */
#ifdef TOPPERS_probe_memory_access

AccessType
probe_memory_access(const MemoryStartAddressType base, MemorySizeType size)
{
	AccessType				access;
	boolean					dummy;
	MemoryStartAddressType	base_adr = base;

	if (run_trusted != FALSE) {
		/* 特権モードの場合は，バックグラウンドをチェックする */
		access = probe_trusted_osap_mem(base, (MemoryStartAddressType) ((uint8 *) base_adr + size));
	}
	else {
		access = check_task_memory(p_runtsk, base, size, &dummy);
	}
	return(access);
}

#endif /* TOPPERS_probe_memory_access */

/*
 *  メモリへの書込み権のチェック
 */
#ifdef TOPPERS_probe_memory_read

boolean
probe_memory_read(const MemoryStartAddressType base, MemorySizeType size)
{
	return((probe_memory_access(base, size) & AP_Readable) == AP_Readable);
}

#endif /* TOPPERS_probe_memory_read */

/*
 *  メモリへの書込み権のチェック
 */
#ifdef TOPPERS_probe_memory_write

boolean
probe_memory_write(const MemoryStartAddressType base, MemorySizeType size)
{
	return((probe_memory_access(base, size) & AP_Writable) == AP_Writable);
}

#endif /* TOPPERS_probe_memory_write */

/*
 *  メモリに対する読書き権のチェック
 */
#ifdef TOPPERS_probe_memory_read_write

boolean
probe_memory_read_write(const MemoryStartAddressType base, MemorySizeType size)
{
	return((probe_memory_access(base, size) & (AP_Readable | AP_Writable)) == (AP_Readable | AP_Writable));
}

#endif /* TOPPERS_probe_memory_read_write */

/*
 *  DATAセクションとBSSセクションの初期化
 */
#ifdef TOPPERS_initialize_sections
#ifndef OMIT_INITIALIZE_SECTIONS

void
initialize_sections(void)
{
	uint32				i;
	uint8				*p_src, *p_dst;
	const DATASECINIB	*p_datasecinib;
	const BSSSECINIB	*p_bsssecinib;

	for (i = 0U; i < tnum_datasec; i++) {
		p_datasecinib = &(datasecinib_table[i]);
		p_src = (uint8 *) p_datasecinib->start_idata;
		for (p_dst = (uint8 *) p_datasecinib->start_data;
			 ((const MemoryStartAddressType) p_dst) < (const MemoryStartAddressType) p_datasecinib->end_data; p_dst++) {
			*p_dst = *p_src;
			p_src++;
		}
	}
	for (i = 0U; i < tnum_bsssec; i++) {
		p_bsssecinib = &(bsssecinib_table[i]);
		for (p_dst = (uint8 *) p_bsssecinib->start_bss;
			 ((const MemoryStartAddressType) p_dst) < (const MemoryStartAddressType) p_bsssecinib->end_bss; p_dst++) {
			*p_dst = 0U;
		}
	}
}

#endif /* OMIT_INITIALIZE_SECTIONS */
#endif /* TOPPERS_initialize_sections */
