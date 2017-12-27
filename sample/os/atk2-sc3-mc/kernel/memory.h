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
 *  $Id: memory.h 485 2015-12-17 08:21:50Z witz-itoyo $
 */

/*
 *		メモリアクセス関連機能
 */

#ifndef TOPPERS_MEMORY_H
#define TOPPERS_MEMORY_H

#include "interrupt.h"
#include "task.h"

/*
 *  メモリ属性（カーネル内部で用いる）
 */
#define TOPPERS_ATTSEC	UINT_C(0x0100)  /* ATT_SEC/ATT_MODで登録されたメモリオブジェクト */
#define TOPPERS_ATTMEM	UINT_C(0x0200)  /* ATT_MEMで登録されたメモリオブジェクト */
#define TOPPERS_USTACK	UINT_C(0x0400)  /* タスク/C2ISRのユーザスタック領域 */

#ifndef TOPPERS_MACRO_ONLY

typedef	uint32 AccessPatternType;      /* アクセス許可パターン */

/*
 *  アクセス許可パターン
 */
#define TACP_KERNEL		((uint32) 0U)               /* カーネルドメインだけにアクセスを許可 */
#define TACP_SHARED		(~(uint32) 0U)              /* すべてのドメインからアクセスを許可 */

#ifndef OMIT_STANDARD_MEMINIB

/*
 *  メモリオブジェクト初期化ブロック
 */
typedef struct memory_initialization_block {
	AttributeType		mematr;         /* メモリオブジェクト属性 */
	AccessPatternType	acptnr;         /* リード許可 OSアプリケーション ビットパターン */
	AccessPatternType	acptnw;         /* ライト許可 OSアプリケーション ビットパターン */
	AccessPatternType	acptnx;         /* 実行許可   OSアプリケーション ビットパターン */
} MEMINIB;

/*
 *  メモリオブジェクト初期化ブロックの数（kernel_mem.c）
 */
extern const uint32		tnum_meminib;

/*
 *  メモリオブジェクトの先頭番地の領域（kernel_mem.c）
 */
extern void * const	memtop_table[];

/*
 *  メモリオブジェクト初期化ブロックの領域（kernel_mem.c）
 */
extern const MEMINIB	meminib_table[];

/*
 *  メモリオブジェクト初期化ブロックの検索
 *
 *  メモリオブジェクト初期化ブロックから，addrを含むメモリオブジェクト
 *  を検索し，そのインデックスを返す
 */

extern uint32 search_meminib(MemoryStartAddressType addr);

#endif /* OMIT_STANDARD_MEMINIB */

/*
 *  メモリ領域がメモリオブジェクトに含まれているかのチェック
 *
 *  先頭番地がbaseでサイズがsizeのメモリ領域が，先頭番地がmobaseでサイ
 *  ズがmosizeのメモリオブジェクトに含まれている場合にTRUE，そうでない
 *  場合にFALSEを返す
 */

LOCAL_INLINE boolean
within_memobj(MemoryStartAddressType base, MemorySizeType size, MemoryStartAddressType mobase, MemorySizeType mosize)
{
	return((mobase <= base) && (size <= mosize)
		   && (((MemorySizeType) base - (MemorySizeType) mobase) <= (mosize - size)));
}

/*
 *  スタックアクセスチェック本体
 */
extern AccessType check_address_stack(const MemoryStartAddressType base, MemorySizeType size, const MemoryStartAddressType mobase, MemorySizeType mosize);

/*
 *  メモリアクセスチェック
 */
extern AccessType check_osap_memory(OSAPCB *p_osapcb, const MemoryStartAddressType adr, MemorySizeType size);

/*
 *  ISRメモリアクセスチェックシステムサービス(カーネル内部用)
 */
extern AccessType check_isr_memory(ISRCB *p_isrcb, const MemoryStartAddressType adr, MemorySizeType size, boolean *over_region);

/*
 *  タスクメモリアクセスチェックシステムサービス(カーネル内部用)
 */
extern AccessType check_task_memory(const TCB *p_tcb, const MemoryStartAddressType adr, MemorySizeType size, boolean *over_region);

/*
 *  メモリアクセス権のチェック
 */
extern AccessType probe_memory_access(const MemoryStartAddressType base, MemorySizeType size);

/*
 *  メモリへの読込み権のチェック
 */
extern boolean probe_memory_read(const MemoryStartAddressType base, MemorySizeType size);

/*
 *  メモリへの書込み権のチェック
 */
extern boolean probe_memory_write(const MemoryStartAddressType base, MemorySizeType size);

/*
 *  メモリに対する読書き権のチェック
 */
extern boolean probe_memory_read_write(const MemoryStartAddressType base, MemorySizeType size);

/*
 *  DATAセクションとBSSセクションの初期化
 */
extern void initialize_sections(void);

/*
 *  メモリアクセス権チェックのためのマクロ
 */
#ifndef PROBE_MEM_WRITE
#define PROBE_MEM_WRITE(p_var, type) \
	(boolean) ((probe_memory_write((MemoryStartAddressType) (p_var), sizeof(type)) != FALSE) && (ALIGNED_TYPE(p_var, type) != FALSE))
#endif /* PROBE_MEM_WRITE */

#ifndef PROBE_MEM_RW
#define PROBE_MEM_RW(p_var, type) \
	(boolean) ((probe_memory_read_write((MemoryStartAddressType) (p_var), sizeof(type)) != FALSE) && (ALIGNED_TYPE(p_var, type) != FALSE))
#endif /* PROBE_MEM_RW */

/*
 *  dataセクション初期化ブロック
 */
typedef struct {
	void	*start_data;        /* dataセクションの先頭番地 */
	void	*end_data;          /* dataセクションの終了番地 */
	void	*start_idata;       /* 初期化データ領域の先頭番地 */
} DATASECINIB;

/*
 *  dataセクションの数と初期化ブロックのエリア（kernel_mem_2.c）
 */
extern const uint32			tnum_datasec;
extern const DATASECINIB	datasecinib_table[];

/*
 *  bssセクション初期化ブロック
 */
typedef struct {
	void	*start_bss;         /* bssセクションの先頭番地 */
	void	*end_bss;           /* bssセクションの終了番地 */
} BSSSECINIB;

/*
 *  bssセクションの数と初期化ブロックのエリア（kernel_mem_2.c）
 */
extern const uint32		tnum_bsssec;
extern const BSSSECINIB	bsssecinib_table[];

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_MEMORY_H_ */
