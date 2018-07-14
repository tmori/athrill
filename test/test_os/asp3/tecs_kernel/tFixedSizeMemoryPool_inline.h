/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2015 by Ushio Laboratory
 *              Graduate School of Engineering Science, Osaka Univ., JAPAN
 *  Copyright (C) 2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: tFixedSizeMemoryPool_inline.h 509 2016-01-12 06:06:14Z ertl-hiro $
 */

#ifndef TOPPERS_TFIXEDSIZEMEMORYPOOL_INLINE_H
#define TOPPERS_TFIXEDSIZEMEMORYPOOL_INLINE_H

/*
 *  固定長メモリブロックの獲得
 */
Inline ER
eFixedSizeMemoryPool_get(CELLIDX idx, void **p_block)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(get_mpf( ATTR_id, p_block));
}

/*
 *  固定長メモリブロックの獲得（ポーリング）
 */
Inline ER
eFixedSizeMemoryPool_getPolling(CELLIDX idx, void **p_block)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(pget_mpf(ATTR_id, p_block));
}

/*
 *  固定長メモリブロックの獲得（タイムアウトあり）
 */
Inline ER
eFixedSizeMemoryPool_getTimeout(CELLIDX idx, void **p_block, TMO timeout)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(tget_mpf(ATTR_id, p_block, timeout));
}

/*
 *  固定長メモリブロックの返却
 */
Inline ER
eFixedSizeMemoryPool_release(CELLIDX idx, const void *block)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(rel_mpf(ATTR_id, (void *) block));
}

/*
 *  固定長メモリプールの再初期化
 */
Inline ER
eFixedSizeMemoryPool_initialize(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(ini_mpf(ATTR_id));
}

/*
 *  固定長メモリプールの状態参照
 */
Inline ER
eFixedSizeMemoryPool_refer(CELLIDX idx, T_RMPF *pk_fixedSizeMemoryPoolStatus)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	return(ref_mpf(ATTR_id, pk_memoryPoolFixedSizeStatus));
}

#endif /* TOPPERS_TFIXEDSIZEMEMORYPOOL_INLINE_H */
