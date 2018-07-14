/*
 *  TECS Generator
 *      Generator for TOPPERS Embedded Component System
 *  
 *   Copyright (C) 2008-2013 by TOPPERS Project
 *--
 *   上記著作権者は，以下の(1)(4)の条件を満たす場合に限り，本ソフトウェ
 *   ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *   変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *   (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *       権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *       スコード中に含まれていること．
 *   (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *       用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *       者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *       の無保証規定を掲載すること．
 *   (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *       用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *       と．
 *     (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *         作権表示，この利用条件および下記の無保証規定を掲載すること．
 *     (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *         報告すること．
 *   (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *       害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *       また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *       由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *       免責すること．
 *  
 *   本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *   よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *   に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *   アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *   の責任を負わない．
 *  
 *   $Id: tPPAllocator.c 2027 2014-01-20 08:36:17Z okuma-top $
 */

/* #[<PREAMBLE>]#
 * #[<...>]# から #[</...>]# で囲まれたコメントは編集しないでください
 * tecsmerge によるマージに使用されます
 *
 * 属性アクセスマクロ #_CAAM_#
 * heap_size        uint32_t         ATTR_heap_size  
 * buf              int8_t*          VAR_buf         
 * allocated_size   uint32_t         VAR_allocated_size
 *
 * #[</PREAMBLE>]# */

#include "tPPAllocator_tecsgen.h"

#ifndef E_OK
#define	E_OK	0		/* success */
#define	E_ID	(-18)	/* illegal ID */
#endif

/* 受け口関数 #_TEPF_# */
/* #[<ENTRY_PORT>]# ePPAllocator
 * entry port: ePPAllocator
 * signature:  sPPAllocator
 * context:    task
 * #[</ENTRY_PORT>]# */

/* #[<ENTRY_FUNC>]# ePPAllocator_alloc
 * name:         ePPAllocator_alloc
 * global_name:  tPPAllocator_ePPAllocator_alloc
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
ER
ePPAllocator_alloc(CELLIDX idx, uint32_t size, void** ptr)
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	if( size + VAR_allocatedSize > ATTR_heapSize ){
		ercd = E_NOMEM;
	}
	else {
		*ptr = (void *)(VAR_buf+VAR_allocatedSize);
		VAR_allocatedSize += size;
	}

	return(ercd);
}

/* #[<ENTRY_FUNC>]# ePPAllocator_dealloc
 * name:         ePPAllocator_dealloc
 * global_name:  tPPAllocator_ePPAllocator_dealloc
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
ER
ePPAllocator_dealloc(CELLIDX idx, const void* ptr)
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	if( (uintptr_t)ptr < (uintptr_t)VAR_buf || (uintptr_t)ptr >= (uintptr_t)(VAR_buf+VAR_allocatedSize) ){
		ercd = E_PAR;
	}
	else {
		VAR_allocatedSize = (uintptr_t)ptr - (uintptr_t)VAR_buf;
	}

	return(ercd);
}

/* #[<ENTRY_FUNC>]# ePPAllocator_dealloc_all
 * name:         ePPAllocator_dealloc_all
 * global_name:  tPPAllocator_ePPAllocator_dealloc_all
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
ER
ePPAllocator_dealloc_all(CELLIDX idx)
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	VAR_allocatedSize = 0;

	return(ercd);
}

/* #[<POSTAMBLE>]#
 *   これより下に非受け口関数を書きます
 * #[</POSTAMBLE>]#*/
