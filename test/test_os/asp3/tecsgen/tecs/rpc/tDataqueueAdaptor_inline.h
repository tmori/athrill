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
 *   $Id: tDataqueueAdaptor_inline.h 2027 2014-01-20 08:36:17Z okuma-top $
 */

/* #[<PREAMBLE>]#
 * #[<...>]# から #[</...>]# で囲まれたコメントは編集しないでください
 * tecsmerge によるマージに使用されます
 *
 * #[</PREAMBLE>]# */

/* 受け口関数 #_TEPF_# */
/* #[<ENTRY_PORT>]# eChannel
 * entry port: eChannel
 * signature:  sChannel
 * context:    task
 * #[</ENTRY_PORT>]# */

/* #[<ENTRY_FUNC>]# eChannel_open
 * name:         eChannel_open
 * global_name:  tDataqueueAdaptor_eChannel_open
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eChannel_open(CELLIDX idx, const int8_t* arg, int16_t size, TMO tmo)
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

	return(ercd);
}

/* #[<ENTRY_FUNC>]# eChannel_close
 * name:         eChannel_close
 * global_name:  tDataqueueAdaptor_eChannel_close
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eChannel_close(CELLIDX idx)
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

	return(ercd);
}

/* #[<ENTRY_FUNC>]# eChannel_reset
 * name:         eChannel_reset
 * global_name:  tDataqueueAdaptor_eChannel_reset
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eChannel_reset(CELLIDX idx)
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
	cEventflag_clear( 0 );

	return(ercd);
}

/* #[<ENTRY_FUNC>]# eChannel_send
 * name:         eChannel_send
 * global_name:  tDataqueueAdaptor_eChannel_send
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eChannel_send(CELLIDX idx, const int8_t* buf, int16_t size, TMO tmo)
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	int     i;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	ercd = 0;
	for( i = 0; i < size; i++ )
		ercd |= cDataqueue_send( (intptr_t)((uint8_t)buf[ i ]) );

	return(ercd);
}

/* #[<ENTRY_FUNC>]# eChannel_flush
 * name:         eChannel_flush
 * global_name:  tDataqueueAdaptor_eChannel_flush
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eChannel_flush(CELLIDX idx, TMO tmo)
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

	return(ercd);
}

/* #[<ENTRY_FUNC>]# eChannel_receive
 * name:         eChannel_receive
 * global_name:  tDataqueueAdaptor_eChannel_receive
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER_UINT
eChannel_receive(CELLIDX idx, int8_t* buf, int16_t size, TMO tmo)
{
	ER_UINT ercd = E_OK;
	CELLCB	*p_cellcb;
	intptr_t data;
	int     i;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	/* printf( "DQA: receive(%d) ", size ); */
	for( i = 0; i < size; i++ ){
		ercd = cDataqueue_receive( &data );
		buf[i] = (int8_t)data;
		if( ercd != E_OK )
			break;
	}
	if( ercd != E_OK )
		return(ercd);
	else
		return i;
}

/* #[<ENTRY_FUNC>]# eChannel_end_receive
 * name:         eChannel_end_receive
 * global_name:  tDataqueueAdaptor_eChannel_end_receive
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eChannel_end_receive(CELLIDX idx)
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

	return(ercd);
}

