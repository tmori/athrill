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
 *   $Id: tTDR_inline.h 2032 2014-03-16 12:10:58Z okuma-top $
 */

/* #[<PREAMBLE>]#
 * #[<...>]# から #[</...>]# で囲まれたコメントは編集しないでください
 * tecsmerge によるマージに使用されます
 *
 * 属性アクセスマクロ #_CAAM_#
 * tmo              TMO              VAR_tmo         
 *
 * #[</PREAMBLE>]# */

/*
 * marhaler, unmarshaler が使用する STRLEN 関数を定義する
 *  marshaler, unmarshaler は必ず TDR は に結合されるため
 */
#include	"rpc_string.h"

/* 受け口関数 #_TEPF_# */
/* #[<ENTRY_PORT>]# eTDR
 * entry port: eTDR
 * signature:  sTDR
 * context:    task
 * #[</ENTRY_PORT>]# */

/* #[<ENTRY_FUNC>]# eTDR_reset
 * name:         eTDR_reset
 * global_name:  tTDR_eTDR_reset
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_reset(CELLIDX idx)
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	syslog( LOG_INFO, "TDR: resetting channel" );
	ercd = cChannel_reset();
	return(ercd);
}

/* #[<ENTRY_FUNC>]# eTDR_sendSOP
 * name:         eTDR_sendSOP
 * global_name:  tTDR_eTDR_sendSOP
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_sendSOP(CELLIDX idx, bool_t b_client )
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	const uint16_t *p_sopMagic;
	const uint16_t SOP_MAGIC1 = TDR_SOP_MAGIC1;
	const uint16_t SOP_MAGIC2 = TDR_SOP_MAGIC2;
	uint8_t		   val;

	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
#ifdef RPC_DEBUG
	syslog(LOG_INFO, "eTDR_sendSOP(b_client=%d)", b_client);
#endif
    if( b_client )
		p_sopMagic = &SOP_MAGIC1;
	else
		p_sopMagic = &SOP_MAGIC2;

	/*
	 * SOP パケットの送信
	 * パケットシーケンスが多少崩れても回復できるように、バイト単位で送る
	 * 下位アドレス、上位アドレスの順に送る
	 */

	val = (uint8_t)(*((uint8_t *)p_sopMagic));
	ercd = eTDR_putUInt8( idx, val );
	// syslog( LOG_INFO, "sendSOP:1 %02X", val );
	if( ercd != E_OK )
		return	ercd;
	val = (uint8_t)*(((uint8_t *)p_sopMagic)+1);
	ercd = eTDR_putUInt8( idx, val );
	// syslog( LOG_INFO, "sendSOP:2 %02X", val );

	return	ercd;
}

/* #[<ENTRY_FUNC>]# eTDR_receiveSOP
 * name:         eTDR_receiveSOP
 * global_name:  tTDR_eTDR_receiveSOP
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_receiveSOP(CELLIDX idx, bool_t b_client)
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	const uint16_t SOP_MAGIC1 = TDR_SOP_MAGIC1;
	const uint16_t SOP_MAGIC2 = TDR_SOP_MAGIC2;
	const uint16_t *p_sopMagic;
	uint16_t magic;

	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
#ifdef RPC_DEBUG
	syslog(LOG_INFO, "eTDR_receiveSOP(b_client=%d)", b_client);
#endif

    if( b_client )
		p_sopMagic = &SOP_MAGIC2;
	else
		p_sopMagic = &SOP_MAGIC1;

	if( (ercd=eTDR_getUInt8( idx, (uint8_t *)&magic )) != E_OK )
		;
	else if( (uint8_t)magic != (uint8_t)(*((uint8_t *)p_sopMagic)) ){
		// syslog( LOG_INFO, "receiveSOP:1 %02X expecting=%02X", (uint8_t)magic, (int8_t)(*((int8_t *)p_sopMagic)) );
		ercd = E_MAGIC;
	}
	else if((ercd=eTDR_getUInt8( idx, ((uint8_t *)&magic)+1 )) != E_OK )
		;
	else if( magic != *p_sopMagic){
		// syslog( LOG_INFO, "receiveSOP:2 %04X expecting=%04X", magic, *p_sopMagic );
		ercd = E_MAGIC;
	}

	if( ercd != E_OK )
		syslog( LOG_INFO, "receiveSOP: ERCD=%d", ercd );

	return ercd;
}

/* #[<ENTRY_FUNC>]# eTDR_sendEOP
 * name:         eTDR_sendEOP
 * global_name:  tTDR_eTDR_sendEOP
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_sendEOP(CELLIDX idx, bool_t b_continue )
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	uint16_t magic;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
#ifdef RPC_DEBUG
	syslog(LOG_INFO, "eTDR_sendEOP(b_continue=%d)", b_continue);
#endif

	if( b_continue )
		magic = TDR_EOP_MAGIC1;
	else
		magic = TDR_EOP_MAGIC2;

	/* EOP magic の送信 */
	if( (ercd = eTDR_putInt16( idx, (int16_t)magic )) != E_OK )
		return ercd;

	return cChannel_flush(VAR_tmo);
}

/* #[<ENTRY_FUNC>]# eTDR_receiveEOP
 * name:         eTDR_receiveEOP
 * global_name:  tTDR_eTDR_receiveEOP
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_receiveEOP(CELLIDX idx, bool_t b_continue)
{
	ER		ercd = E_OK, er2;
	CELLCB	*p_cellcb;
	uint16_t magic;

	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
#ifdef RPC_DEBUG
	syslog(LOG_INFO, "eTDR_receiveEOP(b_continue=%d)",b_continue);
#endif
/* mikan cChannel EOF get_* */
/* mikan magic EOF */

	if( (ercd=eTDR_getInt16( idx, (int16_t *)&magic )) == E_OK ){
		if( (b_continue && magic == TDR_EOP_MAGIC1) || (!b_continue && magic == TDR_EOP_MAGIC2) ){
		} else {
			ercd = E_MAGIC;
		}
	}
	er2 = cChannel_end_receive();
	return ercd != E_OK ? ercd : er2;
}

/* #[<ENTRY_FUNC>]# eTDR_putInt8
 * name:         eTDR_putInt8
 * global_name:  tTDR_eTDR_putInt8
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putInt8(CELLIDX idx, int8_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putInt16
 * name:         eTDR_putInt16
 * global_name:  tTDR_eTDR_putInt16
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putInt16(CELLIDX idx, int16_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putInt32
 * name:         eTDR_putInt32
 * global_name:  tTDR_eTDR_putInt32
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putInt32(CELLIDX idx, int32_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putInt64
 * name:         eTDR_putInt64
 * global_name:  tTDR_eTDR_putInt64
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putInt64(CELLIDX idx, int64_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putInt128
 * name:         eTDR_putInt128
 * global_name:  tTDR_eTDR_putInt128
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putInt128(CELLIDX idx, int128_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_getInt8
 * name:         eTDR_getInt8
 * global_name:  tTDR_eTDR_getInt8
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getInt8(CELLIDX idx, int8_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	 *p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getInt16
 * name:         eTDR_getInt16
 * global_name:  tTDR_eTDR_getInt16
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getInt16(CELLIDX idx, int16_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getInt32
 * name:         eTDR_getInt32
 * global_name:  tTDR_eTDR_getInt32
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getInt32(CELLIDX idx, int32_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getInt64
 * name:         eTDR_getInt64
 * global_name:  tTDR_eTDR_getInt64
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getInt64(CELLIDX idx, int64_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getInt128
 * name:         eTDR_getInt128
 * global_name:  tTDR_eTDR_getInt128
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getInt128(CELLIDX idx, int128_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_putUInt8
 * name:         eTDR_putUInt8
 * global_name:  tTDR_eTDR_putUInt8
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putUInt8(CELLIDX idx, uint8_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putUInt16
 * name:         eTDR_putUInt16
 * global_name:  tTDR_eTDR_putUInt16
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putUInt16(CELLIDX idx, uint16_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putUInt32
 * name:         eTDR_putUInt32
 * global_name:  tTDR_eTDR_putUInt32
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putUInt32(CELLIDX idx, uint32_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putUInt64
 * name:         eTDR_putUInt64
 * global_name:  tTDR_eTDR_putUInt64
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putUInt64(CELLIDX idx, uint64_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putUInt128
 * name:         eTDR_putUInt128
 * global_name:  tTDR_eTDR_putUInt128
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putUInt128(CELLIDX idx, uint128_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_getUInt8
 * name:         eTDR_getUInt8
 * global_name:  tTDR_eTDR_getUInt8
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getUInt8(CELLIDX idx, uint8_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getUInt16
 * name:         eTDR_getUInt16
 * global_name:  tTDR_eTDR_getUInt16
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getUInt16(CELLIDX idx, uint16_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getUInt32
 * name:         eTDR_getUInt32
 * global_name:  tTDR_eTDR_getUInt32
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getUInt32(CELLIDX idx, uint32_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getUInt64
 * name:         eTDR_getUInt64
 * global_name:  tTDR_eTDR_getUInt64
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getUInt64(CELLIDX idx, uint64_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getUInt128
 * name:         eTDR_getUInt128
 * global_name:  tTDR_eTDR_getUInt128
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getUInt128(CELLIDX idx, uint128_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_putBool
 * name:         eTDR_putBool
 * global_name:  tNBOTDR_eTDR_putBool
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putBool(CELLIDX idx, bool_t in)
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	eTDR_putUInt8(idx, (uint8_t)(in != 0) );

	return(ercd);
}

/* #[<ENTRY_FUNC>]# eTDR_getBool
 * name:         eTDR_getBool
 * global_name:  tNBOTDR_eTDR_getBool
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getBool(CELLIDX idx, bool_t* out)
{
	ER		ercd = E_OK;
	CELLCB	*p_cellcb;
	uint8_t	val;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	ercd = eTDR_getUInt8(idx, &val );
	if( ercd != E_OK )
		return ercd;
	*out = (val != 0);

	return(ercd);
}

/* #[<ENTRY_FUNC>]# eTDR_putFloat32
 * name:         eTDR_putFloat32
 * global_name:  tTDR_eTDR_putFloat32
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putFloat32(CELLIDX idx, float32_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putDouble64
 * name:         eTDR_putDouble64
 * global_name:  tTDR_eTDR_putDouble64
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putDouble64(CELLIDX idx, double64_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_getFloat32
 * name:         eTDR_getFloat32
 * global_name:  tTDR_eTDR_getFloat32
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getFloat32(CELLIDX idx, float32_t* out)
{
	ER_UINT		er_sz = E_OK;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getDouble64
 * name:         eTDR_getDouble64
 * global_name:  tTDR_eTDR_getDouble64
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getDouble64(CELLIDX idx, double64_t* out)
{
	ER_UINT		er_sz = E_OK;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_putChar
 * name:         eTDR_putChar
 * global_name:  tTDR_eTDR_putChar
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putChar(CELLIDX idx, char_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_getChar
 * name:         eTDR_getChar
 * global_name:  tTDR_eTDR_getChar
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getChar(CELLIDX idx, char_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_putSChar
 * name:         eTDR_putSChar
 * global_name:  tTDR_eTDR_putSChar
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putSChar(CELLIDX idx, signed char in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putShort
 * name:         eTDR_putShort
 * global_name:  tTDR_eTDR_putShort
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putShort(CELLIDX idx, short_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putInt
 * name:         eTDR_putInt
 * global_name:  tTDR_eTDR_putInt
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putInt(CELLIDX idx, int_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putLong
 * name:         eTDR_putLong
 * global_name:  tTDR_eTDR_putLong
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putLong(CELLIDX idx, long_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_getSChar
 * name:         eTDR_getSChar
 * global_name:  tTDR_eTDR_getSChar
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getSChar(CELLIDX idx, signed char* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getShort
 * name:         eTDR_getShort
 * global_name:  tTDR_eTDR_getShort
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getShort(CELLIDX idx, short* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getInt
 * name:         eTDR_getInt
 * global_name:  tTDR_eTDR_getInt
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getInt(CELLIDX idx, int_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getLong
 * name:         eTDR_getLong
 * global_name:  tTDR_eTDR_getLong
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getLong(CELLIDX idx, long_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_putUChar
 * name:         eTDR_putUChar
 * global_name:  tTDR_eTDR_putUChar
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putUChar(CELLIDX idx, unsigned char in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putUShort
 * name:         eTDR_putUShort
 * global_name:  tTDR_eTDR_putUShort
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putUShort(CELLIDX idx, ushort_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putUInt
 * name:         eTDR_putUInt
 * global_name:  tTDR_eTDR_putUInt
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putUInt(CELLIDX idx, uint_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_putULong
 * name:         eTDR_putULong
 * global_name:  tTDR_eTDR_putULong
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putULong(CELLIDX idx, ulong_t in)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&in, (int16_t)sizeof( in ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_getUChar
 * name:         eTDR_getUChar
 * global_name:  tTDR_eTDR_getUChar
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getUChar(CELLIDX idx, unsigned char* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getUShort
 * name:         eTDR_getUShort
 * global_name:  tTDR_eTDR_getUShort
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getUShort(CELLIDX idx, unsigned short* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getUInt
 * name:         eTDR_getUInt
 * global_name:  tTDR_eTDR_getUInt
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getUInt(CELLIDX idx, uint_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_getULong
 * name:         eTDR_getULong
 * global_name:  tTDR_eTDR_getULong
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getULong(CELLIDX idx, ulong_t* out)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	} /* end if VALID_IDX(idx) */

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)out, (int16_t)sizeof( *out ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *out ) ? E_OK : E_BOVR );
}

/* #[<ENTRY_FUNC>]# eTDR_putIntptr
 * name:         eTDR_putIntptr
 * global_name:  tTDR_eTDR_putIntptr
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_putIntptr(CELLIDX idx, const intptr_t ptr)
{
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	return cChannel_send( (int8_t *)&ptr, (int16_t)sizeof( ptr ), VAR_tmo );
}

/* #[<ENTRY_FUNC>]# eTDR_getIntptr
 * name:         eTDR_getIntptr
 * global_name:  tTDR_eTDR_getIntptr
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
Inline ER
eTDR_getIntptr(CELLIDX idx, intptr_t* ptr)
{
	ER_UINT	 er_sz;
	CELLCB	*p_cellcb;
	if (VALID_IDX(idx)) {
		p_cellcb = GET_CELLCB(idx);
	}
	else {
		return(E_ID);
	}

	/* ここに処理本体を記述します #_TEFB_# */
	er_sz = cChannel_receive( (int8_t *)ptr, (int16_t)sizeof( *ptr ), VAR_tmo );
	return  er_sz < 0 ? er_sz : ( er_sz == sizeof( *ptr ) ? E_OK : E_BOVR );
}


