/*
 *
 *  TECS Generator
 *      Generator for TOPPERS Embedded Component System
 *  
 *   Copyright (C) 2008-2013 by TOPPERS Project
 *--
 *   上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
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
 *   $Id: rpc_string.h 2027 2014-01-20 08:36:17Z okuma-top $
 */
#ifndef RPC_STRING_H
#define RPC_STRING_H

#include <string.h>

/* GenParamCopy.rb で生成する STRLEN, STRNLEN 関数の定義 */
/* 現状では short, int, long, int128_t をサポートしない */

#define   STRLEN8( str )    strlen( (char *)str )

/*
 * STRNLEN
 *  marshaler, unmarshaler は strnlen を使う．
 *  バッファオーバーランへの耐性を明確にするため．
 *  しかし、strnlen は ANSI-C 標準のライブラリ関数ではないため、実装されていないケースがある
 *  GNU (Linux), VC++ では使用できるが、MacOS では使用できない
 *  （他の組込み用 OS での実装状況も不明）
 *  小さな関数ですむので inline 関数として実装しておく
 */

#ifndef USE_STRNLEN_LIB
#define   STRNLEN8( str, n )    strnlen( (char *)str, n )

#else

Inline size_t
#define   STRNLEN8( str, n )    STRNLEN8_( (uint8_t *)str, n )
STRNLEN8_(const uint8_t *s, size_t maxlen)
{
    size_t  i;
    for( i = 0; i < maxlen; i++ )
        if( *s++ == 0 )
            break;

    return  i;
}

#endif

#define	STRLEN16( s )			STRLEN16_( (uint16_t *)s )
#define	STRLEN32( s )			STRLEN32_( (uint32_t *)s )
#define	STRLEN64( s )			STRLEN64_( (uint64_t *)s )

#define	STRNLEN16( s, maxlen )	STRLEN16_( (uint16_t *)s, maxlen )
#define	STRNLEN32( s, maxlen )	STRLEN32_( (uint32_t *)s, maxlen )
#define	STRNLEN64( s, maxlen )	STRLEN64_( (uint64_t *)s, maxlen )

/** STRLENnn_ nn:16,32,64**/
Inline size_t
STRLEN16_(const uint16_t *s )
{
    size_t  i;
    for( i = 0; ; i++ )
        if( *s++ == 0 )
            break;

    return  i;
}

Inline size_t
STRLEN32_(const uint32_t *s )
{
    size_t  i;
    for( i = 0; ; i++ )
        if( *s++ == 0 )
            break;

    return  i;
}

Inline size_t
STRLEN64_(const uint64_t *s )
{
    size_t  i;
    for( i = 0; ; i++ )
        if( *s++ == 0 )
            break;

    return  i;
}

/** STRNLENnn_ nn:16,32,64**/
Inline size_t
STRNLEN16_(const uint16_t *s, size_t maxlen)
{
    size_t  i;
    for( i = 0; i < maxlen; i++ )
        if( *s++ == 0 )
            break;

    return  i;
}

Inline size_t
STRNLEN32_(const uint32_t *s, size_t maxlen)
{
    size_t  i;
    for( i = 0; i < maxlen; i++ )
        if( *s++ == 0 )
            break;

    return  i;
}

Inline size_t
STRNLEN64_(const uint64_t *s, size_t maxlen)
{
    size_t  i;
    for( i = 0; i < maxlen; i++ )
        if( *s++ == 0 )
            break;

    return  i;
}


#endif /* RPC_STRING_H */
