/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2017 by Witz Corporation
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
 *  $Id: log_output.c 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		システムログのフォーマット出力
 */

#include "log_output.h"

/*
 *  数値を文字列に変換時，uintptr型の数値の最大文字数
 *  8進数，10進数，16進数へ変換を想定
 *  動作例：
 *  2Byteの場合：16進数：FFFF  →4文字
 *             ：10進数：65535 →5文字
 *             ： 8進数：177777→6文字
 *               →CONVERT_BUFLENは最大の6文字を返す
 *  計算パラメータ：
 *  8U:1バイトのビット数
 *  2U:3で割った際の余り1ビット，または2ビット分を含めるための補正値
 *  3U:10進数へ変換時，最大3ビットで1文字になる
 *     (4ビットだと2桁数字になるため)
 *  計算例：
 *  2Byteの場合
 *     ビット数は2 * 8U = 16bit
 *     単純に3で割ると，16 / 3 = 5となり，余り1ビット分の文字数が
 *     切り捨てられてしまう
 *     3で割る前に2ビット分足すことで，切捨てを防ぐ
 *     (16 + 2) / 3 = 6文字
 */
#define CONVERT_BUFLEN	(((sizeof(uintptr) * 8U) + 2U) / 3U)

/*
 *  数値を文字列に変換
 */
static void
convert(uintptr val, uint32 radix, const char8 *radchar,
		uint32 width, boolean minus, boolean padzero, void (*outputc)(char8 c))
{
	char8	buf[CONVERT_BUFLEN];
	uint32	i, j;

	i = 0U;
	do {
		buf[i++] = radchar[val % radix];
		val /= radix;
	} while ((i < CONVERT_BUFLEN) && (val != 0U));

	if ((minus != FALSE) && (width > 0U)) {
		width -= 1U;
	}
	if ((minus != FALSE) && (padzero != FALSE)) {
		(*outputc)('-');
	}
	for (j = i; j < width; j++) {
		(*outputc)((char8) ((padzero != FALSE) ? '0' : ' '));
	}
	if ((minus != FALSE) && (padzero == FALSE)) {
		(*outputc)('-');
	}
	while (i > 0U) {
		(*outputc)(buf[--i]);
	}
}

/*
 *  文字列整形出力
 */
static const char8	raddec[] = "0123456789";
static const char8	radhex[] = "0123456789abcdef";
static const char8	radHEX[] = "0123456789ABCDEF";

void
syslog_printf(const char8 *format, const uintptr *p_args, void (*outputc)(char8 c))
{
	char8		c;
	uint32		width;
	boolean		padzero;
	sintptr		val;
	const char8	*str;

	while ((c = *format++) != '\0') {
		if (c != '%') {
			(*outputc)(c);
			continue;
		}

		width = 0U;
		padzero = FALSE;
		if ((c = *format++) == '0') {
			padzero = TRUE;
			c = *format++;
		}
		while (('0' <= c) && (c <= '9')) {
			width = (width * 10U) + (c - '0');
			c = *format++;
		}
		if (c == 'l') {
			c = *format++;
		}
		switch (c) {
		case 'd':
			val = (sintptr) (*p_args++);
			if (val >= 0) {
				convert((uintptr) val, 10U, raddec,
						width, FALSE, padzero, outputc);
			}
			else {
				convert((uintptr) (-val), 10U, raddec,
						width, TRUE, padzero, outputc);
			}
			break;
		case 'u':
			val = (sintptr) (*p_args++);
			convert((uintptr) val, 10U, raddec, width, FALSE, padzero, outputc);
			break;
		case 'x':
		case 'p':
			val = (sintptr) (*p_args++);
			convert((uintptr) val, 16U, radhex, width, FALSE, padzero, outputc);
			break;
		case 'X':
			val = (sintptr) (*p_args++);
			convert((uintptr) val, 16U, radHEX, width, FALSE, padzero, outputc);
			break;
		case 'c':
			(*outputc)((char8) (uintptr) (*p_args++));
			break;
		case 's':
			str = (const char8 *) (*p_args++);
			while ((c = *str++) != '\0') {
				(*outputc)(c);
			}
			break;
		case '%':
			(*outputc)('%');
			break;
		case '\0':
			format--;
			break;
		default:
			/* 上記のケース以外の場合，処理は行わない */
			break;
		}
	}
}

/*
 *  ログ情報の出力
 */
void
syslog_print(const SYSLOG *p_syslog, void (*outputc)(char8 c))
{
	switch (p_syslog->logtype) {
	case LOG_TYPE_COMMENT:
		syslog_printf((const char8 *) (p_syslog->loginfo[0]),
					  &(p_syslog->loginfo[1]), outputc);
		break;
	case LOG_TYPE_ASSERT:
		syslog_printf("%s:%u: Assertion '%s' failed.",
					  &(p_syslog->loginfo[0]), outputc);
		break;
	default:
		/* 上記のケース以外の場合，出力は行わない */
		break;
	}
}
