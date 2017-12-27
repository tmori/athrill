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
 *  $Id: vasyslog.c 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*
 *		可変数引数のシステムログライブラリ
 */
#include "t_syslog.h"
#include <stdarg.h>

#ifndef TOPPERS_OMIT_SYSLOG

void
syslog(uint32 prio, const char8 *format, ...)
{
	SYSLOG	syslog;
	va_list	ap;
	uint32	i;
	char8	c;
	boolean	lflag;

	syslog.logtype = LOG_TYPE_COMMENT;
	syslog.loginfo[0] = (sintptr) format;
	i = 1U;
	va_start(ap, format);

	while (((c = *format++) != '\0') && (i < TMAX_LOGINFO)) {
		if (c != '%') {
			continue;
		}

		lflag = FALSE;
		c = *format++;
		while (('0' <= c) && (c <= '9')) {
			c = *format++;
		}
		if (c == 'l') {
			lflag = TRUE;
			c = *format++;
		}
		switch (c) {
		case 'd':
			syslog.loginfo[i++] = (lflag != FALSE) ? (sintptr) va_arg(ap, sint32)
								  : (sintptr) va_arg(ap, sint32);
			break;
		case 'u':
		case 'x':
		case 'X':
			syslog.loginfo[i++] = (lflag != FALSE) ? (sintptr) va_arg(ap, uint32)
								  : (sintptr) va_arg(ap, uint32);
			break;
		case 'p':
			syslog.loginfo[i++] = (sintptr) va_arg(ap, void *);
			break;
		case 'c':
			syslog.loginfo[i++] = (sintptr) va_arg(ap, sint32);
			break;
		case 's':
			syslog.loginfo[i++] = (sintptr) va_arg(ap, const char8 *);
			break;
		case '\0':
			format--;
			break;
		default:
			/* 上記のケース以外の場合，処理は行わない */
			break;
		}
	}
	va_end(ap);
	(void) syslog_wri_log(prio, &syslog);
}

#endif /* TOPPERS_OMIT_SYSLOG */
