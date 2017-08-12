/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: vasyslog.c 509 2016-01-12 06:06:14Z ertl-hiro $
 */

/*
 *		可変数引数のシステムログライブラリ
 */

#include <t_stddef.h>
#include <t_syslog.h>
#include <stdarg.h>

#ifndef TOPPERS_OMIT_SYSLOG

void
syslog(uint_t prio, const char *format, ...)
{
	SYSLOG	logbuf;
	va_list	ap;
	uint_t	i;
	char	sz;
	char	c;

	logbuf.logtype = LOG_TYPE_COMMENT;
	logbuf.logpar[0] = (LOGPAR) format;
	i = 1U;
	va_start(ap, format);

	while ((c = *format++) != '\0' && i < TNUM_LOGPAR) {
		if (c != '%') {
			continue;
		}

		sz = '\0';
		c = *format++;
		while ('0' <= c && c <= '9') {
			c = *format++;
		}
		if (c == 'l' || c == 't' || c == 'T') {
			sz = c;
			c = *format++;
		}
		switch (c) {
		case 'd':
			logbuf.logpar[i++] = (sz == 'l') ? (LOGPAR) va_arg(ap, long_t)
							   : (sz == 't') ? (LOGPAR) va_arg(ap, int32_t)
							   : (sz == 'T') ? (LOGPAR) va_arg(ap, SYSTIM)
							   : (LOGPAR) va_arg(ap, int_t);
			break;
		case 'u':
		case 'x':
		case 'X':
			logbuf.logpar[i++] = (sz == 'l') ? (LOGPAR) va_arg(ap, ulong_t)
							   : (sz == 't') ? (LOGPAR) va_arg(ap, uint32_t)
							   : (sz == 'T') ? (LOGPAR) va_arg(ap, SYSTIM)
							   : (LOGPAR) va_arg(ap, uint_t);
			break;
		case 'p':
			logbuf.logpar[i++] = (LOGPAR) va_arg(ap, void *);
			break;
		case 'c':
			logbuf.logpar[i++] = (LOGPAR) va_arg(ap, int);
			break;
		case 's':
			logbuf.logpar[i++] = (LOGPAR) va_arg(ap, const char *);
			break;
		case '\0':
			format--;
			break;
		default:
			break;
		}
	}
	va_end(ap);
	(void) tSysLog_eSysLog_write(prio, &logbuf);
}

#endif /* TOPPERS_OMIT_SYSLOG */
