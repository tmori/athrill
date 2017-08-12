/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: tSysLog.c 550 2016-01-17 04:16:26Z ertl-hiro $
 */

/*
 *		システムログ機能
 */

#include <sil.h>
#include "tSysLog_tecsgen.h"
#undef TOPPERS_OMIT_SYSLOG
#include <t_syslog.h>
#include <log_output.h>
#include "target_syssvc.h"

/*
 *  トレースログマクロのデフォルト定義
 *
 *  システムログに出力されたログ情報をトレースログにも記録できるように，
 *  eSyslog_writeの入口にトレースログマクロを置く．
 */
#ifndef LOG_TSYSLOG_ESYSLOG_WRITE_ENTER
#define LOG_TSYSLOG_ESYSLOG_WRITE_ENTER(priority, p_syslog)
#endif /* LOG_TSYSLOG_ESYSLOG_WRITE_ENTER */

/*
 *  ログ時刻の取り出し
 *
 *  デフォルトでは，ログ時刻として，高分解能タイマのカウント値を用いて
 *  いる．ターゲット依存で変更する場合には，SYSLOG_GET_LOGTIMに，ログ時
 *  刻を取り出すマクロを定義する．
 */
#ifndef SYSLOG_GET_LOGTIM
#define target_hrt_get_current		_kernel_target_hrt_get_current
#include "target_timer.h"
#define SYSLOG_GET_LOGTIM(p_logtim) \
				(*(p_logtim) = _kernel_target_hrt_get_current())
#endif /* SYSLOG_GET_LOGTIM */

/*
 *  低レベル出力の呼出し関数
 */
static void
low_putchar(char c)
{
	cPutLog_putChar(c);
}

/*
 *  ログ情報の出力（受け口関数）
 *
 *  CPUロック状態や実行コンテキストによらず動作できるように実装してある．
 */
ER
eSysLog_write(uint_t priority, const SYSLOG *p_syslog)
{
	SIL_PRE_LOC;

	LOG_TSYSLOG_ESYSLOG_WRITE_ENTER(priority, p_syslog);
	SIL_LOC_INT();

	/*
	 *  ログ時刻の設定
	 */
	SYSLOG_GET_LOGTIM(&(((SYSLOG *) p_syslog)->logtim));

	/*
	 *  ログバッファに記録
	 */
	if ((VAR_logMask & LOG_MASK(priority)) != 0U) {
		VAR_logBuffer[VAR_tail] = *p_syslog;
		VAR_tail++;
		if (VAR_tail >= ATTR_logBufferSize) {
			VAR_tail = 0U;
		}
		if (VAR_count < ATTR_logBufferSize) {
			VAR_count++;
		}
		else {
			VAR_head = VAR_tail;
			VAR_lost++;
		}
	}

	/*
	 *  低レベル出力
	 */
	if ((VAR_lowMask & LOG_MASK(priority)) != 0U) {
		syslog_print(p_syslog, low_putchar);
		low_putchar('\n');
	}

	SIL_UNL_INT();
	return(E_OK);
}

/*
 *  ログバッファからの読出し（受け口関数）
 *
 *  CPUロック状態や実行コンテキストによらず動作できるように実装してある．
 */
ER_UINT
eSysLog_read(SYSLOG *p_syslog)
{
	ER_UINT	ercd;
	SIL_PRE_LOC;

	SIL_LOC_INT();

	/*
	 *  ログバッファからの取出し
	 */
	if (VAR_count > 0U) {
		*p_syslog = VAR_logBuffer[VAR_head];
		VAR_count--;
		VAR_head++;
		if (VAR_head >= ATTR_logBufferSize) {
			VAR_head = 0U;
		}
		ercd = (ER_UINT) VAR_lost;
		VAR_lost = 0U;
	}
	else {
		ercd = E_OBJ;
	}

	SIL_UNL_INT();
	return(ercd);
}

/* 
 *  出力すべきログ情報の重要度の設定（受け口関数）
 */
ER
eSysLog_mask(uint_t logMask, uint_t lowMask)
{
	SIL_PRE_LOC;

	SIL_LOC_INT();
	VAR_logMask = logMask;
	VAR_lowMask = lowMask;
	SIL_UNL_INT();
	return(E_OK);
}

/*
 *  ログバッファの状態参照（受け口関数）
 */
ER_UINT
eSysLog_refer(T_SYSLOG_RLOG *pk_rlog)
{
	SIL_PRE_LOC;

	SIL_LOC_INT();
	pk_rlog->count = VAR_count;
	pk_rlog->lost = VAR_lost;
	pk_rlog->logmask = VAR_logMask;
	pk_rlog->lowmask = VAR_lowMask;
	SIL_UNL_INT();
	return(E_OK);
}

/* 
 *  低レベル出力によるすべてのログ情報の出力（受け口関数）
 */
ER
eSysLog_flush(void)
{
	SYSLOG	logbuf;
	ER_UINT	rercd;
	SIL_PRE_LOC;

	SIL_LOC_INT();

	while ((rercd = eSysLog_read(&logbuf)) >= 0) {
		if (rercd > 0) {
			syslog_lostmsg((uint_t) rercd, low_putchar);
		}
		if (logbuf.logtype >= LOG_TYPE_COMMENT) {
			syslog_print(&logbuf, low_putchar);
			low_putchar('\n');
		}
	}

	SIL_UNL_INT();
	return(E_OK);
}
