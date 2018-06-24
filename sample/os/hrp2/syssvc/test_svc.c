/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 * 
 *  Copyright (C) 2005-2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_svc.c 837 2012-12-26 15:09:59Z ertl-hiro $
 */

/* 
 *		テストプログラム用システムサービス
 */

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <log_output.h>
#include "syssvc/syslog.h"
#include "target_syssvc.h"
#include <test_lib.h>

/*
 *	チェックポイント
 */
static uint_t	check_count = 0u;

/*
 *	自己診断関数
 */
static BIT_FUNC	check_bit_func = NULL;

/*
 *	チェックポイント
 */
void
_test_check_point(uint_t count)
{
	bool_t	errorflag = false;
	ER		rercd;
	SIL_PRE_LOC;

	/*
	 *  割込みロック状態に
	 */
	SIL_LOC_INT();

	/*
	 *  シーケンスチェック
	 */
	if (++check_count == count) {
		syslog_1(LOG_NOTICE, "Check point %d passed.", count);
	}
	else {
		syslog_1(LOG_ERROR, "## Unexpected check point %d.", count);
		errorflag = true;
	}

	/*
	 *  カーネルの内部状態の検査
	 */
	if (check_bit_func != NULL) {
		rercd = (*check_bit_func)();
		if (rercd < 0) {
			syslog_2(LOG_ERROR, "## Internal inconsistency detected (%s, %d).",
								itron_strerror(rercd), SERCD(rercd));
			errorflag = true;
		}
	}

	/*
	 *  エラーが検出された場合は，テストプログラムを終了する．
	 */
	if (errorflag) {
		test_finish();
	}

	/*
	 *  割込みロック状態を解除
	 */
	SIL_UNL_INT();
}

/*
 *	自己診断関数の設定
 */
void
_test_set_bit_func(BIT_FUNC bit_func)
{
	check_bit_func = bit_func;
}

/*
 *  システムログの出力処理
 */
void
_test_syslog_flush(void)
{
	SYSLOG	logbuf;
	ER_UINT	rercd;

	/*
	 *  ログバッファに記録されたログ情報を，低レベル出力機能を用いて出
	 *  力する．
	 */
	while ((rercd = syslog_rea_log(&logbuf)) >= 0) {
		if (rercd > 0) {
			syslog_lostmsg((uint_t) rercd, target_fput_log);
		}
		if (logbuf.logtype >= LOG_TYPE_COMMENT) {
			syslog_print(&logbuf, target_fput_log);
			target_fput_log('\n');
		}
	}
}

/*
 *	テストプログラムの終了
 */
void
_test_test_finish(void)
{
	SIL_PRE_LOC;

	SIL_LOC_INT();
	syslog_flush();
	(void) ext_ker();

	/* ここへ来ることはないはず */
	SIL_UNL_INT();
}

/*
 *  テストプログラム用ライブラリを拡張サービスコールとして登録するため
 *  の定義
 */
ER_UINT
extsvc_check_point(intptr_t count, intptr_t par2, intptr_t par3,
							intptr_t par4, intptr_t par5, ID cdmid)
{
	_test_check_point((uint_t) count);
	return(E_OK);
}

ER_UINT
extsvc_set_bit_func(intptr_t bit_func, intptr_t par2, intptr_t par3,
							intptr_t par4, intptr_t par5, ID cdmid)
{
	_test_set_bit_func((BIT_FUNC) bit_func);
	return(E_OK);
}

ER_UINT
extsvc_syslog_flush(intptr_t par1, intptr_t par2, intptr_t par3,
							intptr_t par4, intptr_t par5, ID cdmid)
{
	_test_syslog_flush();
	return(E_OK);
}

ER_UINT
extsvc_test_finish(intptr_t par1, intptr_t par2, intptr_t par3,
							intptr_t par4, intptr_t par5, ID cdmid)
{
	_test_test_finish();
	return(E_OK);
}
