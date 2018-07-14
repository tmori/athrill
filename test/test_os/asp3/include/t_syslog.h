/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
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
 *  $Id: t_syslog.h 509 2016-01-12 06:06:14Z ertl-hiro $
 */

/*
 *		システムログ出力を行うための定義
 *
 *  システムログサービスは，システムのログ情報を出力するためのサービス
 *  である．カーネルからのログ情報の出力にも用いるため，内部で待ち状態
 *  にはいることはない．
 *
 *  ログ情報は，カーネル内のログバッファに書き込むか，低レベルの文字出
 *  力関数を用いて出力する．どちらを使うかは，拡張サービスコールで切り
 *  換えることができる．
 *
 *  ログバッファ領域がオーバフローした場合には，古いログ情報を消して上
 *  書きする．
 *
 *  アセンブリ言語のソースファイルからこのファイルをインクルードする時
 *  は，TOPPERS_MACRO_ONLYを定義しておくことで，マクロ定義以外の記述を
 *  除くことができる．
 */

#ifndef TOPPERS_T_SYSLOG_H
#define TOPPERS_T_SYSLOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <t_stddef.h>

/*
 *  ログ情報の種別の定義
 */
#define LOG_TYPE_COMMENT	UINT_C(0x01)	/* コメント */
#define LOG_TYPE_ASSERT		UINT_C(0x02)	/* アサーションの失敗 */

#define LOG_TYPE_INH		UINT_C(0x11)	/* 割込みハンドラ */
#define LOG_TYPE_ISR		UINT_C(0x12)	/* 割込みサービスルーチン */
#define LOG_TYPE_CYC		UINT_C(0x13)	/* 周期ハンドラ */
#define LOG_TYPE_ALM		UINT_C(0x14)	/* アラームハンドラ */
#define LOG_TYPE_OVR		UINT_C(0x15)	/* オーバランハンドラ */
#define LOG_TYPE_EXC		UINT_C(0x16)	/* CPU例外ハンドラ */
#define LOG_TYPE_TSKSTAT	UINT_C(0x21)	/* タスク状態変化 */
#define LOG_TYPE_DSP		UINT_C(0x31)	/* ディスパッチャ */
#define LOG_TYPE_SVC		UINT_C(0x41)	/* サービスコール */

#define LOG_ENTER			UINT_C(0x00)	/* 入口／開始 */
#define LOG_LEAVE			UINT_C(0x80)	/* 出口／終了 */

/*
 *  ログ情報の重要度の定義
 */
#define LOG_EMERG			UINT_C(0)		/* シャットダウンに値するエラー */
#define LOG_ALERT			UINT_C(1)
#define LOG_CRIT			UINT_C(2)
#define LOG_ERROR			UINT_C(3)		/* システムエラー */
#define LOG_WARNING			UINT_C(4)		/* 警告メッセージ */
#define LOG_NOTICE			UINT_C(5)
#define LOG_INFO			UINT_C(6)
#define LOG_DEBUG			UINT_C(7)		/* デバッグ用メッセージ */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  ログ情報のデータ構造
 */

#ifndef LOGTIM
#define LOGTIM		HRTCNT			/* ログ時刻のデータ型 */
#endif /* LOGTIM */

#define TNUM_LOGPAR		6			/* ログパラメータの数 */

#ifndef LOGPAR
#define LOGPAR		intptr_t		/* ログパラメータのデータ型 */
#define ULOGPAR		uintptr_t
#endif /* LOGPAR */

typedef struct {
	uint_t	logtype;				/* ログ情報の種別 */
	LOGTIM	logtim;					/* ログ時刻 */
	LOGPAR	logpar[TNUM_LOGPAR];	/* ログパラメータ */
} SYSLOG;

/*
 *  ログ情報の重要度のビットマップを作るためのマクロ
 */
#define LOG_MASK(prio)		(1U << (prio))
#define LOG_UPTO(prio)		((1U << ((prio) + 1)) - 1)

/*
 *  パケット形式の定義
 */
typedef struct t_syslog_rlog {
	uint_t	count;		/* ログバッファ中のログの数 */
	uint_t	lost;		/* 失われたログの数 */
	uint_t	logmask;	/* ログバッファに記録すべき重要度 */
	uint_t	lowmask;	/* 低レベル出力すべき重要度 */
} T_SYSLOG_RLOG;

#ifndef TOPPERS_OMIT_SYSLOG

/*
 *  ログ情報を出力するためのライブラリ関数
 *
 *  TECSで記述されたシステムログ機能を直接呼び出す．
 */

extern ER	tSysLog_eSysLog_write(uint_t prio, const SYSLOG *p_syslog) throw();

Inline void
_syslog_0(uint_t prio, uint_t type)
{
	SYSLOG	logbuf;

	logbuf.logtype = type;
	(void) tSysLog_eSysLog_write(prio, &logbuf);
}

Inline void
_syslog_1(uint_t prio, uint_t type, LOGPAR arg1)
{
	SYSLOG	logbuf;

	logbuf.logtype = type;
	logbuf.logpar[0] = arg1;
	(void) tSysLog_eSysLog_write(prio, &logbuf);
}

Inline void
_syslog_2(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2)
{
	SYSLOG	logbuf;

	logbuf.logtype = type;
	logbuf.logpar[0] = arg1;
	logbuf.logpar[1] = arg2;
	(void) tSysLog_eSysLog_write(prio, &logbuf);
}

Inline void
_syslog_3(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2, LOGPAR arg3)
{
	SYSLOG	logbuf;

	logbuf.logtype = type;
	logbuf.logpar[0] = arg1;
	logbuf.logpar[1] = arg2;
	logbuf.logpar[2] = arg3;
	(void) tSysLog_eSysLog_write(prio, &logbuf);
}

Inline void
_syslog_4(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2,
											LOGPAR arg3, LOGPAR arg4)
{
	SYSLOG	logbuf;

	logbuf.logtype = type;
	logbuf.logpar[0] = arg1;
	logbuf.logpar[1] = arg2;
	logbuf.logpar[2] = arg3;
	logbuf.logpar[3] = arg4;
	(void) tSysLog_eSysLog_write(prio, &logbuf);
}

Inline void
_syslog_5(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2,
								LOGPAR arg3, LOGPAR arg4, LOGPAR arg5)
{
	SYSLOG	logbuf;

	logbuf.logtype = type;
	logbuf.logpar[0] = arg1;
	logbuf.logpar[1] = arg2;
	logbuf.logpar[2] = arg3;
	logbuf.logpar[3] = arg4;
	logbuf.logpar[4] = arg5;
	(void) tSysLog_eSysLog_write(prio, &logbuf);
}

Inline void
_syslog_6(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2,
					LOGPAR arg3, LOGPAR arg4, LOGPAR arg5, LOGPAR arg6)
{
	SYSLOG	logbuf;

	logbuf.logtype = type;
	logbuf.logpar[0] = arg1;
	logbuf.logpar[1] = arg2;
	logbuf.logpar[2] = arg3;
	logbuf.logpar[3] = arg4;
	logbuf.logpar[4] = arg5;
	logbuf.logpar[5] = arg6;
	(void) tSysLog_eSysLog_write(prio, &logbuf);
}

/*
 *  ログ情報（コメント）を出力するためのライブラリ関数（vasyslog.c）
 */
extern void	syslog(uint_t prio, const char *format, ...) throw();

#else /* TOPPERS_OMIT_SYSLOG */

/*
 *  システムログ出力を抑止する場合
 */

Inline void
_syslog_0(uint_t prio, uint_t type)
{
}

Inline void
_syslog_1(uint_t prio, uint_t type, LOGPAR arg1)
{
}

Inline void
_syslog_2(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2)
{
}

Inline void
_syslog_3(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2, LOGPAR arg3)
{
}

Inline void
_syslog_4(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2,
											LOGPAR arg3, LOGPAR arg4)
{
}

Inline void
_syslog_5(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2,
								LOGPAR arg3, LOGPAR arg4, LOGPAR arg5)
{
}

Inline void
_syslog_6(uint_t prio, uint_t type, LOGPAR arg1, LOGPAR arg2,
					LOGPAR arg3, LOGPAR arg4, LOGPAR arg5, LOGPAR arg6)
{
}

Inline void
syslog(uint_t prio, const char *format, ...)
{
}

#endif /* TOPPERS_OMIT_SYSLOG */

/*
 *  ログ情報（コメント）を出力するためのマクロ
 *
 *  formatおよび後続の引数から作成したメッセージを，重大度prioでログ情
 *  報として出力するためのマクロ．arg1〜argnはLOGPAR型にキャストするた
 *  め，LOGPAR型に型変換できる任意の型でよい．
 */

#define syslog_0(prio, format) \
				_syslog_1(prio, LOG_TYPE_COMMENT, (LOGPAR)(format))

#define syslog_1(prio, format, arg1) \
				_syslog_2(prio, LOG_TYPE_COMMENT, (LOGPAR)(format), \
														(LOGPAR)(arg1))

#define syslog_2(prio, format, arg1, arg2) \
				_syslog_3(prio, LOG_TYPE_COMMENT, (LOGPAR)(format), \
										(LOGPAR)(arg1), (LOGPAR)(arg2))

#define syslog_3(prio, format, arg1, arg2, arg3) \
				_syslog_4(prio, LOG_TYPE_COMMENT, (LOGPAR)(format), \
						(LOGPAR)(arg1), (LOGPAR)(arg2), (LOGPAR)(arg3))

#define syslog_4(prio, format, arg1, arg2, arg3, arg4) \
				_syslog_5(prio, LOG_TYPE_COMMENT, (LOGPAR)(format), \
						(LOGPAR)(arg1), (LOGPAR)(arg2), (LOGPAR)(arg3), \
														(LOGPAR)(arg4))

#define syslog_5(prio, format, arg1, arg2, arg3, arg4, arg5) \
				_syslog_6(prio, LOG_TYPE_COMMENT, (LOGPAR)(format), \
						(LOGPAR)(arg1), (LOGPAR)(arg2), (LOGPAR)(arg3), \
										(LOGPAR)(arg4), (LOGPAR)(arg5))

/*
 *  ログ情報（アサーションの失敗）を出力するためのマクロ
 */
#ifndef TOPPERS_assert_fail
#define TOPPERS_assert_fail(exp, file, line) \
				_syslog_3(LOG_EMERG, LOG_TYPE_ASSERT, (LOGPAR)(file), \
										(LOGPAR)(line), (LOGPAR)(exp))
#endif /* TOPPERS_assert_fail */

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_T_SYSLOG_H */
