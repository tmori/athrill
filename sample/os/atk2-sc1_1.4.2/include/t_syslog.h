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
 *  $Id: t_syslog.h 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		システムログ出力を行うための定義
 *
 *  システムログサービスは，システムのログ情報を出力するためのサービス
 *  である
 *  カーネルからのログ情報の出力にも用いるため，内部で待ち状態にはいる
 *  ことはない
 *
 *  ログ情報は，カーネル内のログバッファに書き込むか，低レベルの文字出
 *  力関数を用いて出力する
 *  どちらを使うかは，拡張サービスコールで切り換えることができる
 *
 *  ログバッファ領域がオーバフローした場合には，古いログ情報を消して上
 *  書きする
 *
 *  アセンブリ言語のソースファイルからこのファイルをインクルードする時
 *  は，TOPPERS_MACRO_ONLYを定義しておくことで，マクロ定義以外の記述を
 *  除くことができる
 */

#ifndef TOPPERS_T_SYSLOG_H
#define TOPPERS_T_SYSLOG_H

#include "Os.h"

/*
 *  ログ情報の種別の定義
 */
#define LOG_TYPE_COMMENT	UINT_C(0x01)    /* コメント */
#define LOG_TYPE_ASSERT		UINT_C(0x02)    /* アサーションの失敗 */

#define LOG_TYPE_ISR			UINT_C(0x11)    /* 割込みサービスルーチン */
#define LOG_TYPE_ALM			UINT_C(0x12)    /* アラームハンドラ */
#define LOG_TYPE_TSKSTAT		UINT_C(0x13)    /* タスク状態変化 */
#define LOG_TYPE_DSP			UINT_C(0x14)    /* ディスパッチャ */
#define LOG_TYPE_SVC			UINT_C(0x15)    /* サービスコール */
#define LOG_TYPE_SCHTBL			UINT_C(0x16)    /* スケジュールテーブル満了処理 */
#define LOG_TYPE_STAHOOK		UINT_C(0x17)    /* スタートアップフック */
#define LOG_TYPE_ERRHOOK		UINT_C(0x18)    /* エラーフック */
#define LOG_TYPE_PROHOOK		UINT_C(0x19)    /* プロテクションフック */
#define LOG_TYPE_SHUTHOOK		UINT_C(0x1a)    /* シャットダウンフック */
#define LOG_TYPE_STAHOOKOSAP	UINT_C(0x1b)    /* OSAPスタートアップフック */
#define LOG_TYPE_ERRHOOKOSAP	UINT_C(0x1c)    /* OSAPエラーフック */
#define LOG_TYPE_SHUTHOOKOSAP	UINT_C(0x1d)    /* OSAPシャットダウンフック */
#define LOG_TYPE_TFN			UINT_C(0x1e)    /* 信頼関数 */

#define LOG_ENTER			UINT_C(0x00)    /* 入口／開始 */
#define LOG_LEAVE			UINT_C(0x80)    /* 出口／終了 */

/*
 *  ログ情報の重要度の定義
 */
#define LOG_EMERG			UINT_C(0)       /* シャットダウンに値するエラー */
#define LOG_ALERT			UINT_C(1)
#define LOG_CRIT			UINT_C(2)
#define LOG_ERROR			UINT_C(3)       /* システムエラー */
#define LOG_WARNING			UINT_C(4)       /* 警告メッセージ */
#define LOG_NOTICE			UINT_C(5)
#define LOG_INFO			UINT_C(6)
#define LOG_DEBUG			UINT_C(7)       /* デバッグ用メッセージ */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  ログ出力用データ型
 */

typedef uint32 ObjectIDType;                /* オブジェクトのID番号 */

/*
 *  ログ情報のデータ構造
 */

#define TMAX_LOGINFO	UINT_C(6)

typedef struct {
	uint32				logtype;                /* ログ情報の種別 */
	SystemTimeMsType	logtim;                 /* ログ時刻 */
	uintptr				loginfo[TMAX_LOGINFO];  /* その他のログ情報 */
} SYSLOG;

/*
 *  ログ情報の重要度のビットマップを作るためのマクロ
 */
#define LOG_MASK(prio)		(1U << (prio))
#define LOG_UPTO(prio)		((((uint32) 1) << ((prio) + 1U)) - 1U)

#ifndef TOPPERS_OMIT_SYSLOG

/*
 *  ログ情報を出力するためのライブラリ関数(syslog.c)
 */
extern StatusType syslog_wri_log(uint32 prio, const SYSLOG *p_syslog);

LOCAL_INLINE void
_syslog_1(uint32 prio, uint32 type, uintptr arg1)
{
	SYSLOG syslog;

	syslog.logtype = type;
	syslog.loginfo[0] = arg1;
	(void) syslog_wri_log(prio, &syslog);
}

LOCAL_INLINE void
_syslog_2(uint32 prio, uint32 type, uintptr arg1, uintptr arg2)
{
	SYSLOG syslog;

	syslog.logtype = type;
	syslog.loginfo[0] = arg1;
	syslog.loginfo[1] = arg2;
	(void) syslog_wri_log(prio, &syslog);
}

LOCAL_INLINE void
_syslog_3(uint32 prio, uint32 type, uintptr arg1, uintptr arg2,
		  uintptr arg3)
{
	SYSLOG syslog;

	syslog.logtype = type;
	syslog.loginfo[0] = arg1;
	syslog.loginfo[1] = arg2;
	syslog.loginfo[2] = arg3;
	(void) syslog_wri_log(prio, &syslog);
}

LOCAL_INLINE void
_syslog_4(uint32 prio, uint32 type, uintptr arg1, uintptr arg2,
		  uintptr arg3, uintptr arg4)
{
	SYSLOG syslog;

	syslog.logtype = type;
	syslog.loginfo[0] = arg1;
	syslog.loginfo[1] = arg2;
	syslog.loginfo[2] = arg3;
	syslog.loginfo[3] = arg4;
	(void) syslog_wri_log(prio, &syslog);
}

LOCAL_INLINE void
_syslog_5(uint32 prio, uint32 type, uintptr arg1, uintptr arg2,
		  uintptr arg3, uintptr arg4, uintptr arg5)
{
	SYSLOG syslog;

	syslog.logtype = type;
	syslog.loginfo[0] = arg1;
	syslog.loginfo[1] = arg2;
	syslog.loginfo[2] = arg3;
	syslog.loginfo[3] = arg4;
	syslog.loginfo[4] = arg5;
	(void) syslog_wri_log(prio, &syslog);
}

LOCAL_INLINE void
_syslog_6(uint32 prio, uint32 type, uintptr arg1, uintptr arg2,
		  uintptr arg3, uintptr arg4, uintptr arg5, uintptr arg6)
{
	SYSLOG syslog;

	syslog.logtype = type;
	syslog.loginfo[0] = arg1;
	syslog.loginfo[1] = arg2;
	syslog.loginfo[2] = arg3;
	syslog.loginfo[3] = arg4;
	syslog.loginfo[4] = arg5;
	syslog.loginfo[5] = arg6;
	(void) syslog_wri_log(prio, &syslog);
}

/*
 *  ログ情報（コメント）を出力するためのライブラリ関数（vasyslog.c）
 */
extern void syslog(uint32 prio, const char8 *format, ...);

#else /* TOPPERS_OMIT_SYSLOG */

/*
 *  システムログ出力を抑止する場合
 */

LOCAL_INLINE void
_syslog_1(uint32 prio, uint32 type, uintptr arg1)
{
}

LOCAL_INLINE void
_syslog_2(uint32 prio, uint32 type, uintptr arg1, uintptr arg2)
{
}

LOCAL_INLINE void
_syslog_3(uint32 prio, uint32 type, uintptr arg1, uintptr arg2,
		  uintptr arg3)
{
}

LOCAL_INLINE void
_syslog_4(uint32 prio, uint32 type, uintptr arg1, uintptr arg2,
		  uintptr arg3, uintptr arg4)
{
}

LOCAL_INLINE void
_syslog_5(uint32 prio, uint32 type, uintptr arg1, uintptr arg2,
		  uintptr arg3, uintptr arg4, uintptr arg5)
{
}

LOCAL_INLINE void
_syslog_6(uint32 prio, uint32 type, uintptr arg1, uintptr arg2,
		  uintptr arg3, uintptr arg4, uintptr arg5, uintptr arg6)
{
}

LOCAL_INLINE void
syslog(uint32 prio, const char8 *format, ...)
{
}

#endif /* TOPPERS_OMIT_SYSLOG */

/*
 *  ログ情報（コメント）を出力するためのマクロ
 *
 *  formatおよび後続の引数から作成したメッセージを，重大度prioでログ情
 *  報として出力するためのマクロ
 *  arg1〜argnはuintptr型にキャストするため，uintptr型に型変換できる任
 *  意の型でよい
 */

#define syslog_0(prio, format) \
	_syslog_1((prio), LOG_TYPE_COMMENT, (uintptr) (format))

#define syslog_1(prio, format, arg1)						\
	_syslog_2((prio), LOG_TYPE_COMMENT, (uintptr) (format),	\
			  (uintptr) (arg1))

#define syslog_2(prio, format, arg1, arg2)					\
	_syslog_3((prio), LOG_TYPE_COMMENT, (uintptr) (format),	\
			  (uintptr) (arg1), (uintptr) (arg2))

#define syslog_3(prio, format, arg1, arg2, arg3)			\
	_syslog_4((prio), LOG_TYPE_COMMENT, (uintptr) (format),	\
			  (uintptr) (arg1), (uintptr) (arg2), (uintptr) (arg3))

#define syslog_4(prio, format, arg1, arg2, arg3, arg4)				\
	_syslog_5((prio), LOG_TYPE_COMMENT, (uintptr) (format),			\
			  (uintptr) (arg1), (uintptr) (arg2), (uintptr) (arg3),	\
			  (uintptr) (arg4))

#define syslog_5(prio, format, arg1, arg2, arg3, arg4, arg5)		\
	_syslog_6((prio), LOG_TYPE_COMMENT, (uintptr) (format),			\
			  (uintptr) (arg1), (uintptr) (arg2), (uintptr) (arg3),	\
			  (uintptr) (arg4), (uintptr) (arg5))

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_T_SYSLOG_H */
