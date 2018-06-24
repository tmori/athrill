/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: kernel.h 837 2012-12-26 15:09:59Z ertl-hiro $
 */

/*
 *		TOPPERS/HRPカーネル 標準ヘッダファイル
 *
 *  TOPPERS/HRPカーネルがサポートするサービスコールの宣言と，必要なデー
 *  タ型，定数，マクロの定義を含むヘッダファイル．
 *
 *  アセンブリ言語のソースファイルからこのファイルをインクルードする時
 *  は，TOPPERS_MACRO_ONLYを定義しておく．これにより，マクロ定義以外を
 *  除くようになっている．
 *
 *  このファイルをインクルードする前にインクルードしておくべきファイル
 *  はない．
 */

#ifndef TOPPERS_KERNEL_H
#define TOPPERS_KERNEL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *	TOPPERS共通のデータ型・定数・マクロ
 */
#include <t_stddef.h>

/*
 *  ターゲット依存部
 */
#include "target_kernel.h"

#ifndef TOPPERS_MACRO_ONLY

/*
 *  データ型の定義
 */

/*
 *  ビットパターンやオブジェクト番号の型定義
 */
typedef	uint_t		TEXPTN;		/* タスク例外要因のビットパターン */
typedef	uint_t		FLGPTN;		/* イベントフラグのビットパターン */
typedef	ulong_t		OVRTIM;		/* プロセッサ時間 */
typedef	uint_t		INTNO;		/* 割込み番号 */
typedef	uint_t		INHNO;		/* 割込みハンドラ番号 */
typedef	uint_t		EXCNO;		/* CPU例外ハンドラ番号 */

/*
 *  処理単位の型定義
 */
typedef void	(*TASK)(intptr_t exinf);
typedef void	(*TEXRTN)(TEXPTN texptn, intptr_t exinf);
typedef void	(*CYCHDR)(intptr_t exinf);
typedef void	(*ALMHDR)(intptr_t exinf);
typedef void	(*OVRHDR)(ID tskid, intptr_t exinf);
typedef void	(*ISR)(intptr_t exinf);
typedef void	(*INTHDR)(void);
typedef void	(*EXCHDR)(void *p_excinf);
typedef ER_UINT	(*EXTSVC)(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid);
typedef void	(*INIRTN)(intptr_t exinf);
typedef void	(*TERRTN)(intptr_t exinf);

/*
 *  メモリ領域確保のための型定義
 */
#ifndef TOPPERS_STK_T
#define TOPPERS_STK_T	intptr_t
#endif /* TOPPERS_STK_T */
typedef	TOPPERS_STK_T	STK_T;	/* スタック領域を確保するための型 */

#ifndef TOPPERS_MPF_T
#define TOPPERS_MPF_T	intptr_t
#endif /* TOPPERS_MPF_T */
typedef	TOPPERS_MPF_T	MPF_T;	/* 固定長メモリプール領域を確保するための型 */

/*
 *  メッセージヘッダの型定義
 */
typedef	struct t_msg {			/* メールボックスのメッセージヘッダ */
	struct t_msg	*pk_next;
} T_MSG;

typedef	struct t_msg_pri {		/* 優先度付きメッセージヘッダ */
	T_MSG	msgque;				/* メッセージヘッダ */
	PRI		msgpri;				/* メッセージ優先度 */
} T_MSG_PRI;

/*
 *  パケット形式の定義
 */
typedef struct t_rtsk {
	STAT	tskstat;	/* タスク状態 */
	PRI		tskpri;		/* タスクの現在優先度 */
	PRI		tskbpri;	/* タスクのベース優先度 */
	STAT	tskwait;	/* 待ち要因 */
	ID		wobjid;		/* 待ち対象のオブジェクトのID */
	TMO		lefttmo;	/* タイムアウトするまでの時間 */
	uint_t	actcnt;		/* 起動要求キューイング数 */
	uint_t	wupcnt;		/* 起床要求キューイング数 */
	bool_t	texmsk;		/* タスク例外マスク状態か否か */
	bool_t	waifbd;		/* 待ち禁止状態か否か */
	uint_t	svclevel;	/* 拡張サービスコールのネストレベル */
} T_RTSK;

typedef struct t_rtex {
	STAT	texstat;	/* タスク例外処理の状態 */
	TEXPTN	pndptn;		/* 保留例外要因 */
} T_RTEX;

typedef struct t_rsem {
	ID		wtskid;		/* セマフォの待ち行列の先頭のタスクのID番号 */
	uint_t	semcnt;		/* セマフォの現在の資源数 */
} T_RSEM;

typedef struct t_rflg {
	ID		wtskid;		/* イベントフラグの待ち行列の先頭のタスクのID番号 */
	FLGPTN	flgptn;		/* イベントフラグの現在のビットパターン */
} T_RFLG;

typedef struct t_rdtq {
	ID		stskid;		/* データキューの送信待ち行列の先頭のタスクのID番号 */
	ID		rtskid;		/* データキューの受信待ち行列の先頭のタスクのID番号 */
	uint_t	sdtqcnt;	/* データキュー管理領域に格納されているデータの数 */
} T_RDTQ;

typedef struct t_rpdq {
	ID		stskid;		/* 優先度データキューの送信待ち行列の先頭のタスク
						   のID番号 */
	ID		rtskid;		/* 優先度データキューの受信待ち行列の先頭のタスク
						   のID番号 */
	uint_t	spdqcnt;	/* 優先度データキュー管理領域に格納されているデー
						   タの数 */
} T_RPDQ;

typedef struct t_rmtx {
	ID		htskid;		/* ミューテックスをロックしているタスクのID番号 */
	ID		wtskid;		/* ミューテックスの待ち行列の先頭のタスクのID番号 */
} T_RMTX;

typedef struct t_rmpf {
	ID		wtskid;		/* 固定長メモリプールの待ち行列の先頭のタスクの
						   ID番号 */
	uint_t	fblkcnt;	/* 固定長メモリプール領域の空きメモリ領域に割り
						   付けることができる固定長メモリブロックの数 */
} T_RMPF;

typedef struct t_rcyc {
	STAT	cycstat;	/* 周期ハンドラの動作状態 */
	RELTIM	lefttim;	/* 次に周期ハンドラを起動する時刻までの相対時間 */
} T_RCYC;

typedef struct t_ralm {
	STAT	almstat;	/* アラームハンドラの動作状態 */
	RELTIM	lefttim;	/* アラームハンドラを起動する時刻までの相対時間 */
} T_RALM;

typedef struct t_rovr {
	STAT	ovrstat;	/* オーバランハンドラの動作状態 */
	OVRTIM	leftotm;	/* 残りプロセッサ時間 */
} T_ROVR;

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  サービスコール呼出しのための定義と宣言
 */
#include <kernel_fncode.h>				/* 機能コードの定義 */
#include "target_svc.h"					/* ソフトウェア割込みによる呼出し */
#include <svc_call.h>					/* 関数呼出しによる呼出し */

#define SVC_CALL(svc)	_kernel_##svc	/* 関数呼出しによる呼び出す名称 */

/*
 *  オブジェクト属性の定義
 */
#define TA_ACT			UINT_C(0x02)	/* タスクを起動された状態で生成 */

#define TA_TPRI			UINT_C(0x01)	/* タスクの待ち行列を優先度順に */
#define TA_MPRI			UINT_C(0x02)	/* メッセージキューを優先度順に */

#define TA_WMUL			UINT_C(0x02)	/* 複数の待ちタスク */
#define TA_CLR			UINT_C(0x04)	/* イベントフラグのクリア指定 */

#define TA_CEILING		UINT_C(0x03)	/* 優先度上限プロトコル */

#define TA_STA			UINT_C(0x02)	/* 周期ハンドラを動作状態で生成 */

#define	TA_NOWRITE		UINT_C(0x01)	/* 書込みアクセス禁止 */
#define	TA_NOREAD		UINT_C(0x02)	/* 読出しアクセス禁止 */
#define	TA_EXEC			UINT_C(0x04)	/* 実行アクセス許可 */
#define	TA_MEMINI		UINT_C(0x08)	/* メモリの初期化を行う */
#define	TA_MEMPRSV		UINT_C(0x10)	/* メモリの初期化を行わない */
#define	TA_SDATA		UINT_C(0x20)	/* ショートデータ領域に配置 */
#define	TA_UNCACHE		UINT_C(0x40)	/* キャッシュ不可 */
#define	TA_IODEV		UINT_C(0x80)	/* 周辺デバイスの領域 */

#define TA_NONKERNEL	UINT_C(0x02)	/* カーネル管理外の割込み */

#define TA_ENAINT		UINT_C(0x01)	/* 割込み要求禁止フラグをクリア */
#define TA_EDGE			UINT_C(0x02)	/* エッジトリガ */

/*
 *  サービスコールの動作モードの定義
 */
#define TWF_ORW			UINT_C(0x01)	/* イベントフラグのOR待ち */
#define TWF_ANDW		UINT_C(0x02)	/* イベントフラグのAND待ち */

#define	TPM_WRITE		UINT_C(0x01)	/* 書込みアクセス権のチェック */
#define	TPM_READ		UINT_C(0x02)	/* 読出しアクセス権のチェック */
#define	TPM_EXEC		UINT_C(0x04)	/* 実行アクセス権のチェック */

/*
 *  オブジェクトの状態の定義
 */
#define TTS_RUN			UINT_C(0x01)	/* 実行状態 */
#define TTS_RDY			UINT_C(0x02)	/* 実行可能状態 */
#define TTS_WAI			UINT_C(0x04)	/* 待ち状態 */
#define TTS_SUS			UINT_C(0x08)	/* 強制待ち状態 */
#define TTS_WAS			UINT_C(0x0c)	/* 二重待ち状態 */
#define TTS_DMT			UINT_C(0x10)	/* 休止状態 */

#define TTW_SLP			UINT_C(0x0001)	/* 起床待ち */
#define TTW_DLY			UINT_C(0x0002)	/* 時間経過待ち */
#define TTW_SEM			UINT_C(0x0004)	/* セマフォの資源獲得待ち */
#define TTW_FLG			UINT_C(0x0008)	/* イベントフラグ待ち */
#define TTW_SDTQ		UINT_C(0x0010)	/* データキューへの送信待ち */
#define TTW_RDTQ		UINT_C(0x0020)	/* データキューからの受信待ち */
#define TTW_SPDQ		UINT_C(0x0100)	/* 優先度データキューへの送信待ち */
#define TTW_RPDQ		UINT_C(0x0200)	/* 優先度データキューからの受信待ち */
#define TTW_MTX			UINT_C(0x0080)	/* ミューテックスのロック待ち状態 */
#define TTW_MPF			UINT_C(0x2000)	/* 固定長メモリブロックの獲得待ち */

#define TTEX_ENA		UINT_C(0x01)	/* タスク例外処理許可状態 */
#define TTEX_DIS		UINT_C(0x02)	/* タスク例外処理禁止状態 */

#define TCYC_STP		UINT_C(0x01)	/* 周期ハンドラが動作していない */
#define TCYC_STA		UINT_C(0x02)	/* 周期ハンドラが動作している */

#define TALM_STP		UINT_C(0x01)	/* アラームハンドラが動作していない */
#define TALM_STA		UINT_C(0x02)	/* アラームハンドラが動作している */

#define TOVR_STP		UINT_C(0x01)	/* オーバランハンドラが動作していない*/
#define TOVR_STA		UINT_C(0x02)	/* オーバランハンドラが動作している */

/*
 *  保護ドメインID
 */
#define TDOM_SELF		0				/* 自タスクの属する保護ドメイン */
#define TDOM_KERNEL		(-1)			/* カーネルドメイン */
#define TDOM_NONE		(-2)			/* 無所属（保護ドメインに属さない）*/

/*
 *  その他の定数の定義
 */
#define TSK_SELF		0			/* 自タスク指定 */
#define TSK_NONE		0			/* 該当するタスクがない */

#define TPRI_SELF		0			/* 自タスクのベース優先度 */
#define TPRI_INI		0			/* タスクの起動時優先度 */

#define TIPM_ENAALL		0			/* 割込み優先度マスク全解除 */

/*
 *  構成定数とマクロ
 */

/*
 *  サポートする機能
 */
#ifdef TOPPERS_TARGET_SUPPORT_DIS_INT
#define TOPPERS_SUPPORT_DIS_INT			/* dis_intがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_DIS_INT */

#ifdef TOPPERS_TARGET_SUPPORT_ENA_INT
#define TOPPERS_SUPPORT_ENA_INT			/* ena_intがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_ENA_INT */

#ifdef TOPPERS_TARGET_SUPPORT_GET_UTM
#define TOPPERS_SUPPORT_GET_UTM			/* get_utmがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_GET_UTM */

#define TOPPERS_SUPPORT_MUTEX			/* ミューテックス機能拡張 */

#ifdef TOPPERS_TARGET_SUPPORT_OVRHDR
#define TOPPERS_SUPPORT_OVRHDR			/* オーバランハンドラ機能拡張 */
#endif /* TOPPERS_TARGET_SUPPORT_OVRHDR */

#define TOPPERS_SUPPORT_PROTECT			/* 保護機能対応のカーネル */

#ifdef TOPPERS_TARGET_SUPPORT_ATT_MOD
#define TOPPERS_SUPPORT_ATT_MOD			/* ATT_MODがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_ATT_MOD */

#ifdef TOPPERS_TARGET_SUPPORT_ATT_PMA
#define TOPPERS_SUPPORT_ATT_PMA			/* ATT_PMAがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_ATT_PMA */

/*
 *  優先度の範囲
 */
#define TMIN_TPRI		1			/* タスク優先度の最小値（最高値）*/
#define TMAX_TPRI		16			/* タスク優先度の最大値（最低値）*/
#define TMIN_DPRI		1			/* データ優先度の最小値（最高値）*/
#define TMAX_DPRI		16			/* データ優先度の最大値（最低値）*/
#define TMIN_MPRI		1			/* メッセージ優先度の最小値（最高値）*/
#define TMAX_MPRI		16			/* メッセージ優先度の最大値（最低値）*/
#define TMIN_ISRPRI		1			/* 割込みサービスルーチン優先度の最小値 */
#define TMAX_ISRPRI		16			/* 割込みサービスルーチン優先度の最大値 */

/*
 *  バージョン情報
 */
#define TKERNEL_MAKER	UINT_C(0x0118)	/* カーネルのメーカーコード */
#define TKERNEL_PRID	UINT_C(0x0006)	/* カーネルの識別番号 */
#define TKERNEL_SPVER	UINT_C(0xf515)	/* カーネル仕様のバージョン番号 */
#define TKERNEL_PRVER	UINT_C(0x2010)	/* カーネルのバージョン番号 */

/*
 *  キューイング回数の最大値
 */
#define TMAX_ACTCNT		UINT_C(1)		/* 起動要求キューイング数の最大値 */
#define TMAX_WUPCNT		UINT_C(1)		/* 起床要求キューイング数の最大値 */

/*
 *  ビットパターンのビット数
 */
#ifndef TBIT_TEXPTN					/* タスク例外要因のビット数 */
#define TBIT_TEXPTN		(sizeof(TEXPTN) * CHAR_BIT)
#endif /* TBIT_TEXPTN */

#ifndef TBIT_FLGPTN					/* イベントフラグのビット数 */
#define TBIT_FLGPTN		(sizeof(FLGPTN) * CHAR_BIT)
#endif /* TBIT_FLGPTN */

/*
 *  プロセッサ時間（OVRTIM）の最大値
 */
#ifndef TMAX_OVRTIM
#define TMAX_OVRTIM		ULONG_MAX
#endif /* TMAX_OVRTIM */

/*
 *  メモリ領域確保のためのマクロ
 *
 *  以下のTOPPERS_COUNT_SZとTOPPERS_ROUND_SZの定義は，unitが2の巾乗であ
 *  ることを仮定している．
 */
#ifndef TOPPERS_COUNT_SZ
#define TOPPERS_COUNT_SZ(sz, unit)	(((sz) + (unit) - 1) / (unit))
#endif /* TOPPERS_COUNT_SZ */
#ifndef TOPPERS_ROUND_SZ
#define TOPPERS_ROUND_SZ(sz, unit)	(((sz) + (unit) - 1) & ~((unit) - 1))
#endif /* TOPPERS_ROUND_SZ */

#define COUNT_STK_T(sz)		TOPPERS_COUNT_SZ(sz, sizeof(STK_T))
#define ROUND_STK_T(sz)		TOPPERS_ROUND_SZ(sz, sizeof(STK_T))

#define COUNT_MPF_T(blksz)	TOPPERS_COUNT_SZ(blksz, sizeof(MPF_T))
#define ROUND_MPF_T(blksz)	TOPPERS_ROUND_SZ(blksz, sizeof(MPF_T))

/*
 *  その他の構成定数
 */
#define TMAX_MAXSEM		UINT_MAX	/* セマフォの最大資源数の最大値 */

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_KERNEL_H */
