/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	
 *	上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *	ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *	変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *	(1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *		権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *		スコード中に含まれていること．
 *	(2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *		用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *		者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *		の無保証規定を掲載すること．
 *	(3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *		用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *		と．
 *	  (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *		  作権表示，この利用条件および下記の無保証規定を掲載すること．
 *	  (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *		  報告すること．
 *	(4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *		害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *		また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *		由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *		免責すること．
 *	
 *	本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *	よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *	に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *	アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *	の責任を負わない．
 *	
 */

/*
 *	タイマドライバ（SH2AW用）
 */

#ifndef TOPPERS_TARGET_TIMER_H
#define TOPPERS_TARGET_TIMER_H

#include <sil.h>

/*
 *  CMT（コンペアマッチタイマ）関連のレジスタ
 */
#define CM01STR 0xfffec000 /* CMT0, CMT1開始レジスタ */
#define CMT0_STR 0x0001    /* CMT0スタート */
#define CMT1_STR 0x0002    /* CMT1スタート */
#define CM23STR 0xfffec010 /* CMT2, CMT3開始レジスタ */
#define CMT2_STR 0x0001    /* CMT2スタート */
#define CMT3_STR 0x0002    /* CMT3スタート */

#define CMT0CR 0xfffec002 /* CMT0制御レジスタ */
#define CMT1CR 0xfffec008 /* CMT1制御レジスタ */
#define CMT2CR 0xfffec012 /* CMT2制御レジスタ */
#define CMT3CR 0xfffec018 /* CMT3制御レジスタ */
#define CMCR_CMIE 0x0040  /* 割込み許可（ビットが0だと禁止） */
/*
 *	分周比 /8,/32,/128,/512 のいずれかを選択
 */
#define CMSR_CKS_DIV8  0x0000U
#define CMSR_CKS_DIV32  0x0001U
#define CMSR_CKS_DIV128  0x0002U
#define CMSR_CKS_DIV512  0x0003U

#define CMT0CNT 0xfffec004 /* CMT0カウンタ */
#define CMT1CNT 0xfffec00a /* CMT1カウンタ */
#define CMT2CNT 0xfffec014 /* CMT2カウンタ */
#define CMT3CNT 0xfffec01a /* CMT3カウンタ */

#define CMT0CMSE 0xfffec006 /* CMT0コンペアマッチ設定レジスタ */
#define CMT1CMSE 0xfffec00c /* CMT1コンペアマッチ設定レジスタ */
#define CMT2CMSE 0xfffec016 /* CMT2コンペアマッチ設定レジスタ */
#define CMT3CMSE 0xfffec01c /* CMT3コンペアマッチ設定レジスタ */

#define INTNO_CMT0 142 /* CMT0割込み番号 */
#define INTNO_CMT1 143 /* CMT1割込み番号 */
#define INTNO_CMT2 144 /* CMT2割込み番号 */
#define INTNO_CMT3 145 /* CMT3割込み番号 */

/*
 *  CMT0を使用する
 */

/*
 *	タイマ値の内部表現とミリ秒単位との変換
 *	1msecあたりのクロックカウント数
 *	現在の設定：
 *	  f(CPU)は160MHz
 *	  f(PBA)は40MHz(f(CPU)の4分周)
 *	  CMTのカウントソースは32分周
 */
#define TIMER_CLOCK ((40000000/32) / 1000)

/*
 *	タイマ割込みハンドラ登録のための定数
 */
#define INHNO_TIMER 	INTNO_CMT0    /* 割込みハンドラ番号 */
#define INTNO_TIMER 	INTNO_CMT0    /* 割込み番号 */
#define INTPRI_TIMER	(-6)		  /* 割込み優先度 */
#define INTATR_TIMER	0U			  /* 割込み属性 */


#ifndef TOPPERS_TARGET_SUPPORT_OVRHDR
#define TOPPERS_TARGET_SUPPORT_OVRHDR
#endif /* TOPPERS_TARGET_SUPPORT_OVRHDR */

#ifdef TOPPERS_TARGET_SUPPORT_OVRHDR
/*
 *  CMT1を使用する
 */

/*
 *	タイマ割込みハンドラ登録のための定数
 */
#define INHNO_OVRTIMER 	INTNO_CMT1    /* 割込みハンドラ番号 */
#define INTNO_OVRTIMER 	INTNO_CMT1    /* 割込み番号 */
#define INTPRI_OVRTIMER	(-7)		  /* 割込み優先度 */
#define INTATR_OVRTIMER	0U			  /* 割込み属性 */

#endif /* TOPPERS_TARGET_SUPPORT_OVRHDR */

#ifndef TOPPERS_MACRO_ONLY

/*
 *	タイマ値の内部表現の型
 */
typedef uint32_t	CLOCK;

/*
 *	タイマ値の内部表現とミリ秒・μ秒単位との変換
 *
 */
#define TO_CLOCK(nume, deno)	((CLOCK)(TIMER_CLOCK * (nume) / (deno)))
#define TO_USEC(clock)			(((SYSUTM) clock) * 1000U / TIMER_CLOCK)

/*
 *	設定できる最大のタイマ周期（単位は内部表現）
 */
#define MAX_CLOCK	 ((CLOCK) 0xffffU)

/*
 *	タイマの起動処理
 *
 *	タイマを初期化し，周期的なタイマ割込み要求を発生させる．
 */
extern void target_timer_initialize(intptr_t exinf);

/*
 *	タイマの停止処理
 *
 *	タイマの動作を停止させる．
 */
extern void target_timer_terminate(intptr_t exinf);

/*
 *	タイマの現在値の読出し
 */
Inline CLOCK
target_timer_get_current(void)
{
	return (CLOCK)(sil_reh_mem((void*)CMT0CNT));
}

/*
 *	タイマ割込み要求のチェック
 */
Inline bool_t
target_timer_probe_int(void)
{
    return (x_probe_int(INTNO_TIMER));
}

/*
 *	タイマ割込みハンドラ
 */
extern void target_timer_handler(void);

#ifdef TOPPERS_TARGET_SUPPORT_OVRHDR
/*
 *	タイマ値の内部表現とマイクロ秒単位との変換
 *	1μsecあたりのクロックカウント数
 *	現在の設定：
 *	  f(CPU)は160MHz
 *	  f(PBA)は40MHz(f(CPU)の4分周)
 *	  CMTのカウントソースは512分周
 */
// #define OVRTIMER_CLOCK ((40000000/DIV) / 1000000)
#define OVRTIMER_CLOCK(usec) ((40 * usec) / 512)
#define OVRTIMER_USEC(clock) ((512 * clock) / 40)

/*
 *	オーバランタイマの初期化処理
 *
 *	タイマを初期化する．
 */
extern void target_ovrtimer_initialize(intptr_t exinf);

/*
 *	オーバランタイマの終了処理
 *
 *	タイマの動作を終了する．
 */
extern void target_ovrtimer_terminate(intptr_t exinf);

/*
 *	オーバランタイマの起動処理
 *
 *	タイマを起動する．
 */
extern void target_ovrtimer_start(OVRTIM ovrtim);

/*
 *	オーバランタイマの停止処理
 *
 *	タイマの動作を停止させる．
 */
extern OVRTIM target_ovrtimer_stop(void);

/*
 *	オーバランタイマの現在値の読出し
 */
extern OVRTIM target_ovrtimer_get_current(void);

/*
 *	オーバランタイマ割込みハンドラ
 */
extern void target_ovrtimer_handler(void);

/*
 *  オーバランハンドラ起動ルーチン
 */
extern void	call_ovrhdr(void);

#endif /* TOPPERS_TARGET_SUPPORT_OVRHDR */

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_TARGET_TIMER_H */
