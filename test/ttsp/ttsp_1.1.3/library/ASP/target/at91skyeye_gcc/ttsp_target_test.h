/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2012 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2012 by FUJISOFT INCORPORATED
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
 *  $Id: ttsp_target_test.h 2 2012-05-09 02:23:52Z nces-shigihara $
 */

/*
 *		テストプログラムのチップ依存定義（AT91SKYEYE用）
 */

#ifndef TTSP_TARGET_TEST_H
#define TTSP_TARGET_TEST_H

/*
 *  CPU例外を発生させる命令
 */
#if defined(TOPPERS_ENABLE_GCOV_PART) || defined(TOPPERS_ENABLE_GCOV_FULL)
#define RAISE_CPU_EXCEPTION Asm(".long 0x16000010");
#else
#define RAISE_CPU_EXCEPTION Asm(".long 0x06000010");
#endif /* defined(TOPPERS_ENABLE_GCOV_PART) || defined(TOPPERS_ENABLE_GCOV_FULL) */


/*
 *  TTSP用の定義
 */

/*
 *  タスクのスタックサイズ
 */
#define TTSP_TASK_STACK_SIZE  2048

/*
 *  非タスクコンテキストのスタックサイズ
 *  (DEF_ICSのテストでのみ使用する)
 */
#define TTSP_NON_TASK_STACK_SIZE  2048

/*
 *  関数の先頭番地として不正な番地
 */
#define TTSP_INVALID_FUNC_ADDRESS  0x123456

/*
 *  スタック領域として不正な番地
 */
#define TTSP_INVALID_STK_ADDRESS  0x123456

/*
 *  固定長メモリープールの先頭番地として不正な番地
 */
#define TTSP_INVALID_MPF_ADDRESS  0x123456

/*
 *  スタックサイズとして不正なサイズ
 */
#define TTSP_INVALID_STACK_SIZE  0x01

/*  
 *  ターゲット定義の拡張後の割込み優先度最小値
 */
#define TTSP_TMIN_INTPRI  TMIN_INTPRI

/*  
 *  割込み優先度定義
 */
#define TTSP_GE_TIMER_INTPRI  TMIN_INTPRI	/* タイマ割込みの割込み優先度より高い割込み優先度 */
#define TTSP_HIGH_INTPRI      -5			/* 割込み優先度高 */
#define TTSP_MID_INTPRI       -4			/* 割込み優先度中 */
#define TTSP_LOW_INTPRI       -3			/* 割込み優先度低 */

/*  
 *  割込み番号(正常値)
 */
#define TTSP_INTNO_A    9	/* 割込み番号A */
#define TTSP_INTNO_B   10	/* 割込み番号B */
#define TTSP_INTNO_C   11	/* 割込み番号C */
#define TTSP_INTNO_D   12	/* 割込み番号D */
#define TTSP_INTNO_E   13	/* 割込み番号E */
#define TTSP_INTNO_F   14	/* 割込み番号F */

/*
 *  割込み番号(異常値)
 */
#define TTSP_INVALID_INTNO  50	/* ターゲットでサポートしていない割込み番号 */
#define TTSP_NOT_SET_INTNO   1	/* 割込み要求ラインに対して割込み属性が設定されていない割込み番号 */

/*
 *  割込みハンドラ番号(正常値)
 */
#define TTSP_INHNO_A   TTSP_INTNO_A		/* 割込みハンドラ番号A */
#define TTSP_INHNO_B   TTSP_INTNO_B		/* 割込みハンドラ番号B */
#define TTSP_INHNO_C   TTSP_INTNO_C		/* 割込みハンドラ番号C */

/*
 *  割込みハンドラ番号(異常値)
 */
#define TTSP_INVALID_INHNO  50

/*
 *  CPU例外ハンドラ番号(正常値)
 */
#define TTSP_EXCNO_A  1		/* CPU例外発生元のコンテキストへreturn可能(未定義命令) */
#define TTSP_EXCNO_B  2		/* 本番号でCPU例外を発生させるテストケースはない(SWI) */

/*
 *  CPU例外ハンドラ番号(異常値)
 */
#define TTSP_INVALID_EXCNO  0

/*
 *  タイムアウト用変数
 *  [sil_dly_nse(TTSP_SIL_DLY_NSE_TIME) * TTSP_LOOP_COUNT]
 *  デフォルト: 1マイクロ秒 * 50,000回 = 50ミリ秒
 *  ※シミュレータのため体感的に適度な設定とする
 */
#define TTSP_SIL_DLY_NSE_TIME  1000
#define TTSP_LOOP_COUNT        50000


/*
 *  TTSP用の関数
 */

/*
 *  ティック更新の停止
 */ 
extern void ttsp_target_stop_tick(void);

/*
 *  ティック更新の再開
 */ 
extern void ttsp_target_start_tick(void);

/*
 *  ティックの更新
 */
extern void ttsp_target_gain_tick(void);

/*
 *  割込みの発生
 */
extern void ttsp_int_raise(INTNO intno);

/*
 *  CPU例外の発生
 */
extern void ttsp_cpuexc_raise(EXCNO excno);

/*
 *  CPU例外発生時のフック処理(SkyEyeでは不要)
 */
extern void ttsp_cpuexc_hook(EXCNO excno, void* p_excinf);

/*
 *  割込み要求のクリア(SkyEyeでは不要)
 */
extern void ttsp_clear_int_req(INTNO intno);

#endif /* TTSP_TARGET_TEST_H */
