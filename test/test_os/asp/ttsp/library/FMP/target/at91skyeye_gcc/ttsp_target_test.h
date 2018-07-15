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
 *  性能評価プログラムのための定義
 */

/*
 *  カーネル内で宣言している関数の外部宣言
 */
extern void cycle_counter_get(uint32_t *p_time);

/*
 *  性能評価用のマクロ定義
 */
#define HISTTIM  uint32_t
#define HIST_GET_TIM(p_time)  cycle_counter_get(p_time)

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
#define TTSP_INTNO_A      0x10009	/* PE1割込み番号A */
#define TTSP_INTNO_B      0x1000A	/* PE1割込み番号B */
#define TTSP_INTNO_C      0x1000B	/* PE1割込み番号C */
#define TTSP_INTNO_D      0x1000C	/* PE1割込み番号D */
#define TTSP_INTNO_E      0x1000D	/* PE1割込み番号E */
#define TTSP_INTNO_F      0x1000E	/* PE1割込み番号F */

#define TTSP_INTNO_PE2_A  0x20009	/* PE2割込み番号A */
#define TTSP_INTNO_PE2_B  0x2000A	/* PE2割込み番号B */
#define TTSP_INTNO_PE2_C  0x2000B	/* PE2割込み番号C */
#define TTSP_INTNO_PE2_D  0x2000C	/* PE2割込み番号D */
#define TTSP_INTNO_PE2_E  0x2000D	/* PE2割込み番号E */
#define TTSP_INTNO_PE2_F  0x2000E	/* PE2割込み番号F */

#define TTSP_INTNO_PE3_A  0x30009	/* PE3割込み番号A */
#define TTSP_INTNO_PE3_B  0x3000A	/* PE3割込み番号B */
#define TTSP_INTNO_PE3_C  0x3000B	/* PE3割込み番号C */
#define TTSP_INTNO_PE3_D  0x3000C	/* PE3割込み番号D */
#define TTSP_INTNO_PE3_E  0x3000D	/* PE3割込み番号E */
#define TTSP_INTNO_PE3_F  0x3000E	/* PE3割込み番号F */

#define TTSP_INTNO_PE4_A  0x40009	/* PE4割込み番号A */
#define TTSP_INTNO_PE4_B  0x4000A	/* PE4割込み番号B */
#define TTSP_INTNO_PE4_C  0x4000B	/* PE4割込み番号C */
#define TTSP_INTNO_PE4_D  0x4000C	/* PE4割込み番号D */
#define TTSP_INTNO_PE4_E  0x4000D	/* PE4割込み番号E */
#define TTSP_INTNO_PE4_F  0x4000E	/* PE4割込み番号F */

/*
 *  割込み番号(異常値)
 */
#define TTSP_INVALID_INTNO       0x10032	/* PE1ターゲットでサポートしていない割込み番号 */
#define TTSP_INVALID_INTNO_PE2   0x20032	/* PE2ターゲットでサポートしていない割込み番号 */
#define TTSP_INVALID_INTNO_PE3   0x30032	/* PE3ターゲットでサポートしていない割込み番号 */
#define TTSP_INVALID_INTNO_PE4   0x40032	/* PE4ターゲットでサポートしていない割込み番号 */
#define TTSP_NOT_SET_INTNO       0x10001	/* PE1割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_NOT_SET_INTNO_PE2   0x20001	/* PE2割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_NOT_SET_INTNO_PE3   0x30001	/* PE3割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_NOT_SET_INTNO_PE4   0x40001	/* PE4割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_GLOBAL_IRC_INTNO_A  0x00009	/* グローバルIRC用割込み番号A(at91skyeye_gccではサポート外) */
#define TTSP_GLOBAL_IRC_INTNO_B  0x0000A	/* グローバルIRC用割込み番号B(at91skyeye_gccではサポート外) */
#define TTSP_GLOBAL_IRC_INTNO_C  0x0000B	/* グローバルIRC用割込み番号C(at91skyeye_gccではサポート外) */
#define TTSP_GLOBAL_IRC_INTNO_D  0x0000C	/* グローバルIRC用割込み番号D(at91skyeye_gccではサポート外) */
#define TTSP_GLOBAL_IRC_INTNO_E  0x0000D	/* グローバルIRC用割込み番号E(at91skyeye_gccではサポート外) */
#define TTSP_GLOBAL_IRC_INTNO_F  0x0000E	/* グローバルIRC用割込み番号F(at91skyeye_gccではサポート外) */
#define TTSP_NOT_SELF_INTNO_PE1  0x20001	/* PE1上位ビットが発行元プロセッサIDと異なる割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE2  0x10001	/* PE2上位ビットが発行元プロセッサIDと異なる割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE3  0x10001	/* PE3上位ビットが発行元プロセッサIDと異なる割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE4  0x10001	/* PE4上位ビットが発行元プロセッサIDと異なる割込み番号 */

/*
 *  割込みハンドラ番号(正常値)
 */
#define TTSP_INHNO_A       TTSP_INTNO_A			/* PE1割込みハンドラ番号A */
#define TTSP_INHNO_B       TTSP_INTNO_B			/* PE1割込みハンドラ番号B */
#define TTSP_INHNO_C       TTSP_INTNO_C			/* PE1割込みハンドラ番号C */

#define TTSP_INHNO_PE2_A   TTSP_INTNO_PE2_A		/* PE2割込みハンドラ番号A */
#define TTSP_INHNO_PE2_B   TTSP_INTNO_PE2_B		/* PE2割込みハンドラ番号B */
#define TTSP_INHNO_PE2_C   TTSP_INTNO_PE2_C		/* PE2割込みハンドラ番号C */

#define TTSP_INHNO_PE3_A   TTSP_INTNO_PE3_A		/* PE3割込みハンドラ番号A */
#define TTSP_INHNO_PE3_B   TTSP_INTNO_PE3_B		/* PE3割込みハンドラ番号B */
#define TTSP_INHNO_PE3_C   TTSP_INTNO_PE3_C		/* PE3割込みハンドラ番号C */

#define TTSP_INHNO_PE4_A   TTSP_INTNO_PE4_A		/* PE4割込みハンドラ番号A */
#define TTSP_INHNO_PE4_B   TTSP_INTNO_PE4_B		/* PE4割込みハンドラ番号B */
#define TTSP_INHNO_PE4_C   TTSP_INTNO_PE4_C		/* PE4割込みハンドラ番号C */

/*
 *  割込みハンドラ番号(異常値)
 */
#define TTSP_INVALID_INHNO  -1
#define TTSP_GLOBAL_IRC_INHNO_A  TTSP_GLOBAL_IRC_INTNO_A	/* グローバルIRC用割込みハンドラ番号A(at91skyeye_gccではサポート外) */
#define TTSP_GLOBAL_IRC_INHNO_B  TTSP_GLOBAL_IRC_INTNO_B	/* グローバルIRC用割込みハンドラ番号B(at91skyeye_gccではサポート外) */
#define TTSP_GLOBAL_IRC_INHNO_C  TTSP_GLOBAL_IRC_INTNO_C	/* グローバルIRC用割込みハンドラ番号C(at91skyeye_gccではサポート外) */

/*
 *  CPU例外ハンドラ番号(正常値)
 */
#define TTSP_EXCNO_A      0x10001	/* CPU例外発生元のコンテキストへreturn可能(未定義命令) */
#define TTSP_EXCNO_B      0x10002	/* 本番号でCPU例外を発生させるテストケースはない(SWI) */
#define TTSP_EXCNO_PE2_A  0x20001	/* PE2未定義命令 */
#define TTSP_EXCNO_PE2_B  0x20002	/* PE2SWI */
#define TTSP_EXCNO_PE3_A  0x30001	/* PE3未定義命令 */
#define TTSP_EXCNO_PE3_B  0x30002	/* PE3SWI */
#define TTSP_EXCNO_PE4_A  0x40001	/* PE4未定義命令 */
#define TTSP_EXCNO_PE4_B  0x40002	/* PE4SWI */

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
 *  テスト用の関数
 */

/*
 *  ティック更新の停止（全プロセッサ）
 */ 
extern void ttsp_target_stop_tick(void);

/*
 *  ティック更新の停止（特定プロセッサ）
 */ 
extern void ttsp_target_stop_tick_pe(ID prcid);

/*
 *  ティック更新の再開（全プロセッサ）
 */ 
extern void ttsp_target_start_tick(void);

/*
 *  ティック更新の再開（特定プロセッサ）
 */ 
extern void ttsp_target_start_tick_pe(ID prcid);

/*
 *  ティックの更新（全プロセッサ）
 */
extern void ttsp_target_gain_tick(void);

/*
 *  ティックの更新（特定プロセッサ）
 */
extern void ttsp_target_gain_tick_pe(ID prcid, bool_t wait_flg);

/*
 *  タイマハンドラ用関数
 */
extern bool_t ttsp_timer_handler_begin_hook(void);
extern void ttsp_timer_handler_end_hook(void);

/*
 *  タイマハンドラ呼び出し完了確認関数
 */
extern void ttsp_check_timer_handler(void);


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
