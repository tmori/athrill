/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2012 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
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
 *		テストプログラムのチップ依存定義（NaviEngine用）
 */

#include "ttsp_chip_timer.h"

#ifndef TTSP_TARGET_TEST_H
#define TTSP_TARGET_TEST_H


/*
 *  CPU例外を発生させる命令
 */
__svc(0) void _svc_dummy(void);

#define RAISE_CPU_EXCEPTION _svc_dummy()

/*
 *  性能評価プログラムのための定義
 */

/*
 *  カーネル内で宣言している関数の外部宣言
 */
extern void perf_timer_get(uint32_t *p_time);
extern uint32_t perf_timer_conv_tim(uint32_t time);

/*
 *  性能評価用のマクロ定義
 */
#define HISTTIM  uint32_t
#define HIST_GET_TIM(p_time)  perf_timer_get(p_time)
#define HIST_CONV_TIM(time)   perf_timer_conv_tim(time)

/*
 *  チップ依存モジュール（MPCORE用）
 */
#include "arm_gcc/mpcore/chip_test.h"

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
#define TTSP_HIGH_INTPRI  -5				/* 割込み優先度高 */
#define TTSP_MID_INTPRI   -4				/* 割込み優先度中 */
#define TTSP_LOW_INTPRI   -3				/* 割込み優先度低 */

/*  
 *  割込み番号(正常値)
 */
#define TTSP_INTNO_A      0x00021	/* PE1割込み番号A */
#define TTSP_INTNO_B      0x00022	/* PE1割込み番号B */
#define TTSP_INTNO_C      0x00023	/* PE1割込み番号C */
#define TTSP_INTNO_D      0x00024	/* PE1割込み番号D */
#define TTSP_INTNO_E      0x00025	/* PE1割込み番号E */
#define TTSP_INTNO_F      0x00026	/* PE1割込み番号F */

#define TTSP_INTNO_PE2_A  0x00027	/* PE2割込み番号A */
#define TTSP_INTNO_PE2_B  0x00028	/* PE2割込み番号B */
#define TTSP_INTNO_PE2_C  0x00029	/* PE2割込み番号C */
#define TTSP_INTNO_PE2_D  0x0002a	/* PE2割込み番号D */
#define TTSP_INTNO_PE2_E  0x0002b	/* PE2割込み番号E */
#define TTSP_INTNO_PE2_F  0x0002c	/* PE2割込み番号F */

#define TTSP_INTNO_PE3_A  0x0002e	/* PE3割込み番号A */
#define TTSP_INTNO_PE3_B  0x0002f	/* PE3割込み番号B */
#define TTSP_INTNO_PE3_C  0x00030	/* PE3割込み番号C */
#define TTSP_INTNO_PE3_D  0x00031	/* PE3割込み番号D */
#define TTSP_INTNO_PE3_E  0x00032	/* PE3割込み番号E */
#define TTSP_INTNO_PE3_F  0x00033	/* PE3割込み番号F */

#define TTSP_INTNO_PE4_A  0x00034	/* PE4割込み番号A */
#define TTSP_INTNO_PE4_B  0x00035	/* PE4割込み番号B */
#define TTSP_INTNO_PE4_C  0x00036	/* PE4割込み番号C */
#define TTSP_INTNO_PE4_D  0x00037	/* PE4割込み番号D */
#define TTSP_INTNO_PE4_E  0x00038	/* PE4割込み番号E */
#define TTSP_INTNO_PE4_F  0x00039	/* PE4割込み番号F */

#define TTSP_GLOBAL_IRC_INTNO_A  0x00009	/* グローバルIRC用割込み番号A */
#define TTSP_GLOBAL_IRC_INTNO_B  0x0000A	/* グローバルIRC用割込み番号B */
#define TTSP_GLOBAL_IRC_INTNO_C  0x0000B	/* グローバルIRC用割込み番号C */
#define TTSP_GLOBAL_IRC_INTNO_D  0x0000C	/* グローバルIRC用割込み番号D */
#define TTSP_GLOBAL_IRC_INTNO_E  0x0000D	/* グローバルIRC用割込み番号E */
#define TTSP_GLOBAL_IRC_INTNO_F  0x0000E	/* グローバルIRC用割込み番号F */

/*
 *  割込み番号(異常値)
 */
#define TTSP_INVALID_INTNO       0x10080	/* PE1ターゲットでサポートしていない割込み番号 */
#define TTSP_INVALID_INTNO_PE2   0x20080	/* PE2ターゲットでサポートしていない割込み番号 */
#define TTSP_INVALID_INTNO_PE3   0x30080	/* PE3ターゲットでサポートしていない割込み番号 */
#define TTSP_INVALID_INTNO_PE4   0x40080	/* PE4ターゲットでサポートしていない割込み番号 */
#define TTSP_NOT_SET_INTNO       0x10010	/* PE1割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_NOT_SET_INTNO_PE2   0x20010	/* PE2割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_NOT_SET_INTNO_PE3   0x30010	/* PE3割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_NOT_SET_INTNO_PE4   0x40010	/* PE4割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE1  0x20001	/* PE1上位ビットが発行元プロセッサIDと異なる割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE2  0x10001	/* PE2上位ビットが発行元プロセッサIDと異なる割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE3  0x10001	/* PE3上位ビットが発行元プロセッサIDと異なる割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE4  0x10001	/* PE4上位ビットが発行元プロセッサIDと異なる割込み番号 */

/*
 *  割込みハンドラ番号(正常値)
 */
#define TTSP_INHNO_A       0x10021		/* PE1割込みハンドラ番号A */
#define TTSP_INHNO_B       0x10022		/* PE1割込みハンドラ番号B */
#define TTSP_INHNO_C       0x10023		/* PE1割込みハンドラ番号C */

#define TTSP_INHNO_PE2_A   0x20027		/* PE2割込みハンドラ番号A */
#define TTSP_INHNO_PE2_B   0x20028		/* PE2割込みハンドラ番号B */
#define TTSP_INHNO_PE2_C   0x20029		/* PE2割込みハンドラ番号C */

#define TTSP_INHNO_PE3_A   0x3002e		/* PE3割込みハンドラ番号A */
#define TTSP_INHNO_PE3_B   0x3002f		/* PE3割込みハンドラ番号B */
#define TTSP_INHNO_PE3_C   0x30030		/* PE3割込みハンドラ番号C */

#define TTSP_INHNO_PE4_A   0x40034		/* PE4割込みハンドラ番号A */
#define TTSP_INHNO_PE4_B   0x40035		/* PE4割込みハンドラ番号B */
#define TTSP_INHNO_PE4_C   0x40036		/* PE4割込みハンドラ番号C */

/*
 *  割込みハンドラ番号(異常値)
 */
#define TTSP_INVALID_INHNO  -1
#define TTSP_GLOBAL_IRC_INHNO_A  TTSP_GLOBAL_IRC_INTNO_A	/* グローバルIRC用割込みハンドラ番号A */
#define TTSP_GLOBAL_IRC_INHNO_B  TTSP_GLOBAL_IRC_INTNO_B	/* グローバルIRC用割込みハンドラ番号B */
#define TTSP_GLOBAL_IRC_INHNO_C  TTSP_GLOBAL_IRC_INTNO_C	/* グローバルIRC用割込みハンドラ番号C */

/*
 *  CPU例外ハンドラ番号(正常値)
 */
#define TTSP_EXCNO_A      (0x10000|EXCH_NO_SWI)		/* CPU例外発生元のコンテキストへreturn可能(SWI) */
#define TTSP_EXCNO_B      (0x10000|EXCH_NO_UNDEF)	/* 本番号でCPU例外を発生させるテストケースはない(未定義命令) */
#define TTSP_EXCNO_PE2_A  (0x20000|EXCH_NO_SWI)		/* PE2SWI */
#define TTSP_EXCNO_PE2_B  (0x20000|EXCH_NO_UNDEF)	/* PE2未定義命令 */
#define TTSP_EXCNO_PE3_A  (0x30000|EXCH_NO_SWI)		/* PE3SWI */
#define TTSP_EXCNO_PE3_B  (0x30000|EXCH_NO_UNDEF)	/* PE3未定義命令 */
#define TTSP_EXCNO_PE4_A  (0x40000|EXCH_NO_SWI)		/* PE4SWI */
#define TTSP_EXCNO_PE4_B  (0x40000|EXCH_NO_UNDEF)	/* PE4未定義命令 */

/*
 *  CPU例外ハンドラ番号(異常値)
 */
#define TTSP_INVALID_EXCNO  0


/*
 *  テスト用の関数
 */

/*
 *  各同期処理のタイムアウト用変数
 *  [sil_dly_nse(TTSP_SIL_DLY_NSE_TIME) * TTSP_LOOP_COUNT]
 *  デフォルト: 1マイクロ秒 * 3,000,000回 = 3秒
 */
#define TTSP_SIL_DLY_NSE_TIME  1000
#define TTSP_LOOP_COUNT        3000000

/*
 *  割込みの発生
 */
extern void ttsp_int_raise(INTNO intno);

/*
 *  CPU例外の発生
 */
extern void ttsp_cpuexc_raise(EXCNO excno);

/*
 *  CPU例外発生時のフック処理
 */
extern void ttsp_cpuexc_hook(EXCNO excno, void* p_excinf);

/*
 *  割込み要求のクリア
 */
extern void ttsp_clear_int_req(INTNO intno);

#endif /* TTSP_TARGET_TEST_H */
