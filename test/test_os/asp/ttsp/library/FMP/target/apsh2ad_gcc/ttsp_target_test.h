/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
 *  Copyright (C) 2010-2012 by Industrial Technology Institute,
 *								Miyagi Prefectural Government, JAPAN
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
 *		テストプログラムのターゲット依存定義（APSH2AD用）
 */

#ifndef TTSP_TARGET_TEST_H
#define TTSP_TARGET_TEST_H

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
#define TTSP_INVALID_FUNC_ADDRESS  0x123451	/*  2バイト境界でない  */

/*
 *  スタック領域として不正な番地
 */
#define TTSP_INVALID_STK_ADDRESS  0x123456	/*  4バイト境界でない  */

/*
 *  固定長メモリープールの先頭番地として不正な番地
 */
#define TTSP_INVALID_MPF_ADDRESS  0x123456	/*  4バイト境界でない  */

/*
 *  スタックサイズとして不正なサイズ
 */
#define TTSP_INVALID_STACK_SIZE  0x123456	/*  4バイトの倍数でない  */

/*  
 *  ターゲット定義の拡張後の割込み優先度最小値
 */
#define TTSP_TMIN_INTPRI  TMIN_INTPRI

/*  
 *  割込み優先度定義
 */
#define TTSP_GE_TIMER_INTPRI  TMIN_INTPRI	/* タイマ割込みの割込み優先度より高い割込み優先度 */
#define TTSP_HIGH_INTPRI  (-5)	/* 割込み優先度高 */
#define TTSP_MID_INTPRI   (-4)	/* 割込み優先度中 */
#define TTSP_LOW_INTPRI   (-3)	/* 割込み優先度低 */

/*  
 *  割込み番号(正常値)
 */
							/*  CMI0_VECTORはカーネルが使用  */
#define TTSP_INTNO_A      (0x10000 | CMI1_VECTOR)	/* PE1割込み番号A */
#define TTSP_INTNO_B      (0x10000 | CMI2_VECTOR)	/* PE1割込み番号B */
#define TTSP_INTNO_C      (0x10000 | CMI3_VECTOR)	/* PE1割込み番号C */
													/* PE1割込み番号D */
#define TTSP_INTNO_D      (0x10000 | MTU0_TGI0V_VECTOR)

#define TTSP_INTNO_PE2_A  (0x20000 | MTU1_TGI1V_VECTOR)	/* PE2割込み番号A */
#define TTSP_INTNO_PE2_B  (0x20000 | MTU2_TGI2V_VECTOR)	/* PE2割込み番号B */
#define TTSP_INTNO_PE2_C  (0x20000 | MTU3_TGI3V_VECTOR)	/* PE2割込み番号C */
#define TTSP_INTNO_PE2_D  (0x20000 | MTU4_TGI4V_VECTOR)	/* PE2割込み番号D */

/*
 *  割込み番号(異常値)
 */
#define TTSP_INVALID_INTNO   0x10001	/* ターゲットでサポートしていない割込み番号 */
					/* 割込み要求ラインに対して割込み属性が設定されていない割込み番号 */
#define TTSP_NOT_SET_INTNO   	(0x10000 | 68)		/* 不正なPE1割込み番号 */
#define TTSP_INVALID_INTNO_PE2	(0x20000 | 68)		/* 不正なPE2割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE1  0x20001	/* PE1上位ビットが発行元プロセッサIDと異なる割込み番号 */
#define TTSP_NOT_SELF_INTNO_PE2  0x10001	/* PE2上位ビットが発行元プロセッサIDと異なる割込み番号 */

/*  グローバルIRC用割込み番号（このターゲットではサポート外）  */
#define TTSP_GLOBAL_IRC_INTNO_A  CMI1_VECTOR	/* グローバルIRC用割込み番号A */
#define TTSP_GLOBAL_IRC_INTNO_B  CMI2_VECTOR	/* グローバルIRC用割込み番号B */
#define TTSP_GLOBAL_IRC_INTNO_C  CMI3_VECTOR	/* グローバルIRC用割込み番号C */

/*
 *  割込みハンドラ番号(正常値)
 */
#define TTSP_INHNO_A       TTSP_INTNO_A			/* PE1割込みハンドラ番号A */
#define TTSP_INHNO_B       TTSP_INTNO_B			/* PE1割込みハンドラ番号B */
#define TTSP_INHNO_C       TTSP_INTNO_C			/* PE1割込みハンドラ番号C */
#define TTSP_INHNO_D       TTSP_INTNO_D			/* PE1割込みハンドラ番号D */

#define TTSP_INHNO_PE2_A   TTSP_INTNO_PE2_A		/* PE2割込みハンドラ番号A */
#define TTSP_INHNO_PE2_B   TTSP_INTNO_PE2_B		/* PE2割込みハンドラ番号B */
#define TTSP_INHNO_PE2_C   TTSP_INTNO_PE2_C		/* PE2割込みハンドラ番号C */
#define TTSP_INHNO_PE2_D   TTSP_INTNO_PE2_D		/* PE2割込みハンドラ番号D */

/*
 *  割込みハンドラ番号(異常値)
 */
#define TTSP_INVALID_INHNO  TTSP_INVALID_INTNO

/*
 *  CPU例外ハンドラ番号(正常値)
 */
										/* CPU例外発生元のコンテキストへreturn可能 */
#define TTSP_EXCNO_A 		0x10009		/* アドレスエラー例外 */
#define TTSP_EXCNO_B  		0x10004		/*  一般不当命令  */
										/* 本番号はCPU例外登録するが、*/
										/* 実際にCPU例外を発生させる */
										/* テストケースはない */
#define TTSP_EXCNO_PE2_A  	0x20009		/* PE2：アドレスエラー例外 */
#define TTSP_EXCNO_PE2_B  	0x20004		/* PE2：一般不当命令 */


/*
 *  CPU例外ハンドラ番号(異常値)
 */
#define TTSP_INVALID_EXCNO  0x10005


/*
 *  TTSP用の関数
 */

/*
 *  各同期処理のタイムアウト用変数
 *  [sil_dly_nse(TTSP_SIL_DLY_NSE_TIME) * TTSP_LOOP_COUNT]
 *  デフォルト: 1マイクロ秒 * 3,000,000回 = 3秒
 */
#define TTSP_SIL_DLY_NSE_TIME  1000
#define TTSP_LOOP_COUNT        3000000

/*
 *  ティック更新の停止
 */ 
extern void ttsp_target_stop_tick(void);

/*
 *  ティック更新の再開
 */ 
extern void ttsp_target_start_tick(void);

/*
 *  ティックの更新（特定プロセッサ）
 *  　グローバルタイマ方式なので、
 *  　prcidには時刻管理プロセッサしか指定しない。
 */
extern void ttsp_target_gain_tick_pe(ID prcid, bool_t wait_flg);

/*
 *  割込みの発生
 */
extern void ttsp_int_raise(INTNO intno);

/*
 *  割込み要求のクリア
 */
extern void ttsp_clear_int_req(INTNO intno);

/*
 *  CPU例外の発生
 */
extern void ttsp_cpuexc_raise(EXCNO excno);

/*
 *  CPU例外発生時のフック処理
 */
extern void ttsp_cpuexc_hook(EXCNO excno, void* p_excinf);


/*
 *  タスク初期化コンテキストブロックのデータ構造の違いを
 *  変換するマクロ
 *  
 *  　USE_TSKINICTXBを定義しているターゲットでは
 *  　以下の変換マクロが必要
 */
#define ttsp_target_get_stksz(p_tinib)	((p_tinib)->tskinictxb.stksz)
#define ttsp_target_get_stk(p_tinib)	(((p_tinib)->tskinictxb.stk_bottom) \
										 - ((p_tinib)->tskinictxb.stksz))


#endif /* TTSP_TARGET_TEST_H */
