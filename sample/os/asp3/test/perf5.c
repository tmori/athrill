/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2015,2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: perf5.c 544 2016-01-16 05:34:11Z ertl-hiro $
 */

/*
 *		カーネル性能評価プログラム(5)
 *
 *  タイムイベント処理にかかる時間を計測するプログラム．
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/syslog.h"
#include "syssvc/test_svc.h"
#include "syssvc/histogram.h"
#include "kernel_cfg.h"
#include "perf5.h"

/*
 *  計測回数と実行時間分布を記録する最大時間
 */
#define NO_MEASURE	10000U			/* 計測回数 */

/*
 *  アラームハンドラ
 */
void alarm_handler(intptr_t exinf)
{
	/*
	 *  アラームハンドラは実行されないはず
	 */
	syslog_1(LOG_NOTICE, "alarm handler %d executed.", exinf);
}

/*
 *  計測用のアラームハンドラのリスト
 */
static ID alarm1_list[30] = {
	ALM1, ALM2, ALM3, ALM4, ALM5, ALM6, ALM7, ALM8, ALM9, ALM10,
	ALM11, ALM12, ALM13, ALM14, ALM15, ALM16, ALM17, ALM18, ALM19, ALM20,
	ALM21, ALM22, ALM23, ALM24, ALM25, ALM26, ALM27, ALM28, ALM29, ALM30
};

static ID alarm2_list[30] = {
	ALM31, ALM32, ALM33, ALM34, ALM35, ALM36, ALM37, ALM38, ALM39, ALM40,
	ALM41, ALM42, ALM43, ALM44, ALM45, ALM46, ALM47, ALM48, ALM49, ALM50,
	ALM51, ALM52, ALM53, ALM54, ALM55, ALM56, ALM57, ALM58, ALM59, ALM60
};

static ID alarm3_list[30] = {
	ALM61, ALM62, ALM63, ALM64, ALM65, ALM66, ALM67, ALM68, ALM69, ALM70,
	ALM71, ALM72, ALM73, ALM74, ALM75, ALM76, ALM77, ALM78, ALM79, ALM80,
	ALM81, ALM82, ALM83, ALM84, ALM85, ALM86, ALM87, ALM88, ALM89, ALM90
};

/*
 *  メインタスク
 */
void main_task(intptr_t exinf)
{
	uint_t	i, j;

	syslog_0(LOG_NOTICE, "Performance evaluation program (5)");
	init_hist(1);
	init_hist(2);
	init_hist(3);
	init_hist(4);
	init_hist(5);
	init_hist(6);

	/*
	 *  繰り返し計測
	 */
	for (j = 0; j < NO_MEASURE / 10; j++) {
		/*
		 *  アラームハンドラ0短い時間で動作開始
		 *
		 *  性能評価中に高分解能タイマが再設定されるのを避けるため．
		 */
		sta_alm(ALM0, ALM_RELTIM0);

		/*
		 *  30個のアラームハンドラを長い時間で動作開始
		 */
		begin_measure(1);
		for (i = 0; i < 30; i++) {
			sta_alm(alarm1_list[i], ALM_RELTIM1);
		}
		end_measure(1);

		/*
		 *  30個のアラームハンドラを中間の時間で動作開始
		 */
		begin_measure(2);
		for (i = 0; i < 30; i++) {
			sta_alm(alarm2_list[i], ALM_RELTIM2);
		}
		end_measure(2);

		/*
		 *  30個のアラームハンドラを短い時間で動作開始
		 */
		begin_measure(3);
		for (i = 0; i < 30; i++) {
			sta_alm(alarm3_list[i], ALM_RELTIM3);
		}
		end_measure(3);

		/*
		 *  短い時間で動作開始した30個のアラームハンドラを動作停止
		 */
		begin_measure(6);
		for (i = 0; i < 30; i++) {
			stp_alm(alarm3_list[i]);
		}
		end_measure(6);

		/*
		 *  中間の時間で動作開始した30個のアラームハンドラを動作停止
		 */
		begin_measure(5);
		for (i = 0; i < 30; i++) {
			stp_alm(alarm2_list[29 - i]);		/* 逆順で動作停止 */
		}
		end_measure(5);

		/*
		 *  長い時間で動作開始した30個のアラームハンドラを動作停止
		 */
		begin_measure(4);
		for (i = 0; i < 30; i++) {
			stp_alm(alarm1_list[29 - i]);		/* 逆順で動作停止 */
		}
		end_measure(4);
	}

	/*
	 *  測定結果の出力
	 */
	syslog_0(LOG_NOTICE, "Execution times of 30 short sta_alm");
	print_hist(1);

	syslog_0(LOG_NOTICE, "Execution times of 30 medium sta_alm");
	print_hist(2);

	syslog_0(LOG_NOTICE, "Execution times of 30 long sta_alm");
	print_hist(3);

	syslog_0(LOG_NOTICE, "Execution times of 30 short stp_alm");
	print_hist(4);

	syslog_0(LOG_NOTICE, "Execution times of 30 medium stp_alm");
	print_hist(5);

	syslog_0(LOG_NOTICE, "Execution times of 30 long stp_alm");
	print_hist(6);
	check_finish(0);
}
