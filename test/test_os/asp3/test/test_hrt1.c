/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2007-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_hrt1.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/*
 *		fch_hrtに関するテスト(1)
 *
 * 【テストの目的】
 *
 *  高分解能タイマのカウント値が逆行することがないことをテストする．
 *
 * 【テストの内容】
 *
 *  メインタスクでは，高分解能タイマのカウント値を繰り返し取得し，カウ
 *  ント値が小さくならないかをチェックする．それと並行して，周期ハンド
 *  ラを100μ秒周期で実行し，その中でも高分解能タイマのカウント値を取得
 *  して，カウント値が小さくならないかをチェックする．
 *
 *  なお，高分解能タイマのカウント値が最大値を超えて0に戻る状況は考慮し
 *  ていない．
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_hrt1.h"

#define	NO_LOOP		100000UL

HRTCNT	recent_hrtcnt;
char	*recent_hrtcnt_pos;
uint_t	cyclic_count;

void
cyclic_handler(intptr_t exinf)
{
	HRTCNT		hrtcnt, prev_hrtcnt;
	char		*prev_hrtcnt_pos;

	(void) loc_cpu();
	hrtcnt = fch_hrt();
	prev_hrtcnt = recent_hrtcnt;
	prev_hrtcnt_pos = recent_hrtcnt_pos;
	recent_hrtcnt = hrtcnt;
	recent_hrtcnt_pos = "CYC";
	(void) unl_cpu();

	if (prev_hrtcnt > hrtcnt) {
		syslog(LOG_NOTICE,
				"system performance time goes back: %d(%s) %d(CYC)",
								prev_hrtcnt, prev_hrtcnt_pos, hrtcnt);
	}
	cyclic_count += 1U;
}

void
main_task(intptr_t exinf)
{
	HRTCNT		hrtcnt, prev_hrtcnt;
	char		*prev_hrtcnt_pos;
	ulong_t		i;

	cyclic_count = 0U;
	recent_hrtcnt = fch_hrt();
	syslog(LOG_NOTICE, "system performance time test starts.");

	for (i = 0; i < NO_LOOP; i++) {
		(void) loc_cpu();
		hrtcnt = fch_hrt();
		prev_hrtcnt = recent_hrtcnt;
		prev_hrtcnt_pos = recent_hrtcnt_pos;
		recent_hrtcnt = hrtcnt;
		recent_hrtcnt_pos = "TSK";
		(void) unl_cpu();

		if (prev_hrtcnt > hrtcnt) {
			syslog(LOG_NOTICE,
				"system performance time goes back: %d(%s) %d(TSK)",
								prev_hrtcnt, prev_hrtcnt_pos, hrtcnt);
		}
	}

	syslog(LOG_NOTICE, "system performance time test finishes.");
	syslog(LOG_NOTICE, "number of cyclic handler execution: %d", cyclic_count);
	check_finish(0);
}
