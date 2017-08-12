/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2006-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: tHistogram.c 509 2016-01-12 06:06:14Z ertl-hiro $
 */

/*
 *		実行時間分布集計サービス
 */

#include "tHistogram_tecsgen.h"
#define syslog_write	cSysLog_write
#include <t_syslog.h>
#include <log_output.h>

/* 
 *  実行時間分布計測の初期化（受け口関数）
 */
void
eHistogram_initialize(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	uint_t	i;

	for (i = 0; i <= ATTR_maxTime; i++) {
		VAR_histarea[i] = 0U;
	}
	VAR_over = 0U;
	VAR_under = 0U;
}

/*
 *  実行時間計測の開始（受け口関数）
 */
void
eHistogram_beginMeasure(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);

	HIST_BM_HOOK();
	HIST_GET_TIM(&VAR_begin_time);
}

/*
 *  実行時間計測の終了（受け口関数）
 */
void
eHistogram_endMeasure(CELLIDX idx)
{
	CELLCB		*p_cellcb = GET_CELLCB(idx);
	histtim_t	end_time, timediff;
	uint_t		val;

	HIST_GET_TIM(&end_time);

	timediff = end_time - VAR_begin_time;
#ifdef HISTTIM_CYCLE
	if (end_time < VAR_begin_time) {
		timediff += HISTTIM_CYCLE;
	}
#endif /* HISTTIM_CYCLE */
	val = HIST_CONV_TIM(timediff);
	if (val <= ATTR_maxTime) {
		VAR_histarea[val]++;
	}
	else if (val <= ((uint_t) INT_MAX)) {
		VAR_over++;
	}
	else {
		VAR_under++;
	}
}

/*
 *  実行時間分布計測の表示（受け口関数）
 */
void
eHistogram_print(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	uint_t	i;

	for (i = 0; i <= ATTR_maxTime; i++) {
		if (VAR_histarea[i] > 0) {
			syslog_2(LOG_NOTICE, "%d : %d", i, VAR_histarea[i]);
		}
	}
	if (VAR_over > 0) {
		syslog_2(LOG_NOTICE, "> %d : %d", ATTR_maxTime, VAR_over);
	}
	if (VAR_under > 0) {
		syslog_1(LOG_NOTICE, "> INT_MAX : %d", VAR_under);
	}
}
