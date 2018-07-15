/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
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
 *  $Id: ttsp_target_test.c 2 2012-05-09 02:23:52Z nces-shigihara $
 */

#include "kernel_impl.h"
#include "time_event.h"
#include "pcb.h"
#include <sil.h>
#include "ttsp_target_test.h"

/*
 *  ttsp_target_timer.cリンク確認
 */
extern uint_t include_ttsp_target_timer_to_KERNEL_COBJS;
void
ttsp_target_timer_not_be_linked(void)
{
	include_ttsp_target_timer_to_KERNEL_COBJS = 1;
}

/*
 *  ティック用タイマディスエーブル変数
 */
static volatile bool_t target_timer_disable[TNUM_PRCID];

/*
 *  ティック用タイマワンショット実行
 */
static volatile bool_t target_timer_oneshot[TNUM_PRCID];

/*
 *  タイマハンドラ開始時に呼び出すフック
 *  (falseでsignal_time()を呼び出さない)
 */
bool_t
ttsp_timer_handler_begin_hook(void)
{
	return !target_timer_disable[x_prc_index()];
}

/*
 *  タイマハンドラ終了時に呼び出すフック
 */
void
ttsp_timer_handler_end_hook(void)
{
	uint_t prc_index = x_prc_index();
	if (target_timer_oneshot[prc_index]) {
		target_timer_disable[prc_index] = true;
		target_timer_oneshot[prc_index] = false;
	}
}

/*
 *  タイマハンドラ呼び出し完了確認関数
 */
void
ttsp_check_timer_handler(void)
{
	int_t		i;
	ID			pid;
	ulong_t		timeout;

	sil_get_pid(&pid);

	/* 前回のタイマハンドラが完了しているかチェック */
	for(i = 0; i < TNUM_PRCID; i++) {
		timeout = 0;
		while (target_timer_disable[i] == false) {
			timeout++;
			if (timeout > TTSP_LOOP_COUNT) {
				syslog_2(LOG_ERROR, "## PE %d : ttsp_check_timer_handler[PE:%d][check] caused a timeout.", pid, i + 1);
				ext_ker();
			}
			sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
		};
	}
}

/*
 *  ティック更新の停止(全プロセッサ)
 */
void
ttsp_target_stop_tick(void)
{
	uint_t i;

	for(i = 1; i <= TNUM_PRCID; i++) {
		ttsp_target_stop_tick_pe(i);
	}
}

/*
 *  ティック更新の停止(特定プロセッサ)
 */
void
ttsp_target_stop_tick_pe(ID prcid)
{
	uint_t prc_index = prcid - 1;
	target_timer_disable[prc_index] = true;
}

/*
 *  ティック更新の再開（全プロセッサ）
 */
void
ttsp_target_start_tick(void)
{
	uint_t i;

	for(i = 1; i <= TNUM_PRCID; i++) {
		ttsp_target_start_tick_pe(i);
	}
}

/*
 *  ティック更新の再開（特定プロセッサ）
 */
void
ttsp_target_start_tick_pe(ID prcid)
{
	uint_t prc_index = prcid - 1;
	target_timer_disable[prc_index] = false;
}

/*
 *  ティックの更新（特定プロセッサ）
 */
void
ttsp_target_gain_tick_pe(ID prcid, bool_t wait_flg)
{
	int			prc_index = prcid - 1;
	ID			pid;
	ulong_t		timeout;

	sil_get_pid(&pid);

	/*
	 *  前回の更新が終わっていることを確認
	 *  自PEに対してはチェックする必要はないが，チェックしても問題ないため，
	 *  チェックする． 
	 */
	timeout = 0;
	while (target_timer_oneshot[prc_index] == true) {
		timeout++;
		if (timeout > TTSP_LOOP_COUNT) {
			syslog_2(LOG_ERROR, "## PE %d : ttsp_target_gain_tick_pe(%d)[check] caused a timeout.", pid, prcid);
			ext_ker();
		}
		sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
	};

	target_timer_oneshot[prc_index] = true;
	target_timer_disable[prc_index] = false;

	/* wait_flgがtrueの場合は，ハンドラが終了するまで待ち合わせる */
	if (wait_flg == true) {
		timeout = 0;
		while (target_timer_oneshot[prc_index] == true) {
			timeout++;
			if (timeout > TTSP_LOOP_COUNT) {
				syslog_2(LOG_ERROR, "## PE %d : ttsp_target_gain_tick_pe(%d)[wait] caused a timeout.", pid, prcid);
				ext_ker();
			}
			sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
		};
	}
}

/*
 *  ティックの更新（全プロセッサ）
 */
void
ttsp_target_gain_tick(void)
{
	int_t		i;
	ID			pid;
	ulong_t		timeout;
	int			prc_index;

	/* 全PEの前回のティック更新処理が完了したことを確認 */
	ttsp_check_timer_handler();

	/* メインプロセッサID取得 */
	sil_get_pid(&pid);
	prc_index = pid - 1;

	/*
	 *  前回の更新が終わっていることを確認
	 *  自PEに対してはチェックする必要はないが，チェックしても問題ないため，
	 *  チェックする． 
	 */
	for(i = 0; i < TNUM_PRCID; i++) {
		if (i != prc_index) {
			timeout = 0;
			while (target_timer_oneshot[i] == true) {
				timeout++;
				if (timeout > TTSP_LOOP_COUNT) {
					syslog_2(LOG_ERROR, "## PE %d : ttsp_target_gain_tick[PE:%d][check] caused a timeout.", pid, i + 1);
					ext_ker();
				}
				sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
			};
		}
	}

	for(i = 0; i < TNUM_PRCID; i++) {
		if (i != prc_index) {
			target_timer_oneshot[i] = true;
		}
	}

	for(i = 0; i < TNUM_PRCID; i++) {
		if (i != prc_index) {
			target_timer_disable[i] = false;
		}
	}

	/* 全PEに対して，ハンドラが終了するまで待ち合わせる */
	for(i = 0; i < TNUM_PRCID; i++) {
		if (i != prc_index) {
			timeout = 0;
			while (target_timer_oneshot[i] == true) {
				timeout++;
				if (timeout > TTSP_LOOP_COUNT) {
					syslog_2(LOG_ERROR, "## PE %d : ttsp_target_gain_tick[PE:%d][wait] caused a timeout.", pid, i + 1);
					ext_ker();
				}
				sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
			};
		}
	}

	/* 最後にメインプロセッサの時刻を進める */
	timeout = 0;
	while (target_timer_oneshot[prc_index] == true) {
		timeout++;
		if (timeout > TTSP_LOOP_COUNT) {
			syslog_2(LOG_ERROR, "## PE %d : ttsp_target_gain_tick[PE:%d][check] caused a timeout.", pid, pid);
			ext_ker();
		}
		sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
	};
	target_timer_oneshot[prc_index] = true;
	target_timer_disable[prc_index] = false;
	timeout = 0;
	while (target_timer_oneshot[prc_index] == true) {
		timeout++;
		if (timeout > TTSP_LOOP_COUNT) {
			syslog_2(LOG_ERROR, "## PE %d : ttsp_target_gain_tick[PE:%d][wait] caused a timeout.", pid, pid);
			ext_ker();
		}
		sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
	};
}


/*
 * 割込みHWのアドレステーブル
 */
const uint32_t target_ttsp_int_base_table[TNUM_PRCID][3] = {
	{0x07114000,0x07114010,0x07114020},
	{0x07114030,0x07114040,0x07114050},
};

/*
 *  割込みの発生
 */
void
ttsp_int_raise(INTNO intno)
{
	uint_t index;
	uint_t prcid;

	index = INTNO_MASK(intno);
	index -= 7; /* 割込み番号は7から連番であることを想定 */
	prcid = intno >> 16;
	sil_wrw_mem((void *)(target_ttsp_int_base_table[prcid-1][index]), 0x01);
}

/*
 *  CPU例外の発生
 */
void
ttsp_cpuexc_raise(EXCNO excno)
{
	if (excno == TTSP_EXCNO_A) {
		RAISE_CPU_EXCEPTION;
	}
#ifdef TTSP_EXCNO_PE2_A
	if (excno == TTSP_EXCNO_PE2_A) {
		RAISE_CPU_EXCEPTION;
	}
#endif /* TTSP_EXCNO_PE2_A */
#ifdef TTSP_EXCNO_PE3_A
	if (excno == TTSP_EXCNO_PE3_A) {
		RAISE_CPU_EXCEPTION;
	}
#endif /* TTSP_EXCNO_PE3_A */
#ifdef TTSP_EXCNO_PE4_A
	if (excno == TTSP_EXCNO_PE4_A) {
		RAISE_CPU_EXCEPTION;
	}
#endif /* TTSP_EXCNO_PE4_A */
}

/*
 *  CPU例外発生時のフック処理
 */
void
ttsp_cpuexc_hook(EXCNO excno, void* p_excinf)
{

}

/*
 *  割込み要求のクリア
 */
void
ttsp_clear_int_req(INTNO intno)
{
	uint_t index;
	uint_t prcid;

	index = INTNO_MASK(intno);
	index -= 7; /* 割込み番号は7から連番であることを想定 */
	prcid = intno >> 16;
	sil_wrw_mem((void *)(target_ttsp_int_base_table[prcid-1][index]), 0x00);
}

