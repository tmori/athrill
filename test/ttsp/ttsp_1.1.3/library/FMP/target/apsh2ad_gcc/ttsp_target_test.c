/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
 *  Copyright (C) 2010-2011 by Industrial Technology Institute,
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
 *  $Id: ttsp_target_test.c 2 2012-05-09 02:23:52Z nces-shigihara $
 */

#include "kernel_impl.h"
#include "time_event.h"
#include <sil.h>
#include "ttsp_target_test.h"
#include "target_timer.h"
#include "ttsp_target_mtu2.h"

/*
 *  割込み番号の割り当て
 *  　システムタイマ：	CMT0
 *  　TTSP_INTNO_A：	CMT1
 *  　TTSP_INTNO_B：	CMT2
 *  　TTSP_INTNO_C：	CMT3
 *  　TTSP_INTNO_D：	MTU2 ch0
 *  　TTSP_INTNO_PE2_A：MTU2 ch1
 *  　TTSP_INTNO_PE2_B：MTU2 ch2
 *  　TTSP_INTNO_PE2_C：MTU2 ch3
 *  　TTSP_INTNO_PE2_D：MTU2 ch4
 */

/*
 *  CMTのレジスタ定義
 */

/*
 *  ベースアドレスからのオフセットの定義（2バイト単位）
 */
#define OFFSET_CMCSR_h	0U	/* タイマコントロール／ステータスレジスタ */
#define OFFSET_CMCNT_h	1U	/* コンペアマッチカウンタ */
#define OFFSET_CMCOR_h	2U	/* コンペアマッチコンスタントレジスタ */

/*
 *  CMT初期化ブロックの定義
 */
typedef struct cmt_initialization_block {
	uint16_t *base_addr;	/* ベースアドレス(CMCSRレジスタのアドレス)  */
	uint16_t *p_cmstr_h;	/* タイマスタートレジスタ（共用） */
	uint32_t start_bit;		/* スタートビット */
} CMTINIB;

/*
 *  CMT初期化ブロック
 */
const CMTINIB cmtinib_table[] = {
	{
		(uint16_t *)CMCSR_0_h,
		(uint16_t *)CMSTR_01_h,
		CMSTR_STR0
	},
	{
		(uint16_t *)CMCSR_1_h,
		(uint16_t *)CMSTR_01_h,
		CMSTR_STR1
	},
	{
		(uint16_t *)CMCSR_2_h,
		(uint16_t *)CMSTR_23_h,
		CMSTR_STR2
	},
	{
		(uint16_t *)CMCSR_3_h,
		(uint16_t *)CMSTR_23_h,
		CMSTR_STR3
	}
};


/*
 *  タイマスタート
 */
Inline void
ttsp_target_start_cmt(const CMTINIB *p_cmtinib)
{
	/* CMTnスタート */
	sil_orh_reg(p_cmtinib->p_cmstr_h, p_cmtinib->start_bit);
}

/*
 *  タイマ停止
 */
Inline void
ttsp_target_stop_cmt(const CMTINIB *p_cmtinib)
{
	/* CMTn停止 */
	sil_anh_reg(p_cmtinib->p_cmstr_h, ~(p_cmtinib->start_bit));
}

/*
 *  タイマ割込み要求をクリア
 */
Inline void
ttsp_target_clear_cmt_int(const CMTINIB *p_cmtinib)
{
	sil_anh_reg(p_cmtinib->base_addr + OFFSET_CMCSR_h, ~CMCSR_CMF);
}

/*
 *  タイマ割込みを許可
 */
Inline void
ttsp_target_enable_cmt_int(const CMTINIB *p_cmtinib)
{
	sil_orh_reg(p_cmtinib->base_addr + OFFSET_CMCSR_h, CMCSR_CMIE);
}

/*
 *  コンペアマッチ待ち
 */
Inline void
ttsp_target_wait_compare_match_cmt(const CMTINIB *p_cmtinib)
{
	uint16_t *p_cmcsr = p_cmtinib->base_addr + OFFSET_CMCSR_h;
	while((sil_reh_mem((void*)p_cmcsr) & CMCSR_CMF) == 0);
}

/*
 *  CMT割込み番号からチャネル番号への変換
 */
Inline int_t
ttsp_target_cmt_intno2ch(INTNO intno)
{
	INTNO 	local_intno;
	int_t	ch;

	local_intno = intno & 0xffffU;
	ch = local_intno - CMI0_VECTOR;
	return ch;
}


/*
 *  CMT割込みの発生   
 *  　CMT1-3を用いる。
 */
static void
ttsp_target_raise_cmt_int(INTNO intno)
{
	int_t	ch;
	uint_t	bitptn;
	const CMTINIB *p_cmtinib;
	CLOCK	cyc = (TO_CLOCK(TIC_NUME, TIC_DENO)) - 1;
	SIL_PRE_LOC;

	ch = ttsp_target_cmt_intno2ch(intno);
	p_cmtinib = &cmtinib_table[ch];

	/*
	 *	CMTnを初期化
	 */

	/*  CMTnにクロックを供給  */
	switch(ch) {
		case 0:
		case 1:
			bitptn = ~0x80U;
			break;
		case 2:
		case 3:
			bitptn = ~0x40U;
			break;
		default:
			bitptn = 0x00U;
			break;
	}
	SIL_LOC_SPN();
	sil_anb_reg((uint8_t *)STBCR7_b, bitptn);
	sil_reb_mem((void *)STBCR7_b);		/*  ダミーリード  */
	SIL_UNL_SPN();
	sil_dly_nse(100);					/*  クロック安定待ち  */

	SIL_LOC_SPN();
	/*
	 *	タイマ周期を設定し，タイマの動作を開始する．
	 */
	/* CMTn停止 */
	ttsp_target_stop_cmt(p_cmtinib);

	/* タイマ上限値のチェック */
	assert(cyc <= MAX_CLOCK);

	/* 分周比設定(PCLOCK/8)、割込み許可 */
	sil_wrh_mem(p_cmtinib->base_addr + OFFSET_CMCSR_h, (CMCSR_CKS | CMCSR_CMIE));

	/* コンペアマッチ・コンスタント・レジスタをセット */
	sil_wrh_mem(p_cmtinib->base_addr + OFFSET_CMCOR_h, cyc);

	/* 割込み要求をクリア */
	ttsp_target_clear_cmt_int(p_cmtinib);

	/* 割込みの許可 */
	ttsp_target_enable_cmt_int(p_cmtinib);

	/* CMTnスタート */
	ttsp_target_start_cmt(p_cmtinib);

#if 0
	/* コンペアマッチ待ち */
	ttsp_target_wait_compare_match_cmt(p_cmtinib);

	/* CMTn停止 */
	ttsp_target_stop_cmt(p_cmtinib);
#endif

	SIL_UNL_SPN();

	/* ここで割込みが入る */
}

/*
 *  CMT割込み要求のクリア
 */
void
ttsp_clear_int_req_cmt(INTNO intno)
{
	int_t	ch;
	const CMTINIB *p_cmtinib;
	SIL_PRE_LOC;

	ch = ttsp_target_cmt_intno2ch(intno);
	p_cmtinib = &cmtinib_table[ch];

	SIL_LOC_SPN();
	/* CMTn停止 */
	ttsp_target_stop_cmt(p_cmtinib);
	/* 割込み要求をクリア */
	ttsp_target_clear_cmt_int(p_cmtinib);
	SIL_UNL_SPN();
}

/*
 *  ティック用タイマディスエーブル変数
 */
static volatile bool_t target_timer_disable = false;

/*
 *  ティック用タイマワンショット実行
 */
static volatile bool_t target_timer_oneshot = false;

/*
 *  タイマハンドラ開始時に呼び出すフック
 *  (falseでsignal_time()を呼び出さない)
 */
static bool_t
ttsp_timer_handler_begin_hook(void)
{
	return (!target_timer_disable);
}

/*
 *  タイマハンドラ終了時に呼び出すフック
 */
static void
ttsp_timer_handler_end_hook(void)
{
	if (target_timer_oneshot) {
		target_timer_disable = true;
		target_timer_oneshot = false;
	}
}

/************　外部に公開する関数　*************************/

/*
 *	システムタイマには、CMT0を使用
 */

/*
 *  ティック更新の停止
 */
void
ttsp_target_stop_tick(void)
{
	SIL_PRE_LOC;
	SIL_LOC_SPN();

	/* タイマ停止 */
	target_timer_disable = true;
	ttsp_target_stop_cmt(&cmtinib_table[0]);

	SIL_UNL_SPN();
}

/*
 *  ティック更新の再開
 */
void
ttsp_target_start_tick(void)
{
	SIL_PRE_LOC;
	SIL_LOC_SPN();

	/* タイマスタート */
	target_timer_disable = false;
	ttsp_target_start_cmt(&cmtinib_table[0]);

	SIL_UNL_SPN();
}

/*
 *  ティックの更新（特定プロセッサ）
 *  　グローバルタイマ方式なので、
 *  　prcidには時刻管理プロセッサしか指定しない。
 *  　
 *  　bool_t wait_flg
 *  　        true： タイマ割込みハンドラの終了まで待つ
 *  　        false：タイマ割込みハンドラの終了まで待たない
 */
void
ttsp_target_gain_tick_pe(ID prcid, bool_t wait_flg)
{
	ID				pid;
	volatile PCB 	*p_pcb = get_mp_p_pcb(prcid);
	EVTTIM			current_time1, current_time2;
	ulong_t			timeout;
	SIL_PRE_LOC;

	assert(prcid == TOPPERS_SYSTIM_PRCID);

	/*
	 *  前回の更新が終わっていることを確認
	 *  自PEに対してはチェックする必要はないが，チェックしても問題ないため，
	 *  チェックする． 
	 */
	timeout = 0;
	while (target_timer_oneshot == true) {
		timeout++;
		if (timeout > TTSP_LOOP_COUNT) {
			syslog_2(LOG_ERROR, "## PE %d : ttsp_target_gain_tick_pe(%d)[check] caused a timeout.", pid, prcid);
			ext_ker();
		}
		sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
	};

	SIL_LOC_SPN();
	target_timer_oneshot = true;
	target_timer_disable = false;

	sil_get_pid(&pid);
	current_time1 = p_pcb->p_tevtcb->current_time;
	
	/* タイマスタート */
	ttsp_target_start_cmt(&cmtinib_table[0]);

	SIL_UNL_SPN();
	
	/* wait_flgがtrueの場合は，システム時刻が更新されるまで待ち合わせる */
	if (wait_flg) {
		timeout = 0;
		while(1){
			SIL_LOC_SPN();
			current_time2 = p_pcb->p_tevtcb->current_time;
			SIL_UNL_SPN();

			if (current_time1 != current_time2) {
				/* タイマ停止 */
				SIL_LOC_SPN();
				ttsp_target_stop_cmt(&cmtinib_table[0]);
				SIL_UNL_SPN();
				break;
			} else {
				timeout++;
				if (timeout > TTSP_LOOP_COUNT) {
					syslog_2(LOG_ERROR, "## PE %d : ttsp_target_gain_tick_pe(%d)[wait] caused a timeout.",
					          pid, prcid);
					ext_ker();
				}
				sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
			}
		}
	}
}


/*
 *	タイマ割込みハンドラ
 *	　カーネルのアーキテクチャ依存部にあるtarget_timer_handlerの代替関数
 */
void
target_timer_handler(void)
{
								/* 割込み要求をクリア */
	ttsp_target_clear_cmt_int(&cmtinib_table[0]);
	
	i_begin_int(INTNO_TIMER_SYSTIM);
	if (ttsp_timer_handler_begin_hook()) {
		signal_time();                    /* タイムティックの供給 */
		ttsp_timer_handler_end_hook();
	}
	i_end_int(INTNO_TIMER_SYSTIM);
}

/*
 *  割込みの発生   
 */
void
ttsp_int_raise(INTNO intno)
{
#ifdef TTSP_DEBUG_MESSAGE
	syslog_2(LOG_NOTICE, "ttsp_int_raise: intno %d(0x%x)",
		          (intptr_t)intno, (intptr_t)intno);
#endif	/*  TTSP_DEBUG_MESSAGE  */

	switch(intno) {

		case TTSP_INTNO_A:
		case TTSP_INTNO_B:
		case TTSP_INTNO_C:
			ttsp_target_raise_cmt_int(intno);
			break;
		
		case TTSP_INTNO_D:
		case TTSP_INTNO_PE2_A:
		case TTSP_INTNO_PE2_B:
		case TTSP_INTNO_PE2_C:
		case TTSP_INTNO_PE2_D:
			ttsp_target_raise_mtu2_int(intno);
			break;

		default:
			syslog_2(LOG_NOTICE, "Error: intno %d(0x%x) is not supported by ttsp_int_raise.",
		          (intptr_t)intno, (intptr_t)intno);
			ext_ker();
			break;
	}
}


/*
 *  割込み要求のクリア
 */
void
ttsp_clear_int_req(INTNO intno)
{
#ifdef TTSP_DEBUG_MESSAGE
	syslog_2(LOG_NOTICE, "ttsp_clear_int_req: intno %d(0x%x)",
		          (intptr_t)intno, (intptr_t)intno);
#endif	/*  TTSP_DEBUG_MESSAGE  */

	switch(intno) {
		case TTSP_INTNO_A:
		case TTSP_INTNO_B:
		case TTSP_INTNO_C:
			ttsp_clear_int_req_cmt(intno);
			break;

		case TTSP_INTNO_D:
		case TTSP_INTNO_PE2_A:
		case TTSP_INTNO_PE2_B:
		case TTSP_INTNO_PE2_C:
		case TTSP_INTNO_PE2_D:
			ttsp_clear_int_req_mtu2(intno);
			break;

		default:
			syslog_2(LOG_NOTICE, "Error: intno %d(0x%x) is not supported by ttsp_clear_int_req.",
			          (intptr_t)intno, (intptr_t)intno);
			ext_ker();
			break;
	}
}


/*
 *	アドレスエラー例外発生関数
 *	　
 *	　例外発生時にスタックに退避されるPCの値は「最後に実行した命令」
 *	　の次命令の先頭アドレスを指している。
 *	　この「最後に実行した命令」とは実際にアドレスエラー例外を発生
 *	　した命令ではなく、例外発生時にパイプライン上で実行されている
 *	　後続の命令である。
 *	　（例外を発生した命令がメモリアクセスステージで、後続の命令が
 *	　　既に実行ステージにある場合）
 *	　
 *	　また、後続の命令に分岐命令が含まれる場合は、退避すべきPCの値が
 *	　書き換えられているケースがある。
 *	　
 *	　「最後に実行した命令」が例外発生箇所から何命令離れているか
 *	　一概には求められないため、ここではnop命令を挿入している。
 */
Inline void 
ttsp_target_raise_address_error(void)
{
	uint32_t tmp;
	uint32_t adr = 0xfffffec1U;		/*  奇数番地  */
	
	Asm(" mov.l @%1, %0 \n"			/*  アドレスエラー例外を発生  */
	    " nop           \n"
	    " nop           \n"
	    " nop           \n"			/*  ←スタックに退避されるアドレス  */
	    " nop           " : "=r"(tmp): "r"(adr));
}

/*
 *  CPU例外の発生
 */
void
ttsp_cpuexc_raise(EXCNO excno)
{
	switch(excno) {
		case TTSP_EXCNO_A:
		case TTSP_EXCNO_PE2_A:
			ttsp_target_raise_address_error();
			break;

		default:
			syslog_2(LOG_NOTICE, "Error: excno %d(0%x) is not supported by ttsp_cpuexc_raise.",
		          (intptr_t)excno, (intptr_t)excno);
			break;
	}
}

/*
 *  CPU例外発生時のフック処理
 */
void
ttsp_cpuexc_hook(EXCNO excno, void* p_excinf)
{

}
