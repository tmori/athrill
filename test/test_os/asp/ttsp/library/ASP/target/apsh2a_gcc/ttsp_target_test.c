/*
 *  TTSP
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
 *	Copyright (C) 2010-2011 by Industrial Technology Institute,
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
#include <sil.h>
#include "ttsp_target_test.h"
#include "target_timer.h"
#include "ttsp_target_mtu2.h"

/*
 *  割込み番号の割り当て
 *  　システムタイマ：	CMT0
 *  　TTSP_INTNO_A：	CMT1
 *  　TTSP_INTNO_B：	MTU2 ch0
 *  　TTSP_INTNO_C：	MTU2 ch1
 *  　TTSP_INTNO_D：	MTU2 ch2
 *  　TTSP_INTNO_E：	MTU2 ch3
 *  　TTSP_INTNO_F：	MTU2 ch4
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
	uint32_t start_bit;		/* スタートビット */
} CMTINIB;

/*
 *  CMT初期化ブロック
 */
const CMTINIB cmtinib_table[] = {
	{
		(uint16_t *)CMCSR_0_h,
		CMSTR_STR0
	},
	{
		(uint16_t *)CMCSR_1_h,
		CMSTR_STR1
	}
};

/*
 *  タイマスタート
 */
Inline void
ttsp_target_start_cmt(const CMTINIB *p_cmtinib)
{
	/* CMTnスタート */
	sil_orh_reg((uint16_t *)CMSTR_h, p_cmtinib->start_bit);
}

/*
 *  タイマ停止
 */
Inline void
ttsp_target_stop_cmt(const CMTINIB *p_cmtinib)
{
	/* CMTn停止 */
	sil_anh_reg((uint16_t *)CMSTR_h, ~(p_cmtinib->start_bit));
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
 *  CMT割込みの発生   
 *  　システムタイマにCMT0を用いるので、残りはCMT1のみ
 */
static void
ttsp_target_raise_cmt_int(INTNO intno)
{
	const CMTINIB *p_cmtinib = &cmtinib_table[1];	/* CMT1 */
	CLOCK	cyc = (TO_CLOCK(TIC_NUME, TIC_DENO)) - 1;
	SIL_PRE_LOC;

	/*  CMTnを初期化  */
	SIL_LOC_INT();
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

	SIL_UNL_INT();

	/* ここで割込みが入る */
}

/*
 *  CMT割込み要求のクリア
 *   CMT1を用いる。
 */
static void
ttsp_clear_int_req_cmt(INTNO intno)
{
	const CMTINIB *p_cmtinib = &cmtinib_table[1];	/* CMT1 */
	SIL_PRE_LOC;

	SIL_LOC_INT();
	/* CMTn停止 */
	ttsp_target_stop_cmt(p_cmtinib);
	/* 割込み要求をクリア */
	ttsp_target_clear_cmt_int(p_cmtinib);
	SIL_UNL_INT();
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
	    " nop           \n"			/*  ←スタックに退避されるアドレス  */
	    " nop           \n"
	    " nop           \n"
	    " nop           " : "=r"(tmp): "r"(adr));
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
	SIL_LOC_INT();

	/* タイマ停止 */
	ttsp_target_stop_cmt(&cmtinib_table[0]);

	SIL_UNL_INT();
}

/*
 *  ティック更新の再開
 */
void
ttsp_target_start_tick(void)
{
	SIL_PRE_LOC;
	SIL_LOC_INT();

	/* タイマスタート */
	ttsp_target_start_cmt(&cmtinib_table[0]);

	SIL_UNL_INT();
}

/*
 *  ティックの更新
 *  　２回以上、タイマ割込みを発生させないため、
 *  　この関数の中でコンペアマッチまで待ち、タイマを停止する。
 */
void
ttsp_target_gain_tick(void)
{
	const CMTINIB *p_cmtinib = &cmtinib_table[0];	/* CMT0 */
	SIL_PRE_LOC;
	SIL_LOC_INT();

	/* 割込み要求をクリア */
	ttsp_target_clear_cmt_int(p_cmtinib);

	/* タイマのスタート */
	ttsp_target_start_cmt(p_cmtinib);

	/* コンペアマッチ待ち */
	ttsp_target_wait_compare_match_cmt(p_cmtinib);

	/*
	 * タイマの停止
	 * （２回以上、タイマ割込みを発生させないため） 
	 */
	ttsp_target_stop_cmt(p_cmtinib);

	/* 割込みの許可 */
	ttsp_target_enable_cmt_int(p_cmtinib);

	SIL_UNL_INT();	/*  ここでタイマ割込みを受け付ける。  */
}


/*
 *  割込みの発生   
 */
void
ttsp_int_raise(INTNO intno)
{
#if 1
	syslog_2(LOG_NOTICE, "ttsp_int_raise: intno %d(0x%x)",
		          (intptr_t)intno, (intptr_t)intno);
#endif
	
	switch(intno) {

		case TTSP_INTNO_A:
			ttsp_target_raise_cmt_int(intno);
			break;
		
		case TTSP_INTNO_B:
		case TTSP_INTNO_C:
		case TTSP_INTNO_D:
		case TTSP_INTNO_E:
		case TTSP_INTNO_F:
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
#if 1
	syslog_2(LOG_NOTICE, "ttsp_clear_int_req: intno %d(0x%x)",
		          (intptr_t)intno, (intptr_t)intno);
#endif

	switch(intno) {
		case TTSP_INTNO_A:
			ttsp_clear_int_req_cmt(intno);
			break;

		case TTSP_INTNO_B:
		case TTSP_INTNO_C:
		case TTSP_INTNO_D:
		case TTSP_INTNO_E:
		case TTSP_INTNO_F:
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
 *  CPU例外の発生
 */
void
ttsp_cpuexc_raise(EXCNO excno)
{
	switch(excno) {
		case TTSP_EXCNO_A:
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
