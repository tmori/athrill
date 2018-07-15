/*
 *  TTSP
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  $Id: ttsp_target_mtu2.c 2 2012-05-09 02:23:52Z nces-shigihara $
 */

/*
 *  マルチファンクションタイマパルスユニットMTU2ドライバ
 */

#include "kernel_impl.h"
#include <sil.h>
#include "ttsp_target_mtu2.h"


/*
 *  MTU2初期化ブロックの定義
 */
typedef struct mtu2_initialization_block {
	uint8_t  *p_tcr_b;		/* タイマコントロールレジスタ  */
	uint8_t  *p_tier_b;		/* タイマインタラプトイネーブルレジスタ  */
	uint8_t  *p_tsr_b;		/* タイマステータスジスタ  */
	uint16_t *p_tcnt_h;		/* タイマカウンタ */
	uint8_t  start_bit;		/* スタートビット */
} MTU2INIB;

/*
 *  MTU2初期化ブロック
 */
const MTU2INIB mtu2inib_table[] = {
	{
		(uint8_t *)TCR_0_b,
		(uint8_t *)TIER_0_b,
		(uint8_t *)TSR_0_b,
		(uint16_t *)TCNT_0_h,
		TSTR_CST0
	},
	{
		(uint8_t *)TCR_1_b,
		(uint8_t *)TIER_1_b,
		(uint8_t *)TSR_1_b,
		(uint16_t *)TCNT_1_h,
		TSTR_CST1
	},
	{
		(uint8_t *)TCR_2_b,
		(uint8_t *)TIER_2_b,
		(uint8_t *)TSR_2_b,
		(uint16_t *)TCNT_2_h,
		TSTR_CST2
	},
	{
		(uint8_t *)TCR_3_b,
		(uint8_t *)TIER_3_b,
		(uint8_t *)TSR_3_b,
		(uint16_t *)TCNT_3_h,
		TSTR_CST3
	},
	{
		(uint8_t *)TCR_4_b,
		(uint8_t *)TIER_4_b,
		(uint8_t *)TSR_4_b,
		(uint16_t *)TCNT_4_h,
		TSTR_CST4
	},
};


/*
 *  タイマスタート
 */
Inline void
ttsp_target_start_mtu2(const MTU2INIB *p_mtu2inib)
{
	/* MTU2チャネルnスタート */
	sil_orb_reg((uint8_t *)TSTR_b, p_mtu2inib->start_bit);
}

/*
 *  タイマ停止
 */
Inline void
ttsp_target_stop_mtu2(const MTU2INIB *p_mtu2inib)
{
	/* MTU2チャネルn停止 */
	sil_anb_reg((uint8_t *)TSTR_b, ~(p_mtu2inib->start_bit));
}

/*
 *  タイマ割込み要求をクリア
 */
Inline void
ttsp_target_clear_mtu2_int(const MTU2INIB *p_mtu2inib)
{
	sil_anb_reg(p_mtu2inib->p_tsr_b, ~TCR_TCFV);
}

/*
 *  タイマ割込みを許可
 */
Inline void
ttsp_target_enable_mtu2_int(const MTU2INIB *p_mtu2inib)
{
	sil_orb_reg(p_mtu2inib->p_tier_b, TIER_TCIEV);
}

/*
 *  コンペアマッチ待ち
 */
Inline void
ttsp_target_wait_compare_match_mtu2(const MTU2INIB *p_mtu2inib)
{
	uint8_t *p_tsr_b = p_mtu2inib->p_tsr_b;
	while((sil_reb_mem((void*)p_tsr_b) & TCR_TCFV) == 0U);
}

/*
 *  MTU2割込み番号からチャネル番号への変換
 *  　MTU2チャネル0-4を用いる。
 */
static int_t
ttsp_target_mtu2_intno2ch(INTNO intno)
{
	uint_t	ch = 0;
	
	switch(intno) {
		case MTU0_TGI0V_VECTOR:
			ch = 0;
			break;
		case MTU1_TGI1V_VECTOR:
			ch = 1;
			break;
		case MTU2_TGI2V_VECTOR:
			ch = 2;
			break;
		case MTU3_TGI3V_VECTOR:
			ch = 3;
			break;
		case MTU4_TGI4V_VECTOR:
			ch = 4;
			break;
		default:
			syslog_2(LOG_NOTICE, "Error: intno %d(0x%x) is not supported by MTU2 driver.",
		          (intptr_t)intno, (intptr_t)intno);
			ext_ker();
			break;
	}
	return ch;
}

/*
 *  MTU2割込みの発生   
 *  　MTU2チャネル0-4を用いる。
 */
void
ttsp_target_raise_mtu2_int(INTNO intno)
{
	int_t	ch;
	const MTU2INIB *p_mtu2inib;
	SIL_PRE_LOC;

	ch = ttsp_target_mtu2_intno2ch(intno);
	p_mtu2inib = &mtu2inib_table[ch];

	/*
	 *	MTU2チャネルnを初期化
	 */

	/*  MTU2にクロックを供給  */
	SIL_LOC_INT();
	sil_anb_reg((uint8_t *)STBCR3_b, ~STBCR3_MTU2);
	sil_reb_mem((void *)STBCR3_b);		/*  ダミーリード  */
	SIL_UNL_INT();
	sil_dly_nse(100);					/*  クロック安定待ち  */

	/*
	 *	タイマ周期を設定し，タイマの動作を開始する．
	 */
	SIL_LOC_INT();

	/* MTU2チャネルn停止 */
	ttsp_target_stop_mtu2(p_mtu2inib);

	/*
	 *	・カウンタクリア要因
	 *	・クロックソース
	 *	・エッジ選択
	 */
	sil_wrb_mem(p_mtu2inib->p_tcr_b, 0U);

	/* アップカウンタの初期値をセット */
	sil_wrh_mem(p_mtu2inib->p_tcnt_h, 0xfffc);

	/* 割込み要求をクリア */
	ttsp_target_clear_mtu2_int(p_mtu2inib);

	/* 割込みの許可 */
	ttsp_target_enable_mtu2_int(p_mtu2inib);

	/* MTU2チャネルnスタート */
	ttsp_target_start_mtu2(p_mtu2inib);

#if 0
	/* コンペアマッチ待ち */
	ttsp_target_wait_compare_match_mtu2(p_mtu2inib);

	/* MTU2チャネルn停止 */
	ttsp_target_stop_mtu2(p_mtu2inib);
#endif

	SIL_UNL_INT();

	/* ここで割込みが入る */
}

/*
 *  割込み要求のクリア
 */
void
ttsp_clear_int_req_mtu2(INTNO intno)
{
	int_t	ch;
	const MTU2INIB *p_mtu2inib;
	SIL_PRE_LOC;

	ch = ttsp_target_mtu2_intno2ch(intno);
	p_mtu2inib = &mtu2inib_table[ch];

	SIL_LOC_INT();
	ttsp_target_stop_mtu2(p_mtu2inib);
	ttsp_target_clear_mtu2_int(p_mtu2inib);
	SIL_UNL_INT();
}

