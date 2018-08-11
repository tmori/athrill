/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2016 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: prc_config.c 557 2016-01-06 10:37:19Z ertl-honda $
 */

/*
 *		プロセッサ依存モジュール（V850用）
 */
#include "kernel_impl.h"

/*
 *  例外（割込み/CPU例外）のネスト回数のカウント
 *  コンテキスト参照のために使用
 */
uint32			except_nest_cnt;

/*
 * C1ISRのネスト回数のカウント
 */
uint32			c1isr_nest_cnt;

#define C1ISR_OSTKPT(stk, stksz)	((uint8 *) ((sint8 *) (stk) + (stksz)))

#define C1ISR_STACK_SIZE	1280U
static uint8	_c1isr_stack[C1ISR_STACK_SIZE];
uint8 * const	stkpt_c1isr = C1ISR_OSTKPT(_c1isr_stack, C1ISR_STACK_SIZE);

/*
 * 割込みマスク状態管理
 * 0:割込み許可
 * 1:割込み禁止
 */
uint16_t disint_table[IMR_SIZE];

#ifdef CFG_USE_PROTECTIONHOOK
uint32			v850_cpu_exp_no;
uint32			v850_cpu_exp_pc;
uint32			v850_cpu_exp_sp;
#endif /* CFG_USE_PROTECTIONHOOK */

/*
 * 現在の割込み優先度の値(CPU割込み優先度)
 */
uint8			current_iintpri;

/*
 *  x_nested_lock_os_int()のネスト回数
 */
volatile uint8	nested_lock_os_int_cnt;

/*
 *  プロセッサ依存の初期化
 */
void
prc_initialize(void)
{
	uint32 i;
	/*
	 *  カーネル起動時は非タスクコンテキストとして動作させるため1に
	 */
	except_nest_cnt = 1U;

	c1isr_nest_cnt = 0U;

	/*
	 * 割込み優先度マスクの初期値は最低優先度
	 */
	current_iintpri = INT_IPM(0);

	for (i = 0U; i < IMR_SIZE; i++) {
		disint_table[i] = 0x0000U;
	}

}

/*
 *  プロセッサ依存の終了処理
 */
void
prc_terminate(void)
{
}

/* <TRACE> (6-6-2-1) void x_config_int()
 *
 *  割込み要求ラインの属性の設定
 */
void
x_config_int(InterruptNumberType intno, AttributeType intatr, PriorityType intpri)
{
	uint32 eic_address;

	ASSERT(VALID_INTNO(intno));

	eic_address = EIC_ADDRESS(intno);

	/*
	 *  割込みのマスク
	 *
	 *  割込みを受け付けたまま，レベルトリガ／エッジトリガの設定や，割
	 *  込み優先度の設定を行うのは危険なため，割込み属性にかかわらず，
	 *  一旦マスクする．
	 */
	(void) x_disable_int(intno);


	
	/*
	 *  割込み優先度の設定
	 */
	sil_wrb_mem((void *)eic_address ,
		((sil_reb_mem((void *)eic_address) & ~0x7) | (INT_IPM((volatile PriorityType)intpri)))
	);

	if ((intatr & ENABLE) != 0U) {
		/*
		 *  割込みのマスク解除
		 */
		(void) x_enable_int(intno);
	}
}

#ifndef OMIT_DEFAULT_INT_HANDLER

/*
 * <TRACE> (6-6-4-1) void default_int_handler(void)（オプション）
 *
 *  未定義の割込みが入った場合の処理
 */
void
default_int_handler(void)
{
	target_fput_str("Unregistered Interrupt occurs.");
	ASSERT(0);
}

#endif /* OMIT_DEFAULT_INT_HANDLER */

/*
 *  無限ループ処理
 *  (デバッグ時の無限ループ到達確認用 / 最適化防止のためここで定義する)
 */
void
infinite_loop(void)
{
	while (1) {
	}
}

/*
 *  特定の割込み要求ラインの有効/無効を制御可能かを調べる処理
 */
boolean
target_is_int_controllable(InterruptNumberType intno)
{
	/*
	 *  falseを返す
	 */
	return(FALSE);
}


/*
 *	現在の割込み優先度の値(CPU割込み優先度)の設定
 *
 *	インライン関数でないのは，アセンブラからも使用するためである．
 */
void
set_intpri(uint8_t intpri)
{
	SIL_PRE_LOC;

	SIL_LOC_INT();
	sil_wrh_mem((void *)(INTC_IMR0) , imr_table[intpri][0] | disint_table[0]);
	sil_wrh_mem((void *)(INTC_IMR1) , imr_table[intpri][1] | disint_table[1]);
	sil_wrh_mem((void *)(INTC_IMR2) , imr_table[intpri][2] | disint_table[2]);
	sil_wrh_mem((void *)(INTC_IMR3) , imr_table[intpri][3] | disint_table[3]);
	sil_wrh_mem((void *)(INTC_IMR4) , imr_table[intpri][4] | disint_table[4]);
	sil_wrh_mem((void *)(INTC_IMR5) , imr_table[intpri][5] | disint_table[5]);
	sil_wrh_mem((void *)(INTC_IMR6) , imr_table[intpri][6] | disint_table[6]);
	sil_wrb_mem((void *)(INTC_IMR7) , (uint8)( imr_table[intpri][7] | disint_table[7] ) );
	SIL_UNL_INT();
}

void
set_intpri_lock_os_int(void)
{
	set_intpri(c2isr_iintpri);
}
void
set_intpri_unlock_os_int(void)
{
	set_intpri(current_iintpri);
}
