/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011-2012 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	
 *	上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *	ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *	変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *	(1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *		権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *		スコード中に含まれていること．
 *	(2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *		用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *		者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *		の無保証規定を掲載すること．
 *	(3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *		用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *		と．
 *	  (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *		  作権表示，この利用条件および下記の無保証規定を掲載すること．
 *	  (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *		  報告すること．
 *	(4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *		害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *		また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *		由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *		免責すること．
 *	
 *	本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *	よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *	に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *	アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *	の責任を負わない．
 *	
 */

/*
 *	ターゲット依存モジュール（SH2A-MG-EVB用）
 */

#include "kernel_impl.h"
#include <sil.h>
#include "sh72aw.h"
#include "target_serial.h"
#include "task.h"
#include "memory.h"

/*
 *	ターゲット依存の初期化
 */
void
target_initialize(void)
{
	/*
	 *	プロセッサ依存の初期化
	 */
	prc_initialize();

    /*
     *  MPUの初期化
     */
    target_mpu_initialize();

	/*
	 *  低レベル出力用のシリアルコントローラの初期化
	 */
    target_fput_initialize();
}

/*
 *	ターゲット依存の終了処理
 */
void
target_exit(void)
{
	/*
	 *	プロセッサ依存の終了処理
	 */
	prc_exit();

	/*
	 *	開発環境依存の終了処理
     *	特になし
	 */
    while(1);

	/*
	 * ここには来ない
	 */
	while(1);
}

/*
 *  低レベル出力用シリアルポートの初期化
 */
void
target_fput_initialize(void)
{
    SIOPCB *p_siopcb = &(siopcb_table[TARGET_PUTC_PORTID - 1]);
    const SIOPINIB *p_siopinib = &(siopinib_table[TARGET_PUTC_PORTID - 1]);

    if(!(p_siopcb->initialized_flag)){
        p_siopcb->initialized_flag = true;

        sio_hardware_initialize(p_siopinib);
    }
}

/*
 *	システムログの低レベル出力のための文字出力
 */
Inline void
sio_send_char_pol(char c)
{
    const SIOPINIB *p_siopinib = &(siopinib_table[TARGET_PUTC_PORTID - 1]);
    SIL_PRE_LOC;

    /*
     *  送信バッファが空くまで待つ
     */
    while((sil_reb_mem(p_siopinib->scsr) & SCI_SR_TBEF) == 0);

    /*
     *  他の処理単位に送信バッファを使われないように，
     *  割込みを禁止する
     */
    SIL_LOC_INT();
    
    /*
     *  送信バッファに文字を書き込む
     */
    sil_wrb_mem(p_siopinib->sctb, (uint8_t)c);

    /*
     *  割込み禁止を解除
     */
    SIL_UNL_INT();
}

void
target_fput_log(char c)
{
	if (c == '\n') {
		sio_send_char_pol('\r');
	}
    sio_send_char_pol(c);
}

/*
 *	割込み要求ライン属性の設定
 */
void
x_config_int(INTNO intno, ATR intatr, PRI intpri)
{
    uint16_t set_icr;

	assert(VALID_INTNO_CFGINT(intno));
	assert((-15 <= intpri) && (intpri <= TMAX_INTPRI));

	/*
	 *	一旦割込みを禁止する
	 */
	(void)x_disable_int(intno);

    /*
     *  割込み優先度の設定と割込み許可
     */
    set_icr = (intatr & TA_ENAINT)? INTC_PB_ICR_INTEN: 0U;
    intpri *= -1; // +-の反転
    set_icr |= ((uint16_t)intpri & INTC_PB_ICR_IPR_MASK);

    sil_wrh_mem((void *)(INTC_PB_ICR_BASE + (intno - INTC_PB_INTNO_BASE) * 2), set_icr);

}

/*
 *  メモリ領域がユーザスタック領域に含まれているかのチェック
 *
 *  先頭番地がbaseでサイズがsizeのメモリ領域が，p_tcbで指定されるタスク
 *  のユーザスタック領域に含まれている場合にtrue，そうでない場合に
 *  falseを返す．
 *
 *  メモリ領域の先頭番地からユーザスタックの底までの長さが
 *  メモリ領域のサイズよりも大きく，かつ，
 *  ユーザスタックのサイズよりも小さければ，メモリ領域は，
 *  ユーザスタックの範囲内である．
 *
 */
bool_t
within_ustack(const void *base, SIZE size, TCB *p_tcb)
{
    SIZE stk_bottom = (SIZE)(p_tcb->p_tinib->tskinictxb.stk_bottom);
    SIZE len = stk_bottom - (SIZE)base;

    return ((SIZE)base < stk_bottom
            && len >= size
            && len <= (SIZE)(p_tcb->p_tinib->tskinictxb.stksz));
}

/*
 *  ユーザタスクのタスク例外処理呼び出し時に
 *  使用するスタックサイズが足りないか？
 */
bool_t
i_check_tex_runtsk()
{
    SIZE ustk_top = (SIZE)(p_runtsk->tskctxb.usp);

    if(!PROBE_STACK((void *)(ustk_top - USRTEX_STKSZ), 
                USRTEX_STKSZ)){
        return (true);
    }

    return (false);
}  

