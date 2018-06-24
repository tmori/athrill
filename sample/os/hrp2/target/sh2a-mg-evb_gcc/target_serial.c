/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011 by Embedded and Real-Time Systems Laboratory
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
 *	シリアルI/Oデバイス（SIO）ドライバ（SH2A-MG-EVB用）
 *	SCI0 を使用
 */

#include "kernel/kernel_impl.h"
#include <t_syslog.h>
#include <sil.h>
#include "target_serial.h"

const SIOPINIB siopinib_table[TNUM_SCI_PORT] = {
#if TNUM_SCI_PORT > 0
    {
        (void *)(SCI0_BASE + OFFSET_SCI_CR),     /* 制御レジスタ */
        (void *)(SCI0_BASE + OFFSET_SCI_MD),     /* モードレジスタ */
        (void *)(SCI0_BASE + OFFSET_SCI_EMD),    /* 拡張モードレジスタ */
        (void *)(SCI0_BASE + OFFSET_SCI_BR),     /* ビットレートレジスタ */
        (void *)(SCI0_BASE + OFFSET_SCI_TB),     /* 送信バッファレジスタ */
        (void *)(SCI0_BASE + OFFSET_SCI_RB),     /* 受信バッファレジスタ */
        (void *)(SCI0_BASE + OFFSET_SCI_SR),     /* ステータスレジスタ */
        (void *)(PIO_PA10CR),  /* RXD端子のポート制御レジスタ */
        (void *)(PIO_PA11CR),  /* TXD端子のポート制御レジスタ */
        (void *)(PIO_PA12CR),  /* SCK端子のポート制御レジスタ */
        (void *)(PIO_PAIN),    /* RXD端子のポート入力レジスタ */
        (10),          /* RXDに対応するPIOの番号：A/Bの後の数字 */
        (INTNO_SCI0_RXF), /* 受信バッファフルの割込み番号 */
        (INTNO_SCI0_TXC), /* 送信完了の割込み番号 */
    },
#endif /* TNUM_SCI_PORT > 0 */
#if TNUM_SCI_PORT > 1
    {
        (void *)(SCI1_BASE + OFFSET_SCI_CR),     /* 制御レジスタ */
        (void *)(SCI1_BASE + OFFSET_SCI_MD),     /* モードレジスタ */
        (void *)(SCI1_BASE + OFFSET_SCI_EMD),    /* 拡張モードレジスタ */
        (void *)(SCI1_BASE + OFFSET_SCI_BR),     /* ビットレートレジスタ */
        (void *)(SCI1_BASE + OFFSET_SCI_TB),     /* 送信バッファレジスタ */
        (void *)(SCI1_BASE + OFFSET_SCI_RB),     /* 受信バッファレジスタ */
        (void *)(SCI1_BASE + OFFSET_SCI_SR),     /* ステータスレジスタ */
        (void *)(PIO_PA13CR),  /* RXD端子のポート制御レジスタ */
        (void *)(PIO_PA14CR),  /* TXD端子のポート制御レジスタ */
        (void *)(PIO_PA15CR),  /* SCK端子のポート制御レジスタ */
        (void *)(PIO_PAIN),    /* RXD端子のポート入力レジスタ */
        (13),          /* RXDに対応するPIOの番号：A/Bの後の数字 */
        (INTNO_SCI1_RXF), /* 受信バッファフルの割込み番号 */
        (INTNO_SCI1_TXC), /* 送信完了の割込み番号 */
    },
#endif /* TNUM_SCI_PORT > 1 */
#if TNUM_SCI_PORT > 2
    {
        (void *)(SCI2_BASE + OFFSET_SCI_CR),     /* 制御レジスタ */
        (void *)(SCI2_BASE + OFFSET_SCI_MD),     /* モードレジスタ */
        (void *)(SCI2_BASE + OFFSET_SCI_EMD),    /* 拡張モードレジスタ */
        (void *)(SCI2_BASE + OFFSET_SCI_BR),     /* ビットレートレジスタ */
        (void *)(SCI2_BASE + OFFSET_SCI_TB),     /* 送信バッファレジスタ */
        (void *)(SCI2_BASE + OFFSET_SCI_RB),     /* 受信バッファレジスタ */
        (void *)(SCI2_BASE + OFFSET_SCI_SR),     /* ステータスレジスタ */
        (void *)(PIO_PB02CR),  /* RXD端子のポート制御レジスタ */
        (void *)(PIO_PB03CR),  /* TXD端子のポート制御レジスタ */
        (void *)(PIO_PB04CR),  /* SCK端子のポート制御レジスタ */
        (void *)(PIO_PBIN),    /* RXD端子のポート入力レジスタ */
        (2),          /* RXDに対応するPIOの番号：A/Bの後の数字 */
        (INTNO_SCI2_RXF), /* 受信バッファフルの割込み番号 */
        (INTNO_SCI2_TXC), /* 送信完了の割込み番号 */
    },
#endif /* TNUM_SCI_PORT > 2 */
#if TNUM_SCI_PORT > 3
    // PIOの番号は，リセット後の設定 
    {
        (void *)(SCI3_BASE + OFFSET_SCI_CR),     /* 制御レジスタ */
        (void *)(SCI3_BASE + OFFSET_SCI_MD),     /* モードレジスタ */
        (void *)(SCI3_BASE + OFFSET_SCI_EMD),    /* 拡張モードレジスタ */
        (void *)(SCI3_BASE + OFFSET_SCI_BR),     /* ビットレートレジスタ */
        (void *)(SCI3_BASE + OFFSET_SCI_TB),     /* 送信バッファレジスタ */
        (void *)(SCI3_BASE + OFFSET_SCI_RB),     /* 受信バッファレジスタ */
        (void *)(SCI3_BASE + OFFSET_SCI_SR),     /* ステータスレジスタ */
        (void *)(PIO_PA06CR),  /* RXD端子のポート制御レジスタ */
        (void *)(PIO_PA07CR),  /* TXD端子のポート制御レジスタ */
        (void *)(PIO_PA08CR),  /* SCK端子のポート制御レジスタ */
        (void *)(PIO_PAIN),    /* RXD端子のポート入力レジスタ */
        (6),          /* RXDに対応するPIOの番号：A/Bの後の数字 */
        (INTNO_SCI3_RXF), /* 受信バッファフルの割込み番号 */
        (INTNO_SCI3_TXC), /* 送信完了の割込み番号 */
    },
#endif /* TNUM_SCI_PORT > 3 */
};

SIOPCB siopcb_table[TNUM_SCI_PORT];

/*
 *	SIOドライバの初期化
 */
void
sio_initialize(intptr_t exinf)
{
    SIOPCB *p_siopcb;
    int8_t i;

    for(i = 0, p_siopcb = &(siopcb_table[0]); i < TNUM_SCI_PORT; i++, p_siopcb++){
        p_siopcb->p_siopinib = &(siopinib_table[i]);
    }
}

void
sio_hardware_initialize(const SIOPINIB *p_siopinib)
{
    /*
     *  ポート制御レジスタの設定（入力）
     */
    sil_wrb_mem((void *)PIO_PPR, PIO_PPR_UNPROTECT); /* プロテクトの解除 */

    /* 入力を許可，ポート機能は周辺機能入力 */
    sil_wrh_mem(p_siopinib->rxdiocr, (PIO_CR_INE | PIO_CR_PSEL_IO));

    sil_wrb_mem((void *)PIO_PPR, 0); /* プロテクト */

    /*
     *  送受信を禁止
     */
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) & ~(SCI_CR_TE | SCI_CR_RE)));

    /*
     *  クロック選択 
     */
    sil_wrb_mem(p_siopinib->sccr,
                ((sil_reb_mem(p_siopinib->sccr) & ~(SCI_CR_CKS_MASK)) | (SCI_CR_CKS)));

    /*
     *  モードレジスタの設定
     *  調歩同期式，
     *  データ長8ビット，パリティなし，1ストップビット，
     *  カウントソース：分周なし
     */
    sil_wrb_mem(p_siopinib->scmd,
                (sil_reb_mem(p_siopinib->scmd) & 
                 ~(SCI_MD_SMS | SCI_MD_SDLS | SCI_MD_PE | SCI_MD_TSTLS | SCI_MD_SCSS_MASK)));
    sil_wrb_mem(p_siopinib->scmd,
                (sil_reb_mem(p_siopinib->scmd) | SCI_MD_SCSS_DIV0));

    /*
     *  ビットレートの設定：38400
     */
    sil_wrb_mem(p_siopinib->scbr, CONVERT_BITRATE(38400));

    sil_dly_nse(40000);

    /*
     *  送受信割込みの禁止
     */
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) & ~(SCI_CR_TIE)));
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) & ~(SCI_CR_TEIE | SCI_CR_RIE)));

    /*
     *  ポート制御レジスタの設定（出力）
     */
    sil_wrb_mem((void *)PIO_PPR, PIO_PPR_UNPROTECT); /* プロテクトの解除 */

    /* 
     *  出力を一時禁止した後，
     *  ポート機能をSCI機能出力に設定し，同時に出力許可
     *  別々に設定することはできない
     */
    sil_wrh_mem(p_siopinib->txdiocr, (~(PIO_CR_OUTE) & PIO_CR_PSEL_IO));
    sil_wrh_mem(p_siopinib->txdiocr, 
                (sil_reb_mem(p_siopinib->txdiocr) | (PIO_CR_OUTE | PIO_CR_PSEL_SCI)));

    sil_wrb_mem((void *)PIO_PPR, 0); /* プロテクト */

    /*
     *  RXD端子がhighであることをチェック
     */
    assert((sil_reh_mem(p_siopinib->rxdioin) & (1 << p_siopinib->rxpio)) != 0);

    /*
     *  送受信を許可
     */
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) | (SCI_CR_TE | SCI_CR_RE)));

}

Inline void
enable_int_send(const SIOPINIB *p_siopinib)
{
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) | (SCI_CR_TEIE)));
}

Inline void
enable_int_receive(const SIOPINIB *p_siopinib)
{
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) | (SCI_CR_RIE)));
}

Inline void
disable_int_send(const SIOPINIB *p_siopinib)
{
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) & ~(SCI_CR_TEIE)));
}

Inline void
disable_int_receive(const SIOPINIB *p_siopinib)
{
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) & ~(SCI_CR_RIE)));
}

Inline bool_t
sio_ready_send_char(const SIOPINIB *p_siopinib)
{
    if((sil_reb_mem(p_siopinib->scsr) & SCI_SR_TBEF) != 0){
        return true;
    } else {
        return false;
    }
}

Inline bool_t
sio_ready_receive_char(const SIOPINIB *p_siopinib)
{
    if((sil_reb_mem(p_siopinib->scsr) & SCI_SR_RBFF) != 0){
        return true;
    } else {
        return false;
    }
}

/*
 *	シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB *p_siopcb = &(siopcb_table[siopid - 1]);
	const SIOPINIB *p_siopinib = p_siopcb->p_siopinib;

    /*
     *  拡張情報を保存する
     *  これは，コールバックルーチンに渡す引数となる
     */
    p_siopcb->exinf = exinf;

    /*
	 *	シリアルI/O割込みをマスクする．
	 */
	dis_int(p_siopinib->intno_rx);
	dis_int(p_siopinib->intno_tx);

    if(!(p_siopcb->initialized_flag)){
        p_siopcb->initialized_flag = true;

        sio_hardware_initialize(p_siopinib);
    }

    /*
	 *	シリアルI/O割込みを許可する．
	 */
	ena_int(p_siopinib->intno_rx);
	ena_int(p_siopinib->intno_tx);

	return(p_siopcb);
}

/*
 *	シリアルI/Oポートのクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
    const SIOPINIB *p_siopinib = p_siopcb->p_siopinib;

    /*
     *  送受信を禁止
     */
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) & ~(SCI_CR_TE | SCI_CR_RE)));

    /*
     *  送受信割込みを禁止
     */
    sil_wrb_mem(p_siopinib->sccr,
                (sil_reb_mem(p_siopinib->sccr) & ~(SCI_CR_TIE | SCI_CR_RIE)));

	/*
	 *	シリアルI/O割込みをマスクする．
	 */
	dis_int(p_siopinib->intno_rx);
	dis_int(p_siopinib->intno_tx);
}

/*
 *	SIOの割込みハンドラ
 */
void
sio_tx_isr(intptr_t exinf)
{
    /*
     *  exinfには，割込みの発生したチャネル番号が入っているので，
     *  そこから，オープン処理で保存した拡張情報を取得し，
     *  コールバックルーチンの引数とする
     */
    exinf = (siopcb_table[exinf - 1]).exinf;

	sio_irdy_snd(exinf);
}


/*
 *	SIOの割込みハンドラ
 */
void
sio_rx_isr(intptr_t exinf)
{
    /*
     *  エッジトリガの割込み要因をクリア
     */
    x_clear_int(siopinib_table[exinf - 1].intno_rx);
    /*
     *  exinfには，割込みの発生したチャネル番号が入っているので，
     *  そこから，オープン処理で保存した拡張情報を取得し，
     *  コールバックルーチンの引数とする
     */
    exinf = (siopcb_table[exinf - 1]).exinf;

	sio_irdy_rcv(exinf);
}

/*
 *	シリアルI/Oポートへの文字送信
 */
bool_t
sio_snd_chr(SIOPCB *p_siopcb, char c)
{
    const SIOPINIB *p_siopinib = p_siopcb->p_siopinib;

    if(sio_ready_send_char(p_siopinib)){
        sil_wrb_mem(p_siopinib->sctb, (uint8_t)c);

        return true;
    } else {
        return false;
    }
}

/*
 *	シリアルI/Oポートからの文字受信
 */
int_t
sio_rcv_chr(SIOPCB *p_siopcb)
{
    const SIOPINIB *p_siopinib = p_siopcb->p_siopinib;

    if(sio_ready_receive_char(p_siopinib)){
        return (int_t)sil_reb_mem(p_siopinib->scrb);
    }

    return 0;
}

/*
 *	シリアルI/Oポートからのコールバックの許可
 */
void
sio_ena_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
    const SIOPINIB *p_siopinib = p_siopcb->p_siopinib;

    switch(cbrtn){
        case SIO_RDY_RCV:
            /*
             *	受信割込みのマスクを解除する
             */
            enable_int_receive(p_siopinib);
            break;
        case SIO_RDY_SND:
            /*
             *	送信割込みのマスクを解除する
             */
            enable_int_send(p_siopinib);
            break;
        default:
            break;
    }
}

/*
 *	シリアルI/Oポートからのコールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *p_siopcb, uint_t cbrtn)
{
    const SIOPINIB *p_siopinib = p_siopcb->p_siopinib;

    switch(cbrtn){
        case SIO_RDY_RCV:
            /*
             *	受信割込みをマスクする
             */
            disable_int_receive(p_siopinib);
            break;
        case SIO_RDY_SND:
            /*
             *	送信割込みをマスクする
             */
            disable_int_send(p_siopinib);
            break;
        default:
            break;
    }
}

