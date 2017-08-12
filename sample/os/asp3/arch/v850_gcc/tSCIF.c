/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
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
 *  $Id: tSCIF.c 720 2016-04-01 22:16:17Z ertl-hiro $
 */

/*
 *		FIFO内蔵シリアルコミュニケーションインタフェース用 簡易SIOドライバ
 */

#include <sil.h>
#include "tSCIF_tecsgen.h"
#include "prc_config.h"

/*
 *  プリミティブな送信／受信関数
 */

/*
 *  受信バッファに文字があるか？
 */
Inline bool_t
scif_getready(CELLCB *p_cellcb)
{
	uint8_t str = sil_reb_mem((void *)UDnSTR(UDnCH0));

	if ((str & 0x10) == 0x10) {
		return true;
	}
	return false;
}

/*
 *  送信バッファに空きがあるか？
 */
Inline bool_t
scif_putready(CELLCB *p_cellcb)
{
	uint8_t str = sil_reb_mem((void *)UDnSTR(UDnCH0));

	if ((str & 0x80) == 0x00) {
		return true;
	}
	return false;
}

/*
 *  受信した文字の取出し
 */
Inline bool_t
scif_getchar(CELLCB *p_cellcb, char *p_c)
{
	uint8_t dat;
	uint8_t str;

	str = sil_reb_mem((void *)UDnSTR(UDnCH0));
	if ((str & 0x10) == 0x10) {
		dat = sil_reb_mem((void *) UDnRX(UDnCH0)); /* 受信データを読み込み */
		*p_c = dat;
		str &= ~0x10;
		sil_wrb_mem((void *)UDnSTR(UDnCH0), str);
		return true;
	}

	return(false);
}

/*
 *  送信する文字の書込み
 */
Inline void
scif_putchar(CELLCB *p_cellcb, char c)
{
	sil_wrb_mem((void *)UDnTX(UDnCH0), c);
}

/*
 *  シリアルI/Oポートのオープン
 */
void
eSIOPort_open(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);

	if (VAR_initialized) {
		/*
		 *  既に初期化している場合は、二重に初期化しない．
		 */
		return;
	}
	/*
	 * 通信設定
	 *
	 * UARTD0制御レジスタ0(UD0CTL0)
	 * UARTD0動作許可、送受信禁止、転送方向：LSB、パリティ：なし、データ：８ビット、ストップビット：１ビット
	 */
	sil_wrb_mem((void *) UDnCTL0(UDnCH0), 0x92);

	/*
	 * ボーレート設定
	 *
	 * UARTD0制御レジスタ1(UD0CTL1)
	 *  UARTD0クロック：fxx/2 (PRSI =0)
	 *
	 * UARTD0制御レジスタ2(UD0CTL2)
	 *  規定値：130(0x82) 、シリアルクロック：fuclk/130
	 */
	sil_wrb_mem((void *) UDnCTL1(UDnCH0), 0x01);
	sil_wrb_mem((void *) UDnCTL2(UDnCH0), 0x82);

	/*
	 * オプション設定
	 *
	 * UARTD0オプション制御レジスタ0(UD0OPT0)
	 *  送信データ通常出力、受信データ通常入力
	 *
	 * UARTD0オプション制御レジスタ1(UD0OPT1)
	 *  データ一貫性チェックなし
	 */
	sil_wrb_mem((void *) UDnOPT0(UDnCH0), 0x14);
	sil_wrb_mem((void *) UDnOPT1(UDnCH0), 0x00);

	/*
	 * 割込み許可設定
	 */
	/* 本API終了後，OS側で割り込み許可するためNOP */

	/*
	 * 送受信の許可
	 * UARTD0動作許可、送受信許可、転送方向：LSB、パリティ：なし、データ：８ビット、ストップビット：１ビット
	 */
	sil_wrb_mem((void *) UDnCTL0(UDnCH0), 0xF2);

	VAR_initialized = true;


}
/*
 *  シリアルI/Oポートのクローズ
 */
void
eSIOPort_close(CELLIDX idx)
{
	//TODO
}
/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
eSIOPort_putChar(CELLIDX idx, char c)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);

	if (scif_putready(p_cellcb)){
		scif_putchar(p_cellcb, c);
		return(true);
	}
	return(false);
}
/*
 *  シリアルI/Oポートからの文字受信
 */
int_t
eSIOPort_getChar(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	char	c;

	if (scif_getready(p_cellcb)) {
		if (scif_getchar(p_cellcb, &c)) {
			return((int_t) c);
		}
	}
	return(-1);
}
/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
eSIOPort_enableCBR(CELLIDX idx, uint_t cbrtn)
{
	//TODO
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
eSIOPort_disableCBR(CELLIDX idx, uint_t cbrtn)
{
	//TODO

}

/*
 *  シリアルI/Oポートに対する受信割込み処理
 */
void
eiRxISR_main(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);
	(void)x_clear_int(INTNO_INTUD0R);

	if (scif_getready(p_cellcb)) {
		/*
		 *  受信通知コールバックルーチンを呼び出す．
		 */
		ciSIOCBR_readyReceive();
	}

}

/*
 *  シリアルI/Oポートに対する送信割込み処理
 */
void
eiTxISR_main(CELLIDX idx)
{
	CELLCB	*p_cellcb = GET_CELLCB(idx);

	if (scif_putready(p_cellcb)) {
		/*
		 *  送信可能コールバックルーチンを呼び出す．
		 */
		ciSIOCBR_readySend();
	}

}
