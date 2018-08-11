/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
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
 *  $Id: uart.c 540 2015-12-29 01:00:01Z ertl-honda $
 */

/*
 *		シリアルI/Oデバイス（SIO）ドライバ（UART用）
 */

#include "Os.h"
#include "target_serial.h"
#include "uart.h"
#include "Os_Lcfg.h"
#include "kernel/kernel_impl.h"



/*
 *  カーネルの低レベル出力用関数
 */
void
uart_putc(char8 c)
{
	sil_wrb_mem((void *)UDnTX(UDnCH0), c);
}

/*
 *  初期化処理
 *
 *   本APIは，OS割込み禁止状態で呼び出される．
 *
 *   SCI0：38400bps,調歩同期式モード，8bit, パリティなし，stop=1bit, LSB
 */
void
InitHwSerial(void)
{
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
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
TermHwSerial(void)
{
}

/*
 *  受信完了割込みハンドラ
 */
ISR(RxHwSerialInt)
{
	uint8 dat;
	uint8 str;

	dat = sil_reb_mem((void *) UDnRX(UDnCH0)); /* 受信データを読み込み */

	str = sil_reb_mem((void *)UDnSTR(UDnCH0));
	str &= ~0x10;
	sil_wrb_mem((void *)UDnSTR(UDnCH0), str);
	RxSerialInt(dat);

	x_clear_int(36);
}

/*
 *  送信完了割込みハンドラ
 */
ISR(TxHwSerialInt)
{
}
