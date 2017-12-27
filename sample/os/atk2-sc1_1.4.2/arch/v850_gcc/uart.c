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

/*
 *  受信用バッファ
 */
static uint8 rx_buf;

/* 内部関数のプロトタイプ宣言 */
LOCAL_INLINE boolean uart_getready(void);
LOCAL_INLINE uint8 uart_getchar(void);

/*
 *  文字を受信したかチェック
 */
LOCAL_INLINE boolean
uart_getready(void)
{
	boolean ans;
	uint8	stat;
	uint8	rx_tmp;

	stat = sil_reb_mem((void *) URTEnSTR1);                     /* ステータスの読み込み */
	rx_tmp = sil_reb_mem((void *) URTEnRX);                     /* 受信データを読み込み */

	ans = FALSE;
	if ((stat & 0x07) == 0)	{
		rx_buf = rx_tmp;
		ans = TRUE;
	}
	sil_wrb_mem((void *) URTEnSTC, 0x1f);                            /* ステータスクリア */

	return(ans);
}

/*
 *  受信した文字の取り出し
 */
LOCAL_INLINE uint8
uart_getchar(void)
{
	return(rx_buf);
}

/*
 *  初期化処理
 */
void
InitHwSerial(void)
{
	sil_wrh_mem((void *) URTEnCTL2, URTEnCTL2_VAL);
	sil_wrh_mem((void *) URTEnCTL1, 0x0103);        /* 8bit, LSB First */
	sil_wrb_mem((void *) URTEnCTL0, 0x80);
	sil_wrb_mem((void *) URTEnCTL0, 0xe0);
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
TermHwSerial(void)
{
	/* 受信割込みの禁止 */
	sil_wrb_mem((void *) URTEnCTL0, 0x00);
}

/*
 *  SIOの割込みハンドラ
 */
ISR(RxHwSerialInt)
{
	if (uart_getready() != FALSE) {
		/*
		 *  受信通知コールバックルーチンを呼び出す
		 */
		RxSerialInt(uart_getchar());
	}
}
