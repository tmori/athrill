/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJISOFT INCORPORATED, JAPAN
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
 *  $Id: v850.h 549 2015-12-30 10:06:17Z ertl-honda $
 */

/*
 *		V850のハードウェア資源の定義（開発環境共通）
 */

#ifndef TOPPERS_V850_H
#define TOPPERS_V850_H

#define MAGIC_START	UINT_C(0x87654321) /* 同期用のマジックナンバー */

#ifdef __v850e2v3__


/*
 *  V850E2用の割込みコントローラ操作ルーチン
 */
#ifndef TOPPERS_MACRO_ONLY

#include "prc_sil.h"


#endif /* TOPPERS_MACRO_ONLY */

#elif defined(__v850e3v5__)

#ifdef _RH850G3M_
#define TNUM_INTPRI	16
#elif defined(_RH850G3K_)
#define TNUM_INTPRI	8
#else
#error please define ether _RH850G3M_ or _RH850G3K_
#endif /* _RH850G3M_ */

#else /* __v850e3v5__ */
#error please define ether __v850e2v3__ or __v850e3v5__
#endif /* __v850e2v3__ */


#define TMIN_INTNO			0U
#define TMAX_INTNO			116U
/*
 * <TRACE> 7.1 設定ファイルとターゲット依存部の位置付け (b) 値取得シンボルテーブル
 *
 * 割込み番号54は欠番であるため，
 * 使用可能な総数は116個．
 */
#define TNUM_INT			117U

/*
 * <TRACE> 7.1 設定ファイルとターゲット依存部の位置付け (b) 値取得シンボルテーブル
 * <TRACE> (7-2-1-3) TNUM_INTPRI
 *
 * 割込み優先度の数
 */
#define TNUM_INTPRI			8U
#define IMR_SIZE		8U

/*
 * 割込みコントローラ
 */
/*
 * IMR0-IMR7
 */
#define INTC_BASE	UINT_C(0xFFFFF110)
#define INTC_IMR0	UINT_C(0xFFFFF100)
#define INTC_IMR1	UINT_C(0xFFFFF102)
#define INTC_IMR2	UINT_C(0xFFFFF104)
#define INTC_IMR3	UINT_C(0xFFFFF106)
#define INTC_IMR4	UINT_C(0xFFFFF108)
#define INTC_IMR5	UINT_C(0xFFFFF10A)
#define INTC_IMR6	UINT_C(0xFFFFF10C)
#define INTC_IMR7	UINT_C(0xFFFFF10E)



/*
 * アシンクロナス・シリアル・インタフェース（ UARTD）
 */
#define UDnChannelNum			UINT_C(8)
#define UDnCH0					UINT_C(0)
#define UDnCH1					UINT_C(1)
#define UDnCH2					UINT_C(2)
#define UDnCH3					UINT_C(3)
#define UDnCH4					UINT_C(4)
#define UDnCH5					UINT_C(5)
#define UDnCH6					UINT_C(6)
#define UDnCH7					UINT_C(7)

/*
 * UARTDn制御レジスタ 0（ UDnCTL0）
 */
#define UDnCTL0_BASE			UINT_C(0xFFFFFA00)
#define UDnCTL0(CH)				(UDnCTL0_BASE + ((CH) * 16U))

/*
 * UARTDn制御レジスタ 1（ UDnCTL1）
 */
#define UDnCTL1_BASE			UINT_C(0xFFFFFA01)
#define UDnCTL1(CH)				(UDnCTL1_BASE + ((CH) * 16U))
/*
 * UARTDn制御レジスタ 2（ UDnCTL2）
 */
#define UDnCTL2_BASE			UINT_C(0xFFFFFA02)
#define UDnCTL2(CH)				(UDnCTL2_BASE + ((CH) * 16U))

/*
 * UARTDn オプション制御レジスタ 0（ UDnOPT0）
 */
#define UDnOPT0_BASE			UINT_C(0xFFFFFA03)
#define UDnOPT0(CH)				(UDnOPT0_BASE + ((CH) * 16U))

/*
 * UARTDn オプション制御レジスタ 1（ UDnOPT1）
 */
#define UDnOPT1_BASE			UINT_C(0xFFFFFA05)
#define UDnOPT1(CH)				(UDnOPT1_BASE + ((CH) * 16U))

/*
 * UARTDn状態レジスタ（ UDnSTR）
 */
#define UDnSTR_BASE				UINT_C(0xFFFFFA04)
#define UDnSTR(CH)				(UDnSTR_BASE + ((CH) * 16U))

/*
 * UARTDn送信データ・レジスタ（ UDnTX）
 */
#define UDnTX_BASE				UINT_C(0xFFFFFA07)
#define UDnTX(CH)				(UDnTX_BASE + ((CH) * 16U))

/*
 * UARTDn受信データ・レジスタ（ UDnRX）
 */
#define UDnRX_BASE				UINT_C(0xFFFFFA06)
#define UDnRX(CH)				(UDnRX_BASE + ((CH) * 16U))

/*
 * ISPR
 */
#define INTC_ISPR	UINT_C(0xFFFFF1FA)

/*
 *  V850ESFK3用の割込みコントローラ操作ルーチン
 */
 /* intno は unsigned を想定 */
#define EIC_ADDRESS(intno)	(INTC_BASE + (intno * 2U))
#endif /* TOPPERS_V850_H */
