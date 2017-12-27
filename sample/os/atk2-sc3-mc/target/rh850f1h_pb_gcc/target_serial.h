/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
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
 *  $Id: target_serial.h 62 2014-09-08 12:19:38Z ertl-honda $
 */

/*
 *		シリアルI/Oデバイス（SIO）ドライバ（RH850F1H_PB用）
 */

#ifndef TOPPERS_TARGET_SERIAL_H
#define TOPPERS_TARGET_SERIAL_H

/*
 *  PE 毎に使用するチャネルのベースアドレス
 *  target_config.c で定義
 */
extern const uint32	rlin3x_base_table[TNUM_HWCORE];

/*
 *   ボーレートレジスタへの設定値 MainOSC=16MHz, 115200bps
 */
#define RLIN3xLWBR_VAL	0xf0    /* 1/1 分周 */
#define RLIN3xLBRP01_VAL	8   /* 9(9+1)分周  source clock / プリスケーラ / BRP01 / 16*/

/*
 *  SIOの割込みハンドラのベクタ番号
 */
#ifdef G_SYSLOG
#ifdef G_SYSLOG_RLIN30
#define INTNO_SIO_UART0	(0xffff0000 | (RLIN30_RX_INTNO))         /* 割込み番号 */
#else /* G_SYSLOG_RLIN31 */
#define INTNO_SIO_UART0	(0xffff0000 | (RLIN31_RX_INTNO))         /* 割込み番号 */
#endif /* G_SYSLOG_RLIN30 */
#else /* !G_SYSLOG */
#define INTNO_SIO_UART0	(0xffff0000 | (RLIN30_RX_INTNO))            /* 割込み番号 */
#define INTNO_SIO_UART1	(0xffff0000 | (RLIN31_RX_INTNO))            /* 割込み番号 */
#endif /* G_SYSLOG */

/*
 *  割込み優先度
 */
#ifdef G_SYSLOG
#ifdef G_SYSLOG_RLIN30
#define INTPRI_SIO_UART0	2
#else /* G_SYSLOG_RLIN31 */
#define INTPRI_SIO_UART0	2
#endif /* G_SYSLOG_RLIN30 */
#else /* !G_SYSLOG */
#define INTPRI_SIO_UART0	2
#define INTPRI_SIO_UART1	2
#endif /* G_SYSLOG */

/*
 *  プロセッサ依存モジュール
 */
#include "v850_gcc/uart_rlin.h"

#endif /* TOPPERS_TARGET_SERIAL_H */
