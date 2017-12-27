/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2011-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2017 by Witz Corporation
 *  Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION
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
 *  $Id: kernel_fncode.h 727 2017-01-23 09:27:59Z witz-itoyo $
 */

#ifndef TOPPERS_KERNEL_FNCODE_H
#define TOPPERS_KERNEL_FNCODE_H

#define TMAX_SVCID	UINT_C(36)

#define TFN_STARTOS						UINT_C(0)
#define TFN_SHUTDOWNOS					UINT_C(1)
#define TFN_ACTIVATETASK				UINT_C(2)
#define TFN_TERMINATETASK				UINT_C(3)
#define TFN_CHAINTASK					UINT_C(4)
#define TFN_SCHEDULE					UINT_C(5)
#define TFN_GETTASKID					UINT_C(6)
#define TFN_GETTASKSTATE				UINT_C(7)
#define TFN_ENABLEALLINTERRUPTS			UINT_C(8)
#define TFN_DISABLEALLINTERRUPTS		UINT_C(9)
#define TFN_RESUMEALLINTERRUPTS			UINT_C(10)
#define TFN_SUSPENDALLINTERRUPTS		UINT_C(11)
#define TFN_RESUMEOSINTERRUPTS			UINT_C(12)
#define TFN_SUSPENDOSINTERRUPTS			UINT_C(13)
#define TFN_GETISRID					UINT_C(14)
#define TFN_GETRESOURCE					UINT_C(15)
#define TFN_RELEASERESOURCE				UINT_C(16)
#define TFN_SETEVENT					UINT_C(17)
#define TFN_CLEAREVENT					UINT_C(18)
#define TFN_GETEVENT					UINT_C(19)
#define TFN_WAITEVENT					UINT_C(20)
#define TFN_GETALARMBASE				UINT_C(21)
#define TFN_GETALARM					UINT_C(22)
#define TFN_SETRELALARM					UINT_C(23)
#define TFN_SETABSALARM					UINT_C(24)
#define TFN_CANCELALARM					UINT_C(25)
#define TFN_INCREMENTCOUNTER			UINT_C(26)
#define TFN_GETCOUNTERVALUE				UINT_C(27)
#define TFN_GETELAPSEDVALUE				UINT_C(28)
#define TFN_GETACTIVEAPPLICATIONMODE	UINT_C(29)
#define TFN_STARTSCHEDULETABLEREL		UINT_C(30)
#define TFN_STARTSCHEDULETABLEABS		UINT_C(31)
#define TFN_STOPSCHEDULETABLE			UINT_C(32)
#define TFN_NEXTSCHEDULETABLE			UINT_C(33)
#define TFN_GETSCHEDULETABLESTATUS		UINT_C(34)
#define TFN_DISABLEINTERRUPTSOURCE		UINT_C(35)
#define TFN_ENABLEINTERRUPTSOURCE		UINT_C(36)

#endif /* TOPPERS_KERNEL_FNCODE_H */
