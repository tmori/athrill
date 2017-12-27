/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2011-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2015 by Witz Corporation
 *  Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015 by SUZUKI MOTOR CORPORATION
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
 *  $Id: kernel_fncode.h 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*  This file is generated from svc.def by gensvc_atk. */

#ifndef TOPPERS_KERNEL_FNCODE_H
#define TOPPERS_KERNEL_FNCODE_H

#define TMAX_SVCID	(72 + TARGET_SVC_NUM)

#define TFN_GETCOREID					UINT_C(0)
#define TFN_GETNUMBEROFACTIVATEDCORES	UINT_C(1)
#define TFN_STARTCORE					UINT_C(2)
#define TFN_STARTNONAUTOSARCORE			UINT_C(3)
#define TFN_STARTOS						UINT_C(4)
#define TFN_SHUTDOWNALLCORES			UINT_C(5)
#define TFN_ACTIVATETASK				UINT_C(6)
#define TFN_TERMINATETASK				UINT_C(7)
#define TFN_CHAINTASK					UINT_C(8)
#define TFN_SCHEDULE					UINT_C(9)
#define TFN_GETTASKID					UINT_C(10)
#define TFN_GETTASKSTATE				UINT_C(11)
#define TFN_ENABLEALLINTERRUPTS			UINT_C(12)
#define TFN_DISABLEALLINTERRUPTS		UINT_C(13)
#define TFN_RESUMEALLINTERRUPTS			UINT_C(14)
#define TFN_SUSPENDALLINTERRUPTS		UINT_C(15)
#define TFN_RESUMEOSINTERRUPTS			UINT_C(16)
#define TFN_SUSPENDOSINTERRUPTS			UINT_C(17)
#define TFN_GETISRID					UINT_C(18)
#define TFN_DISABLEINTERRUPTSOURCE		UINT_C(19)
#define TFN_ENABLEINTERRUPTSOURCE		UINT_C(20)
#define TFN_GETRESOURCE					UINT_C(21)
#define TFN_RELEASERESOURCE				UINT_C(22)
#define TFN_SETEVENT					UINT_C(23)
#define TFN_CLEAREVENT					UINT_C(24)
#define TFN_GETEVENT					UINT_C(25)
#define TFN_WAITEVENT					UINT_C(26)
#define TFN_GETALARMBASE				UINT_C(27)
#define TFN_GETALARM					UINT_C(28)
#define TFN_SETRELALARM					UINT_C(29)
#define TFN_SETABSALARM					UINT_C(30)
#define TFN_CANCELALARM					UINT_C(31)
#define TFN_INCREMENTCOUNTER			UINT_C(32)
#define TFN_GETCOUNTERVALUE				UINT_C(33)
#define TFN_GETELAPSEDVALUE				UINT_C(34)
#define TFN_CHECKISRMEMORYACCESS		UINT_C(35)
#define TFN_CHECKTASKMEMORYACCESS		UINT_C(36)
#define TFN_CHECKTASKACCESS				UINT_C(37)
#define TFN_CHECKISRACCESS				UINT_C(38)
#define TFN_CHECKALARMACCESS			UINT_C(39)
#define TFN_CHECKRESOURCEACCESS			UINT_C(40)
#define TFN_CHECKCOUNTERACCESS			UINT_C(41)
#define TFN_CHECKSCHEDULETABLEACCESS	UINT_C(42)
#define TFN_CHECKSPINLOCKACCESS			UINT_C(43)
#define TFN_CHECKTASKOWNERSHIP			UINT_C(44)
#define TFN_CHECKISROWNERSHIP			UINT_C(45)
#define TFN_CHECKALARMOWNERSHIP			UINT_C(46)
#define TFN_CHECKCOUNTEROWNERSHIP		UINT_C(47)
#define TFN_CHECKSCHEDULETABLEOWNERSHIP	UINT_C(48)
#define TFN_GETAPPLICATIONID			UINT_C(49)
#define TFN_CALLTRUSTEDFUNCTION			UINT_C(50)
#define TFN_GETAPPLICATIONSTATE			UINT_C(51)
#define TFN_GETACTIVEAPPLICATIONMODE	UINT_C(52)
#define TFN_STARTSCHEDULETABLEREL		UINT_C(53)
#define TFN_STARTSCHEDULETABLEABS		UINT_C(54)
#define TFN_STOPSCHEDULETABLE			UINT_C(55)
#define TFN_NEXTSCHEDULETABLE			UINT_C(56)
#define TFN_GETSCHEDULETABLESTATUS		UINT_C(57)
#define TFN_GETFAULTYCONTEXT			UINT_C(58)
#define TFN_GETSPINLOCK					UINT_C(59)
#define TFN_RELEASESPINLOCK				UINT_C(60)
#define TFN_TRYTOGETSPINLOCK			UINT_C(61)
#define TFN_GET_ERROR_SVCID				UINT_C(62)
#define TFN_GET_ERROR_PAR				UINT_C(63)
#define TFN_IOC_SEND_GENERIC			UINT_C(64)
#define TFN_IOC_WRITE_GENERIC			UINT_C(65)
#define TFN_IOC_RECEIVE_GENERIC			UINT_C(66)
#define TFN_IOC_READ_GENERIC			UINT_C(67)
#define TFN_IOC_EMPTY_QUEUE_GENERIC		UINT_C(68)
#define TFN_RAISEINTERCOREINTERRUPT		UINT_C(69)
#define TFN_ALLOWACCESS					UINT_C(70)
#define TFN_TERMINATEAPPLICATION		UINT_C(71)

#endif /* TOPPERS_KERNEL_FNCODE_H */
