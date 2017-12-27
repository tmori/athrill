/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2013 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2013 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by FUJITSU VLSI LIMITED, JAPAN
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2013 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2013 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2013 by Witz Corporation, JAPAN
 *  Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: prc_kernel.h 187 2015-06-25 03:39:04Z t_ishikawa $
 */

/*
 *		Os.hのプロセッサ依存部（V850E2/FL4用）
 *
 *  このインクルードファイルは，Os.hでインクルードされる．他のファ
 *  イルから直接インクルードすることはない
 */

#ifndef TOPPERS_PRC_KERNEL_H
#define TOPPERS_PRC_KERNEL_H

#ifndef TOPPERS_MACRO_ONLY

typedef uint32	SystemTimeMsType;
typedef uint32	SystemTimeUsType;
typedef uint32	SystemTime100NsType;

#ifdef CFG_USE_PROTECTIONHOOK
/*
 *  CPU例外要因情報保持変数外部参照宣言（prc_config.c）
 *  ユーザ作成するプロテクションフックから参照するため，
 *  Os.hからインクルードされるprc_kernel.hで参照宣言
 */
/* CPU例外発生時，CPU例外要因番号を保持する変数 */
extern uint32	v850_cpu_exp_no;

/* CPU例外発生時，プログラムカウンタ（PC - 4）を保持する変数 */
extern uint32	v850_cpu_exp_pc;

/* CPU例外発生時，スタックポインタを保持する変数 */
extern uint32	v850_cpu_exp_sp;
#endif /* CFG_USE_PROTECTIONHOOK */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  サポートする機能の定義
 */
#define TOPPERS_SUPPORT_GET_UTM         /* get_utmをサポートする */
#define TOPPERS_TARGET_SUPPORT_ATT_MOD		/* ATT_MOD/ATA_MOD */


#endif /* TOPPERS_PRC_KERNEL_H */
