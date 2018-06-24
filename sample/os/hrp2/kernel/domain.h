/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 * 
 *  Copyright (C) 2008-2010 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: domain.h 263 2010-02-14 05:32:25Z ertl-hiro $
 */

/*
 *		保護ドメイン管理モジュール
 */

#ifndef TOPPERS_DOMAIN_H
#define TOPPERS_DOMAIN_H

#ifndef TOPPERS_MACRO_ONLY

/*
 *  保護ドメイン初期化ブロック
 */
typedef struct domain_initialization_block {
	ACPTN		domptn;			/* 保護ドメインのビットパターン */
#ifdef USE_DOMINICTXB
	DOMINICTXB	domctxb;		/* 保護ドメイン初期化コンテキストブロック */
#endif /* USE_DOMINICTXB */
} DOMINIB;

/*
 *  保護ドメインIDの最大値（kernel_cfg.c）
 */
extern const ID	tmax_domid;

/*
 *  保護ドメイン初期化ブロックのエリア（kernel_cfg.c）
 */
extern const DOMINIB	dominib_table[];

/*
 *  カーネルドメインの保護ドメイン初期化ブロック（kernel_cfg.c）
 */
extern const DOMINIB	dominib_kernel;

/*
 *  保護ドメインIDから保護ドメイン初期化ブロックを取り出すためのマクロ
 */
#define INDEX_DOM(domid)	((uint_t)((domid) - TMIN_DOMID))
#define get_dominib(domid)	(&(dominib_table[INDEX_DOM(domid)]))

/*
 *  保護ドメイン初期化ブロックから保護ドメインIDを取り出すためのマクロ
 */
#define	DOMID(p_dominib)	(((p_dominib) == &dominib_kernel) ? TDOM_KERNEL \
							: (ID)(((p_dominib) - dominib_table) + TMIN_DOMID))

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_DOMAIN_H_ */
