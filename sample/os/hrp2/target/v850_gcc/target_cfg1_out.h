/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 * 
 *  Copyright (C) 2010 by Meika Sugimoto
 * 
 *  上記著作権者は，以下の(1)~(4)の条件を満たす場合に限り，本ソフトウェ
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
 */

#ifndef TARGET_CFG1_OUT_H
#define TARGET_CFG1_OUT_H

void sta_ker(void){}

/* 
 *  オフセットファイルを生成するための定義 
 */ 
const uint8_t   MAGIC_1 = 0x12; 
const uint16_t  MAGIC_2 = 0x1234; 
const uint32_t  MAGIC_4 = 0x12345678;

const TCB	TCB_enatex = {
	{ NULL, NULL },			/* task_queue */
	NULL,					/* p_tinib */
	0U,						/* tstat */
	0U,						/* svclevel */
	0U,						/* bpriority */
	0U,						/* priority */
	false,					/* acqeue */
	false,					/* wupque */
	true,					/* enatex */
	false,					/* waifbd */
	0U,						/* texprn */
	NULL,					/* p_winifo */
	{ NULL, NULL },			/* mutex_queue */
#ifdef TOPPERS_SUPPORT_OVRHDR
	0U,						/* leftotm */
#endif /* TOPPERS_SUPPORT_OVRHDR */
	{ NULL, NULL, 0, 0 }		/* tskctxb */
};

const TCB	TCB_waifbd = {
	{ NULL, NULL },		/* タスクキュー */
	NULL,				/* 初期化ブロックへのポインタ */
	0,					/* タスク状態（内部表現）*/
	0,					/* 拡張サービスコールのネストレベル */
	0,					/* ベース優先度（内部表現）*/
	0,					/* 現在の優先度（内部表現）*/
	false,				/* 起動要求キューイング */
	false,				/* 起床要求キューイング */
	false,				/* タスク例外処理許可状態 */
	true,				/* 待ち禁止状態 */
	0,					/* 保留例外要因 */
	NULL,				/* 待ち情報ブロックへのポインタ */
	{ NULL, NULL },		/* ロックしているミューテックスのキュー */
#ifdef TOPPERS_SUPPORT_OVRHDR
	0U,						/* leftotm */
#endif /* TOPPERS_SUPPORT_OVRHDR */
	{ NULL, NULL, 0, 0 }		/* tskctxb */
};

const TCB	TCB_svclevel = {
	{ NULL, NULL },		/* タスクキュー */
	NULL,				/* 初期化ブロックへのポインタ */
	0,					/* タスク状態（内部表現）*/
	1,					/* 拡張サービスコールのネストレベル */
	0,					/* ベース優先度（内部表現）*/
	0,					/* 現在の優先度（内部表現）*/
	false,				/* 起動要求キューイング */
	false,				/* 起床要求キューイング */
	false,				/* タスク例外処理許可状態 */
	false,				/* 待ち禁止状態 */
	0,					/* 保留例外要因 */
	NULL,				/* 待ち情報ブロックへのポインタ */
	{ NULL, NULL },		/* ロックしているミューテックスのキュー */
#ifdef TOPPERS_SUPPORT_OVRHDR
	0U,						/* leftotm */
#endif /* TOPPERS_SUPPORT_OVRHDR */
	{ NULL, NULL, 0, 0 }		/* tskctxb */
};


#endif	/* TARGET_CFG1_OUT_H */
