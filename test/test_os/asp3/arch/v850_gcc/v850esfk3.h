/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 * 
 *  Copyright (C) 2010 by Meika Sugimoto
 * 
 *  上記著作権者は，以下の (1)～(4) の条件か，Free Software Foundation
 *  によって公表されている GNU General Public License の Version 2 に記
 *  述されている条件を満たす場合に限り，本ソフトウェア（本ソフトウェア
 *  を改変したものを含む．以下同じ）を使用・複製・改変・再配布（以下，
 *  利用と呼ぶ）することを無償で許諾する．
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
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，その適用可能性も
 *  含めて，いかなる保証も行わない．また，本ソフトウェアの利用により直
 *  接的または間接的に生じたいかなる損害に関しても，その責任を負わない．
 * 
 */

#ifndef TOPPERS_V850ESFK3_H
#define TOPPERS_V850ESFK3_H

#define DEFAULT_STK_TOP		((STK_T *const)0xFFFF0000)
/*
 *  型キャストを行うマクロの定義
 */
#ifndef CAST
#define CAST(type, val)		((type)(val))
#endif /* CAST */

/* I/Oアドレス(必要なもののみ) */

/* ポート */
#define PCT			(0xFFFFF00A)
#define PCMT		(0xFFFFF02A)
#define PMC3L		(0xFFFFF446)
#define PFC3		(0xFFFFF466)

/* UART */
#define UA0CTL0		(0xFFFFFA00)
#define UA0CTL1		(0xFFFFFA01)
#define UA0CTL2		(0xFFFFFA02)
#define UA0STR		(0xFFFFFA04)

#define UA0TX		(0xFFFFFA07)
#define UA1TX		(0xFFFFFA17)
#define UA2TX		(0xFFFFFA27)

#define UA0RX		(0xFFFFFA06)
#define UA1RX		(0xFFFFFA16)
#define UA2RX		(0xFFFFFA26)

#define UA0STR		(0xFFFFFA04)
#define UA1STR		(0xFFFFFA14)
#define UA2STR		(0xFFFFFA24)

/* クロック */
#define PRCMD		(0xFFFFF1FC)
#define CKC			(0xFFFFF822)
#define LOCKR 		(0xFFFFF824)
#define VSWC		(0xFFFFF06E)
#define PLLCTL		(0xFFFFF82C)

/* ウォッチドッグ */
#define WDTM2		(0xFFFFF6D0)
#define RCM			(0xFFFFF80C)

/* タイマ */
#define TM0CMP0		(0xFFFFF694)
#define TM0CTL0		(0xFFFFF690)

/* 割込み */
#define INTF0		(0xFFFFFC00)
#define INTR0		(0xFFFFFC20)
#define INTF3		(0xFFFFFC00)
#define INTR3		(0xFFFFFC20)
#define INTF9H		(0xFFFFFC13)
#define INTR9H		(0xFFFFFC33)

#define IMR_SIZE	(8)
#define IMR0		(0xFFFFF100)
#define IMR1		(0xFFFFF102)
#define IMR2		(0xFFFFF104)
#define IMR3		(0xFFFFF106)
#define IMR4		(0xFFFFF108)
#define IMR5		(0xFFFFF10A)
#define IMR6		(0xFFFFF10C)
#define IMR7		(0xFFFFF10E)


/*
 *	割込み制御レジスタの番地を算出するためのマクロ
 *
 *	割込み制御レジスタは割込み番号順に並んでいるため，
 *	ベースアドレスからのオフセットでアドレスを求めることができる．
 */

#define INTREG_BASE				(0xFFFFF110)
#define INTREG_ADDRESS(intno)	(INTREG_BASE + ((intno) * 2U))

/*
 *	外部割込み極性設定レジスタテーブル
 */

#define INT_POLREG_TABLE											\
	{ 0 , 0 , 0 } , 			/* リセット番地なので無効 */		\
	{ INTF0 , INTR0 , 2 } ,		/* NMI */							\
	{ 0, 0 , 0 } , 				/* 外部割込みで出ないので無効 */	\
	{ 0 , 0 , 0 } , 			/* 外部割込みで出ないので無効 */	\
	{ 0 , 0 , 0 } , 			/* 外部割込みで出ないので無効 */	\
	{ 0 , 0 , 0 } , 			/* 外部割込みで出ないので無効 */	\
	{ 0 , 0 , 0 } , 			/* 外部割込みで出ないので無効 */	\
	{ 0 , 0 , 0 } , 			/* 外部割込みで出ないので無効 */	\
	{ INTF0 , INTR0 , 3 } , 	/* INT0 */							\
	{ INTF0 , INTR0 , 4 } , 	/* INT1 */							\
	{ INTF0 , INTR0 , 5 } , 	/* INT2 */							\
	{ INTF0 , INTR0 , 6 } ,		/* INT3 */							\
	{ INTF9H , INTR9H , 5 } , 	/* INT4 */							\
	{ INTF9H , INTR9H , 6 } , 	/* INT5 */							\
	{ INTF9H , INTR9H , 7 } , 	/* INT6 */							\
	{ INTF3 , INTR3 , 1 } 		/* INT7 */

#define SCIF2_BASE			0xe8008000


#define INTNO_SCIF0_BRI		10		/* SCIF0 ブレーク割込み */
#define INTNO_SCIF0_ERI		11		/* SCIF0 エラー割込み */
#define INTNO_SCIF0_RXI		12		/* SCIF0 受信割込み */
#define INTNO_SCIF0_TXI		13		/* SCIF0 送信割込み */
#define INTNO_SCIF1_BRI		14		/* SCIF1 ブレーク割込み */
#define INTNO_SCIF1_ERI		15		/* SCIF1 エラー割込み */
#define INTNO_SCIF1_RXI		16		/* SCIF1 受信割込み */
#define INTNO_SCIF1_TXI		17		/* SCIF1 送信割込み */
#define INTNO_SCIF2_BRI		18		/* SCIF2 ブレーク割込み */
#define INTNO_SCIF2_ERI		19		/* SCIF2 エラー割込み */
#define INTNO_SCIF2_RXI		20		/* SCIF2 受信割込み */
#define INTNO_SCIF2_TXI		21		/* SCIF2 送信割込み */
#define INTNO_SCIF3_BRI		22		/* SCIF3 ブレーク割込み */
#define INTNO_SCIF3_ERI		23		/* SCIF3 エラー割込み */
#define INTNO_SCIF3_RXI		24		/* SCIF3 受信割込み */
#define INTNO_SCIF3_TXI		25		/* SCIF3 送信割込み */
#define INTNO_SCIF4_BRI		26		/* SCIF4 ブレーク割込み */
#define INTNO_SCIF4_ERI		27		/* SCIF4 エラー割込み */
#define INTNO_SCIF4_RXI		28		/* SCIF4 受信割込み */
#define INTNO_SCIF4_TXI		29		/* SCIF4 送信割込み */


#define TNUM_INT			117U
#define TMIN_INTNO			0U
#define TMAX_INTNO			116U
#define TNUM_INTNO			TNUM_INT

/*************************************************
 * 16ビットタイマ／イベントカウンタAA(TAA)
 *************************************************/

#define TAAnChannelNum			UINT_C(8)
#define TAAnCH0					UINT_C(0)
#define TAAnCH1					UINT_C(1)
#define TAAnCH2					UINT_C(2)
#define TAAnCH3					UINT_C(3)
#define TAAnCH4					UINT_C(4)
#define TAAnCH5					UINT_C(5)
#define TAAnCH6					UINT_C(6)
#define TAAnCH7					UINT_C(7)




#endif	/* TOPPERS_V850ESFK3_H */
